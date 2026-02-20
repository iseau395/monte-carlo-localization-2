import { distance_sensor_max, gaussian_random, height, use_normal_dist, width } from "./constants";
import { DistanceSensor, predict_distance, Robot } from "./robot";
import { erf } from "mathjs";
import color_convert from "color-convert";

function cdf_normal(x: number, mean: number, standardDeviation: number) {
    return (1 - erf((mean - x) / (Math.sqrt(2) * standardDeviation))) / 2
}

function prob_normal(x: number, mean: number, standardDeviation: number)
{
    if (x < mean)
    {
        return cdf_normal(x, mean, standardDeviation);
    }
    else
    {
        return cdf_normal(mean - (x - mean), mean, standardDeviation);
    }
}

interface Particle
{
    x: number;
    y: number;
    weight: number;
}

export class MonteCarlo
{
    private readonly distance_sensors: DistanceSensor[];
    private particles = new Array<Particle>();

    private margin = 0;

    readonly odom_stdev = 0.1;
    readonly percent_random = 0.05;

    constructor(...distance_sensors: DistanceSensor[])
    {
        this.distance_sensors = distance_sensors;
    }

    initialize(start_x: number, start_y: number, particle_count: number, robot: Robot, initial_stdev = 0.5)
    {
        this.particles.length = 0;
        
        this.margin = 0.5 * Math.min(robot.width, robot.length)

        for (let i = 0; i < particle_count; i++)
        {
            const x = this.margin + Math.random() * (width - 2*this.margin);
            const y = this.margin + Math.random() * (height - 2*this.margin);

            this.particles.push({
                x: x,
                y: y,
                weight: prob_normal(x, start_x, initial_stdev) * prob_normal(y, start_y, initial_stdev)
            });
        }
    }

    private motion_update(delta_x: number, delta_y: number)
    {
        for (let i = 0; i < this.particles.length; i++) {
            this.particles[i].x += gaussian_random(delta_x, Math.max(0.1, delta_x * this.odom_stdev));
            this.particles[i].y += gaussian_random(delta_y, Math.max(0.1, delta_y * this.odom_stdev));
        }
    }

    private resample()
    {
        let max_weight = 0;
        for (const particle of this.particles) {
            if (!isNaN(particle.weight) && particle.weight > max_weight)
            {
                max_weight = particle.weight;
            }
        }

        // console.log(this.particles.map(v => v.weight));

        const full_random_particles = max_weight != 0 ? this.particles.length * this.percent_random : this.particles.length;
        // const full_random_particles = 0;

        const new_particles = new Array<Particle>();
        
        
        let index = Math.floor(Math.random() * this.particles.length);
        let beta = 0;
        for (let i = 0; i < this.particles.length - full_random_particles; i++) {
            beta += Math.random() * 2 * max_weight;
            while (beta > this.particles[index].weight) {
                beta -= this.particles[index].weight;
                index = (index + 1) % this.particles.length;
            }

            new_particles.push({
                x: this.particles[index].x,
                y: this.particles[index].y,
                weight: 1
            });
        }

        for (let i = 0; i < full_random_particles; i++) {
            new_particles.push({
                x: this.margin + Math.random() * (width - 2*this.margin),
                y: this.margin + Math.random() * (height - 2*this.margin),
                weight: 1
            });
        }

        this.particles = new_particles;
    }

    private get_sensor_probability(actual: number, x: number, y: number, theta: number)
    {
        if (0 > x || x > width) return 0;
        if (0 > y || y > height) return 0;

        const expected = predict_distance(x, y, theta, false);

        if (use_normal_dist)
        {
            if (isFinite(actual))
            {
                return Math.max(prob_normal(actual, expected, DistanceSensor.predict_stdev(expected)), 0.001);
            }
            else if (expected > distance_sensor_max)
            {
                return 1;
            }
            else
            {
                return 0.001;
            }
        }
        else
        {
            if (isFinite(actual) && (0.95 * actual < expected) && (1.05 * actual > expected))
            {
                return 1;
            }
            else if (!isFinite(actual) && expected > distance_sensor_max)
            {
                return 1;
            }
            else
            {
                return 0.5 / (Math.abs(actual - expected) + 1);
            }
        }
    }

    private get_particle_probability(particle_x: number, particle_y: number, robot_theta: number, distance_values: number[])
    {
        let weight = 1;

        // if (particle_x == 72 && particle_y == 72)
        //     console.log(distance_values);
        
        for (const [i, distance_sensor] of this.distance_sensors.entries())
        {
            const [x, y, theta] = distance_sensor.get_position(particle_x, particle_y, robot_theta);

            weight *= this.get_sensor_probability(distance_values[i], x, y, theta);

            if (weight == 0) return 0;
        }

        return weight;
    }

    private sensor_update(robot: Robot)
    {
        const distance_values = this.distance_sensors.map(v => v.get_distance(robot));

        for (const particle of this.particles)
        {
            if (this.margin >= particle.x || width - this.margin <= particle.x)
            {
                particle.weight = 0;
                continue;
            }
            if (this.margin >= particle.y || height - this.margin <= particle.y)
            {
                particle.weight = 0;
                continue;
            }

            particle.weight = this.get_particle_probability(particle.x, particle.y, robot.theta, distance_values);
        }
    }

    tick(delta_x: number, delta_y: number, robot: Robot)
    {
        this.resample();
        this.motion_update(delta_x, delta_y);
        this.sensor_update(robot);

        let weighted_total_x = 0;
        let weighted_total_y = 0;
        let total_weight = 0;

        for (const particle of this.particles)
        {
            weighted_total_x += particle.x * particle.weight;
            weighted_total_y += particle.y * particle.weight;
            total_weight += particle.weight;
        }

        return [weighted_total_x / total_weight, weighted_total_y / total_weight, total_weight / this.particles.length];
    }

    private max_weight_cache = 0;
    draw(ctx: CanvasRenderingContext2D, robot: Robot)
    {
        for (const distance_sensor of this.distance_sensors)
        {
            distance_sensor.draw(ctx, robot);
        }

        let max_weight = 0;
        for (const particle of this.particles)
        {
            if (particle.weight > max_weight)
            {
                max_weight = particle.weight;
            }
        }


        // this.max_weight_cache = Math.max(max_weight, this.max_weight_cache);
        this.max_weight_cache = max_weight;

        for (const particle of this.particles)
        {
            const t = particle.weight / max_weight;

            ctx.beginPath();
            ctx.arc(particle.x, particle.y, 0.25, 0, 2*Math.PI);
            // ctx.fillStyle = `rgba(0, 0, 255, ${255 * t})`;
            ctx.fillStyle = `hsl(${120 * t}, 100%, 50%)`;
            ctx.fill();
        }
    }

    async draw_heat_map(ctx: CanvasRenderingContext2D, robot: Robot)
    {
        const img_width = 2*width;
        const img_height = 2*height;

        const img = ctx.createImageData(img_width, img_height);
        
        const distance_values = this.distance_sensors.map(v => v.get_distance(robot));

        for (let i = 0; i < img.data.length; i += 4)
        {
            const i_pixel = Math.floor(i / 4)

            const x = i_pixel % img_width;
            const y = Math.floor(i_pixel / img_width);

            const weight = this.get_particle_probability(x/2, y/2, robot.theta, distance_values);

            const rgb = color_convert.hsv.rgb((weight ?? 0) * (120 / (this.max_weight_cache ?? 0.001)), 100, 100);

            if (!rgb) continue;

            img.data[i+0] = rgb[0];
            img.data[i+1] = rgb[1];
            img.data[i+2] = rgb[2];
            img.data[i+3] = 0.5 * 255;
        }

        ctx.putImageData(img, 0, 0);
    }
}