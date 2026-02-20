import { distance_sensor_max, gaussian_random, height, use_normal_dist, width } from "./constants";

function get_vector(length: number, theta: number): [number, number]
{
    return [length * Math.cos(theta), length * Math.sin(theta)];
}

function add(...vectors: [number, number][])
{
    return vectors.reduce((previous, current) => [previous[0] + current[0], previous[1] + current[1]]);
}

function is_bounded(x: number, a: number, b: number)
{
    if (b < a)
    {
        let temp = a;
        a = b;
        b = temp;
    }

    return a <= x && x <= b;
}

function get_distance_to_line_segment(x: number, y: number, theta: number, point_a: [number, number], point_b: [number, number])
{
    const vertical_1 = point_b[0] - point_a[0] == 0;
    const vertical_2 = Math.cos(theta) == 0;
    
    let m1 = (point_b[1] - point_a[1]) / (point_b[0] - point_a[0]);
    let m2 = Math.tan(theta);

    let hit_x: number;
    let hit_y: number;

    if (vertical_1 && vertical_2)
    {
        return Infinity;
    }
    if (vertical_1)
    {
        hit_x = point_a[0];
        hit_y = m2 * (hit_x - x) + y;
    }
    else if (vertical_2)
    {
        hit_x = x;
        hit_y = m1 * (hit_x - point_a[0]) + point_a[1];
    }
    else
    {
        const b1 = -m1 * point_a[0] + point_a[1];
        const b2 = -m2 * x + y;

        hit_x = (b2 - b1) / (m1 - m2);
        hit_y = m1 * hit_x + b1;
    }

    if (is_bounded(hit_x, point_a[0], point_b[0]) && is_bounded(hit_y, point_a[1], point_b[1]))
    {
        if (
            (hit_x - x > 0) == (Math.cos(theta) > 0) &&
            (hit_y - y > 0) == (Math.sin(theta) > 0)
        )
        {
            const delta_x = hit_x - x;
            const delta_y = hit_y - y;

            const distance =  Math.sqrt(delta_x * delta_x + delta_y * delta_y);

            return distance;
        }
    }

    return Infinity;
}

function get_distance_to_circle(ray_x: number, ray_y: number, theta: number, center_x: number, center_y: number, r: number)
{
    const x = ray_x - center_x;
    const y = ray_y - center_y;

    const cos = Math.cos(theta);
    const sin = Math.sin(theta);

    const a = -x * cos - y * sin;
    const b = Math.sqrt(2 * x * y * sin * cos + (r*r - y*y) * cos*cos + (r*r - x*x) * sin*sin);

    if (a - b > 0)
    {
        return a - b;
    }
    else if (a + b > 0)
    {
        return a + b;
    }
    else
    {
        return Infinity
    }
}

const obstacles: [number, number, number][] = [];
export function append_obstacle(x: number, y: number, r: number)
{
    obstacles.push([x, y, r]);
}
export function draw_obstacles(ctx: CanvasRenderingContext2D)
{
    for (const obstacle of obstacles)
    {
        ctx.beginPath();
        ctx.arc(obstacle[0], obstacle[1], obstacle[2], 0, Math.PI * 2);
        ctx.closePath();

        ctx.fillStyle = "#AA00AA";
        ctx.fill();
    }
}

export function predict_distance(x: number, y: number, theta: number, check_obstacles = false)
{
    const out = Math.min(
        get_distance_to_line_segment(x, y, theta, [0,     0     ], [width, 0     ]),
        get_distance_to_line_segment(x, y, theta, [width, 0     ], [width, height]),
        get_distance_to_line_segment(x, y, theta, [width, height], [0,     height]),
        get_distance_to_line_segment(x, y, theta, [0,     height], [0,     0     ]),
        ...(check_obstacles ? obstacles.map(v => get_distance_to_circle(x, y, theta, v[0], v[1], v[2])) : [Infinity])
    );

    return out;
}

export class Robot
{
    readonly width: number;
    readonly length: number;

    x: number = 50;
    y: number = 50;
    theta: number = -Math.PI/2 + 0.000000001;

    constructor(width: number, length: number)
    {
        this.width = width
        this.length = length
    }

    move(delta_y_local: number, delta_x_local: number, delta_theta: number)
    {
        this.theta += delta_theta;
        const delta = add(
            get_vector(delta_y_local, this.theta),
            get_vector(delta_x_local, this.theta + Math.PI/2)
        );

        this.x += delta[0];
        this.y += delta[1];
    }

    render(ctx: CanvasRenderingContext2D)
    {
        const center: [number, number] = [this.x, this.y];

        const top_left = add(
            center,
            get_vector(this.length/2, this.theta),
            get_vector(-this.width/2, this.theta + Math.PI/2)
        );
        const top_right = add(
            center,
            get_vector(this.length/2, this.theta),
            get_vector(this.width/2, this.theta + Math.PI/2)
        );
        const bottom_left = add(
            center,
            get_vector(-this.length/2, this.theta),
            get_vector(-this.width/2, this.theta + Math.PI/2)
        );
        const bottom_right = add(
            center,
            get_vector(-this.length/2, this.theta),
            get_vector(this.width/2, this.theta + Math.PI/2)
        );

        let region = new Path2D();
        region.moveTo(top_left[0],     top_left[1]);
        region.lineTo(top_right[0],    top_right[1]);
        region.lineTo(bottom_right[0], bottom_right[1]);
        region.lineTo(bottom_left[0],  bottom_left[1]);
        region.closePath();

        ctx.fillStyle = "#555555";
        ctx.fill(region);
    }
}

export class DistanceSensor
{
    x_offset: number;
    y_offset: number;
    theta_offset: number;

    readonly rand_1 = Math.random();
    readonly rand_2 = Math.random();

    constructor(x_offset: number, y_offset: number, theta_offset: number)
    {
        this.x_offset = x_offset;
        this.y_offset = y_offset;
        this.theta_offset = theta_offset;
    }

    static predict_stdev(predicted_distance: number)
    {
        if (predicted_distance > 7.874015748)
        {
            return 0.5 * (0.05 * predicted_distance);
        }
        else
        {
            return 0.5 * 0.5905511811;
        }
        // if (predicted_distance > 7.874015748)
        // {
        //     return (0.05 * predicted_distance);
        // }
        // else
        // {
        //     return 0.5905511811;
        // }
    }

    get_distance(robot: Robot)
    {
        const [x, y, theta] = this.get_position(robot.x, robot.y, robot.theta);
        const exact_distance = predict_distance(x, y, theta, true);

        if (exact_distance > distance_sensor_max)
        {
            return Infinity;
        }
        else if (use_normal_dist)
        {
            return gaussian_random(exact_distance, DistanceSensor.predict_stdev(exact_distance), this.rand_1, this.rand_2);
        }
        else
        {
            return 0.95 * exact_distance + 0.1 * exact_distance * this.rand_1;
        }
    }

    get_position(robot_x: number, robot_y: number, robot_theta: number): [number, number, number]
    {
        const theta = robot_theta + this.theta_offset;

        const global_offset = add(
            get_vector(this.y_offset, robot_theta),
            get_vector(this.x_offset, robot_theta + Math.PI/2)
        )

        const x = robot_x + global_offset[0];
        const y = robot_y + global_offset[1];

        return [x, y, theta];
    }

    draw(ctx: CanvasRenderingContext2D, robot: Robot)
    {
        const [x, y, theta] = this.get_position(robot.x, robot.y, robot.theta);

        const distance = predict_distance(x, y, theta, true);

        if (distance < distance_sensor_max)
        {
            ctx.beginPath();
            ctx.moveTo(x, y);
            ctx.lineTo(
                x + Math.cos(theta) * Math.min(distance, 200),
                y + Math.sin(theta) * Math.min(distance, 200)
            );
            ctx.lineWidth = 0.5;
            ctx.strokeStyle = "#CC0000";
            ctx.stroke();
        }

        ctx.beginPath();
        ctx.arc(x, y, 1, 0, 2*Math.PI);
        ctx.fillStyle = "#000000";
        ctx.fill();
    }
}