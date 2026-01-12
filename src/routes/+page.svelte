<script lang="ts">
    import AnimatedCanvas from "$lib/AnimatedCanvas.svelte";
    import { InputSystem } from "$lib/input";
    import { width, height, gaussian_random } from "$lib/constants";
    import { MonteCarlo } from "$lib/monte_carlo"
    import { DistanceSensor, Robot } from "$lib/robot";
    import { onDestroy, onMount } from "svelte";

    const scale = 5;

    const robot = new Robot(18, 18);
    let canvas: HTMLCanvasElement | undefined = $state(undefined);
    let input = $state(new InputSystem());

    const particle_count = 1000;
    const distance_1 = new DistanceSensor(9, 0, Math.PI/2);
    const distance_2 = new DistanceSensor(0, 9, 0);
    const distance_3 = new DistanceSensor(-9, 0, -Math.PI/2);
    const distance_4 = new DistanceSensor(0, -9, Math.PI);
    const monte_carlo = new MonteCarlo(distance_1, distance_2, distance_3, distance_4);
    monte_carlo.initialize(robot.x, robot.y, particle_count, robot);

    onMount(() => input.register_events(window, canvas!));

    const move_speed = 60;
    const turn_speed = Math.PI;

    let predicted_position = [robot.x, robot.y];
    let total_weight = $state(0);

    let last_time = Date.now() / 1000;
    function render(ctx: CanvasRenderingContext2D)
    {
        const dt = Date.now()/1000 - last_time;
        last_time += dt;

        ctx.fillStyle = "#909090";
        ctx.fillRect(0, 0, width, height);
        
        robot.render(ctx);
        
        monte_carlo.draw(ctx, robot);

        ctx.beginPath();
        ctx.arc(predicted_position[0], predicted_position[1], 1, 0, Math.PI * 2);
        ctx.fillStyle = "#00FF00";
        ctx.fill();

        robot.move(
            dt * move_speed * (Number(input.is_key_down("w")) - Number(input.is_key_down("s"))),
            dt * move_speed * (Number(input.is_key_down("d")) - Number(input.is_key_down("a"))),
            dt * turn_speed * (Number(input.is_key_down("e")) - Number(input.is_key_down("q"))),
        );
    }


    let last_robot_x = robot.x;
    let last_robot_y = robot.y;
    function tick()
    {
        const true_delta_x = robot.x - last_robot_x;
        const true_delta_y = robot.y - last_robot_y;

        const delta_x = gaussian_random(true_delta_x, true_delta_x * 0.05);
        const delta_y = gaussian_random(true_delta_y, true_delta_y * 0.05);

        const new_predicted_position = monte_carlo.tick(delta_x, delta_y, robot);
        if (!isNaN(new_predicted_position[0]) && !isNaN(new_predicted_position[1]))
        {
            predicted_position[0] = 0.75 * (predicted_position[0] + delta_x) + 0.25 * new_predicted_position[0];
            predicted_position[1] = 0.75 * (predicted_position[1] + delta_y) + 0.25 * new_predicted_position[1];
        }
        else
        {
            predicted_position[0] += delta_x;
            predicted_position[1] += delta_y;
        }

        total_weight = new_predicted_position[2];


        last_robot_x = robot.x;
        last_robot_y = robot.y;
    }

    const interval_id = setInterval(tick, 20);
    onDestroy(() => clearTimeout(interval_id));
</script>

<AnimatedCanvas {render} bind:input bind:canvas {width} {height} {scale} />
{(total_weight / particle_count * 10000000).toFixed(8).padStart(8, "0")}