<script lang="ts">
    import { onDestroy, onMount } from "svelte";
    import { InputSystem } from "./input";

    // let canvas: HTMLCanvasElement;
    let animation_frame_id: number;
    
    let {
        render,
        input = $bindable(),
        canvas = $bindable(undefined),
        width,
        height,
        scale,
    }: {
        render: (ctx: CanvasRenderingContext2D) => void,
        input: InputSystem,
        canvas: HTMLCanvasElement | undefined,
        width: number,
        height: number,
        scale: number,
    } = $props();

    onMount(() => {
        const ctx = canvas!.getContext("2d");

        ctx?.scale(scale, scale);

        if (ctx)
        {
            function draw() {

                if (render)
                {
                    render(ctx!);
                }

                animation_frame_id = requestAnimationFrame(draw);
            }

            draw();
        }
    });

    onDestroy(() => {
        if (animation_frame_id)
        {
            cancelAnimationFrame(animation_frame_id);
        }
    });
</script>

<canvas width={width * scale} height={height * scale} bind:this={canvas}></canvas>