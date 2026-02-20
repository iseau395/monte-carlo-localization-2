export class InputSystem
{
    private keymap = new Map<string, boolean>();
    private readonly click_callbacks: ((ev: PointerEvent) => void)[] = [];

    register_events(window: Window, canvas: HTMLElement)
    {
        window.addEventListener("keydown", (ev) => this.keydown_event(ev));
        window.addEventListener("keyup",   (ev) => this.keyup_event(ev));
        window.addEventListener("focus",   () => this.focus_event());

        canvas.addEventListener("click", (ev) => this.click_event(ev));
    }

    private focus_event()
    {
        this.keymap.clear();
    }

    private keydown_event(ev: KeyboardEvent)
    {
        ev.preventDefault();
        this.keymap.set(ev.key, true);
    }

    private keyup_event(ev: KeyboardEvent)
    {
        ev.preventDefault();
        this.keymap.set(ev.key, false);
    }

    private click_event(ev: PointerEvent)
    {
        for (const callback of this.click_callbacks)
        {
            callback(ev);
        }
    }

    is_key_down(key: string)
    {
        return this.keymap.has(key) && this.keymap.get(key);
    }

    on_click(callback: (ev: PointerEvent) => void)
    {
        this.click_callbacks.push(callback);
    }
}