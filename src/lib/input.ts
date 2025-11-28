export class InputSystem
{
    private keymap = new Map<string, boolean>();

    register_events(window: Window, canvas: HTMLElement)
    {
        window.addEventListener("keydown", (ev) => this.keydown_event(ev));
        window.addEventListener("keyup",   (ev) => this.keyup_event(ev));
        window.addEventListener("focus",   () => this.focus_event());
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

    is_key_down(key: string)
    {
        return this.keymap.has(key) && this.keymap.get(key);
    }
}