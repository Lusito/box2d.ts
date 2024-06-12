export interface HotKey {
    description: string;
    key: string;
    step: boolean;
    callback: (down: boolean) => void;
}

export function hotKey(key: string, description: string, callback: (down: boolean) => void, step = false): HotKey {
    return {
        key,
        description,
        callback,
        step,
    };
}

export function hotKeyPress(key: string, description: string, callback: () => void) {
    return hotKey(key, description, (down) => {
        if (down) callback();
    });
}

export function hotKeyRelease(key: string, description: string, callback: () => void) {
    return hotKey(key, description, (down) => {
        if (!down) callback();
    });
}

export function hotKeyStep(key: string, description: string, callback: () => void) {
    return hotKey(key, description, callback, true);
}

export function hotKeyState<TK extends string>(
    key: string,
    description: string,
    state: { [key in TK]: boolean },
    attribute: TK,
) {
    return hotKey(key, description, (down) => {
        state[attribute] = down;
    });
}
