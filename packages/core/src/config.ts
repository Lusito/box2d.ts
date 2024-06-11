export interface b2Settings {
    /**
     * You can use this to change the length scale used by your game.
     * For example for inches you could use 39.4.
     */
    lengthUnitsPerMeter: number;

    /**
     * The maximum number of vertices on a convex polygon. You cannot increase
     * this too much because b2BlockAllocator has a maximum object size.
     */
    maxPolygonVertices: number;
}

const defaultSettings: b2Settings = {
    lengthUnitsPerMeter: 1,
    maxPolygonVertices: 8,
};

export const settings: b2Settings = { ...defaultSettings };

export function configure(changes: Partial<b2Settings>) {
    Object.assign(settings, defaultSettings, changes);
}
