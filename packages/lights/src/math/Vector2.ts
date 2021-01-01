export interface XY {
    x: number;
    y: number;
}

export class Vector2 implements XY {
    public x = 0;

    public y = 0;

    public set(x: number, y: number) {
        this.x = x;
        this.y = y;
        return this;
    }

    public copy(other: XY) {
        this.x = other.x;
        this.y = other.y;
        return this;
    }

    public length() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    public normalize() {
        const length = this.length();
        if (length > 0) {
            const inv_length = 1 / length;
            this.x *= inv_length;
            this.y *= inv_length;
        }
        return this;
    }

    public add(other: XY) {
        this.x += other.x;
        this.y += other.y;
        return this;
    }

    public addXY(x: number, y: number) {
        this.x += x;
        this.y += y;
        return this;
    }

    public subXY(x: number, y: number) {
        this.x -= x;
        this.y -= y;
        return this;
    }

    public sub(other: XY) {
        this.x -= other.x;
        this.y -= other.y;
        return this;
    }

    public scale(f: number) {
        this.x *= f;
        this.y *= f;
        return this;
    }

    public rotate(radians: number) {
        const c = Math.cos(radians);
        const s = Math.sin(radians);
        const { x } = this;
        this.x = c * x - s * this.y;
        this.y = s * x + c * this.y;
        return this;
    }

    /** Rotates the Vector2 by 90 degrees in the specified direction, where >= 0 is counter-clockwise and < 0 is clockwise. */
    public rotate90(dir: number) {
        const { x } = this;
        if (dir >= 0) {
            this.x = -this.y;
            this.y = x;
        } else {
            this.x = this.y;
            this.y = -x;
        }
        return this;
    }

    public getAngle() {
        return Math.atan2(this.y, this.x);
    }

    public static makeArray(length: number) {
        const result = new Array<Vector2>(length);
        for (let i = 0; i < length; i++) result[i] = new Vector2();
        return result;
    }
}
