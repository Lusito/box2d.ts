// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

import { Transform, XY } from "./b2_math";

export interface RGB {
    r: number;
    g: number;
    b: number;
}

export interface RGBA extends RGB {
    a: number;
}

/**
 * Color for debug drawing. Each value has the range [0,1].
 */
export class Color implements RGBA {
    public static readonly ZERO: Readonly<RGBA> = new Color(0, 0, 0, 0);

    public static readonly RED: Readonly<RGBA> = new Color(1, 0, 0);

    public static readonly GREEN: Readonly<RGBA> = new Color(0, 1, 0);

    public static readonly BLUE: Readonly<RGBA> = new Color(0, 0, 1);

    public static readonly WHITE: Readonly<RGBA> = new Color(1, 1, 1);

    public static readonly BLACK: Readonly<RGBA> = new Color(0, 0, 0);

    public r: number;

    public g: number;

    public b: number;

    public a: number;

    public constructor(r = 0.5, g = 0.5, b = 0.5, a = 1) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.a = a;
    }

    public Clone(): Color {
        return new Color(this.r, this.g, this.b, this.a);
    }

    public Copy(other: RGBA) {
        this.r = other.r;
        this.g = other.g;
        this.b = other.b;
        this.a = other.a;
        return this;
    }

    public IsEqual(color: RGBA): boolean {
        return this.r === color.r && this.g === color.g && this.b === color.b && this.a === color.a;
    }

    public IsZero(): boolean {
        return this.r === 0 && this.g === 0 && this.b === 0 && this.a === 0;
    }

    public SetByteRGB(r: number, g: number, b: number) {
        this.r = r / 0xff;
        this.g = g / 0xff;
        this.b = b / 0xff;
        return this;
    }

    public SetByteRGBA(r: number, g: number, b: number, a: number) {
        this.r = r / 0xff;
        this.g = g / 0xff;
        this.b = b / 0xff;
        this.a = a / 0xff;
        return this;
    }

    public SetRGB(r: number, g: number, b: number) {
        this.r = r;
        this.g = g;
        this.b = b;
        return this;
    }

    public SetRGBA(r: number, g: number, b: number, a: number) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.a = a;
        return this;
    }

    public Add(color: RGBA) {
        this.r += color.r;
        this.g += color.g;
        this.b += color.b;
        this.a += color.a;
        return this;
    }

    public Subtract(color: RGBA) {
        this.r -= color.r;
        this.g -= color.g;
        this.b -= color.b;
        this.a -= color.a;
        return this;
    }

    public Scale(s: number) {
        this.r *= s;
        this.g *= s;
        this.b *= s;
        this.a *= s;
        return this;
    }

    public Mix(mixColor: RGBA, strength: number): void {
        Color.MixColors(this, mixColor, strength);
    }

    public static Add<T extends RGBA>(colorA: RGBA, colorB: RGBA, out: T): T {
        out.r = colorA.r + colorB.r;
        out.g = colorA.g + colorB.g;
        out.b = colorA.b + colorB.b;
        out.a = colorA.a + colorB.a;
        return out;
    }

    public static Subtract<T extends RGBA>(colorA: RGBA, colorB: RGBA, out: T): T {
        out.r = colorA.r - colorB.r;
        out.g = colorA.g - colorB.g;
        out.b = colorA.b - colorB.b;
        out.a = colorA.a - colorB.a;
        return out;
    }

    public static Scale<T extends RGBA>(color: RGBA, s: number, out: T): T {
        out.r = color.r * s;
        out.g = color.g * s;
        out.b = color.b * s;
        out.a = color.a * s;
        return out;
    }

    public static MixColors(colorA: RGBA, colorB: RGBA, strength: number): void {
        const dr = strength * (colorB.r - colorA.r);
        const dg = strength * (colorB.g - colorA.g);
        const db = strength * (colorB.b - colorA.b);
        const da = strength * (colorB.a - colorA.a);
        colorA.r += dr;
        colorA.g += dg;
        colorA.b += db;
        colorA.a += da;
        colorB.r -= dr;
        colorB.g -= dg;
        colorB.b -= db;
        colorB.a -= da;
    }
}

/**
 * Implement and register this class with a World to provide debug drawing of physics
 * entities in your game.
 */
export interface Draw {
    PushTransform(xf: Transform): void;

    PopTransform(xf: Transform): void;

    DrawPolygon(vertices: XY[], vertexCount: number, color: RGBA): void;

    DrawSolidPolygon(vertices: XY[], vertexCount: number, color: RGBA): void;

    DrawCircle(center: XY, radius: number, color: RGBA): void;

    DrawSolidCircle(center: XY, radius: number, axis: XY, color: RGBA): void;

    DrawSegment(p1: XY, p2: XY, color: RGBA): void;

    DrawTransform(xf: Transform): void;

    DrawPoint(p: XY, size: number, color: RGBA): void;
}

export const debugColors = {
    badBody: new Color(1, 0, 0),
    disabledBody: new Color(0.5, 0.5, 0.3),
    staticBody: new Color(0.5, 0.9, 0.5),
    kinematicBody: new Color(0.5, 0.5, 0.9),
    sleepingBody: new Color(0.6, 0.6, 0.6),
    body: new Color(0.9, 0.7, 0.7),
    pair: new Color(0.3, 0.9, 0.9),
    aabb: new Color(0.9, 0.3, 0.9),

    joint1: new Color(0.7, 0.7, 0.7),
    joint2: new Color(0.3, 0.9, 0.3),
    joint3: new Color(0.9, 0.3, 0.3),
    joint4: new Color(0.3, 0.3, 0.9),
    joint5: new Color(0.4, 0.4, 0.4),
    joint6: new Color(0.5, 0.8, 0.8),
    joint7: new Color(0, 1, 0),
    joint8: new Color(0.8, 0.8, 0.8),

    rope: new Color(0.4, 0.5, 0.7),
    ropePointG: new Color(0.1, 0.8, 0.1),
    ropePointD: new Color(0.7, 0.2, 0.4),
};
