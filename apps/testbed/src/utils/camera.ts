/*
 * Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

import { b2Vec2, XY } from "@box2d/core";
import { vec3, mat4 } from "gl-matrix";

const up: vec3 = [0, -1, 0];
const forward: vec3 = [0, 0, 1];
const tmpVector = vec3.create();
const tmpXY = { x: 0, y: 0 };

function vec2Project(x: number, y: number, m: mat4, out: XY) {
    const w = 1 / (x * m[3] + y * m[7] + m[15]);
    out.x = (x * m[0] + y * m[4] + m[12]) * w;
    out.y = (x * m[1] + y * m[5] + m[13]) * w;
}

export class Camera {
    private readonly center = new b2Vec2(0, 20);

    private zoom = 1;

    public readonly projection = mat4.create();

    public readonly modelView = mat4.create();

    public readonly combined = mat4.create();

    public readonly inverse = mat4.create();

    private width = 0;

    private height = 0;

    public getZoom() {
        return this.zoom;
    }

    public getCenter(): Readonly<XY> {
        return this.center;
    }

    public getWidth() {
        return this.width;
    }

    public getHeight() {
        return this.height;
    }

    public update() {
        const f = 1 / (2 * this.zoom);
        const hw = this.width * f;
        const hh = this.height * f;
        mat4.ortho(this.projection, -hw, hw, hh, -hh, 0, 10000);
        const position: vec3 = [this.center.x, this.center.y, 0];
        mat4.lookAt(this.modelView, position, vec3.add(tmpVector, position, forward), up);
        mat4.mul(this.combined, this.projection, this.modelView);
        mat4.invert(this.inverse, this.combined);
    }

    public resize(width: number, height: number) {
        this.width = width;
        this.height = height;
        this.update();
    }

    public setPositionAndZoom(x: number, y: number, zoom: number) {
        this.center.Set(x, y);
        this.zoom = zoom;
        this.update();
    }

    public setPosition(x: number, y: number) {
        this.center.Set(x, y);
        this.update();
    }

    public setZoom(zoom: number) {
        this.zoom = zoom;
        this.update();
    }

    public project(world: Readonly<XY>, viewport: b2Vec2) {
        vec2Project(world.x, world.y, this.combined, tmpXY);
        viewport.x = (this.width * (tmpXY.x + 1)) / 2;
        viewport.y = this.height - (this.height * (tmpXY.y + 1)) / 2;
        return viewport;
    }

    public unproject({ x, y }: Readonly<XY>, world: b2Vec2) {
        vec2Project((2 * x) / this.width - 1, (2 * (this.height - y)) / this.height - 1, this.inverse, world);

        return world;
    }
}

export const g_camera = new Camera();
