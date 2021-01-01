/*
 * Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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

import {
    Vec2,
    Transform,
    PolygonShape,
    DistanceInput,
    SimplexCache,
    DistanceOutput,
    distance,
    Color,
    XY,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { g_debugDraw } from "../../utils/draw";
import { hotKeyPress, HotKey } from "../../utils/hotkeys";

class DistanceTest extends Test {
    public m_positionB = new Vec2();

    public m_angleB = 0;

    public m_transformA = new Transform();

    public m_transformB = new Transform();

    public m_polygonA = new PolygonShape();

    public m_polygonB = new PolygonShape();

    public constructor() {
        super();

        this.m_transformA.setIdentity();
        this.m_transformA.p.set(0, -0.2);
        this.m_polygonA.setAsBox(10, 0.2);

        this.m_positionB.set(12.017401, 0.13678508);
        this.m_angleB = -0.0109265;
        this.m_transformB.setPositionAngle(this.m_positionB, this.m_angleB);

        this.m_polygonB.setAsBox(2, 0.1);
    }

    public getDefaultViewZoom() {
        return 200;
    }

    public getCenter(): XY {
        return {
            x: 10,
            y: -0.5,
        };
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("a", "Move Left", () => this.adjust(-0.1, 0, 0)),
            hotKeyPress("d", "Move Right", () => this.adjust(0.1, 0, 0)),
            hotKeyPress("s", "Move Down", () => this.adjust(0, -0.1, 0)),
            hotKeyPress("w", "Move Up", () => this.adjust(0, 0.1, 0)),
            hotKeyPress("q", "Turn Left", () => this.adjust(0, 0, 0.1 * Math.PI)),
            hotKeyPress("e", "Turn Right", () => this.adjust(0, 0, -0.1 * Math.PI)),
        ];
    }

    private adjust(x: number, y: number, angle: number) {
        this.m_positionB.x += x;
        this.m_positionB.y += y;
        this.m_angleB += angle;
        this.m_transformB.setPositionAngle(this.m_positionB, this.m_angleB);
    }

    public step(settings: Settings, timeStep: number): void {
        super.step(settings, timeStep);

        const input = new DistanceInput();
        input.proxyA.setShape(this.m_polygonA, 0);
        input.proxyB.setShape(this.m_polygonB, 0);
        input.transformA.copy(this.m_transformA);
        input.transformB.copy(this.m_transformB);
        input.useRadii = true;
        const cache = new SimplexCache();
        cache.count = 0;
        const output = new DistanceOutput();
        distance(output, cache, input);

        this.addDebug("Distance", output.distance.toFixed(2));
        this.addDebug("Iterations", output.iterations);

        {
            const color = new Color(0.9, 0.9, 0.9);
            const v = [];
            for (let i = 0; i < this.m_polygonA.m_count; ++i) {
                v[i] = Transform.multiplyVec2(this.m_transformA, this.m_polygonA.m_vertices[i], new Vec2());
            }
            g_debugDraw.drawPolygon(v, this.m_polygonA.m_count, color);

            for (let i = 0; i < this.m_polygonB.m_count; ++i) {
                v[i] = Transform.multiplyVec2(this.m_transformB, this.m_polygonB.m_vertices[i], new Vec2());
            }
            g_debugDraw.drawPolygon(v, this.m_polygonB.m_count, color);
        }

        const x1 = output.pointA;
        const x2 = output.pointB;

        const c1 = new Color(1, 0, 0);
        g_debugDraw.drawPoint(x1, 4, c1);

        const c2 = new Color(1, 1, 0);
        g_debugDraw.drawPoint(x2, 4, c2);
    }
}

registerTest("Examples", "Distance", DistanceTest);
