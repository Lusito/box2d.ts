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

import { Vec2, RandomFloat, Clamp, PolygonShape, Color } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { g_debugDraw } from "../../utils/draw";
import { hotKeyPress, HotKey } from "../../utils/hotkeys";

class ConvexHullTest extends Test {
    public static readonly e_count = 10;

    public m_test_points: Vec2[] = [];

    public m_count = 0;

    public m_auto = false;

    public constructor() {
        super();

        this.Generate();
    }

    public GetDefaultViewZoom() {
        return 50;
    }

    public Generate(): void {
        for (let i = 0; i < ConvexHullTest.e_count; ++i) {
            let x = RandomFloat(-10, 10);
            let y = RandomFloat(-10, 10);

            // Clamp onto a square to help create collinearities.
            // This will stress the convex hull algorithm.
            x = Clamp(x, -8, 8);
            y = Clamp(y, -8, 8);
            this.m_test_points[i] = new Vec2(x, y);
        }

        this.m_count = ConvexHullTest.e_count;
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("a", "Toggle Autogeneration", () => {
                this.m_auto = !this.m_auto;
            }),
            hotKeyPress("g", "Generate a new random convex hull", () => this.Generate()),
        ];
    }

    public Step(settings: Settings, timeStep: number): void {
        super.Step(settings, timeStep);

        const shape = new PolygonShape();
        shape.Set(this.m_test_points, this.m_count);

        g_debugDraw.DrawPolygon(shape.m_vertices, shape.m_count, new Color(0.9, 0.9, 0.9));

        for (let i = 0; i < this.m_count; ++i) {
            g_debugDraw.DrawPoint(this.m_test_points[i], 3, new Color(0.3, 0.9, 0.3));
            g_debugDraw.DrawStringWorld(this.m_test_points[i].x + 0.05, this.m_test_points[i].y + 0.05, `${i}`);
        }

        shape.Validate();

        if (this.m_auto) {
            this.Generate();
        }
    }
}

registerTest("Geometry", "Convex Hull", ConvexHullTest);
