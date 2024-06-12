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
    b2Vec2,
    b2RandomFloat,
    b2Clamp,
    b2Color,
    b2Rot,
    b2Random,
    b2_maxPolygonVertices,
    b2Hull,
    b2ComputeHull,
    b2ValidateHull,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { hotKeyPress, HotKey } from "../../utils/hotkeys";

class ConvexHull extends Test {
    public static readonly e_count = b2_maxPolygonVertices;

    public m_test_points: b2Vec2[] = [];

    public m_count = 0;

    public m_generation = 0;

    public m_bulk = false;

    public m_auto = false;

    public constructor() {
        super();

        this.Generate();
    }

    public GetDefaultViewZoom() {
        return 50;
    }

    public Generate(): void {
        const angle = Math.PI * b2Random();
        const r = new b2Rot(angle);

        for (let i = 0; i < ConvexHull.e_count; ++i) {
            let x = b2RandomFloat(-10, 10);
            let y = b2RandomFloat(-10, 10);

            // Clamp onto a square to help create collinearities.
            // This will stress the convex hull algorithm.
            x = b2Clamp(x, -4, 4);
            y = b2Clamp(y, -4, 4);
            this.m_test_points[i] = b2Rot.MultiplyVec2(r, new b2Vec2(x, y), new b2Vec2());
        }

        this.m_count = ConvexHull.e_count;
        this.m_generation += 1;
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("a", "Toggle Autogeneration", () => {
                this.m_auto = !this.m_auto;
            }),
            hotKeyPress("b", "Toggle Bulk", () => {
                this.m_bulk = !this.m_bulk;
            }),
            hotKeyPress("g", "Generate a new random convex hull", () => this.Generate()),
        ];
    }

    public Step(settings: Settings, timeStep: number): void {
        super.Step(settings, timeStep);

        let hull!: Readonly<b2Hull>;
        let valid = false;

        if (this.m_bulk) {
            // defect hunting
            for (let i = 0; i < 10000; ++i) {
                this.Generate();
                hull = b2ComputeHull(this.m_test_points, this.m_count);
                if (hull.length === 0) {
                    continue;
                }

                valid = b2ValidateHull(hull, hull.length);
                if (valid === false) {
                    this.m_bulk = false;
                    break;
                }
            }
        } else {
            if (this.m_auto) {
                this.Generate();
            }

            hull = b2ComputeHull(this.m_test_points, this.m_count);
            if (hull.length > 0) {
                valid = b2ValidateHull(hull, hull.length);
                if (valid === false) {
                    this.m_auto = false;
                }
            }
        }

        const draw = settings.m_debugDraw;

        if (valid === false) {
            this.addText(`generation = ${this.m_generation}, FAILED`);
        } else {
            this.addText(`generation = ${this.m_generation}, count = ${hull.length}`);
        }

        draw.DrawPolygon(hull, hull.length, new b2Color(0.9, 0.9, 0.9));

        for (let i = 0; i < this.m_count; ++i) {
            const p = this.m_test_points[i];
            draw.DrawPoint(p, 5, new b2Color(0.3, 0.3, 0.9));
            draw.DrawStringWorld(p.x + 0.1, p.y + 0.1, i.toFixed(0));
        }

        for (const p of hull) {
            draw.DrawPoint(p, 6, new b2Color(0.3, 0.7, 0.3));
        }
    }
}

registerTest("Geometry", "Convex Hull", ConvexHull);
