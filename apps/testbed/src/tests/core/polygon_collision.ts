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
    b2PolygonShape,
    b2Transform,
    b2Vec2,
    b2Manifold,
    b2CollidePolygons,
    b2WorldManifold,
    b2Color,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { HotKey, hotKeyPress } from "../../utils/hotkeys";

class PolygonCollision extends Test {
    public m_polygonA = new b2PolygonShape();

    public m_polygonB = new b2PolygonShape();

    public m_transformA = new b2Transform();

    public m_transformB = new b2Transform();

    public m_positionB = new b2Vec2();

    public m_angleB = 0;

    public constructor() {
        super();

        this.m_polygonA.SetAsBox(0.2, 0.4);
        this.m_transformA.SetPositionAngle(new b2Vec2(), 0);
        this.m_polygonB.SetAsBox(0.5, 0.5);
        this.m_positionB.Set(1, 1);
        this.m_angleB = 1.9160721;
        this.m_transformB.SetPositionAngle(this.m_positionB, this.m_angleB);
    }

    public GetDefaultViewZoom() {
        return 100;
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("a", "Move Left", () => this.Adjust(-0.1, 0, 0)),
            hotKeyPress("d", "Move Right", () => this.Adjust(0.1, 0, 0)),
            hotKeyPress("s", "Move Down", () => this.Adjust(0, -0.1, 0)),
            hotKeyPress("w", "Move Up", () => this.Adjust(0, 0.1, 0)),
            hotKeyPress("q", "Turn Left", () => this.Adjust(0, 0, 0.1 * Math.PI)),
            hotKeyPress("e", "Turn Right", () => this.Adjust(0, 0, -0.1 * Math.PI)),
        ];
    }

    private Adjust(x: number, y: number, angle: number) {
        this.m_positionB.x += x;
        this.m_positionB.y += y;
        this.m_angleB += angle;
        this.m_transformB.SetPositionAngle(this.m_positionB, this.m_angleB);
    }

    public Step(settings: Settings, timeStep: number): void {
        super.Step(settings, timeStep);
        const manifold = new b2Manifold();
        b2CollidePolygons(manifold, this.m_polygonA, this.m_transformA, this.m_polygonB, this.m_transformB);

        const worldManifold = new b2WorldManifold();
        worldManifold.Initialize(
            manifold,
            this.m_transformA,
            this.m_polygonA.m_radius,
            this.m_transformB,
            this.m_polygonB.m_radius,
        );

        this.addDebug("Point Count", manifold.pointCount);

        const draw = settings.m_debugDraw;
        {
            const color = new b2Color(0.9, 0.9, 0.9);
            const v = [];
            for (let i = 0; i < this.m_polygonA.m_count; ++i) {
                v[i] = b2Transform.MultiplyVec2(this.m_transformA, this.m_polygonA.m_vertices[i], new b2Vec2());
            }
            draw.DrawPolygon(v, this.m_polygonA.m_count, color);

            for (let i = 0; i < this.m_polygonB.m_count; ++i) {
                v[i] = b2Transform.MultiplyVec2(this.m_transformB, this.m_polygonB.m_vertices[i], new b2Vec2());
            }
            draw.DrawPolygon(v, this.m_polygonB.m_count, color);
        }

        for (let i = 0; i < manifold.pointCount; ++i) {
            draw.DrawPoint(worldManifold.points[i], 4, new b2Color(0.9, 0.3, 0.3));
        }
    }
}

registerTest("Geometry", "Polygon Collision", PolygonCollision);
