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

import { PolygonShape, Transform, Vec2, Manifold, collidePolygons, WorldManifold, Color } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { g_debugDraw } from "../../utils/draw";
import { HotKey, hotKeyPress } from "../../utils/hotkeys";

class PolygonCollisionTest extends Test {
    public polygonA = new PolygonShape();

    public polygonB = new PolygonShape();

    public transformA = new Transform();

    public transformB = new Transform();

    public positionB = new Vec2();

    public angleB = 0;

    public constructor() {
        super();

        this.polygonA.setAsBox(0.2, 0.4);
        this.transformA.setPositionAngle(new Vec2(), 0);
        this.polygonB.setAsBox(0.5, 0.5);
        this.positionB.set(1, 1);
        this.angleB = 1.9160721;
        this.transformB.setPositionAngle(this.positionB, this.angleB);
    }

    public getDefaultViewZoom() {
        return 100;
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
        this.positionB.x += x;
        this.positionB.y += y;
        this.angleB += angle;
        this.transformB.setPositionAngle(this.positionB, this.angleB);
    }

    public step(settings: Settings, timeStep: number): void {
        super.step(settings, timeStep);
        const manifold = new Manifold();
        collidePolygons(manifold, this.polygonA, this.transformA, this.polygonB, this.transformB);

        const worldManifold = new WorldManifold();
        worldManifold.initialize(
            manifold,
            this.transformA,
            this.polygonA.radius,
            this.transformB,
            this.polygonB.radius,
        );

        this.addDebug("Point Count", manifold.pointCount);

        {
            const color = new Color(0.9, 0.9, 0.9);
            const v = [];
            for (let i = 0; i < this.polygonA.count; ++i) {
                v[i] = Transform.multiplyVec2(this.transformA, this.polygonA.vertices[i], new Vec2());
            }
            g_debugDraw.drawPolygon(v, this.polygonA.count, color);

            for (let i = 0; i < this.polygonB.count; ++i) {
                v[i] = Transform.multiplyVec2(this.transformB, this.polygonB.vertices[i], new Vec2());
            }
            g_debugDraw.drawPolygon(v, this.polygonB.count, color);
        }

        for (let i = 0; i < manifold.pointCount; ++i) {
            g_debugDraw.drawPoint(worldManifold.points[i], 4, new Color(0.9, 0.3, 0.3));
        }
    }
}

registerTest("Geometry", "Polygon Collision", PolygonCollisionTest);
