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

import { Fixture, Vec2, Body, PolygonShape, CircleShape, EdgeShape, randomFloat, BodyType, Color } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { g_debugDraw } from "../../utils/draw";
import { hotKeyPress, HotKey } from "../../utils/hotkeys";

class EdgeShapesTest extends Test {
    public static readonly e_maxBodies = 256;

    public bodyIndex = 0;

    public bodies: Array<Body | null>;

    public polygons: PolygonShape[];

    public circle = new CircleShape();

    public angle = 0;

    public constructor() {
        super();

        this.bodies = new Array<Body>(EdgeShapesTest.e_maxBodies);
        this.polygons = new Array<PolygonShape>(4);
        for (let i = 0; i < 4; ++i) {
            this.polygons[i] = new PolygonShape();
        }

        // Ground body
        {
            const ground = this.world.createBody();

            let x1 = -20;
            let y1 = 2 * Math.cos((x1 / 10) * Math.PI);
            for (let i = 0; i < 80; ++i) {
                const x2 = x1 + 0.5;
                const y2 = 2 * Math.cos((x2 / 10) * Math.PI);

                const shape = new EdgeShape();
                shape.setTwoSided(new Vec2(x1, y1), new Vec2(x2, y2));
                ground.createFixture({ shape });

                x1 = x2;
                y1 = y2;
            }
        }

        {
            const vertices = new Array(3);
            vertices[0] = new Vec2(-0.5, 0);
            vertices[1] = new Vec2(0.5, 0);
            vertices[2] = new Vec2(0, 1.5);
            this.polygons[0].set(vertices, 3);
        }

        {
            const vertices = new Array(3);
            vertices[0] = new Vec2(-0.1, 0);
            vertices[1] = new Vec2(0.1, 0);
            vertices[2] = new Vec2(0, 1.5);
            this.polygons[1].set(vertices, 3);
        }

        {
            const w = 1;
            const b = w / (2 + Math.sqrt(2));
            const s = Math.sqrt(2) * b;

            const vertices = new Array(8);
            vertices[0] = new Vec2(0.5 * s, 0);
            vertices[1] = new Vec2(0.5 * w, b);
            vertices[2] = new Vec2(0.5 * w, b + s);
            vertices[3] = new Vec2(0.5 * s, w);
            vertices[4] = new Vec2(-0.5 * s, w);
            vertices[5] = new Vec2(-0.5 * w, b + s);
            vertices[6] = new Vec2(-0.5 * w, b);
            vertices[7] = new Vec2(-0.5 * s, 0);

            this.polygons[2].set(vertices, 8);
        }

        this.polygons[3].setAsBox(0.5, 0.5);
        this.circle.radius = 0.5;

        for (let i = 0; i < EdgeShapesTest.e_maxBodies; ++i) {
            this.bodies[i] = null;
        }
    }

    public createBody(index: number) {
        const old_body = this.bodies[this.bodyIndex];
        if (old_body !== null) {
            this.world.destroyBody(old_body);
            this.bodies[this.bodyIndex] = null;
        }

        const new_body = (this.bodies[this.bodyIndex] = this.world.createBody({
            position: { x: randomFloat(-10, 10), y: randomFloat(10, 20) },
            angle: randomFloat(-Math.PI, Math.PI),
            type: BodyType.Dynamic,
            angularDamping: index === 4 ? 0.02 : 0,
        }));

        if (index < 4) {
            new_body.createFixture({
                shape: this.polygons[index],
                friction: 0.3,
                density: 20,
            });
        } else {
            new_body.createFixture({
                shape: this.circle,
                friction: 0.3,
                density: 20,
            });
        }

        this.bodyIndex = (this.bodyIndex + 1) % EdgeShapesTest.e_maxBodies;
    }

    public destroyBody() {
        for (let i = 0; i < EdgeShapesTest.e_maxBodies; ++i) {
            const body = this.bodies[i];
            if (body !== null) {
                this.world.destroyBody(body);
                this.bodies[i] = null;
                return;
            }
        }
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("1", "Create Triangle", () => this.createBody(0)),
            hotKeyPress("2", "Create Flat Triangle", () => this.createBody(1)),
            hotKeyPress("3", "Create Octagon", () => this.createBody(2)),
            hotKeyPress("4", "Create Box", () => this.createBody(3)),
            hotKeyPress("5", "Create Circle", () => this.createBody(4)),
            hotKeyPress("d", "Destroy Body", () => this.destroyBody()),
        ];
    }

    public step(settings: Settings, timeStep: number): void {
        const advanceRay = !settings.pause || settings.singleStep;
        super.step(settings, timeStep);

        const L = 25;
        const point1 = new Vec2(0, 10);
        const d = new Vec2(L * Math.cos(this.angle), -L * Math.abs(Math.sin(this.angle)));
        const point2 = Vec2.add(point1, d, new Vec2());

        let resultFixture: Fixture | null = null;
        const resultPoint = new Vec2();
        const resultNormal = new Vec2();
        this.world.rayCast(point1, point2, (fixture, point, normal, fraction) => {
            resultFixture = fixture;
            resultPoint.copy(point);
            resultNormal.copy(normal);
            return fraction;
        });

        if (resultFixture) {
            g_debugDraw.drawPoint(resultPoint, 5, new Color(0.4, 0.9, 0.4));
            g_debugDraw.drawSegment(point1, resultPoint, new Color(0.8, 0.8, 0.8));
            const head = Vec2.add(resultPoint, Vec2.scale(0.5, resultNormal, Vec2.s_t0), new Vec2());
            g_debugDraw.drawSegment(resultPoint, head, new Color(0.9, 0.9, 0.4));
        } else {
            g_debugDraw.drawSegment(point1, point2, new Color(0.8, 0.8, 0.8));
        }

        if (advanceRay) {
            this.angle += (0.25 * Math.PI) / 180;
        }
    }
}

registerTest("Geometry", "Edge Shapes", EdgeShapesTest);
