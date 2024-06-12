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
    b2Fixture,
    b2Vec2,
    b2Body,
    b2PolygonShape,
    b2CircleShape,
    b2EdgeShape,
    b2RandomFloat,
    b2BodyType,
    b2Color,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { hotKeyPress, HotKey } from "../../utils/hotkeys";

class EdgeShapes extends Test {
    public static readonly e_maxBodies = 256;

    public m_bodyIndex = 0;

    public m_bodies: Array<b2Body | null>;

    public m_polygons: b2PolygonShape[];

    public m_circle = new b2CircleShape();

    public m_angle = 0;

    public constructor() {
        super();

        this.m_bodies = new Array<b2Body>(EdgeShapes.e_maxBodies);
        this.m_polygons = new Array<b2PolygonShape>(4);
        for (let i = 0; i < 4; ++i) {
            this.m_polygons[i] = new b2PolygonShape();
        }

        // Ground body
        {
            const ground = this.m_world.CreateBody();

            let x1 = -20;
            let y1 = 2 * Math.cos((x1 / 10) * Math.PI);
            for (let i = 0; i < 80; ++i) {
                const x2 = x1 + 0.5;
                const y2 = 2 * Math.cos((x2 / 10) * Math.PI);

                const shape = new b2EdgeShape();
                shape.SetTwoSided(new b2Vec2(x1, y1), new b2Vec2(x2, y2));
                ground.CreateFixture({ shape });

                x1 = x2;
                y1 = y2;
            }
        }

        {
            const vertices = new Array(3);
            vertices[0] = new b2Vec2(-0.5, 0);
            vertices[1] = new b2Vec2(0.5, 0);
            vertices[2] = new b2Vec2(0, 1.5);
            this.m_polygons[0].Set(vertices, 3);
        }

        {
            const vertices = new Array(3);
            vertices[0] = new b2Vec2(-0.1, 0);
            vertices[1] = new b2Vec2(0.1, 0);
            vertices[2] = new b2Vec2(0, 1.5);
            this.m_polygons[1].Set(vertices, 3);
        }

        {
            const w = 1;
            const b = w / (2 + Math.sqrt(2));
            const s = Math.sqrt(2) * b;

            const vertices = new Array(8);
            vertices[0] = new b2Vec2(0.5 * s, 0);
            vertices[1] = new b2Vec2(0.5 * w, b);
            vertices[2] = new b2Vec2(0.5 * w, b + s);
            vertices[3] = new b2Vec2(0.5 * s, w);
            vertices[4] = new b2Vec2(-0.5 * s, w);
            vertices[5] = new b2Vec2(-0.5 * w, b + s);
            vertices[6] = new b2Vec2(-0.5 * w, b);
            vertices[7] = new b2Vec2(-0.5 * s, 0);

            this.m_polygons[2].Set(vertices, 8);
        }

        this.m_polygons[3].SetAsBox(0.5, 0.5);
        this.m_circle.m_radius = 0.5;

        for (let i = 0; i < EdgeShapes.e_maxBodies; ++i) {
            this.m_bodies[i] = null;
        }
    }

    public CreateBody(index: number) {
        const old_body = this.m_bodies[this.m_bodyIndex];
        if (old_body !== null) {
            this.m_world.DestroyBody(old_body);
            this.m_bodies[this.m_bodyIndex] = null;
        }

        const new_body = (this.m_bodies[this.m_bodyIndex] = this.m_world.CreateBody({
            position: { x: b2RandomFloat(-10, 10), y: b2RandomFloat(10, 20) },
            angle: b2RandomFloat(-Math.PI, Math.PI),
            type: b2BodyType.b2_dynamicBody,
            angularDamping: index === 4 ? 0.02 : 0,
        }));

        if (index < 4) {
            new_body.CreateFixture({
                shape: this.m_polygons[index],
                friction: 0.3,
                density: 20,
            });
        } else {
            new_body.CreateFixture({
                shape: this.m_circle,
                friction: 0.3,
                density: 20,
            });
        }

        this.m_bodyIndex = (this.m_bodyIndex + 1) % EdgeShapes.e_maxBodies;
    }

    public DestroyBody() {
        for (let i = 0; i < EdgeShapes.e_maxBodies; ++i) {
            const body = this.m_bodies[i];
            if (body !== null) {
                this.m_world.DestroyBody(body);
                this.m_bodies[i] = null;
                return;
            }
        }
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("1", "Create Triangle", () => this.CreateBody(0)),
            hotKeyPress("2", "Create Flat Triangle", () => this.CreateBody(1)),
            hotKeyPress("3", "Create Octagon", () => this.CreateBody(2)),
            hotKeyPress("4", "Create Box", () => this.CreateBody(3)),
            hotKeyPress("5", "Create Circle", () => this.CreateBody(4)),
            hotKeyPress("d", "Destroy Body", () => this.DestroyBody()),
        ];
    }

    public Step(settings: Settings, timeStep: number): void {
        const advanceRay = !settings.m_pause || settings.m_singleStep;
        super.Step(settings, timeStep);

        const L = 25;
        const point1 = new b2Vec2(0, 10);
        const d = new b2Vec2(L * Math.cos(this.m_angle), -L * Math.abs(Math.sin(this.m_angle)));
        const point2 = b2Vec2.Add(point1, d, new b2Vec2());

        let resultFixture: b2Fixture | null = null;
        const resultPoint = new b2Vec2();
        const resultNormal = new b2Vec2();
        this.m_world.RayCast(point1, point2, (fixture, point, normal, fraction) => {
            resultFixture = fixture;
            resultPoint.Copy(point);
            resultNormal.Copy(normal);
            return fraction;
        });

        const draw = settings.m_debugDraw;
        if (resultFixture) {
            draw.DrawPoint(resultPoint, 5, new b2Color(0.4, 0.9, 0.4));
            draw.DrawSegment(point1, resultPoint, new b2Color(0.8, 0.8, 0.8));
            const head = b2Vec2.Add(resultPoint, b2Vec2.Scale(0.5, resultNormal, b2Vec2.s_t0), new b2Vec2());
            draw.DrawSegment(resultPoint, head, new b2Color(0.9, 0.9, 0.4));
        } else {
            draw.DrawSegment(point1, point2, new b2Color(0.8, 0.8, 0.8));
        }

        if (advanceRay) {
            this.m_angle += (0.25 * Math.PI) / 180;
        }
    }
}

registerTest("Geometry", "Edge Shapes", EdgeShapes);
