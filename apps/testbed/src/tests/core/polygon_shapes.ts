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
    b2CircleShape,
    b2Transform,
    b2TestOverlap,
    b2Color,
    b2Body,
    b2PolygonShape,
    b2EdgeShape,
    b2Vec2,
    b2BodyType,
    b2RandomFloat,
    b2AABB,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { HotKey, hotKeyPress } from "../../utils/hotkeys";

const queryCallbackMaxCount = 4;

const temp = {
    Step: {
        circle: new b2CircleShape(),
        transform: new b2Transform(),
    },
};

/**
 * This tests stacking. It also shows how to use b2World::Query
 * and b2TestOverlap.
 */
class PolygonShapes extends Test {
    public static readonly e_maxBodies = 256;

    public m_bodyIndex = 0;

    public m_bodies: Array<b2Body | null> = Array.from({ length: PolygonShapes.e_maxBodies }, () => null);

    public m_polygons = Array.from({ length: 4 }, () => new b2PolygonShape());

    public m_circle = new b2CircleShape();

    public constructor() {
        super();

        // Ground body
        {
            const ground = this.m_world.CreateBody();

            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(-40, 0), new b2Vec2(40, 0));
            ground.CreateFixture({ shape });
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

        for (let i = 0; i < PolygonShapes.e_maxBodies; ++i) {
            this.m_bodies[i] = null;
        }
    }

    public CreateBody(index: number) {
        let body = this.m_bodies[this.m_bodyIndex];
        if (body) {
            this.m_world.DestroyBody(body);
            this.m_bodies[this.m_bodyIndex] = null;
        }

        body = this.m_bodies[this.m_bodyIndex] = this.m_world.CreateBody({
            type: b2BodyType.b2_dynamicBody,
            position: { x: b2RandomFloat(-2, 2), y: 10 },
            angle: b2RandomFloat(-Math.PI, Math.PI),
            angularDamping: index === 4 ? 0.02 : 0,
        });

        if (index < 4) {
            body.CreateFixture({
                shape: this.m_polygons[index],
                density: 1,
                friction: 0.3,
            });
        } else {
            body.CreateFixture({
                shape: this.m_circle,
                density: 1,
                friction: 0.3,
            });
        }

        this.m_bodyIndex = (this.m_bodyIndex + 1) % PolygonShapes.e_maxBodies;
    }

    public DestroyBody() {
        for (let i = 0; i < PolygonShapes.e_maxBodies; ++i) {
            const body = this.m_bodies[i];
            if (body) {
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
            hotKeyPress("a", "Toggle Enabled of Even Bodies", () => {
                for (let i = 0; i < PolygonShapes.e_maxBodies; i += 2) {
                    const body = this.m_bodies[i];
                    if (body) {
                        body.SetEnabled(!body.IsEnabled());
                    }
                }
            }),
            hotKeyPress("d", "Destroy Body", () => this.DestroyBody()),
        ];
    }

    public Step(settings: Settings, timeStep: number): void {
        super.Step(settings, timeStep);

        const draw = settings.m_debugDraw;

        let count = 0;
        const { circle, transform } = temp.Step;
        circle.m_radius = 2;
        circle.m_p.Set(0, 1.1);
        transform.SetIdentity();

        const aabb = new b2AABB();
        circle.ComputeAABB(aabb, transform, 0);

        /**
         * We find all the fixtures that overlap an AABB. Of those, we use
         * b2TestOverlap to determine which fixtures overlap a circle.
         * Up to 4 overlapped fixtures will be highlighted with a yellow
         * border.
         */
        this.m_world.QueryAABB(aabb, (fixture) => {
            if (count === queryCallbackMaxCount) return false;

            const body = fixture.GetBody();
            const shape = fixture.GetShape();

            const overlap = b2TestOverlap(shape, 0, circle, 0, body.GetTransform(), transform);

            if (overlap) {
                const color = new b2Color(0.95, 0.95, 0.6);
                const center = body.GetWorldCenter();
                draw.DrawPoint(center, 5, color);
                ++count;
            }

            return true;
        });

        const color = new b2Color(0.4, 0.7, 0.8);
        draw.DrawCircle(circle.m_p, circle.m_radius, color);
    }
}

registerTest("Geometry", "Polygon Shapes", PolygonShapes);
