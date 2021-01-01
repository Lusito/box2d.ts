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
    CircleShape,
    Transform,
    testOverlap,
    Color,
    Body,
    PolygonShape,
    EdgeShape,
    Vec2,
    BodyType,
    randomFloat,
    AABB,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { g_debugDraw } from "../../utils/draw";
import { HotKey, hotKeyPress } from "../../utils/hotkeys";

const queryCallbackMaxCount = 4;

const temp = {
    Step: {
        circle: new CircleShape(),
        transform: new Transform(),
    },
};

/**
 * This tests stacking. It also shows how to use World::Query
 * and testOverlap.
 */
class PolygonShapesTest extends Test {
    public static readonly e_maxBodies = 256;

    public m_bodyIndex = 0;

    public m_bodies: Array<Body | null> = Array.from({ length: PolygonShapesTest.e_maxBodies }, () => null);

    public m_polygons = Array.from({ length: 4 }, () => new PolygonShape());

    public m_circle = new CircleShape();

    public constructor() {
        super();

        // Ground body
        {
            const ground = this.m_world.createBody();

            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-40, 0), new Vec2(40, 0));
            ground.createFixture({ shape });
        }

        {
            const vertices = new Array(3);
            vertices[0] = new Vec2(-0.5, 0);
            vertices[1] = new Vec2(0.5, 0);
            vertices[2] = new Vec2(0, 1.5);
            this.m_polygons[0].set(vertices, 3);
        }

        {
            const vertices = new Array(3);
            vertices[0] = new Vec2(-0.1, 0);
            vertices[1] = new Vec2(0.1, 0);
            vertices[2] = new Vec2(0, 1.5);
            this.m_polygons[1].set(vertices, 3);
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

            this.m_polygons[2].set(vertices, 8);
        }

        this.m_polygons[3].setAsBox(0.5, 0.5);
        this.m_circle.m_radius = 0.5;

        for (let i = 0; i < PolygonShapesTest.e_maxBodies; ++i) {
            this.m_bodies[i] = null;
        }
    }

    public createBody(index: number) {
        let body = this.m_bodies[this.m_bodyIndex];
        if (body) {
            this.m_world.destroyBody(body);
            this.m_bodies[this.m_bodyIndex] = null;
        }

        body = this.m_bodies[this.m_bodyIndex] = this.m_world.createBody({
            type: BodyType.Dynamic,
            position: { x: randomFloat(-2, 2), y: 10 },
            angle: randomFloat(-Math.PI, Math.PI),
            angularDamping: index === 4 ? 0.02 : 0,
        });

        if (index < 4) {
            body.createFixture({
                shape: this.m_polygons[index],
                density: 1,
                friction: 0.3,
            });
        } else {
            body.createFixture({
                shape: this.m_circle,
                density: 1,
                friction: 0.3,
            });
        }

        this.m_bodyIndex = (this.m_bodyIndex + 1) % PolygonShapesTest.e_maxBodies;
    }

    public destroyBody() {
        for (let i = 0; i < PolygonShapesTest.e_maxBodies; ++i) {
            const body = this.m_bodies[i];
            if (body) {
                this.m_world.destroyBody(body);
                this.m_bodies[i] = null;
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
            hotKeyPress("a", "Toggle Enabled of Even Bodies", () => {
                for (let i = 0; i < PolygonShapesTest.e_maxBodies; i += 2) {
                    const body = this.m_bodies[i];
                    if (body) {
                        body.setEnabled(!body.isEnabled());
                    }
                }
            }),
            hotKeyPress("d", "Destroy Body", () => this.destroyBody()),
        ];
    }

    public step(settings: Settings, timeStep: number): void {
        super.step(settings, timeStep);

        let count = 0;
        const { circle, transform } = temp.Step;
        circle.m_radius = 2;
        circle.m_p.set(0, 1.1);
        transform.setIdentity();

        const aabb = new AABB();
        circle.computeAABB(aabb, transform, 0);

        /**
         * We find all the fixtures that overlap an AABB. Of those, we use
         * testOverlap to determine which fixtures overlap a circle.
         * Up to 4 overlapped fixtures will be highlighted with a yellow
         * border.
         */
        this.m_world.queryAABB(aabb, (fixture) => {
            if (count === queryCallbackMaxCount) return false;

            const body = fixture.getBody();
            const shape = fixture.getShape();

            const overlap = testOverlap(shape, 0, circle, 0, body.getTransform(), transform);

            if (overlap) {
                const color = new Color(0.95, 0.95, 0.6);
                const center = body.getWorldCenter();
                g_debugDraw.drawPoint(center, 5, color);
                ++count;
            }

            return true;
        });

        const color = new Color(0.4, 0.7, 0.8);
        g_debugDraw.drawCircle(circle.m_p, circle.m_radius, color);
    }
}

registerTest("Geometry", "Polygon Shapes", PolygonShapesTest);
