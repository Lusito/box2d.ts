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

import { Body, EdgeShape, Vec2, PolygonShape, FixtureDef, RevoluteJointDef, BodyType, CircleShape } from "@box2d/core";

import { registerTest, Test } from "../../test";

class BridgeTest extends Test {
    public static readonly e_count = 30;

    public middle!: Body;

    public constructor() {
        super();

        let ground = null;

        {
            ground = this.world.createBody();

            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-40, 0), new Vec2(40, 0));
            ground.createFixture({ shape });
        }

        {
            const shape = new PolygonShape();
            shape.setAsBox(0.5, 0.125);

            const fd: FixtureDef = {
                shape,
                density: 20,
                friction: 0.2,
            };

            const jd = new RevoluteJointDef();

            let prevBody = ground;
            for (let i = 0; i < BridgeTest.e_count; ++i) {
                const body = this.world.createBody({
                    type: BodyType.Dynamic,
                    position: { x: -14.5 + 1 * i, y: 5 },
                });
                body.createFixture(fd);

                const anchor = new Vec2(-15 + 1 * i, 5);
                jd.initialize(prevBody, body, anchor);
                this.world.createJoint(jd);

                if (i === BridgeTest.e_count >> 1) {
                    this.middle = body;
                }
                prevBody = body;
            }

            const anchor = new Vec2(-15 + 1 * BridgeTest.e_count, 5);
            jd.initialize(prevBody, ground, anchor);
            this.world.createJoint(jd);
        }

        for (let i = 0; i < 2; ++i) {
            const vertices = [];
            vertices[0] = new Vec2(-0.5, 0);
            vertices[1] = new Vec2(0.5, 0);
            vertices[2] = new Vec2(0, 1.5);

            const shape = new PolygonShape();
            shape.set(vertices);

            const body = this.world.createBody({
                type: BodyType.Dynamic,
                position: { x: -8 + 8 * i, y: 12 },
            });
            body.createFixture({
                shape,
                density: 1,
            });
        }

        for (let i = 0; i < 3; ++i) {
            const shape = new CircleShape();
            shape.radius = 0.5;

            const body = this.world.createBody({
                type: BodyType.Dynamic,
                position: { x: -6 + 6 * i, y: 10 },
            });
            body.createFixture({
                shape,
                density: 1,
            });
        }
    }
}

registerTest("Joints", "Bridge", BridgeTest);
