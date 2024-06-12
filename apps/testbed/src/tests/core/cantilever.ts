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
    b2EdgeShape,
    b2Vec2,
    b2PolygonShape,
    b2FixtureDef,
    b2WeldJointDef,
    b2BodyType,
    b2AngularStiffness,
    b2CircleShape,
} from "@box2d/core";

import { registerTest, Test } from "../../test";

// It is difficult to make a cantilever made of links completely rigid with weld joints.
// You will have to use a high number of iterations to make them stiff.
// So why not go ahead and use soft weld joints? They behave like a revolute
// joint with a rotational spring.
class Cantilever extends Test {
    public static readonly e_count = 8;

    public constructor() {
        super();

        let ground = null;

        {
            ground = this.m_world.CreateBody();

            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(-40, 0), new b2Vec2(40, 0));
            ground.CreateFixture({ shape });
        }

        {
            const shape = new b2PolygonShape();
            shape.SetAsBox(0.5, 0.125);

            const fd: b2FixtureDef = {
                shape,
                density: 20,
            };

            const jd = new b2WeldJointDef();

            let prevBody = ground;
            for (let i = 0; i < Cantilever.e_count; ++i) {
                const body = this.m_world.CreateBody({
                    type: b2BodyType.b2_dynamicBody,
                    position: { x: -14.5 + 1 * i, y: 5 },
                });
                body.CreateFixture(fd);

                const anchor = new b2Vec2(-15 + 1 * i, 5);
                jd.Initialize(prevBody, body, anchor);
                this.m_world.CreateJoint(jd);

                prevBody = body;
            }
        }

        {
            const shape = new b2PolygonShape();
            shape.SetAsBox(1, 0.125);

            const fd: b2FixtureDef = {
                shape,
                density: 20,
            };

            const jd = new b2WeldJointDef();
            const frequencyHz = 5;
            const dampingRatio = 0.7;

            let prevBody = ground;
            for (let i = 0; i < 3; ++i) {
                const body = this.m_world.CreateBody({
                    type: b2BodyType.b2_dynamicBody,
                    position: { x: -14 + 2 * i, y: 15 },
                });
                body.CreateFixture(fd);

                const anchor = new b2Vec2(-15 + 2 * i, 15);
                jd.Initialize(prevBody, body, anchor);
                b2AngularStiffness(jd, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
                this.m_world.CreateJoint(jd);

                prevBody = body;
            }
        }

        {
            const shape = new b2PolygonShape();
            shape.SetAsBox(0.5, 0.125);

            const fd: b2FixtureDef = {
                shape,
                density: 20,
            };

            const jd = new b2WeldJointDef();

            let prevBody = ground;
            for (let i = 0; i < Cantilever.e_count; ++i) {
                const body = this.m_world.CreateBody({
                    type: b2BodyType.b2_dynamicBody,
                    position: { x: -4.5 + 1 * i, y: 5 },
                });
                body.CreateFixture(fd);

                if (i > 0) {
                    const anchor = new b2Vec2(-5 + 1 * i, 5);
                    jd.Initialize(prevBody, body, anchor);
                    this.m_world.CreateJoint(jd);
                }

                prevBody = body;
            }
        }

        {
            const shape = new b2PolygonShape();
            shape.SetAsBox(0.5, 0.125);

            const fd: b2FixtureDef = {
                shape,
                density: 20,
            };

            const jd = new b2WeldJointDef();
            const frequencyHz = 8;
            const dampingRatio = 0.7;

            let prevBody = ground;
            for (let i = 0; i < Cantilever.e_count; ++i) {
                const body = this.m_world.CreateBody({
                    type: b2BodyType.b2_dynamicBody,
                    position: { x: 5.5 + 1 * i, y: 10 },
                });
                body.CreateFixture(fd);

                if (i > 0) {
                    const anchor = new b2Vec2(5 + 1 * i, 10);
                    jd.Initialize(prevBody, body, anchor);
                    b2AngularStiffness(jd, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
                    this.m_world.CreateJoint(jd);
                }

                prevBody = body;
            }
        }

        for (let i = 0; i < 2; ++i) {
            const vertices = [];
            vertices[0] = new b2Vec2(-0.5, 0);
            vertices[1] = new b2Vec2(0.5, 0);
            vertices[2] = new b2Vec2(0, 1.5);

            const shape = new b2PolygonShape();
            shape.Set(vertices);

            const body = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: { x: -8 + 8 * i, y: 12 },
            });
            body.CreateFixture({
                shape,
                density: 1,
            });
        }

        for (let i = 0; i < 2; ++i) {
            const shape = new b2CircleShape();
            shape.m_radius = 0.5;

            const body = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: { x: -6 + 6 * i, y: 10 },
            });
            body.CreateFixture({
                shape,
                density: 1,
            });
        }
    }
}

registerTest("Joints", "Cantilever", Cantilever);
