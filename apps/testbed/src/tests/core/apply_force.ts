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
    b2Body,
    b2Vec2,
    b2EdgeShape,
    b2FixtureDef,
    b2Transform,
    b2Rot,
    b2PolygonShape,
    b2BodyType,
    b2FrictionJointDef,
    XY,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { HotKey, hotKeyStep } from "../../utils/hotkeys";

// This test shows how to apply forces and torques to a body.
// It also shows how to use the friction joint that can be useful
// for overhead games.
class ApplyForce extends Test {
    public m_body: b2Body;

    public positiveForce = false;

    public negativeForce = false;

    public cwTorque = false;

    public ccwTorque = false;

    public constructor() {
        super(b2Vec2.ZERO);

        const k_restitution = 0.4;

        const ground = this.m_world.CreateBody({
            position: {
                x: 0,
                y: 20,
            },
        });
        {
            const shape = new b2EdgeShape();

            const sd: b2FixtureDef = {
                shape,
                density: 0,
                restitution: k_restitution,
            };

            // Left vertical
            shape.SetTwoSided(new b2Vec2(-20, -20), new b2Vec2(-20, 20));
            ground.CreateFixture(sd);

            // Right vertical
            shape.SetTwoSided(new b2Vec2(20, -20), new b2Vec2(20, 20));
            ground.CreateFixture(sd);

            // Top horizontal
            shape.SetTwoSided(new b2Vec2(-20, 20), new b2Vec2(20, 20));
            ground.CreateFixture(sd);

            // Bottom horizontal
            shape.SetTwoSided(new b2Vec2(-20, -20), new b2Vec2(20, -20));
            ground.CreateFixture(sd);
        }

        {
            const xf1 = new b2Transform();
            xf1.q.Set(0.3524 * Math.PI);
            xf1.q.GetXAxis(xf1.p);

            const vertices = [];
            vertices[0] = b2Transform.MultiplyVec2(xf1, new b2Vec2(-1, 0), new b2Vec2());
            vertices[1] = b2Transform.MultiplyVec2(xf1, new b2Vec2(1, 0), new b2Vec2());
            vertices[2] = b2Transform.MultiplyVec2(xf1, new b2Vec2(0, 0.5), new b2Vec2());

            const poly1 = new b2PolygonShape();
            poly1.Set(vertices, 3);

            const xf2 = new b2Transform();
            xf2.q.Set(-0.3524 * Math.PI);
            xf2.q.GetXAxis(xf2.p).Negate();
            xf2.p.Copy(b2Rot.MultiplyVec2(xf2.q, new b2Vec2(-1, 0), new b2Vec2()));

            vertices[0] = b2Transform.MultiplyVec2(xf2, new b2Vec2(-1, 0), new b2Vec2());
            vertices[1] = b2Transform.MultiplyVec2(xf2, new b2Vec2(1, 0), new b2Vec2());
            vertices[2] = b2Transform.MultiplyVec2(xf2, new b2Vec2(0, 0.5), new b2Vec2());

            const poly2 = new b2PolygonShape();
            poly2.Set(vertices, 3);

            this.m_body = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: { x: 0, y: 3 },
                angle: Math.PI,
                allowSleep: false,
            });
            this.m_body.CreateFixture({
                shape: poly1,
                density: 2,
            });
            this.m_body.CreateFixture({
                shape: poly2,
                density: 2,
            });

            const gravity = 10;
            const I = this.m_body.GetInertia();
            const mass = this.m_body.GetMass();

            // Compute an effective radius that can be used to
            // set the max torque for a friction joint
            // For a circle = 0.5 * m * r * r ==> r = sqrt(2 * I / m)
            const radius = Math.sqrt((2 * I) / mass);

            const jd = new b2FrictionJointDef();
            jd.bodyA = ground;
            jd.bodyB = this.m_body;
            jd.localAnchorA.SetZero();
            jd.localAnchorB.Copy(this.m_body.GetLocalCenter());
            jd.collideConnected = true;
            jd.maxForce = 0.5 * mass * gravity;
            jd.maxTorque = 0.2 * mass * radius * gravity;

            this.m_world.CreateJoint(jd);
        }

        {
            const shape = new b2PolygonShape();
            shape.SetAsBox(0.5, 0.5);

            const fd: b2FixtureDef = {
                shape,
                density: 1,
                friction: 0.3,
            };

            for (let i = 0; i < 10; ++i) {
                const body = this.m_world.CreateBody({
                    type: b2BodyType.b2_dynamicBody,

                    position: {
                        x: 0,
                        y: 7 + 1.54 * i,
                    },
                });

                body.CreateFixture(fd);

                const gravity = 10;
                const I = body.GetInertia();
                const mass = body.GetMass();

                // For a circle: I = 0.5 * m * r * r ==> r = sqrt(2 * I / m)
                const radius = Math.sqrt((2 * I) / mass);

                const jd = new b2FrictionJointDef();
                jd.localAnchorA.SetZero();
                jd.localAnchorB.SetZero();
                jd.bodyA = ground;
                jd.bodyB = body;
                jd.collideConnected = true;
                jd.maxForce = mass * gravity;
                jd.maxTorque = 0.1 * mass * radius * gravity;

                this.m_world.CreateJoint(jd);
            }
        }
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyStep("w", "Apply Force", () => this.ApplyForce(-50)),
            hotKeyStep("s", "Apply Backward Force", () => this.ApplyForce(50)),
            hotKeyStep("a", "Apply Torque Counter-Clockwise", () => this.m_body.ApplyTorque(10)),
            hotKeyStep("d", "Apply Torque Clockwise", () => this.m_body.ApplyTorque(-10)),
        ];
    }

    private ApplyForce(value: number) {
        const f = this.m_body.GetWorldVector(new b2Vec2(0, value), new b2Vec2());
        const p = this.m_body.GetWorldPoint(new b2Vec2(0, 3), new b2Vec2());
        this.m_body.ApplyForce(f, p);
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 15,
        };
    }
}

registerTest("Forces", "Apply Force", ApplyForce);
