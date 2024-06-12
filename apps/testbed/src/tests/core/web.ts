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
    b2Joint,
    b2EdgeShape,
    b2Vec2,
    b2PolygonShape,
    b2BodyType,
    b2DistanceJointDef,
    b2LinearStiffness,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { HotKey, hotKeyPress } from "../../utils/hotkeys";

// Test distance joints, body destruction, and joint destruction.
class Web extends Test {
    public m_bodies = new Array<b2Body | null>(4);

    public m_joints = new Array<b2Joint | null>(8);

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
            shape.SetAsBox(0.5, 0.5);

            const body0 = (this.m_bodies[0] = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: { x: -5, y: 5 },
            }));
            body0.CreateFixture({ shape, density: 5 });

            const body1 = (this.m_bodies[1] = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: { x: 5, y: 5 },
            }));
            body1.CreateFixture({ shape, density: 5 });

            const body2 = (this.m_bodies[2] = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: { x: 5, y: 15 },
            }));
            body2.CreateFixture({ shape, density: 5 });

            const body3 = (this.m_bodies[3] = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: { x: -5, y: 15 },
            }));
            body3.CreateFixture({ shape, density: 5 });

            const jd = new b2DistanceJointDef();
            let p1;
            let p2;
            let d;

            const frequencyHz = 2;
            const dampingRatio = 0;

            jd.bodyA = ground;
            jd.bodyB = body0;
            jd.localAnchorA.Set(-10, 0);
            jd.localAnchorB.Set(-0.5, -0.5);
            p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA, new b2Vec2());
            p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB, new b2Vec2());
            d = b2Vec2.Subtract(p2, p1, new b2Vec2());
            jd.length = d.Length();
            b2LinearStiffness(jd, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
            this.m_joints[0] = this.m_world.CreateJoint(jd);

            jd.bodyA = ground;
            jd.bodyB = body1;
            jd.localAnchorA.Set(10, 0);
            jd.localAnchorB.Set(0.5, -0.5);
            p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA, new b2Vec2());
            p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB, new b2Vec2());
            d = b2Vec2.Subtract(p2, p1, new b2Vec2());
            jd.length = d.Length();
            b2LinearStiffness(jd, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
            this.m_joints[1] = this.m_world.CreateJoint(jd);

            jd.bodyA = ground;
            jd.bodyB = body2;
            jd.localAnchorA.Set(10, 20);
            jd.localAnchorB.Set(0.5, 0.5);
            p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA, new b2Vec2());
            p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB, new b2Vec2());
            d = b2Vec2.Subtract(p2, p1, new b2Vec2());
            jd.length = d.Length();
            b2LinearStiffness(jd, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
            this.m_joints[2] = this.m_world.CreateJoint(jd);

            jd.bodyA = ground;
            jd.bodyB = body3;
            jd.localAnchorA.Set(-10, 20);
            jd.localAnchorB.Set(-0.5, 0.5);
            p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA, new b2Vec2());
            p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB, new b2Vec2());
            d = b2Vec2.Subtract(p2, p1, new b2Vec2());
            jd.length = d.Length();
            b2LinearStiffness(jd, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
            this.m_joints[3] = this.m_world.CreateJoint(jd);

            jd.bodyA = body0;
            jd.bodyB = body1;
            jd.localAnchorA.Set(0.5, 0);
            jd.localAnchorB.Set(-0.5, 0);
            p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA, new b2Vec2());
            p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB, new b2Vec2());
            d = b2Vec2.Subtract(p2, p1, new b2Vec2());
            jd.length = d.Length();
            b2LinearStiffness(jd, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
            this.m_joints[4] = this.m_world.CreateJoint(jd);

            jd.bodyA = body1;
            jd.bodyB = body2;
            jd.localAnchorA.Set(0, 0.5);
            jd.localAnchorB.Set(0, -0.5);
            p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA, new b2Vec2());
            p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB, new b2Vec2());
            d = b2Vec2.Subtract(p2, p1, new b2Vec2());
            jd.length = d.Length();
            b2LinearStiffness(jd, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
            this.m_joints[5] = this.m_world.CreateJoint(jd);

            jd.bodyA = body2;
            jd.bodyB = body3;
            jd.localAnchorA.Set(-0.5, 0);
            jd.localAnchorB.Set(0.5, 0);
            p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA, new b2Vec2());
            p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB, new b2Vec2());
            d = b2Vec2.Subtract(p2, p1, new b2Vec2());
            jd.length = d.Length();
            b2LinearStiffness(jd, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
            this.m_joints[6] = this.m_world.CreateJoint(jd);

            jd.bodyA = body3;
            jd.bodyB = body0;
            jd.localAnchorA.Set(0, -0.5);
            jd.localAnchorB.Set(0, 0.5);
            p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA, new b2Vec2());
            p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB, new b2Vec2());
            d = b2Vec2.Subtract(p2, p1, new b2Vec2());
            jd.length = d.Length();
            b2LinearStiffness(jd, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
            this.m_joints[7] = this.m_world.CreateJoint(jd);
        }
    }

    public JointDestroyed(joint: b2Joint) {
        for (let i = 0; i < 8; ++i) {
            if (this.m_joints[i] === joint) {
                this.m_joints[i] = null;
                break;
            }
        }
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("b", "Delete a Body", () => {
                for (let i = 0; i < 4; ++i) {
                    const body = this.m_bodies[i];
                    if (body) {
                        this.m_world.DestroyBody(body);
                        this.m_bodies[i] = null;
                        break;
                    }
                }
            }),
            hotKeyPress("j", "Delete a Joint", () => {
                for (let i = 0; i < 8; ++i) {
                    const joint = this.m_joints[i];
                    if (joint) {
                        this.m_world.DestroyJoint(joint);
                        this.m_joints[i] = null;
                        break;
                    }
                }
            }),
        ];
    }

    public Step(settings: Settings, timeStep: number): void {
        super.Step(settings, timeStep);
        this.addText("This demonstrates a soft distance joint.");
    }
}

registerTest("Examples", "Web", Web);
