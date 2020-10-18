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

import { b2RevoluteJoint, b2BodyDef, b2BodyType, b2PolygonShape, b2Vec2, b2RevoluteJointDef, XY } from "@box2d/core";

import { Test } from "../../test";
import { Settings } from "../../settings";

export class Tumbler extends Test {
    public static readonly e_count = 800;

    public m_joint: b2RevoluteJoint;

    public m_count = 0;

    constructor() {
        super();

        const ground = this.m_world.CreateBody(new b2BodyDef());

        {
            const bd = new b2BodyDef();
            bd.type = b2BodyType.b2_dynamicBody;
            bd.allowSleep = false;
            bd.position.Set(0.0, 10.0);
            const body = this.m_world.CreateBody(bd);

            const shape = new b2PolygonShape();
            shape.SetAsBox(0.5, 10.0, new b2Vec2(10.0, 0.0), 0.0);
            body.CreateFixture(shape, 5.0);
            shape.SetAsBox(0.5, 10.0, new b2Vec2(-10.0, 0.0), 0.0);
            body.CreateFixture(shape, 5.0);
            shape.SetAsBox(10.0, 0.5, new b2Vec2(0.0, 10.0), 0.0);
            body.CreateFixture(shape, 5.0);
            shape.SetAsBox(10.0, 0.5, new b2Vec2(0.0, -10.0), 0.0);
            body.CreateFixture(shape, 5.0);

            const jd = new b2RevoluteJointDef();
            jd.bodyA = ground;
            jd.bodyB = body;
            jd.localAnchorA.Set(0.0, 10.0);
            jd.localAnchorB.Set(0.0, 0.0);
            jd.referenceAngle = 0.0;
            jd.motorSpeed = 0.05 * Math.PI;
            jd.maxMotorTorque = 1e8;
            jd.enableMotor = true;
            this.m_joint = this.m_world.CreateJoint(jd);
        }

        this.m_count = 0;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 5,
        };
    }

    public Step(settings: Settings, timeStep: number): void {
        super.Step(settings, timeStep);

        if (this.m_count < Tumbler.e_count) {
            const bd = new b2BodyDef();
            bd.type = b2BodyType.b2_dynamicBody;
            bd.position.Set(0.0, 10.0);
            const body = this.m_world.CreateBody(bd);

            const shape = new b2PolygonShape();
            shape.SetAsBox(0.125, 0.125);
            body.CreateFixture(shape, 1.0);

            ++this.m_count;
        }
    }
}