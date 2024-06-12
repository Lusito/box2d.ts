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

import { b2Body, b2EdgeShape, b2Vec2, b2BodyType, b2CircleShape, b2MotorJointDef, b2MotorJoint } from "@box2d/core";

import { registerTest, Test } from "../../test";

// Adapted from MotorJoint.h

class MotorJoint2 extends Test {
    public constructor() {
        super();

        let ground: b2Body;
        {
            ground = this.m_world.CreateBody();

            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(-20, 0), new b2Vec2(20, 0));

            ground.CreateFixture({ shape });
        }

        // b2Body * body1 = NULL;
        let body1: b2Body;
        {
            body1 = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: { x: 0, y: 4 },
            });

            const shape = new b2CircleShape();
            shape.m_radius = 1;

            body1.CreateFixture({
                shape,
                friction: 0.6,
                density: 2,
            });
        }

        // b2Body * body2 = NULL;
        let body2: b2Body;
        {
            body2 = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: { x: 4, y: 8 },
            });

            const shape = new b2CircleShape();
            shape.m_radius = 1;

            body2.CreateFixture({
                shape,
                friction: 0.6,
                density: 2,
            });
        }

        {
            const mjd = new b2MotorJointDef();
            mjd.Initialize(body1, body2);
            mjd.maxForce = 1000;
            mjd.maxTorque = 1000;
            this.m_joint = this.m_world.CreateJoint(mjd) as b2MotorJoint;
        }
    }

    // b2MotorJoint* m_joint;
    public m_joint: b2MotorJoint;
}

registerTest("Bugs", "Motor Joint (Bug #487)", MotorJoint2);
