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

import { Body, EdgeShape, Vec2, BodyType, CircleShape, MotorJointDef, MotorJoint } from "@box2d/core";

import { registerTest, Test } from "../../test";

// Adapted from MotorJoint.h

class MotorJoint2Test extends Test {
    public constructor() {
        super();

        let ground: Body;
        {
            ground = this.world.createBody();

            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-20, 0), new Vec2(20, 0));

            ground.createFixture({ shape });
        }

        // Body * body1 = NULL;
        let body1: Body;
        {
            body1 = this.world.createBody({
                type: BodyType.Dynamic,
                position: { x: 0, y: 4 },
            });

            const shape = new CircleShape();
            shape.radius = 1;

            body1.createFixture({
                shape,
                friction: 0.6,
                density: 2,
            });
        }

        // Body * body2 = NULL;
        let body2: Body;
        {
            body2 = this.world.createBody({
                type: BodyType.Dynamic,
                position: { x: 4, y: 8 },
            });

            const shape = new CircleShape();
            shape.radius = 1;

            body2.createFixture({
                shape,
                friction: 0.6,
                density: 2,
            });
        }

        {
            const mjd = new MotorJointDef();
            mjd.initialize(body1, body2);
            mjd.maxForce = 1000;
            mjd.maxTorque = 1000;
            this.joint = this.world.createJoint(mjd) as MotorJoint;
        }
    }

    // MotorJoint* joint;
    public joint: MotorJoint;
}

registerTest("Bugs", "Motor Joint (Bug #487)", MotorJoint2Test);
