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

import { RevoluteJoint, BodyType, PolygonShape, Vec2, RevoluteJointDef, XY } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";

class TumblerTest extends Test {
    public static readonly e_count = 800;

    public joint: RevoluteJoint;

    public count = 0;

    public constructor() {
        super();

        const ground = this.world.createBody();

        {
            const body = this.world.createBody({
                type: BodyType.Dynamic,
                allowSleep: false,
                position: { x: 0, y: 10 },
            });

            const shape = new PolygonShape();
            shape.setAsBox(0.5, 10, new Vec2(10, 0), 0);
            body.createFixture({ shape, density: 5 });
            shape.setAsBox(0.5, 10, new Vec2(-10, 0), 0);
            body.createFixture({ shape, density: 5 });
            shape.setAsBox(10, 0.5, new Vec2(0, 10), 0);
            body.createFixture({ shape, density: 5 });
            shape.setAsBox(10, 0.5, new Vec2(0, -10), 0);
            body.createFixture({ shape, density: 5 });

            const jd = new RevoluteJointDef();
            jd.bodyA = ground;
            jd.bodyB = body;
            jd.localAnchorA.set(0, 10);
            jd.localAnchorB.set(0, 0);
            jd.referenceAngle = 0;
            jd.motorSpeed = 0.05 * Math.PI;
            jd.maxMotorTorque = 1e8;
            jd.enableMotor = true;
            this.joint = this.world.createJoint(jd);
        }

        this.count = 0;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 5,
        };
    }

    public step(settings: Settings, timeStep: number): void {
        super.step(settings, timeStep);

        if (this.count < TumblerTest.e_count) {
            const body = this.world.createBody({
                type: BodyType.Dynamic,
                position: { x: 0, y: 10 },
            });

            const shape = new PolygonShape();
            shape.setAsBox(0.125, 0.125);
            body.createFixture({ shape, density: 1 });

            ++this.count;
        }
    }
}

registerTest("Benchmark", "Tumbler", TumblerTest);
