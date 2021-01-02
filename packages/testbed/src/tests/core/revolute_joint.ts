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
    Body,
    RevoluteJoint,
    EdgeShape,
    Vec2,
    CircleShape,
    BodyType,
    RevoluteJointDef,
    PolygonShape,
    XY,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { checkboxDef } from "../../ui/controls/Checkbox";
import { sliderDef } from "../../ui/controls/Slider";

class RevoluteJointTest extends Test {
    public ball: Body;

    public joint1: RevoluteJoint;

    public joint2: RevoluteJoint;

    public motorSpeed = 1;

    public enableMotor = false;

    public enableLimit = true;

    public constructor() {
        super();

        let ground = null;

        {
            ground = this.world.createBody();

            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-40, 0), new Vec2(40, 0));

            ground.createFixture({
                shape,
                // filter.categoryBits = 2;
            });
        }

        {
            const shape = new PolygonShape();
            shape.setAsBox(0.25, 3, new Vec2(0, 3), 0);

            const body = this.world.createBody({
                type: BodyType.Dynamic,
                position: new Vec2(-10, 20),
            });
            body.createFixture({ shape, density: 5 });

            const jd = new RevoluteJointDef();
            jd.initialize(ground, body, new Vec2(-10, 20.5));
            jd.motorSpeed = this.motorSpeed;
            jd.maxMotorTorque = 10000;
            jd.enableMotor = this.enableMotor;
            jd.lowerAngle = -0.25 * Math.PI;
            jd.upperAngle = 0.5 * Math.PI;
            jd.enableLimit = this.enableLimit;

            this.joint1 = this.world.createJoint(jd);
        }

        {
            const circle_shape = new CircleShape(2);
            this.ball = this.world.createBody({
                type: BodyType.Dynamic,
                position: new Vec2(5, 30),
            });
            this.ball.createFixture({
                density: 5,
                filter: { maskBits: 1 },
                shape: circle_shape,
            });

            const polygon_shape = new PolygonShape();
            polygon_shape.setAsBox(10, 0.5, new Vec2(-10, 0), 0);

            const polygon_body = this.world.createBody({
                position: new Vec2(20, 10),
                type: BodyType.Dynamic,
                bullet: true,
            });
            polygon_body.createFixture({
                shape: polygon_shape,
                density: 2,
            });

            const jd = new RevoluteJointDef();
            jd.initialize(ground, polygon_body, new Vec2(19, 10));
            jd.lowerAngle = -0.25 * Math.PI;
            jd.upperAngle = 0 * Math.PI;
            jd.enableLimit = true;
            jd.enableMotor = true;
            jd.motorSpeed = 0;
            jd.maxMotorTorque = 10000;

            this.joint2 = this.world.createJoint(jd);
        }

        this.addTestControlGroup("Joint Controls", [
            checkboxDef("Limit", this.enableLimit, (value: boolean) => {
                this.enableLimit = this.joint1.enableLimit(value);
            }),
            checkboxDef("Motor", this.enableMotor, (value: boolean) => {
                this.enableMotor = this.joint1.enableMotor(value);
            }),
            sliderDef("Speed", -20, 20, 1, this.motorSpeed, (value: number) => {
                this.motorSpeed = this.joint1.setMotorSpeed(value);
            }),
        ]);
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 5,
        };
    }

    public step(settings: Settings, timeStep: number): void {
        super.step(settings, timeStep);

        const torque1 = this.joint1.getMotorTorque(settings.hertz);
        this.addDebug("Motor Torque 1", torque1.toFixed(0));

        const torque2 = this.joint2.getMotorTorque(settings.hertz);
        this.addDebug("Motor Torque 2", torque2.toFixed(0));
    }
}

registerTest("Joints", "Revolute", RevoluteJointTest);
