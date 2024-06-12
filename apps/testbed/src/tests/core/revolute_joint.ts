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
    b2RevoluteJoint,
    b2EdgeShape,
    b2Vec2,
    b2CircleShape,
    b2BodyType,
    b2RevoluteJointDef,
    b2PolygonShape,
    XY,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { checkboxDef } from "../../ui/controls/Checkbox";
import { sliderDef } from "../../ui/controls/Slider";

class RevoluteJoint extends Test {
    public m_ball: b2Body;

    public m_joint1: b2RevoluteJoint;

    public m_joint2: b2RevoluteJoint;

    public m_motorSpeed = 1;

    public m_enableMotor = false;

    public m_enableLimit = true;

    public constructor() {
        super();

        let ground = null;

        {
            ground = this.m_world.CreateBody();

            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(-40, 0), new b2Vec2(40, 0));

            ground.CreateFixture({
                shape,
                // filter.categoryBits = 2;
            });
        }

        {
            const shape = new b2PolygonShape();
            shape.SetAsBox(0.25, 3, new b2Vec2(0, 3), 0);

            const body = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: new b2Vec2(-10, 20),
            });
            body.CreateFixture({ shape, density: 5 });

            const jd = new b2RevoluteJointDef();
            jd.Initialize(ground, body, new b2Vec2(-10, 20.5));
            jd.motorSpeed = this.m_motorSpeed;
            jd.maxMotorTorque = 10000;
            jd.enableMotor = this.m_enableMotor;
            jd.lowerAngle = -0.25 * Math.PI;
            jd.upperAngle = 0.5 * Math.PI;
            jd.enableLimit = this.m_enableLimit;

            this.m_joint1 = this.m_world.CreateJoint(jd);
        }

        {
            const circle_shape = new b2CircleShape(2);
            this.m_ball = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: new b2Vec2(5, 30),
            });
            this.m_ball.CreateFixture({
                density: 5,
                filter: { maskBits: 1 },
                shape: circle_shape,
            });

            const polygon_shape = new b2PolygonShape();
            polygon_shape.SetAsBox(10, 0.5, new b2Vec2(-10, 0), 0);

            const polygon_body = this.m_world.CreateBody({
                position: new b2Vec2(20, 10),
                type: b2BodyType.b2_dynamicBody,
                bullet: true,
            });
            polygon_body.CreateFixture({
                shape: polygon_shape,
                density: 2,
            });

            const jd = new b2RevoluteJointDef();
            jd.Initialize(ground, polygon_body, new b2Vec2(19, 10));
            jd.lowerAngle = -0.25 * Math.PI;
            jd.upperAngle = 0 * Math.PI;
            jd.enableLimit = true;
            jd.enableMotor = true;
            jd.motorSpeed = 0;
            jd.maxMotorTorque = 10000;

            this.m_joint2 = this.m_world.CreateJoint(jd);
        }

        this.addTestControlGroup("Joint Controls", [
            checkboxDef("Limit", this.m_enableLimit, (value: boolean) => {
                this.m_enableLimit = this.m_joint1.EnableLimit(value);
            }),
            checkboxDef("Motor", this.m_enableMotor, (value: boolean) => {
                this.m_enableMotor = this.m_joint1.EnableMotor(value);
            }),
            sliderDef("Speed", -20, 20, 1, this.m_motorSpeed, (value: number) => {
                this.m_motorSpeed = this.m_joint1.SetMotorSpeed(value);
            }),
        ]);
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 5,
        };
    }

    public Step(settings: Settings, timeStep: number): void {
        super.Step(settings, timeStep);

        const torque1 = this.m_joint1.GetMotorTorque(settings.m_hertz);
        this.addDebug("Motor Torque 1", torque1.toFixed(0));

        const torque2 = this.m_joint2.GetMotorTorque(settings.m_hertz);
        this.addDebug("Motor Torque 2", torque2.toFixed(0));
    }
}

registerTest("Joints", "Revolute", RevoluteJoint);
