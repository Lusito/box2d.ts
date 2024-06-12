// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

import {
    b2WheelJoint,
    b2EdgeShape,
    b2Vec2,
    b2BodyType,
    b2CircleShape,
    b2WheelJointDef,
    b2LinearStiffness,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { checkboxDef } from "../../ui/controls/Checkbox";
import { sliderDef } from "../../ui/controls/Slider";

// Test the wheel joint with motor, spring, and limit options.
class WheelJoint extends Test {
    public m_joint: b2WheelJoint;

    public m_motorSpeed = 10;

    public m_enableMotor = false;

    public m_enableLimit = true;

    public constructor() {
        super();

        const ground = this.m_world.CreateBody();
        {
            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(-40, 0), new b2Vec2(40, 0));
            ground.CreateFixture({ shape, density: 0 });
        }

        {
            const shape = new b2CircleShape(2);

            const position = new b2Vec2(0, 10);
            const body = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position,
                allowSleep: false,
            });
            body.CreateFixture({ shape, density: 5 });

            const jd = new b2WheelJointDef();

            // Horizontal
            jd.Initialize(ground, body, position, new b2Vec2(0, 1));

            jd.motorSpeed = this.m_motorSpeed;
            jd.maxMotorTorque = 10000;
            jd.enableMotor = this.m_enableMotor;
            jd.lowerTranslation = -3;
            jd.upperTranslation = 3;
            jd.enableLimit = this.m_enableLimit;

            const hertz = 1;
            const dampingRatio = 0.7;
            b2LinearStiffness(jd, hertz, dampingRatio, ground, body);

            this.m_joint = this.m_world.CreateJoint(jd);
        }

        this.addTestControlGroup("Joint", [
            checkboxDef("Limit", this.m_enableLimit, (value: boolean) => {
                this.m_enableLimit = this.m_joint.EnableLimit(value);
            }),
            checkboxDef("Motor", this.m_enableMotor, (value: boolean) => {
                this.m_enableMotor = this.m_joint.EnableMotor(value);
            }),
            sliderDef("Speed", -100, 100, 1, this.m_motorSpeed, (value: number) => {
                this.m_motorSpeed = this.m_joint.SetMotorSpeed(value);
            }),
        ]);
    }

    public Step(settings: Settings, timeStep: number): void {
        super.Step(settings, timeStep);

        const torque = this.m_joint.GetMotorTorque(settings.m_hertz);
        this.addDebug("Motor Torque", torque);

        const F = this.m_joint.GetReactionForce(settings.m_hertz, new b2Vec2());
        this.addDebug("Reaction Force", `(${F.x.toFixed(1)}, ${F.y.toFixed(1)})`);
    }
}

registerTest("Joints", "Wheel", WheelJoint);
