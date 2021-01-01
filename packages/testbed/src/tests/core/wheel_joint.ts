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

import { WheelJoint, EdgeShape, Vec2, BodyType, CircleShape, WheelJointDef, linearStiffness } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { checkboxDef } from "../../ui/controls/Checkbox";
import { sliderDef } from "../../ui/controls/Slider";

// Test the wheel joint with motor, spring, and limit options.
class WheelJointTest extends Test {
    public m_joint: WheelJoint;

    public m_motorSpeed = 10;

    public m_enableMotor = false;

    public m_enableLimit = true;

    public constructor() {
        super();

        const ground = this.m_world.createBody();
        {
            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-40, 0), new Vec2(40, 0));
            ground.createFixture({ shape, density: 0 });
        }

        {
            const shape = new CircleShape(2);

            const position = new Vec2(0, 10);
            const body = this.m_world.createBody({
                type: BodyType.Dynamic,
                position,
                allowSleep: false,
            });
            body.createFixture({ shape, density: 5 });

            const jd = new WheelJointDef();

            // Horizontal
            jd.initialize(ground, body, position, new Vec2(0, 1));

            jd.motorSpeed = this.m_motorSpeed;
            jd.maxMotorTorque = 10000;
            jd.enableMotor = this.m_enableMotor;
            jd.lowerTranslation = -3;
            jd.upperTranslation = 3;
            jd.enableLimit = this.m_enableLimit;

            const hertz = 1;
            const dampingRatio = 0.7;
            linearStiffness(jd, hertz, dampingRatio, ground, body);

            this.m_joint = this.m_world.createJoint(jd);
        }

        this.addTestControlGroup("Joint", [
            checkboxDef("Limit", this.m_enableLimit, (value: boolean) => {
                this.m_enableLimit = this.m_joint.enableLimit(value);
            }),
            checkboxDef("Motor", this.m_enableMotor, (value: boolean) => {
                this.m_enableMotor = this.m_joint.enableMotor(value);
            }),
            sliderDef("Speed", -100, 100, 1, this.m_motorSpeed, (value: number) => {
                this.m_motorSpeed = this.m_joint.setMotorSpeed(value);
            }),
        ]);
    }

    public step(settings: Settings, timeStep: number): void {
        super.step(settings, timeStep);

        const torque = this.m_joint.getMotorTorque(settings.m_hertz);
        this.addDebug("Motor Torque", torque);

        const F = this.m_joint.getReactionForce(settings.m_hertz, new Vec2());
        this.addDebug("Reaction Force", `(${F.x.toFixed(1)}, ${F.y.toFixed(1)})`);
    }
}

registerTest("Joints", "Wheel", WheelJointTest);
