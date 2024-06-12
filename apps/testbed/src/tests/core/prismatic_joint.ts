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

import { b2PrismaticJoint, b2EdgeShape, b2Vec2, b2PolygonShape, b2BodyType, b2PrismaticJointDef } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { HotKey, hotKeyPress } from "../../utils/hotkeys";
import { checkboxDef } from "../../ui/controls/Checkbox";
import { sliderDef } from "../../ui/controls/Slider";

// Test the prismatic joint with limits and motor options.
class PrismaticJoint extends Test {
    public m_joint: b2PrismaticJoint;

    public m_motorSpeed = 10;

    public m_enableMotor = false;

    public m_enableLimit = true;

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
            shape.SetAsBox(1, 1);

            const position = { x: 0, y: 10 };
            const body = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position,
                angle: 0.5 * Math.PI,
                allowSleep: false,
            });
            body.CreateFixture({ shape, density: 5 });

            const pjd = new b2PrismaticJointDef();

            // Horizontal
            pjd.Initialize(ground, body, position, new b2Vec2(1, 0));

            pjd.motorSpeed = this.m_motorSpeed;
            pjd.maxMotorForce = 10000;
            pjd.enableMotor = this.m_enableMotor;
            pjd.lowerTranslation = -10;
            pjd.upperTranslation = 10;
            pjd.enableLimit = this.m_enableLimit;

            this.m_joint = this.m_world.CreateJoint(pjd);
        }
    }

    public setupControls() {
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

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("l", "Toggle Limit", () => this.m_joint.EnableLimit(!this.m_joint.IsLimitEnabled())),
            hotKeyPress("m", "Start/Stop", () => this.m_joint.EnableMotor(!this.m_joint.IsMotorEnabled())),
            hotKeyPress("s", "Reverse Direction", () => this.m_joint.SetMotorSpeed(-this.m_joint.GetMotorSpeed())),
        ];
    }

    public Step(settings: Settings, timeStep: number): void {
        super.Step(settings, timeStep);
        const force = this.m_joint.GetMotorForce(settings.m_hertz);
        this.addDebug("Motor Force", force.toFixed());
    }
}

registerTest("Joints", "Prismatic", PrismaticJoint);
