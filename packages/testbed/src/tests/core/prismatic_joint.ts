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

import { PrismaticJoint, EdgeShape, Vec2, PolygonShape, BodyType, PrismaticJointDef } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { HotKey, hotKeyPress } from "../../utils/hotkeys";
import { checkboxDef } from "../../ui/controls/Checkbox";
import { sliderDef } from "../../ui/controls/Slider";

// Test the prismatic joint with limits and motor options.
class PrismaticJointTest extends Test {
    public m_joint: PrismaticJoint;

    public m_motorSpeed = 10;

    public m_enableMotor = false;

    public m_enableLimit = true;

    public constructor() {
        super();

        let ground = null;

        {
            ground = this.m_world.createBody();

            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-40, 0), new Vec2(40, 0));
            ground.createFixture({ shape });
        }

        {
            const shape = new PolygonShape();
            shape.setAsBox(1, 1);

            const position = { x: 0, y: 10 };
            const body = this.m_world.createBody({
                type: BodyType.Dynamic,
                position,
                angle: 0.5 * Math.PI,
                allowSleep: false,
            });
            body.createFixture({ shape, density: 5 });

            const pjd = new PrismaticJointDef();

            // Horizontal
            pjd.initialize(ground, body, position, new Vec2(1, 0));

            pjd.motorSpeed = this.m_motorSpeed;
            pjd.maxMotorForce = 10000;
            pjd.enableMotor = this.m_enableMotor;
            pjd.lowerTranslation = -10;
            pjd.upperTranslation = 10;
            pjd.enableLimit = this.m_enableLimit;

            this.m_joint = this.m_world.createJoint(pjd);
        }
    }

    public setupControls() {
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

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("l", "Toggle Limit", () => this.m_joint.enableLimit(!this.m_joint.isLimitEnabled())),
            hotKeyPress("m", "Start/Stop", () => this.m_joint.enableMotor(!this.m_joint.isMotorEnabled())),
            hotKeyPress("s", "Reverse Direction", () => this.m_joint.setMotorSpeed(-this.m_joint.getMotorSpeed())),
        ];
    }

    public step(settings: Settings, timeStep: number): void {
        super.step(settings, timeStep);
        const force = this.m_joint.getMotorForce(settings.m_hertz);
        this.addDebug("Motor Force", force.toFixed());
    }
}

registerTest("Joints", "Prismatic", PrismaticJointTest);
