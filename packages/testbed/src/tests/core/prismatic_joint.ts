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
    public joint: PrismaticJoint;

    public motorSpeed = 10;

    public enableMotor = false;

    public enableLimit = true;

    public constructor() {
        super();

        let ground = null;

        {
            ground = this.world.createBody();

            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-40, 0), new Vec2(40, 0));
            ground.createFixture({ shape });
        }

        {
            const shape = new PolygonShape();
            shape.setAsBox(1, 1);

            const position = { x: 0, y: 10 };
            const body = this.world.createBody({
                type: BodyType.Dynamic,
                position,
                angle: 0.5 * Math.PI,
                allowSleep: false,
            });
            body.createFixture({ shape, density: 5 });

            const pjd = new PrismaticJointDef();

            // Horizontal
            pjd.initialize(ground, body, position, new Vec2(1, 0));

            pjd.motorSpeed = this.motorSpeed;
            pjd.maxMotorForce = 10000;
            pjd.enableMotor = this.enableMotor;
            pjd.lowerTranslation = -10;
            pjd.upperTranslation = 10;
            pjd.enableLimit = this.enableLimit;

            this.joint = this.world.createJoint(pjd);
        }
    }

    public setupControls() {
        this.addTestControlGroup("Joint", [
            checkboxDef("Limit", this.enableLimit, (value: boolean) => {
                this.enableLimit = this.joint.enableLimit(value);
            }),
            checkboxDef("Motor", this.enableMotor, (value: boolean) => {
                this.enableMotor = this.joint.enableMotor(value);
            }),
            sliderDef("Speed", -100, 100, 1, this.motorSpeed, (value: number) => {
                this.motorSpeed = this.joint.setMotorSpeed(value);
            }),
        ]);
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("l", "Toggle Limit", () => this.joint.enableLimit(!this.joint.isLimitEnabled())),
            hotKeyPress("m", "Start/Stop", () => this.joint.enableMotor(!this.joint.isMotorEnabled())),
            hotKeyPress("s", "Reverse Direction", () => this.joint.setMotorSpeed(-this.joint.getMotorSpeed())),
        ];
    }

    public step(settings: Settings, timeStep: number): void {
        super.step(settings, timeStep);
        const force = this.joint.getMotorForce(settings.hertz);
        this.addDebug("Motor Force", force.toFixed());
    }
}

registerTest("Joints", "Prismatic", PrismaticJointTest);
