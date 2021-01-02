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

import { Body, EdgeShape, Vec2, BodyType, PolygonShape, RevoluteJointDef, PrismaticJointDef } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { hotKeyPress, HotKey } from "../../utils/hotkeys";

class BodyTypesTest extends Test {
    public attachment: Body;

    public platform: Body;

    public speed = 0;

    public constructor() {
        super();

        const ground = this.world.createBody();
        {
            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-20, 0), new Vec2(20, 0));

            ground.createFixture({ shape });
        }

        // Define attachment
        {
            this.attachment = this.world.createBody({
                type: BodyType.Dynamic,
                position: {
                    x: 0,
                    y: 3,
                },
            });

            const shape = new PolygonShape();
            shape.setAsBox(0.5, 2);
            this.attachment.createFixture({ shape, density: 2 });
        }

        // Define platform
        {
            this.platform = this.world.createBody({
                type: BodyType.Dynamic,
                position: { x: -4, y: 5 },
            });

            const shape = new PolygonShape();
            shape.setAsBox(0.5, 4, new Vec2(4, 0), 0.5 * Math.PI);

            this.platform.createFixture({
                shape,
                friction: 0.6,
                density: 2,
            });

            const rjd = new RevoluteJointDef();
            rjd.initialize(this.attachment, this.platform, new Vec2(0, 5));
            rjd.maxMotorTorque = 50;
            rjd.enableMotor = true;
            this.world.createJoint(rjd);

            const pjd = new PrismaticJointDef();
            pjd.initialize(ground, this.platform, new Vec2(0, 5), new Vec2(1, 0));

            pjd.maxMotorForce = 1000;
            pjd.enableMotor = true;
            pjd.lowerTranslation = -10;
            pjd.upperTranslation = 10;
            pjd.enableLimit = true;

            this.world.createJoint(pjd);

            this.speed = 3;
        }

        // Create a payload
        {
            const body = this.world.createBody({
                type: BodyType.Dynamic,
                position: { x: 0, y: 8 },
            });

            const shape = new PolygonShape();
            shape.setAsBox(0.75, 0.75);

            body.createFixture({
                shape,
                friction: 0.6,
                density: 2,
            });
        }
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("d", "Set Dynamic Body", () => this.platform.setType(BodyType.Dynamic)),
            hotKeyPress("s", "Set Static Body", () => this.platform.setType(BodyType.Static)),
            hotKeyPress("k", "Set Kinematic Body", () => {
                this.platform.setType(BodyType.Kinematic);
                this.platform.setLinearVelocity(new Vec2(-this.speed, 0));
                this.platform.setAngularVelocity(0);
            }),
        ];
    }

    public step(settings: Settings, timeStep: number): void {
        // Drive the kinematic body.
        if (this.platform.getType() === BodyType.Kinematic) {
            const { p } = this.platform.getTransform();
            const v = this.platform.getLinearVelocity();

            if ((p.x < -10 && v.x < 0) || (p.x > 10 && v.x > 0)) {
                this.platform.setLinearVelocity(new Vec2(-v.x, v.y));
            }
        }

        super.step(settings, timeStep);
    }
}

registerTest("Examples", "Body Types", BodyTypesTest);
