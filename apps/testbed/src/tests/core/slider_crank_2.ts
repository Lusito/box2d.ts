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
    b2RevoluteJoint,
    b2PrismaticJoint,
    b2EdgeShape,
    b2Vec2,
    b2PolygonShape,
    b2BodyType,
    b2RevoluteJointDef,
    b2PrismaticJointDef,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { HotKey, hotKeyPress } from "../../utils/hotkeys";

// A motor driven slider crank with joint friction.
class SliderCrank2 extends Test {
    public m_joint1: b2RevoluteJoint;

    public m_joint2: b2PrismaticJoint;

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
            let prevBody = ground;

            // Define crank.
            {
                const shape = new b2PolygonShape();
                shape.SetAsBox(0.5, 2);

                const body = this.m_world.CreateBody({
                    type: b2BodyType.b2_dynamicBody,
                    position: { x: 0, y: 7 },
                });
                body.CreateFixture({ shape, density: 2 });

                const rjd = new b2RevoluteJointDef();
                rjd.Initialize(prevBody, body, new b2Vec2(0, 5));
                rjd.motorSpeed = 1 * Math.PI;
                rjd.maxMotorTorque = 10000;
                rjd.enableMotor = true;
                this.m_joint1 = this.m_world.CreateJoint(rjd);

                prevBody = body;
            }

            // Define follower.
            {
                const shape = new b2PolygonShape();
                shape.SetAsBox(0.5, 4);

                const body = this.m_world.CreateBody({
                    type: b2BodyType.b2_dynamicBody,
                    position: { x: 0, y: 13 },
                });
                body.CreateFixture({ shape, density: 2 });

                const rjd = new b2RevoluteJointDef();
                rjd.Initialize(prevBody, body, new b2Vec2(0, 9));
                rjd.enableMotor = false;
                this.m_world.CreateJoint(rjd);

                prevBody = body;
            }

            // Define piston
            {
                const shape = new b2PolygonShape();
                shape.SetAsBox(1.5, 1.5);

                const body = this.m_world.CreateBody({
                    type: b2BodyType.b2_dynamicBody,
                    fixedRotation: true,
                    position: { x: 0, y: 17 },
                });
                body.CreateFixture({ shape, density: 2 });

                const rjd = new b2RevoluteJointDef();
                rjd.Initialize(prevBody, body, new b2Vec2(0, 17));
                this.m_world.CreateJoint(rjd);

                const pjd = new b2PrismaticJointDef();
                pjd.Initialize(ground, body, new b2Vec2(0, 17), new b2Vec2(0, 1));

                pjd.maxMotorForce = 1000;
                pjd.enableMotor = true;

                this.m_joint2 = this.m_world.CreateJoint(pjd);
            }

            // Create a payload
            {
                const shape = new b2PolygonShape();
                shape.SetAsBox(1.5, 1.5);

                const body = this.m_world.CreateBody({
                    type: b2BodyType.b2_dynamicBody,
                    position: { x: 0, y: 23 },
                });
                body.CreateFixture({ shape, density: 2 });
            }
        }
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("f", "Toggle Friction", () => {
                this.m_joint2.EnableMotor(!this.m_joint2.IsMotorEnabled());
                this.m_joint2.GetBodyB().SetAwake(true);
            }),
            hotKeyPress("m", "Toggle Motor", () => {
                this.m_joint1.EnableMotor(!this.m_joint1.IsMotorEnabled());
                this.m_joint1.GetBodyB().SetAwake(true);
            }),
        ];
    }

    public Step(settings: Settings, timeStep: number): void {
        super.Step(settings, timeStep);
        const torque = this.m_joint1.GetMotorTorque(settings.m_hertz);
        this.addDebug("Motor Torque", torque.toFixed(0));
    }
}

registerTest("Examples", "Slider Crank 2", SliderCrank2);
