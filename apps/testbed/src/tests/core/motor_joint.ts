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

import { b2MotorJoint, b2EdgeShape, b2Vec2, b2BodyType, b2PolygonShape, b2MotorJointDef, b2Color } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { HotKey, hotKeyPress } from "../../utils/hotkeys";

/**
 * This test shows how to use a motor joint. A motor joint
 * can be used to animate a dynamic body. With finite motor forces
 * the body can be blocked by collision with other bodies.
 */
class MotorJoint extends Test {
    public m_joint: b2MotorJoint;

    public m_time = 0;

    public m_go = false;

    public constructor() {
        super();

        let ground = null;

        {
            ground = this.m_world.CreateBody();

            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(-20, 0), new b2Vec2(20, 0));

            ground.CreateFixture({ shape });
        }

        // Define motorized body
        {
            const body = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: { x: 0, y: 8 },
            });

            const shape = new b2PolygonShape();
            shape.SetAsBox(2, 0.5);

            body.CreateFixture({
                shape,
                friction: 0.6,
                density: 2,
            });

            const mjd = new b2MotorJointDef();
            mjd.Initialize(ground, body);
            mjd.maxForce = 1000;
            mjd.maxTorque = 1000;
            this.m_joint = this.m_world.CreateJoint(mjd);
        }

        this.m_go = false;
        this.m_time = 0;
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("s", "Start/Stop", () => {
                this.m_go = !this.m_go;
            }),
        ];
    }

    public Step(settings: Settings, timeStep: number): void {
        if (this.m_go && settings.m_hertz > 0) {
            this.m_time += 1 / settings.m_hertz;
        }

        const linearOffset = new b2Vec2();
        linearOffset.x = 6 * Math.sin(2 * this.m_time);
        linearOffset.y = 8 + 4 * Math.sin(1 * this.m_time);

        const angularOffset = 4 * this.m_time;

        this.m_joint.SetLinearOffset(linearOffset);
        this.m_joint.SetAngularOffset(angularOffset);

        settings.m_debugDraw.DrawPoint(linearOffset, 4, new b2Color(0.9, 0.9, 0.9));

        super.Step(settings, timeStep);
    }
}

registerTest("Joints", "Motor Joint", MotorJoint);
