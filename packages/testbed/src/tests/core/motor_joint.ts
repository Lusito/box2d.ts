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

import { MotorJoint, EdgeShape, Vec2, BodyType, PolygonShape, MotorJointDef, Color } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { g_debugDraw } from "../../utils/draw";
import { HotKey, hotKeyPress } from "../../utils/hotkeys";

/**
 * This test shows how to use a motor joint. A motor joint
 * can be used to animate a dynamic body. With finite motor forces
 * the body can be blocked by collision with other bodies.
 */
class MotorJointTest extends Test {
    public m_joint: MotorJoint;

    public m_time = 0;

    public m_go = false;

    public constructor() {
        super();

        let ground = null;

        {
            ground = this.m_world.createBody();

            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-20, 0), new Vec2(20, 0));

            ground.createFixture({ shape });
        }

        // Define motorized body
        {
            const body = this.m_world.createBody({
                type: BodyType.Dynamic,
                position: { x: 0, y: 8 },
            });

            const shape = new PolygonShape();
            shape.setAsBox(2, 0.5);

            body.createFixture({
                shape,
                friction: 0.6,
                density: 2,
            });

            const mjd = new MotorJointDef();
            mjd.initialize(ground, body);
            mjd.maxForce = 1000;
            mjd.maxTorque = 1000;
            this.m_joint = this.m_world.createJoint(mjd);
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

    public step(settings: Settings, timeStep: number): void {
        if (this.m_go && settings.m_hertz > 0) {
            this.m_time += 1 / settings.m_hertz;
        }

        const linearOffset = new Vec2();
        linearOffset.x = 6 * Math.sin(2 * this.m_time);
        linearOffset.y = 8 + 4 * Math.sin(1 * this.m_time);

        const angularOffset = 4 * this.m_time;

        this.m_joint.setLinearOffset(linearOffset);
        this.m_joint.setAngularOffset(angularOffset);

        g_debugDraw.drawPoint(linearOffset, 4, new Color(0.9, 0.9, 0.9));

        super.step(settings, timeStep);
    }
}

registerTest("Joints", "Motor Joint", MotorJointTest);
