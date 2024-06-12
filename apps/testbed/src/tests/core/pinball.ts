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
    b2Body,
    b2Vec2,
    b2ChainShape,
    b2FixtureDef,
    b2BodyType,
    b2PolygonShape,
    b2RevoluteJointDef,
    b2CircleShape,
    XY,
    b2MakeArray,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { hotKey, HotKey } from "../../utils/hotkeys";

/**
 * This tests bullet collision and provides an example of a
 * gameplay scenario. This also uses a loop shape.
 */

class Pinball extends Test {
    public m_leftJoint: b2RevoluteJoint;

    public m_rightJoint: b2RevoluteJoint;

    public m_ball: b2Body;

    public constructor() {
        super();

        // Ground body

        let ground = null;
        {
            ground = this.m_world.CreateBody();

            const vs = b2MakeArray(5, b2Vec2);
            vs[0].Set(-8, 6);
            vs[1].Set(-8, 20);
            vs[2].Set(8, 20);
            vs[3].Set(8, 6);
            vs[4].Set(0, -2);

            const loop = new b2ChainShape();
            loop.CreateLoop(vs, 5);
            ground.CreateFixture({
                shape: loop,
                density: 0,
            });
        }

        // Flippers
        {
            const p1 = new b2Vec2(-2, 0);
            const p2 = new b2Vec2(2, 0);

            const leftFlipper = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: p1,
            });

            const rightFlipper = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: p2,
            });

            const box = new b2PolygonShape();
            box.SetAsBox(1.75, 0.1);

            const fd: b2FixtureDef = {
                shape: box,
                density: 1,
            };
            leftFlipper.CreateFixture(fd);
            rightFlipper.CreateFixture(fd);

            const jd = new b2RevoluteJointDef();
            jd.bodyA = ground;
            jd.localAnchorB.SetZero();
            jd.enableMotor = true;
            jd.maxMotorTorque = 1000;
            jd.enableLimit = true;

            jd.motorSpeed = -10;
            jd.localAnchorA.Copy(p1);
            jd.bodyB = leftFlipper;
            jd.lowerAngle = (-30 * Math.PI) / 180;
            jd.upperAngle = (5 * Math.PI) / 180;
            this.m_leftJoint = this.m_world.CreateJoint(jd);

            jd.motorSpeed = 10;
            jd.localAnchorA.Copy(p2);
            jd.bodyB = rightFlipper;
            jd.lowerAngle = (-5 * Math.PI) / 180;
            jd.upperAngle = (30 * Math.PI) / 180;
            this.m_rightJoint = this.m_world.CreateJoint(jd);
        }

        // Circle character
        {
            this.m_ball = this.m_world.CreateBody({
                position: { x: 1, y: 15 },
                type: b2BodyType.b2_dynamicBody,
                bullet: true,
            });

            const shape = new b2CircleShape();
            shape.m_radius = 0.2;

            const fd: b2FixtureDef = {
                shape,
                density: 1,
            };
            this.m_ball.CreateFixture(fd);
        }
    }

    public GetDefaultViewZoom() {
        return 40;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 5,
        };
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKey("a", "Hold Flipper", (down) => {
                if (down) {
                    this.m_leftJoint.SetMotorSpeed(20);
                    this.m_rightJoint.SetMotorSpeed(-20);
                } else {
                    this.m_leftJoint.SetMotorSpeed(-10);
                    this.m_rightJoint.SetMotorSpeed(10);
                }
            }),
        ];
    }
}

registerTest("Examples", "Pinball", Pinball);
