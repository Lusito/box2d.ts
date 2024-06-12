/*
 * Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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
    b2Body,
    b2WheelJoint,
    b2BodyDef,
    b2EdgeShape,
    b2FixtureDef,
    b2Vec2,
    b2BodyType,
    b2PolygonShape,
    b2RevoluteJointDef,
    b2CircleShape,
    b2WheelJointDef,
    b2MakeArray,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { g_camera } from "../../utils/camera";
import { HotKey, hotKey } from "../../utils/hotkeys";

// This is a fun demo that shows off the wheel joint
class Car extends Test {
    public m_car: b2Body;

    public m_wheel1: b2Body;

    public m_wheel2: b2Body;

    public m_speed = 0;

    public m_spring1: b2WheelJoint;

    public m_spring2: b2WheelJoint;

    public constructor() {
        super();

        this.m_speed = 50;

        let ground: b2Body;
        {
            ground = this.m_world.CreateBody();

            const shape = new b2EdgeShape();

            const fd: b2FixtureDef = {
                shape,
                density: 0,
                friction: 0.6,
            };

            shape.SetTwoSided(new b2Vec2(-20, 0), new b2Vec2(20, 0));
            ground.CreateFixture(fd);

            const hs = [0.25, 1, 4, 0, 0, -1, -2, -2, -1.25, 0];

            let x = 20;
            let y1 = 0;
            const dx = 5;

            for (let i = 0; i < 10; ++i) {
                const y2 = hs[i];
                shape.SetTwoSided(new b2Vec2(x, y1), new b2Vec2(x + dx, y2));
                ground.CreateFixture(fd);
                y1 = y2;
                x += dx;
            }

            for (let i = 0; i < 10; ++i) {
                const y2 = hs[i];
                shape.SetTwoSided(new b2Vec2(x, y1), new b2Vec2(x + dx, y2));
                ground.CreateFixture(fd);
                y1 = y2;
                x += dx;
            }

            shape.SetTwoSided(new b2Vec2(x, 0), new b2Vec2(x + 40, 0));
            ground.CreateFixture(fd);

            x += 80;
            shape.SetTwoSided(new b2Vec2(x, 0), new b2Vec2(x + 40, 0));
            ground.CreateFixture(fd);

            x += 40;
            shape.SetTwoSided(new b2Vec2(x, 0), new b2Vec2(x + 10, 5));
            ground.CreateFixture(fd);

            x += 20;
            shape.SetTwoSided(new b2Vec2(x, 0), new b2Vec2(x + 40, 0));
            ground.CreateFixture(fd);

            x += 40;
            shape.SetTwoSided(new b2Vec2(x, 0), new b2Vec2(x, 20));
            ground.CreateFixture(fd);
        }

        // Teeter
        {
            const body = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: { x: 140, y: 1 },
            });

            const box = new b2PolygonShape();
            box.SetAsBox(10, 0.25);
            body.CreateFixture({ shape: box, density: 1 });

            const jd = new b2RevoluteJointDef();
            jd.Initialize(ground, body, body.GetPosition());
            jd.lowerAngle = (-8 * Math.PI) / 180;
            jd.upperAngle = (8 * Math.PI) / 180;
            jd.enableLimit = true;
            this.m_world.CreateJoint(jd);

            body.ApplyAngularImpulse(100);
        }

        // Bridge
        {
            const N = 20;
            const shape = new b2PolygonShape();
            shape.SetAsBox(1, 0.125);

            const fd: b2FixtureDef = {
                shape,
                density: 1,
                friction: 0.6,
            };
            const jd = new b2RevoluteJointDef();

            let prevBody = ground;
            for (let i = 0; i < N; ++i) {
                const body = this.m_world.CreateBody({
                    type: b2BodyType.b2_dynamicBody,
                    position: { x: 161 + 2 * i, y: -0.125 },
                });
                body.CreateFixture(fd);

                const anchor = new b2Vec2(160 + 2 * i, -0.125);
                jd.Initialize(prevBody, body, anchor);
                this.m_world.CreateJoint(jd);

                prevBody = body;
            }

            const anchor = new b2Vec2(160 + 2 * N, -0.125);
            jd.Initialize(prevBody, ground, anchor);
            this.m_world.CreateJoint(jd);
        }

        // Boxes
        {
            const box = new b2PolygonShape();
            box.SetAsBox(0.5, 0.5);

            let body: b2Body;
            const position = new b2Vec2();
            const bd: b2BodyDef = {
                type: b2BodyType.b2_dynamicBody,
                position,
            };

            position.Set(230, 0.5);
            body = this.m_world.CreateBody(bd);
            body.CreateFixture({ shape: box, density: 0.5 });

            position.Set(230, 1.5);
            body = this.m_world.CreateBody(bd);
            body.CreateFixture({ shape: box, density: 0.5 });

            position.Set(230, 2.5);
            body = this.m_world.CreateBody(bd);
            body.CreateFixture({ shape: box, density: 0.5 });

            position.Set(230, 3.5);
            body = this.m_world.CreateBody(bd);
            body.CreateFixture({ shape: box, density: 0.5 });

            position.Set(230, 4.5);
            body = this.m_world.CreateBody(bd);
            body.CreateFixture({ shape: box, density: 0.5 });
        }

        // Car
        {
            const chassis = new b2PolygonShape();
            const vertices = b2MakeArray(8, b2Vec2);
            vertices[0].Set(-1.5, -0.5);
            vertices[1].Set(1.5, -0.5);
            vertices[2].Set(1.5, 0);
            vertices[3].Set(0, 0.9);
            vertices[4].Set(-1.15, 0.9);
            vertices[5].Set(-1.5, 0.2);
            chassis.Set(vertices, 6);

            const circle = new b2CircleShape();
            circle.m_radius = 0.4;

            const position = new b2Vec2();
            const bd: b2BodyDef = { type: b2BodyType.b2_dynamicBody, position };
            position.Set(0, 1);
            this.m_car = this.m_world.CreateBody(bd);
            this.m_car.CreateFixture({ shape: chassis, density: 1 });

            const fd: b2FixtureDef = {
                shape: circle,
                density: 1,
                friction: 0.9,
            };

            position.Set(-1, 0.35);
            this.m_wheel1 = this.m_world.CreateBody(bd);
            this.m_wheel1.CreateFixture(fd);

            position.Set(1, 0.4);
            this.m_wheel2 = this.m_world.CreateBody(bd);
            this.m_wheel2.CreateFixture(fd);

            const jd = new b2WheelJointDef();
            const axis = new b2Vec2(0, 1);

            const mass1 = this.m_wheel1.GetMass();
            const mass2 = this.m_wheel2.GetMass();

            const hertz = 4;
            const dampingRatio = 0.7;
            const omega = 2 * Math.PI * hertz;

            jd.Initialize(this.m_car, this.m_wheel1, this.m_wheel1.GetPosition(), axis);
            jd.motorSpeed = 0;
            jd.maxMotorTorque = 20;
            jd.enableMotor = true;
            jd.stiffness = mass1 * omega * omega;
            jd.damping = 2 * mass1 * dampingRatio * omega;
            jd.lowerTranslation = -0.25;
            jd.upperTranslation = 0.25;
            jd.enableLimit = true;
            this.m_spring1 = this.m_world.CreateJoint(jd);

            jd.Initialize(this.m_car, this.m_wheel2, this.m_wheel2.GetPosition(), axis);
            jd.motorSpeed = 0;
            jd.maxMotorTorque = 10;
            jd.enableMotor = false;
            jd.stiffness = mass2 * omega * omega;
            jd.damping = 2 * mass2 * dampingRatio * omega;
            jd.lowerTranslation = -0.25;
            jd.upperTranslation = 0.25;
            jd.enableLimit = true;
            this.m_spring2 = this.m_world.CreateJoint(jd);
        }
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKey("a", "Decelerate", (down) => this.m_spring1.SetMotorSpeed(down ? this.m_speed : 0)),
            hotKey("d", "Accelerate", (down) => this.m_spring1.SetMotorSpeed(down ? -this.m_speed : 0)),
        ];
    }

    public Step(settings: Settings, timeStep: number): void {
        const center = g_camera.getCenter();
        g_camera.setPosition(this.m_car.GetPosition().x, center.y);
        super.Step(settings, timeStep);
    }
}

registerTest("Examples", "Car", Car);
