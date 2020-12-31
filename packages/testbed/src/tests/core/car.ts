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
    Body,
    WheelJoint,
    BodyDef,
    EdgeShape,
    FixtureDef,
    Vec2,
    BodyType,
    PolygonShape,
    RevoluteJointDef,
    CircleShape,
    WheelJointDef,
    MakeArray,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { g_camera } from "../../utils/camera";
import { HotKey, hotKey } from "../../utils/hotkeys";

// This is a fun demo that shows off the wheel joint
class CarTest extends Test {
    public m_car: Body;

    public m_wheel1: Body;

    public m_wheel2: Body;

    public m_speed = 0;

    public m_spring1: WheelJoint;

    public m_spring2: WheelJoint;

    public constructor() {
        super();

        this.m_speed = 50;

        let ground: Body;
        {
            ground = this.m_world.CreateBody();

            const shape = new EdgeShape();

            const fd: FixtureDef = {
                shape,
                density: 0,
                friction: 0.6,
            };

            shape.SetTwoSided(new Vec2(-20, 0), new Vec2(20, 0));
            ground.CreateFixture(fd);

            const hs = [0.25, 1, 4, 0, 0, -1, -2, -2, -1.25, 0];

            let x = 20;
            let y1 = 0;
            const dx = 5;

            for (let i = 0; i < 10; ++i) {
                const y2 = hs[i];
                shape.SetTwoSided(new Vec2(x, y1), new Vec2(x + dx, y2));
                ground.CreateFixture(fd);
                y1 = y2;
                x += dx;
            }

            for (let i = 0; i < 10; ++i) {
                const y2 = hs[i];
                shape.SetTwoSided(new Vec2(x, y1), new Vec2(x + dx, y2));
                ground.CreateFixture(fd);
                y1 = y2;
                x += dx;
            }

            shape.SetTwoSided(new Vec2(x, 0), new Vec2(x + 40, 0));
            ground.CreateFixture(fd);

            x += 80;
            shape.SetTwoSided(new Vec2(x, 0), new Vec2(x + 40, 0));
            ground.CreateFixture(fd);

            x += 40;
            shape.SetTwoSided(new Vec2(x, 0), new Vec2(x + 10, 5));
            ground.CreateFixture(fd);

            x += 20;
            shape.SetTwoSided(new Vec2(x, 0), new Vec2(x + 40, 0));
            ground.CreateFixture(fd);

            x += 40;
            shape.SetTwoSided(new Vec2(x, 0), new Vec2(x, 20));
            ground.CreateFixture(fd);
        }

        // Teeter
        {
            const body = this.m_world.CreateBody({
                type: BodyType.Dynamic,
                position: { x: 140, y: 1 },
            });

            const box = new PolygonShape();
            box.SetAsBox(10, 0.25);
            body.CreateFixture({ shape: box, density: 1 });

            const jd = new RevoluteJointDef();
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
            const shape = new PolygonShape();
            shape.SetAsBox(1, 0.125);

            const fd: FixtureDef = {
                shape,
                density: 1,
                friction: 0.6,
            };
            const jd = new RevoluteJointDef();

            let prevBody = ground;
            for (let i = 0; i < N; ++i) {
                const body = this.m_world.CreateBody({
                    type: BodyType.Dynamic,
                    position: { x: 161 + 2 * i, y: -0.125 },
                });
                body.CreateFixture(fd);

                const anchor = new Vec2(160 + 2 * i, -0.125);
                jd.Initialize(prevBody, body, anchor);
                this.m_world.CreateJoint(jd);

                prevBody = body;
            }

            const anchor = new Vec2(160 + 2 * N, -0.125);
            jd.Initialize(prevBody, ground, anchor);
            this.m_world.CreateJoint(jd);
        }

        // Boxes
        {
            const box = new PolygonShape();
            box.SetAsBox(0.5, 0.5);

            let body: Body;
            const position = new Vec2();
            const bd: BodyDef = {
                type: BodyType.Dynamic,
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
            const chassis = new PolygonShape();
            const vertices = MakeArray(8, Vec2);
            vertices[0].Set(-1.5, -0.5);
            vertices[1].Set(1.5, -0.5);
            vertices[2].Set(1.5, 0);
            vertices[3].Set(0, 0.9);
            vertices[4].Set(-1.15, 0.9);
            vertices[5].Set(-1.5, 0.2);
            chassis.Set(vertices, 6);

            const circle = new CircleShape();
            circle.m_radius = 0.4;

            const position = new Vec2();
            const bd: BodyDef = { type: BodyType.Dynamic, position };
            position.Set(0, 1);
            this.m_car = this.m_world.CreateBody(bd);
            this.m_car.CreateFixture({ shape: chassis, density: 1 });

            const fd: FixtureDef = {
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

            const jd = new WheelJointDef();
            const axis = new Vec2(0, 1);

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

registerTest("Examples", "Car", CarTest);
