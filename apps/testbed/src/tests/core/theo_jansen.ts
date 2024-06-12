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
    b2Vec2,
    b2Body,
    b2RevoluteJoint,
    b2PolygonShape,
    b2BodyType,
    b2DistanceJointDef,
    b2LinearStiffness,
    b2RevoluteJointDef,
    b2EdgeShape,
    b2CircleShape,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { HotKey, hotKeyPress } from "../../utils/hotkeys";

// Inspired by a contribution by roman_m
// Dimensions scooped from APE (http://www.cove.org/ape/index.htm)

class TheoJansen extends Test {
    public m_offset = new b2Vec2();

    public m_chassis!: b2Body;

    public m_wheel!: b2Body;

    public m_motorJoint!: b2RevoluteJoint;

    public m_motorOn = true;

    public m_motorSpeed = 2;

    public constructor() {
        super();

        this.m_offset.Set(0, 8);
        const pivot = new b2Vec2(0, 0.8);

        // Ground
        {
            const ground = this.m_world.CreateBody();

            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(-50, 0), new b2Vec2(50, 0));
            ground.CreateFixture({ shape });

            shape.SetTwoSided(new b2Vec2(-50, 0), new b2Vec2(-50, 10));
            ground.CreateFixture({ shape });

            shape.SetTwoSided(new b2Vec2(50, 0), new b2Vec2(50, 10));
            ground.CreateFixture({ shape });
        }

        // Balls
        for (let i = 0; i < 40; ++i) {
            const shape = new b2CircleShape();
            shape.m_radius = 0.25;

            const body = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: { x: -40 + 2 * i, y: 0.5 },
            });
            body.CreateFixture({ shape, density: 1 });
        }

        // Chassis
        {
            const shape = new b2PolygonShape();
            shape.SetAsBox(2.5, 1);

            this.m_chassis = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: {
                    x: pivot.x + this.m_offset.x,
                    y: pivot.y + this.m_offset.y,
                },
            });
            this.m_chassis.CreateFixture({
                density: 1,
                shape,
                filter: {
                    groupIndex: -1,
                },
            });
        }

        {
            const shape = new b2CircleShape();
            shape.m_radius = 1.6;

            this.m_wheel = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: {
                    x: pivot.x + this.m_offset.x,
                    y: pivot.y + this.m_offset.y,
                },
            });
            this.m_wheel.CreateFixture({
                density: 1,
                shape,
                filter: {
                    groupIndex: -1,
                },
            });
        }

        {
            const jd = new b2RevoluteJointDef();
            jd.Initialize(this.m_wheel, this.m_chassis, b2Vec2.Add(pivot, this.m_offset, new b2Vec2()));
            jd.collideConnected = false;
            jd.motorSpeed = this.m_motorSpeed;
            jd.maxMotorTorque = 400;
            jd.enableMotor = this.m_motorOn;
            this.m_motorJoint = this.m_world.CreateJoint(jd);
        }

        const wheelAnchor = b2Vec2.Add(pivot, new b2Vec2(0, -0.8), new b2Vec2());

        this.CreateLeg(-1, wheelAnchor);
        this.CreateLeg(1, wheelAnchor);

        this.m_wheel.SetTransformVec(this.m_wheel.GetPosition(), (120 * Math.PI) / 180);
        this.CreateLeg(-1, wheelAnchor);
        this.CreateLeg(1, wheelAnchor);

        this.m_wheel.SetTransformVec(this.m_wheel.GetPosition(), (-120 * Math.PI) / 180);
        this.CreateLeg(-1, wheelAnchor);
        this.CreateLeg(1, wheelAnchor);
    }

    public CreateLeg(s: number, wheelAnchor: b2Vec2) {
        const p1 = new b2Vec2(5.4 * s, -6.1);
        const p2 = new b2Vec2(7.2 * s, -1.2);
        const p3 = new b2Vec2(4.3 * s, -1.9);
        const p4 = new b2Vec2(3.1 * s, 0.8);
        const p5 = new b2Vec2(6 * s, 1.5);
        const p6 = new b2Vec2(2.5 * s, 3.7);

        const poly1 = new b2PolygonShape();
        const poly2 = new b2PolygonShape();

        if (s > 0) {
            const vertices = [];

            vertices[0] = p1;
            vertices[1] = p2;
            vertices[2] = p3;
            poly1.Set(vertices);

            vertices[0] = b2Vec2.ZERO;
            vertices[1] = b2Vec2.Subtract(p5, p4, new b2Vec2());
            vertices[2] = b2Vec2.Subtract(p6, p4, new b2Vec2());
            poly2.Set(vertices);
        } else {
            const vertices = [];

            vertices[0] = p1;
            vertices[1] = p3;
            vertices[2] = p2;
            poly1.Set(vertices);

            vertices[0] = b2Vec2.ZERO;
            vertices[1] = b2Vec2.Subtract(p6, p4, new b2Vec2());
            vertices[2] = b2Vec2.Subtract(p5, p4, new b2Vec2());
            poly2.Set(vertices);
        }

        const body1 = this.m_world.CreateBody({
            type: b2BodyType.b2_dynamicBody,
            position: this.m_offset,
            angularDamping: 10,
        });
        const body2 = this.m_world.CreateBody({
            type: b2BodyType.b2_dynamicBody,
            position: b2Vec2.Add(p4, this.m_offset, new b2Vec2()),
            angularDamping: 10,
        });

        body1.CreateFixture({
            filter: { groupIndex: -1 },
            shape: poly1,
            density: 1,
        });
        body2.CreateFixture({
            filter: { groupIndex: -1 },
            density: 1,
            shape: poly2,
        });

        {
            const jd = new b2DistanceJointDef();

            // Using a soft distance constraint can reduce some jitter.
            // It also makes the structure seem a bit more fluid by
            // acting like a suspension system.
            const dampingRatio = 0.5;
            const frequencyHz = 10;

            jd.Initialize(
                body1,
                body2,
                b2Vec2.Add(p2, this.m_offset, new b2Vec2()),
                b2Vec2.Add(p5, this.m_offset, new b2Vec2()),
            );
            b2LinearStiffness(jd, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
            this.m_world.CreateJoint(jd);

            jd.Initialize(
                body1,
                body2,
                b2Vec2.Add(p3, this.m_offset, new b2Vec2()),
                b2Vec2.Add(p4, this.m_offset, new b2Vec2()),
            );
            b2LinearStiffness(jd, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
            this.m_world.CreateJoint(jd);

            jd.Initialize(
                body1,
                this.m_wheel,
                b2Vec2.Add(p3, this.m_offset, new b2Vec2()),
                b2Vec2.Add(wheelAnchor, this.m_offset, new b2Vec2()),
            );
            b2LinearStiffness(jd, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
            this.m_world.CreateJoint(jd);

            jd.Initialize(
                body2,
                this.m_wheel,
                b2Vec2.Add(p6, this.m_offset, new b2Vec2()),
                b2Vec2.Add(wheelAnchor, this.m_offset, new b2Vec2()),
            );
            b2LinearStiffness(jd, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
            this.m_world.CreateJoint(jd);
        }

        {
            const jd = new b2RevoluteJointDef();

            jd.Initialize(body2, this.m_chassis, b2Vec2.Add(p4, this.m_offset, new b2Vec2()));
            this.m_world.CreateJoint(jd);
        }
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("a", "Left", () => this.m_motorJoint.SetMotorSpeed(-this.m_motorSpeed)),
            hotKeyPress("s", "Brake", () => this.m_motorJoint.SetMotorSpeed(0)),
            hotKeyPress("d", "Right", () => this.m_motorJoint.SetMotorSpeed(this.m_motorSpeed)),
            hotKeyPress("m", "Toggle Enabled", () =>
                this.m_motorJoint.EnableMotor(!this.m_motorJoint.IsMotorEnabled()),
            ),
        ];
    }
}

registerTest("Examples", "Theo Jansen's Walker", TheoJansen);
