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
    b2BodyDef,
    b2Vec2,
    b2ChainShape,
    b2FixtureDef,
    b2RevoluteJointDef,
    b2BodyType,
    b2CircleShape,
    b2PolygonShape,
    b2DegToRad,
    XY,
} from "@box2d/core";

import { registerTest, Test } from "../../test";

class Ragdolls extends Test {
    public constructor() {
        super();

        {
            const ground = this.m_world.CreateBody();

            const shape = new b2ChainShape();
            shape.CreateLoop([new b2Vec2(-30, 0), new b2Vec2(-30, 40), new b2Vec2(30, 40), new b2Vec2(30, 0)]);
            ground.CreateFixture({ shape });
        }

        const position = new b2Vec2();
        const bd: b2BodyDef = {
            position,
        };
        const jd = new b2RevoluteJointDef();

        // Add 2 ragdolls along the top
        for (let i = 0; i < 2; ++i) {
            const startX = -20 + Math.random() * 2 + 40 * i;
            const startY = 30 + Math.random() * 5;

            // BODIES
            // Set these to dynamic bodies
            bd.type = b2BodyType.b2_dynamicBody;

            // Head
            const fd: b2FixtureDef = {
                shape: new b2CircleShape(1.25),
                density: 1,
                friction: 0.4,
                restitution: 0.3,
            };
            position.Set(startX, startY);
            const head = this.m_world.CreateBody(bd);
            head.CreateFixture(fd);
            // if (i === 0)
            // {
            head.ApplyLinearImpulse(
                new b2Vec2(Math.random() * 1000 - 500, Math.random() * 1000 - 500),
                head.GetWorldCenter(),
            );
            // }

            // Torso1
            const shape = new b2PolygonShape();
            fd.shape = shape;
            shape.SetAsBox(1.5, 1);
            fd.density = 1;
            fd.friction = 0.4;
            fd.restitution = 0.1;
            position.Set(startX, startY - 2.8);
            const torso1 = this.m_world.CreateBody(bd);
            torso1.CreateFixture(fd);
            // Torso2
            shape.SetAsBox(1.5, 1);
            position.Set(startX, startY - 4.3);
            const torso2 = this.m_world.CreateBody(bd);
            torso2.CreateFixture(fd);
            // Torso3
            shape.SetAsBox(1.5, 1);
            position.Set(startX, startY - 5.8);
            const torso3 = this.m_world.CreateBody(bd);
            torso3.CreateFixture(fd);

            // UpperArm
            fd.density = 1;
            fd.friction = 0.4;
            fd.restitution = 0.1;
            // L
            shape.SetAsBox(1.8, 0.65);
            position.Set(startX - 3, startY - 2);
            const upperArmL = this.m_world.CreateBody(bd);
            upperArmL.CreateFixture(fd);
            // R
            shape.SetAsBox(1.8, 0.65);
            position.Set(startX + 3, startY - 2);
            const upperArmR = this.m_world.CreateBody(bd);
            upperArmR.CreateFixture(fd);

            // LowerArm
            fd.density = 1;
            fd.friction = 0.4;
            fd.restitution = 0.1;
            // L
            shape.SetAsBox(1.7, 0.6);
            position.Set(startX - 5.7, startY - 2);
            const lowerArmL = this.m_world.CreateBody(bd);
            lowerArmL.CreateFixture(fd);
            // R
            shape.SetAsBox(1.7, 0.6);
            position.Set(startX + 5.7, startY - 2);
            const lowerArmR = this.m_world.CreateBody(bd);
            lowerArmR.CreateFixture(fd);

            // UpperLeg
            fd.density = 1;
            fd.friction = 0.4;
            fd.restitution = 0.1;
            // L
            shape.SetAsBox(0.75, 2.2);
            position.Set(startX - 0.8, startY - 8.5);
            const upperLegL = this.m_world.CreateBody(bd);
            upperLegL.CreateFixture(fd);
            // R
            shape.SetAsBox(0.75, 2.2);
            position.Set(startX + 0.8, startY - 8.5);
            const upperLegR = this.m_world.CreateBody(bd);
            upperLegR.CreateFixture(fd);

            // LowerLeg
            fd.density = 1;
            fd.friction = 0.4;
            fd.restitution = 0.1;
            // L
            shape.SetAsBox(0.6, 2);
            position.Set(startX - 0.8, startY - 12);
            const lowerLegL = this.m_world.CreateBody(bd);
            lowerLegL.CreateFixture(fd);
            // R
            shape.SetAsBox(0.6, 2);
            position.Set(startX + 0.8, startY - 12);
            const lowerLegR = this.m_world.CreateBody(bd);
            lowerLegR.CreateFixture(fd);

            // JOINTS
            jd.enableLimit = true;

            // Head to shoulders
            jd.lowerAngle = b2DegToRad(-40);
            jd.upperAngle = b2DegToRad(40);
            jd.Initialize(torso1, head, new b2Vec2(startX, startY - 1.5));
            this.m_world.CreateJoint(jd);

            // Upper arm to shoulders
            // L
            jd.lowerAngle = b2DegToRad(-85);
            jd.upperAngle = b2DegToRad(130);
            jd.Initialize(torso1, upperArmL, new b2Vec2(startX - 1.8, startY - 2));
            this.m_world.CreateJoint(jd);
            // R
            jd.lowerAngle = b2DegToRad(-130);
            jd.upperAngle = b2DegToRad(85);
            jd.Initialize(torso1, upperArmR, new b2Vec2(startX + 1.8, startY - 2));
            this.m_world.CreateJoint(jd);

            // Lower arm to upper arm
            // L
            jd.lowerAngle = b2DegToRad(-130);
            jd.upperAngle = b2DegToRad(10);
            jd.Initialize(upperArmL, lowerArmL, new b2Vec2(startX - 4.5, startY - 2));
            this.m_world.CreateJoint(jd);
            // R
            jd.lowerAngle = b2DegToRad(-10);
            jd.upperAngle = b2DegToRad(130);
            jd.Initialize(upperArmR, lowerArmR, new b2Vec2(startX + 4.5, startY - 2));
            this.m_world.CreateJoint(jd);

            // Shoulders/stomach
            jd.lowerAngle = b2DegToRad(-15);
            jd.upperAngle = b2DegToRad(15);
            jd.Initialize(torso1, torso2, new b2Vec2(startX, startY - 3.5));
            this.m_world.CreateJoint(jd);
            // Stomach/hips
            jd.Initialize(torso2, torso3, new b2Vec2(startX, startY - 5));
            this.m_world.CreateJoint(jd);

            // Torso to upper leg
            // L
            jd.lowerAngle = b2DegToRad(-25);
            jd.upperAngle = b2DegToRad(45);
            jd.Initialize(torso3, upperLegL, new b2Vec2(startX - 0.8, startY - 7.2));
            this.m_world.CreateJoint(jd);
            // R
            jd.lowerAngle = b2DegToRad(-45);
            jd.upperAngle = b2DegToRad(25);
            jd.Initialize(torso3, upperLegR, new b2Vec2(startX + 0.8, startY - 7.2));
            this.m_world.CreateJoint(jd);

            // Upper leg to lower leg
            // L
            jd.lowerAngle = b2DegToRad(-25);
            jd.upperAngle = b2DegToRad(115);
            jd.Initialize(upperLegL, lowerLegL, new b2Vec2(startX - 0.8, startY - 10.5));
            this.m_world.CreateJoint(jd);
            // R
            jd.lowerAngle = b2DegToRad(-115);
            jd.upperAngle = b2DegToRad(25);
            jd.Initialize(upperLegR, lowerLegR, new b2Vec2(startX + 0.8, startY - 10.5));
            this.m_world.CreateJoint(jd);
        }

        // these are static bodies so set the type accordingly
        bd.type = b2BodyType.b2_staticBody;
        const shape = new b2PolygonShape();
        const fd: b2FixtureDef = {
            shape,
            density: 0,
            friction: 0.4,
            restitution: 0.3,
        };

        // Add stairs on the left
        for (let j = 1; j <= 10; ++j) {
            shape.SetAsBox(1 * j, 1);
            position.Set(1 * j - 30, 21 - 2 * j);
            this.m_world.CreateBody(bd).CreateFixture(fd);
        }

        // Add stairs on the right
        for (let k = 1; k <= 10; ++k) {
            shape.SetAsBox(1 * k, 1);
            position.Set(30 - 1 * k, 21 - 2 * k);
            this.m_world.CreateBody(bd).CreateFixture(fd);
        }

        shape.SetAsBox(3, 4);
        position.Set(0, 4);
        this.m_world.CreateBody(bd).CreateFixture(fd);
    }

    public GetDefaultViewZoom() {
        return 15;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 15,
        };
    }
}

registerTest("Examples", "Ragdolls", Ragdolls);
