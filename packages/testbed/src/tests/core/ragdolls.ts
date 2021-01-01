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
    BodyDef,
    Vec2,
    ChainShape,
    FixtureDef,
    RevoluteJointDef,
    BodyType,
    CircleShape,
    PolygonShape,
    degToRad,
    XY,
} from "@box2d/core";

import { registerTest, Test } from "../../test";

class RagdollsTest extends Test {
    public constructor() {
        super();

        {
            const ground = this.m_world.createBody();

            const shape = new ChainShape();
            shape.createLoop([new Vec2(-30, 0), new Vec2(-30, 40), new Vec2(30, 40), new Vec2(30, 0)]);
            ground.createFixture({ shape });
        }

        const position = new Vec2();
        const bd: BodyDef = {
            position,
        };
        const jd = new RevoluteJointDef();

        // Add 2 ragdolls along the top
        for (let i = 0; i < 2; ++i) {
            const startX = -20 + Math.random() * 2 + 40 * i;
            const startY = 30 + Math.random() * 5;

            // BODIES
            // Set these to dynamic bodies
            bd.type = BodyType.Dynamic;

            // Head
            const fd: FixtureDef = {
                shape: new CircleShape(1.25),
                density: 1,
                friction: 0.4,
                restitution: 0.3,
            };
            position.set(startX, startY);
            const head = this.m_world.createBody(bd);
            head.createFixture(fd);
            // if (i === 0)
            // {
            head.applyLinearImpulse(
                new Vec2(Math.random() * 1000 - 500, Math.random() * 1000 - 500),
                head.getWorldCenter(),
            );
            // }

            // Torso1
            const shape = new PolygonShape();
            fd.shape = shape;
            shape.setAsBox(1.5, 1);
            fd.density = 1;
            fd.friction = 0.4;
            fd.restitution = 0.1;
            position.set(startX, startY - 2.8);
            const torso1 = this.m_world.createBody(bd);
            torso1.createFixture(fd);
            // Torso2
            shape.setAsBox(1.5, 1);
            position.set(startX, startY - 4.3);
            const torso2 = this.m_world.createBody(bd);
            torso2.createFixture(fd);
            // Torso3
            shape.setAsBox(1.5, 1);
            position.set(startX, startY - 5.8);
            const torso3 = this.m_world.createBody(bd);
            torso3.createFixture(fd);

            // UpperArm
            fd.density = 1;
            fd.friction = 0.4;
            fd.restitution = 0.1;
            // L
            shape.setAsBox(1.8, 0.65);
            position.set(startX - 3, startY - 2);
            const upperArmL = this.m_world.createBody(bd);
            upperArmL.createFixture(fd);
            // R
            shape.setAsBox(1.8, 0.65);
            position.set(startX + 3, startY - 2);
            const upperArmR = this.m_world.createBody(bd);
            upperArmR.createFixture(fd);

            // LowerArm
            fd.density = 1;
            fd.friction = 0.4;
            fd.restitution = 0.1;
            // L
            shape.setAsBox(1.7, 0.6);
            position.set(startX - 5.7, startY - 2);
            const lowerArmL = this.m_world.createBody(bd);
            lowerArmL.createFixture(fd);
            // R
            shape.setAsBox(1.7, 0.6);
            position.set(startX + 5.7, startY - 2);
            const lowerArmR = this.m_world.createBody(bd);
            lowerArmR.createFixture(fd);

            // UpperLeg
            fd.density = 1;
            fd.friction = 0.4;
            fd.restitution = 0.1;
            // L
            shape.setAsBox(0.75, 2.2);
            position.set(startX - 0.8, startY - 8.5);
            const upperLegL = this.m_world.createBody(bd);
            upperLegL.createFixture(fd);
            // R
            shape.setAsBox(0.75, 2.2);
            position.set(startX + 0.8, startY - 8.5);
            const upperLegR = this.m_world.createBody(bd);
            upperLegR.createFixture(fd);

            // LowerLeg
            fd.density = 1;
            fd.friction = 0.4;
            fd.restitution = 0.1;
            // L
            shape.setAsBox(0.6, 2);
            position.set(startX - 0.8, startY - 12);
            const lowerLegL = this.m_world.createBody(bd);
            lowerLegL.createFixture(fd);
            // R
            shape.setAsBox(0.6, 2);
            position.set(startX + 0.8, startY - 12);
            const lowerLegR = this.m_world.createBody(bd);
            lowerLegR.createFixture(fd);

            // JOINTS
            jd.enableLimit = true;

            // Head to shoulders
            jd.lowerAngle = degToRad(-40);
            jd.upperAngle = degToRad(40);
            jd.initialize(torso1, head, new Vec2(startX, startY - 1.5));
            this.m_world.createJoint(jd);

            // Upper arm to shoulders
            // L
            jd.lowerAngle = degToRad(-85);
            jd.upperAngle = degToRad(130);
            jd.initialize(torso1, upperArmL, new Vec2(startX - 1.8, startY - 2));
            this.m_world.createJoint(jd);
            // R
            jd.lowerAngle = degToRad(-130);
            jd.upperAngle = degToRad(85);
            jd.initialize(torso1, upperArmR, new Vec2(startX + 1.8, startY - 2));
            this.m_world.createJoint(jd);

            // Lower arm to upper arm
            // L
            jd.lowerAngle = degToRad(-130);
            jd.upperAngle = degToRad(10);
            jd.initialize(upperArmL, lowerArmL, new Vec2(startX - 4.5, startY - 2));
            this.m_world.createJoint(jd);
            // R
            jd.lowerAngle = degToRad(-10);
            jd.upperAngle = degToRad(130);
            jd.initialize(upperArmR, lowerArmR, new Vec2(startX + 4.5, startY - 2));
            this.m_world.createJoint(jd);

            // Shoulders/stomach
            jd.lowerAngle = degToRad(-15);
            jd.upperAngle = degToRad(15);
            jd.initialize(torso1, torso2, new Vec2(startX, startY - 3.5));
            this.m_world.createJoint(jd);
            // Stomach/hips
            jd.initialize(torso2, torso3, new Vec2(startX, startY - 5));
            this.m_world.createJoint(jd);

            // Torso to upper leg
            // L
            jd.lowerAngle = degToRad(-25);
            jd.upperAngle = degToRad(45);
            jd.initialize(torso3, upperLegL, new Vec2(startX - 0.8, startY - 7.2));
            this.m_world.createJoint(jd);
            // R
            jd.lowerAngle = degToRad(-45);
            jd.upperAngle = degToRad(25);
            jd.initialize(torso3, upperLegR, new Vec2(startX + 0.8, startY - 7.2));
            this.m_world.createJoint(jd);

            // Upper leg to lower leg
            // L
            jd.lowerAngle = degToRad(-25);
            jd.upperAngle = degToRad(115);
            jd.initialize(upperLegL, lowerLegL, new Vec2(startX - 0.8, startY - 10.5));
            this.m_world.createJoint(jd);
            // R
            jd.lowerAngle = degToRad(-115);
            jd.upperAngle = degToRad(25);
            jd.initialize(upperLegR, lowerLegR, new Vec2(startX + 0.8, startY - 10.5));
            this.m_world.createJoint(jd);
        }

        // these are static bodies so set the type accordingly
        bd.type = BodyType.Static;
        const shape = new PolygonShape();
        const fd: FixtureDef = {
            shape,
            density: 0,
            friction: 0.4,
            restitution: 0.3,
        };

        // Add stairs on the left
        for (let j = 1; j <= 10; ++j) {
            shape.setAsBox(1 * j, 1);
            position.set(1 * j - 30, 21 - 2 * j);
            this.m_world.createBody(bd).createFixture(fd);
        }

        // Add stairs on the right
        for (let k = 1; k <= 10; ++k) {
            shape.setAsBox(1 * k, 1);
            position.set(30 - 1 * k, 21 - 2 * k);
            this.m_world.createBody(bd).createFixture(fd);
        }

        shape.setAsBox(3, 4);
        position.set(0, 4);
        this.m_world.createBody(bd).createFixture(fd);
    }

    public getDefaultViewZoom() {
        return 15;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 15,
        };
    }
}

registerTest("Examples", "Ragdolls", RagdollsTest);
