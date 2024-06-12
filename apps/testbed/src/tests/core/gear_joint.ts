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
    b2GearJoint,
    b2EdgeShape,
    b2Vec2,
    b2CircleShape,
    b2PolygonShape,
    b2BodyType,
    b2RevoluteJointDef,
    b2GearJointDef,
    b2PrismaticJointDef,
} from "@box2d/core";

import { registerTest, Test } from "../../test";

class GearJoint extends Test {
    public m_joint1: b2RevoluteJoint;

    public m_joint2: b2RevoluteJoint;

    public m_joint3: b2PrismaticJoint;

    public m_joint4: b2GearJoint;

    public m_joint5: b2GearJoint;

    public constructor() {
        super();

        let ground = null;
        {
            ground = this.m_world.CreateBody();

            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(-50, 0), new b2Vec2(50, 0));
            ground.CreateFixture({ shape });
        }

        {
            const circle1 = new b2CircleShape();
            circle1.m_radius = 1;

            const box = new b2PolygonShape();
            box.SetAsBox(0.5, 5);

            const circle2 = new b2CircleShape();
            circle2.m_radius = 2;

            const bdPosition1 = {
                x: 10,
                y: 9,
            };
            const body1 = this.m_world.CreateBody({
                type: b2BodyType.b2_staticBody,
                position: bdPosition1,
            });
            body1.CreateFixture({ shape: circle1, density: 5 });

            const body2 = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: { x: 10, y: 8 },
            });
            body2.CreateFixture({ shape: box, density: 5 });

            const bdPosition3 = { x: 10, y: 6 };
            const body3 = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: bdPosition3,
            });
            body3.CreateFixture({ shape: circle2, density: 5 });

            const jd1 = new b2RevoluteJointDef();
            jd1.Initialize(body2, body1, bdPosition1);
            const joint1 = this.m_world.CreateJoint(jd1);

            const jd2 = new b2RevoluteJointDef();
            jd2.Initialize(body2, body3, bdPosition3);
            const joint2 = this.m_world.CreateJoint(jd2);

            const jd4 = new b2GearJointDef();
            jd4.bodyA = body1;
            jd4.bodyB = body3;
            jd4.joint1 = joint1;
            jd4.joint2 = joint2;
            jd4.ratio = circle2.m_radius / circle1.m_radius;
            this.m_world.CreateJoint(jd4);
        }

        {
            const circle1 = new b2CircleShape();
            circle1.m_radius = 1;

            const circle2 = new b2CircleShape();
            circle2.m_radius = 2;

            const box = new b2PolygonShape();
            box.SetAsBox(0.5, 5);

            const bdPosition1 = { x: -3, y: 12 };
            const body1 = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: bdPosition1,
            });
            body1.CreateFixture({ shape: circle1, density: 5 });

            const jd1 = new b2RevoluteJointDef();
            jd1.bodyA = ground;
            jd1.bodyB = body1;
            ground.GetLocalPoint(bdPosition1, jd1.localAnchorA);
            body1.GetLocalPoint(bdPosition1, jd1.localAnchorB);
            jd1.referenceAngle = body1.GetAngle() - ground.GetAngle();
            this.m_joint1 = this.m_world.CreateJoint(jd1);

            const bdPosition2 = { x: 0, y: 12 };
            const body2 = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: bdPosition2,
            });
            body2.CreateFixture({ shape: circle2, density: 5 });

            const jd2 = new b2RevoluteJointDef();
            jd2.Initialize(ground, body2, bdPosition2);
            this.m_joint2 = this.m_world.CreateJoint(jd2);

            const bdPosition3 = { x: 2.5, y: 12 };
            const body3 = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: bdPosition3,
            });
            body3.CreateFixture({ shape: box, density: 5 });

            const jd3 = new b2PrismaticJointDef();
            jd3.Initialize(ground, body3, bdPosition3, new b2Vec2(0, 1));
            jd3.lowerTranslation = -5;
            jd3.upperTranslation = 5;
            jd3.enableLimit = true;

            this.m_joint3 = this.m_world.CreateJoint(jd3);

            const jd4 = new b2GearJointDef();
            jd4.bodyA = body1;
            jd4.bodyB = body2;
            jd4.joint1 = this.m_joint1;
            jd4.joint2 = this.m_joint2;
            jd4.ratio = circle2.m_radius / circle1.m_radius;
            this.m_joint4 = this.m_world.CreateJoint(jd4);

            const jd5 = new b2GearJointDef();
            jd5.bodyA = body2;
            jd5.bodyB = body3;
            jd5.joint1 = this.m_joint2;
            jd5.joint2 = this.m_joint3;
            jd5.ratio = -1 / circle2.m_radius;
            this.m_joint5 = this.m_world.CreateJoint(jd5);
        }
    }
}

registerTest("Joints", "Gear", GearJoint);
