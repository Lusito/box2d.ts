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
    RevoluteJoint,
    PrismaticJoint,
    GearJoint,
    EdgeShape,
    Vec2,
    CircleShape,
    PolygonShape,
    BodyType,
    RevoluteJointDef,
    GearJointDef,
    PrismaticJointDef,
} from "@box2d/core";

import { registerTest, Test } from "../../test";

class GearJointTest extends Test {
    public joint1: RevoluteJoint;

    public joint2: RevoluteJoint;

    public joint3: PrismaticJoint;

    public joint4: GearJoint;

    public joint5: GearJoint;

    public constructor() {
        super();

        let ground = null;
        {
            ground = this.world.createBody();

            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-50, 0), new Vec2(50, 0));
            ground.createFixture({ shape });
        }

        {
            const circle1 = new CircleShape();
            circle1.radius = 1;

            const box = new PolygonShape();
            box.setAsBox(0.5, 5);

            const circle2 = new CircleShape();
            circle2.radius = 2;

            const bdPosition1 = {
                x: 10,
                y: 9,
            };
            const body1 = this.world.createBody({
                type: BodyType.Static,
                position: bdPosition1,
            });
            body1.createFixture({ shape: circle1, density: 5 });

            const body2 = this.world.createBody({
                type: BodyType.Dynamic,
                position: { x: 10, y: 8 },
            });
            body2.createFixture({ shape: box, density: 5 });

            const bdPosition3 = { x: 10, y: 6 };
            const body3 = this.world.createBody({
                type: BodyType.Dynamic,
                position: bdPosition3,
            });
            body3.createFixture({ shape: circle2, density: 5 });

            const jd1 = new RevoluteJointDef();
            jd1.initialize(body2, body1, bdPosition1);
            const joint1 = this.world.createJoint(jd1);

            const jd2 = new RevoluteJointDef();
            jd2.initialize(body2, body3, bdPosition3);
            const joint2 = this.world.createJoint(jd2);

            const jd4 = new GearJointDef();
            jd4.bodyA = body1;
            jd4.bodyB = body3;
            jd4.joint1 = joint1;
            jd4.joint2 = joint2;
            jd4.ratio = circle2.radius / circle1.radius;
            this.world.createJoint(jd4);
        }

        {
            const circle1 = new CircleShape();
            circle1.radius = 1;

            const circle2 = new CircleShape();
            circle2.radius = 2;

            const box = new PolygonShape();
            box.setAsBox(0.5, 5);

            const bdPosition1 = { x: -3, y: 12 };
            const body1 = this.world.createBody({
                type: BodyType.Dynamic,
                position: bdPosition1,
            });
            body1.createFixture({ shape: circle1, density: 5 });

            const jd1 = new RevoluteJointDef();
            jd1.bodyA = ground;
            jd1.bodyB = body1;
            ground.getLocalPoint(bdPosition1, jd1.localAnchorA);
            body1.getLocalPoint(bdPosition1, jd1.localAnchorB);
            jd1.referenceAngle = body1.getAngle() - ground.getAngle();
            this.joint1 = this.world.createJoint(jd1);

            const bdPosition2 = { x: 0, y: 12 };
            const body2 = this.world.createBody({
                type: BodyType.Dynamic,
                position: bdPosition2,
            });
            body2.createFixture({ shape: circle2, density: 5 });

            const jd2 = new RevoluteJointDef();
            jd2.initialize(ground, body2, bdPosition2);
            this.joint2 = this.world.createJoint(jd2);

            const bdPosition3 = { x: 2.5, y: 12 };
            const body3 = this.world.createBody({
                type: BodyType.Dynamic,
                position: bdPosition3,
            });
            body3.createFixture({ shape: box, density: 5 });

            const jd3 = new PrismaticJointDef();
            jd3.initialize(ground, body3, bdPosition3, new Vec2(0, 1));
            jd3.lowerTranslation = -5;
            jd3.upperTranslation = 5;
            jd3.enableLimit = true;

            this.joint3 = this.world.createJoint(jd3);

            const jd4 = new GearJointDef();
            jd4.bodyA = body1;
            jd4.bodyB = body2;
            jd4.joint1 = this.joint1;
            jd4.joint2 = this.joint2;
            jd4.ratio = circle2.radius / circle1.radius;
            this.joint4 = this.world.createJoint(jd4);

            const jd5 = new GearJointDef();
            jd5.bodyA = body2;
            jd5.bodyB = body3;
            jd5.joint1 = this.joint2;
            jd5.joint2 = this.joint3;
            jd5.ratio = -1 / circle2.radius;
            this.joint5 = this.world.createJoint(jd5);
        }
    }
}

registerTest("Joints", "Gear", GearJointTest);
