/*
 * Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

import { PolygonShape, BodyType, RevoluteJointDef, Vec2, PrismaticJointDef, XY } from "@box2d/core";

import { registerTest, Test } from "../../test";

// A basic slider crank created for GDC tutorial: Understanding Constraints
class SliderCrank1Test extends Test {
    public constructor() {
        super();

        const ground = this.world.createBody({
            position: {
                x: 0,
                y: 17,
            },
        });

        {
            let prevBody = ground;

            // Define crank.
            {
                const shape = new PolygonShape();
                shape.setAsBox(4, 1);

                const body = this.world.createBody({
                    type: BodyType.Dynamic,
                    position: {
                        x: -8,
                        y: 20,
                    },
                });
                body.createFixture({ shape, density: 2 });

                const rjd = new RevoluteJointDef();
                rjd.initialize(prevBody, body, new Vec2(-12, 20));
                this.world.createJoint(rjd);

                prevBody = body;
            }

            // Define connecting rod
            {
                const shape = new PolygonShape();
                shape.setAsBox(8, 1);

                const body = this.world.createBody({
                    type: BodyType.Dynamic,
                    position: {
                        x: 4,
                        y: 20,
                    },
                });
                body.createFixture({ shape, density: 2 });

                const rjd = new RevoluteJointDef();
                rjd.initialize(prevBody, body, new Vec2(-4, 20));
                this.world.createJoint(rjd);

                prevBody = body;
            }

            // Define piston
            {
                const shape = new PolygonShape();
                shape.setAsBox(3, 3);

                const body = this.world.createBody({
                    type: BodyType.Dynamic,
                    fixedRotation: true,
                    position: {
                        x: 12,
                        y: 20,
                    },
                });
                body.createFixture({ shape, density: 2 });

                const rjd = new RevoluteJointDef();
                rjd.initialize(prevBody, body, new Vec2(12, 20));
                this.world.createJoint(rjd);

                const pjd = new PrismaticJointDef();
                pjd.initialize(ground, body, new Vec2(12, 17), new Vec2(1, 0));
                this.world.createJoint(pjd);
            }
        }
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 15,
        };
    }
}

registerTest("Examples", "Slider Crank 1", SliderCrank1Test);
