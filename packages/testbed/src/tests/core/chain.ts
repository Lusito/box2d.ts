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

import { EdgeShape, Vec2, PolygonShape, FixtureDef, RevoluteJointDef, BodyType } from "@box2d/core";

import { registerTest, Test } from "../../test";

const TEST_BAD_BODY = false;

class ChainTest extends Test {
    public constructor() {
        super();

        let ground = null;

        {
            ground = this.world.createBody();

            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-40, 0), new Vec2(40, 0));
            ground.createFixture({ shape });
        }

        {
            const shape = new PolygonShape();
            shape.setAsBox(0.6, 0.125);

            const fd: FixtureDef = {
                shape,
                density: 20,
                friction: 0.2,
            };

            const jd = new RevoluteJointDef();
            jd.collideConnected = false;

            const y = 25;
            let prevBody = ground;
            for (let i = 0; i < 30; ++i) {
                const body = this.world.createBody({
                    type: BodyType.Dynamic,
                    position: { x: 0.5 + i, y },
                });

                if (TEST_BAD_BODY) {
                    if (i === 10) {
                        // Test zero density dynamic body
                        fd.density = 0;
                    } else {
                        fd.density = 20;
                    }
                }

                body.createFixture(fd);

                const anchor = new Vec2(i, y);
                jd.initialize(prevBody, body, anchor);
                this.world.createJoint(jd);

                prevBody = body;
            }
        }
    }
}

registerTest("Joints", "Chain", ChainTest);
