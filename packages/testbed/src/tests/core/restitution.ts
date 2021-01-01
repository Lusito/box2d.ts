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

import { EdgeShape, Vec2, CircleShape, FixtureDef, BodyType } from "@box2d/core";

import { registerTest, Test } from "../../test";

// Note: even with a restitution of 1, there is some energy change
// due to position correction.
class RestitutionTest extends Test {
    public constructor() {
        super();

        const restitutionThreshold = 10;
        {
            const ground = this.m_world.createBody();

            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-40, 0), new Vec2(40, 0));
            ground.createFixture({ shape, restitutionThreshold });
        }

        {
            const shape = new CircleShape();
            shape.m_radius = 1;

            const fd: FixtureDef = {
                shape,
                density: 1,
                restitutionThreshold,
            };

            const restitution = [0, 0.1, 0.3, 0.5, 0.75, 0.9, 1];

            for (let i = 0; i < 7; ++i) {
                const body = this.m_world.createBody({
                    type: BodyType.Dynamic,
                    position: { x: -10 + 3 * i, y: 20 },
                });

                fd.restitution = restitution[i];
                body.createFixture(fd);
            }
        }
    }
}

registerTest("Forces", "Restitution", RestitutionTest);
