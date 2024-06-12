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

import { b2EdgeShape, b2Vec2, b2CircleShape, b2FixtureDef, b2BodyType } from "@box2d/core";

import { registerTest, Test } from "../../test";

// Note: even with a restitution of 1, there is some energy change
// due to position correction.
class Restitution extends Test {
    public constructor() {
        super();

        const restitutionThreshold = 10;
        {
            const ground = this.m_world.CreateBody();

            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(-40, 0), new b2Vec2(40, 0));
            ground.CreateFixture({ shape, restitutionThreshold });
        }

        {
            const shape = new b2CircleShape();
            shape.m_radius = 1;

            const fd: b2FixtureDef = {
                shape,
                density: 1,
                restitutionThreshold,
            };

            const restitution = [0, 0.1, 0.3, 0.5, 0.75, 0.9, 1];

            for (let i = 0; i < 7; ++i) {
                const body = this.m_world.CreateBody({
                    type: b2BodyType.b2_dynamicBody,
                    position: { x: -10 + 3 * i, y: 20 },
                });

                fd.restitution = restitution[i];
                body.CreateFixture(fd);
            }
        }
    }
}

registerTest("Forces", "Restitution", Restitution);
