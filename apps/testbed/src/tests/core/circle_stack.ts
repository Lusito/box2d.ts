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

import { b2Body, b2EdgeShape, b2Vec2, b2CircleShape, b2BodyType } from "@box2d/core";

import { registerTest, Test } from "../../test";

class CircleStack extends Test {
    public static readonly e_count = 10;

    public m_bodies: b2Body[] = [];

    public constructor() {
        super();

        {
            const ground = this.m_world.CreateBody();

            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(-40, 0), new b2Vec2(40, 0));
            ground.CreateFixture({ shape });
        }

        {
            const shape = new b2CircleShape();
            shape.m_radius = 1;

            for (let i = 0; i < CircleStack.e_count; ++i) {
                this.m_bodies[i] = this.m_world.CreateBody({
                    type: b2BodyType.b2_dynamicBody,
                    position: { x: 0, y: 4 + 3 * i },
                });

                this.m_bodies[i].CreateFixture({ shape, density: 1 });

                this.m_bodies[i].SetLinearVelocity(new b2Vec2(0, -50));
            }
        }
    }
}

registerTest("Stacking", "Circle Stack", CircleStack);
