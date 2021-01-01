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

import { Body, EdgeShape, Vec2, CircleShape, BodyType } from "@box2d/core";

import { registerTest, Test } from "../../test";

class CircleStackTest extends Test {
    public static readonly e_count = 10;

    public m_bodies: Body[] = [];

    public constructor() {
        super();

        {
            const ground = this.m_world.createBody();

            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-40, 0), new Vec2(40, 0));
            ground.createFixture({ shape });
        }

        {
            const shape = new CircleShape();
            shape.m_radius = 1;

            for (let i = 0; i < CircleStackTest.e_count; ++i) {
                this.m_bodies[i] = this.m_world.createBody({
                    type: BodyType.Dynamic,
                    position: { x: 0, y: 4 + 3 * i },
                });

                this.m_bodies[i].createFixture({ shape, density: 1 });

                this.m_bodies[i].setLinearVelocity(new Vec2(0, -50));
            }
        }
    }
}

registerTest("Stacking", "Circle Stack", CircleStackTest);
