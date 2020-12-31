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

import { EdgeShape, Vec2, PolygonShape, BodyType } from "@box2d/core";

import { registerTest, Test } from "../../test";

class PyramidTest extends Test {
    public static readonly e_count = 20;

    public constructor() {
        super();

        {
            const ground = this.m_world.CreateBody();

            const shape = new EdgeShape();
            shape.SetTwoSided(new Vec2(-40, 0), new Vec2(40, 0));
            ground.CreateFixture({ shape });
        }

        {
            const a = 0.5;
            const shape = new PolygonShape();
            shape.SetAsBox(a, a);

            const x = new Vec2(-7, 0.75);
            const y = new Vec2();
            const deltaX = new Vec2(0.5625, 1.25);
            const deltaY = new Vec2(1.125, 0);

            for (let i = 0; i < PyramidTest.e_count; ++i) {
                y.Copy(x);

                for (let j = i; j < PyramidTest.e_count; ++j) {
                    const body = this.m_world.CreateBody({
                        type: BodyType.Dynamic,
                        position: y,
                    });
                    body.CreateFixture({ shape, density: 5 });

                    y.Add(deltaY);
                }

                x.Add(deltaX);
            }
        }
    }
}

registerTest("Stacking", "Pyramid", PyramidTest);
