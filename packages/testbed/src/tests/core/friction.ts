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

import { EdgeShape, Vec2, PolygonShape, FixtureDef, BodyType, XY } from "@box2d/core";

import { registerTest, Test } from "../../test";

class FrictionTest extends Test {
    public constructor() {
        super();

        {
            const ground = this.world.createBody();

            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-40, 0), new Vec2(40, 0));
            ground.createFixture({ shape });
        }

        {
            const shape = new PolygonShape();
            shape.setAsBox(13, 0.25);

            const ground = this.world.createBody({
                position: { x: -4, y: 22 },
                angle: -0.25,
            });
            ground.createFixture({ shape });
        }

        {
            const shape = new PolygonShape();
            shape.setAsBox(0.25, 1);

            const ground = this.world.createBody({
                position: { x: 10.5, y: 19 },
            });
            ground.createFixture({ shape });
        }

        {
            const shape = new PolygonShape();
            shape.setAsBox(13, 0.25);
            const ground = this.world.createBody({
                position: { x: 4, y: 14 },
                angle: 0.25,
            });
            ground.createFixture({ shape });
        }

        {
            const shape = new PolygonShape();
            shape.setAsBox(0.25, 1);

            const ground = this.world.createBody({
                position: { x: -10.5, y: 11 },
            });
            ground.createFixture({ shape });
        }

        {
            const shape = new PolygonShape();
            shape.setAsBox(13, 0.25);
            const ground = this.world.createBody({
                position: { x: -4, y: 6 },
                angle: -0.25,
            });
            ground.createFixture({ shape });
        }

        {
            const shape = new PolygonShape();
            shape.setAsBox(0.5, 0.5);

            const fd: FixtureDef = {
                shape,
                density: 25,
            };

            const friction = [0.75, 0.5, 0.35, 0.1, 0];

            for (let i = 0; i < 5; ++i) {
                const body = this.world.createBody({
                    type: BodyType.Dynamic,
                    position: { x: -15 + 4 * i, y: 28 },
                });

                fd.friction = friction[i];
                body.createFixture(fd);
            }
        }
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 10,
        };
    }
}

registerTest("Forces", "Friction", FrictionTest);
