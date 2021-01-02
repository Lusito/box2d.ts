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

import { Vec2, ChainShape, FixtureDef, BodyType, PolygonShape, CircleShape, XY } from "@box2d/core";

import { registerTest, Test } from "../../test";

class ContinousCollisionTest extends Test {
    public constructor() {
        super();

        {
            const ground = this.world.createBody();

            const shape = new ChainShape();
            shape.createLoop([new Vec2(-30, 0), new Vec2(-30, 40), new Vec2(30, 40), new Vec2(30, 0)]);
            ground.createFixture({ shape });
        }

        // Always on, even if default is off
        this.world.setContinuousPhysics(true);

        // Create 'basket'
        {
            const body = this.world.createBody({
                type: BodyType.Dynamic,
                bullet: true,
                position: { x: 15, y: 5 },
            });

            const sd_bottom = new PolygonShape();
            sd_bottom.setAsBox(4.5, 0.45);
            // These values are used for all the parts of the 'basket'
            const fd: FixtureDef = {
                density: 4,
                restitution: 1.4,
                shape: sd_bottom,
            };

            body.createFixture(fd);

            const sd_left = new PolygonShape();
            sd_left.setAsBox(0.45, 8.1, new Vec2(-4.35, 7.05), 0.2);
            fd.shape = sd_left;
            body.createFixture(fd);

            const sd_right = new PolygonShape();
            sd_right.setAsBox(0.45, 8.1, new Vec2(4.35, 7.05), -0.2);
            fd.shape = sd_right;
            body.createFixture(fd);
        }

        // add some small circles for effect
        for (let i = 0; i < 5; i++) {
            const cd = new CircleShape(Math.random() * 1 + 0.5);
            const body = this.world.createBody({
                type: BodyType.Dynamic,
                bullet: true,
                position: {
                    x: Math.random() * 30 - 25,
                    y: Math.random() * 32 + 2,
                },
            });
            body.createFixture({
                shape: cd,
                friction: 0.3,
                density: 1,
                restitution: 1.1,
            });
        }
    }

    public getDefaultViewZoom() {
        return 20;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 15,
        };
    }
}

registerTest("Continuous", "Continuous Collision", ContinousCollisionTest);
