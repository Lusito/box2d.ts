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

import { b2Vec2, b2ChainShape, b2FixtureDef, b2BodyType, b2PolygonShape, b2CircleShape, XY } from "@box2d/core";

import { registerTest, Test } from "../../test";

class ContinousCollision extends Test {
    public constructor() {
        super();

        {
            const ground = this.m_world.CreateBody();

            const shape = new b2ChainShape();
            shape.CreateLoop([new b2Vec2(-30, 0), new b2Vec2(-30, 40), new b2Vec2(30, 40), new b2Vec2(30, 0)]);
            ground.CreateFixture({ shape });
        }

        // Always on, even if default is off
        this.m_world.SetContinuousPhysics(true);

        // Create 'basket'
        {
            const body = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                bullet: true,
                position: { x: 15, y: 5 },
            });

            const sd_bottom = new b2PolygonShape();
            sd_bottom.SetAsBox(4.5, 0.45);
            // These values are used for all the parts of the 'basket'
            const fd: b2FixtureDef = {
                density: 4,
                restitution: 1.4,
                shape: sd_bottom,
            };

            body.CreateFixture(fd);

            const sd_left = new b2PolygonShape();
            sd_left.SetAsBox(0.45, 8.1, new b2Vec2(-4.35, 7.05), 0.2);
            fd.shape = sd_left;
            body.CreateFixture(fd);

            const sd_right = new b2PolygonShape();
            sd_right.SetAsBox(0.45, 8.1, new b2Vec2(4.35, 7.05), -0.2);
            fd.shape = sd_right;
            body.CreateFixture(fd);
        }

        // add some small circles for effect
        for (let i = 0; i < 5; i++) {
            const cd = new b2CircleShape(Math.random() * 1 + 0.5);
            const body = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                bullet: true,
                position: {
                    x: Math.random() * 30 - 25,
                    y: Math.random() * 32 + 2,
                },
            });
            body.CreateFixture({
                shape: cd,
                friction: 0.3,
                density: 1,
                restitution: 1.1,
            });
        }
    }

    public GetDefaultViewZoom() {
        return 20;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 15,
        };
    }
}

registerTest("Continuous", "Continuous Collision", ContinousCollision);
