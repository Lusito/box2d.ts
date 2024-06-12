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

import { b2Body, b2EdgeShape, b2Vec2, b2BodyType, b2FixtureDef, b2PolygonShape, b2CircleShape, XY } from "@box2d/core";
import { b2BuoyancyController } from "@box2d/controllers";

import { registerTest, Test } from "../../test";

class BuoyancyTest extends Test {
    public m_bodies: b2Body[];

    public m_controller: b2BuoyancyController;

    public constructor() {
        super();

        this.m_bodies = [];

        const bc = new b2BuoyancyController();
        this.m_controller = bc;

        bc.normal.Set(0, 1);
        bc.offset = 20;
        bc.density = 2;
        bc.linearDrag = 5;
        bc.angularDrag = 2;

        const ground = this.m_world.CreateBody();

        {
            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(-40, 0), new b2Vec2(40, 0));
            ground.CreateFixture({ shape });
            shape.SetTwoSided(new b2Vec2(-40, 0), new b2Vec2(-40, 25));
            ground.CreateFixture({ shape });
            shape.SetTwoSided(new b2Vec2(40, 0), new b2Vec2(40, 25));
            ground.CreateFixture({ shape });
        }

        // Spawn in a bunch of crap
        for (let i = 0; i < 5; i++) {
            const body = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                // isBullet: true,
                position: {
                    x: Math.random() * 40 - 20,
                    y: Math.random() * 15 + 5,
                },
                angle: Math.random() * Math.PI,
            });

            const polygon = new b2PolygonShape();
            polygon.SetAsBox(Math.random() * 0.5 + 1, Math.random() * 0.5 + 1);
            body.CreateFixture({
                density: 1,
                // Override the default friction.
                friction: 0.3,
                restitution: 0.1,
                shape: polygon,
            });

            this.m_bodies.push(body);
        }

        for (let i = 0; i < 5; i++) {
            const body = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                // isBullet: true,
                position: {
                    x: Math.random() * 40 - 20,
                    y: Math.random() * 15 + 5,
                },
                angle: Math.random() * Math.PI,
            });

            body.CreateFixture({
                density: 1,
                // Override the default friction.
                friction: 0.3,
                restitution: 0.1,
                shape: new b2CircleShape(Math.random() * 0.5 + 1),
            });

            this.m_bodies.push(body);
        }

        for (let i = 0; i < 15; i++) {
            const body = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                // isBullet: true,
                position: {
                    x: Math.random() * 40 - 20,
                    y: Math.random() * 15 + 5,
                },
                angle: Math.random() * Math.PI,
            });

            const polygon = new b2PolygonShape();
            if (Math.random() > 0.66) {
                polygon.Set([
                    new b2Vec2(-1 - Math.random() * 1, 1 + Math.random() * 1),
                    new b2Vec2(-0.5 - Math.random() * 1, -1 - Math.random() * 1),
                    new b2Vec2(0.5 + Math.random() * 1, -1 - Math.random() * 1),
                    new b2Vec2(1 + Math.random() * 1, 1 + Math.random() * 1),
                ]);
            } else if (Math.random() > 0.5) {
                const array = [];
                array[0] = new b2Vec2(0, 1 + Math.random() * 1);
                array[2] = new b2Vec2(-0.5 - Math.random() * 1, -1 - Math.random() * 1);
                array[3] = new b2Vec2(0.5 + Math.random() * 1, -1 - Math.random() * 1);
                array[1] = new b2Vec2(array[0].x + array[2].x, array[0].y + array[2].y);
                array[1].Scale(Math.random() / 2 + 0.8);
                array[4] = new b2Vec2(array[3].x + array[0].x, array[3].y + array[0].y);
                array[4].Scale(Math.random() / 2 + 0.8);
                polygon.Set(array);
            } else {
                polygon.Set([
                    new b2Vec2(0, 1 + Math.random() * 1),
                    new b2Vec2(-0.5 - Math.random() * 1, -1 - Math.random() * 1),
                    new b2Vec2(0.5 + Math.random() * 1, -1 - Math.random() * 1),
                ]);
            }
            body.CreateFixture({
                density: 1,
                friction: 0.3,
                restitution: 0.1,
                shape: polygon,
            });

            this.m_bodies.push(body);
        }

        // Add some exciting bath toys
        {
            const body = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: { x: 0, y: 40 },
                angle: 0,
            });

            const polygon = new b2PolygonShape();
            polygon.SetAsBox(4, 1);
            body.CreateFixture({
                density: 3,
                shape: polygon,
            });

            this.m_bodies.push(body);
        }

        {
            const body = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: {
                    x: 0,
                    y: 30,
                },
            });

            const circle = new b2CircleShape(0.7);
            const fd: b2FixtureDef = {
                density: 2,
                shape: circle,
            };
            circle.m_p.Set(3, 0);
            body.CreateFixture(fd);
            circle.m_p.Set(-3, 0);
            body.CreateFixture(fd);
            circle.m_p.Set(0, 3);
            body.CreateFixture(fd);
            circle.m_p.Set(0, -3);
            body.CreateFixture(fd);

            fd.density = 2;
            const polygon = new b2PolygonShape();
            fd.shape = polygon;
            polygon.SetAsBox(3, 0.2);
            body.CreateFixture(fd);
            polygon.SetAsBox(0.2, 3);
            body.CreateFixture(fd);

            this.m_bodies.push(body);
        }

        // if (DEBUG) {
        //   for (let body_i = 0; i < this.m_bodies.length; ++i)
        //     this.m_controller.AddBody(this.m_bodies[body_i]);
        //   for (let body_i = 0; i < this.m_bodies.length; ++i)
        //     this.m_controller.RemoveBody(this.m_bodies[body_i]);
        // }
        for (const body of this.m_bodies) {
            this.m_controller.AddBody(body);
        }
        // if (DEBUG) {
        //   this.m_world.AddController(this.m_controller);
        //   this.m_world.RemoveController(this.m_controller);
        // }
        this.m_world.AddController(this.m_controller);
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 10,
        };
    }
}

registerTest("Controllers", "Boyancy Test", BuoyancyTest);
