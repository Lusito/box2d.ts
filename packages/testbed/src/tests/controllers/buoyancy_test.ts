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

import { Body, EdgeShape, Vec2, BodyType, FixtureDef, PolygonShape, CircleShape, XY } from "@box2d/core";
import { BuoyancyController } from "@box2d/controllers";

import { registerTest, Test } from "../../test";

class BuoyancyTest extends Test {
    public bodies: Body[];

    public controller: BuoyancyController;

    public constructor() {
        super();

        this.bodies = [];

        const bc = new BuoyancyController();
        this.controller = bc;

        bc.normal.set(0, 1);
        bc.offset = 20;
        bc.density = 2;
        bc.linearDrag = 5;
        bc.angularDrag = 2;

        const ground = this.world.createBody();

        {
            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-40, 0), new Vec2(40, 0));
            ground.createFixture({ shape });
            shape.setTwoSided(new Vec2(-40, 0), new Vec2(-40, 25));
            ground.createFixture({ shape });
            shape.setTwoSided(new Vec2(40, 0), new Vec2(40, 25));
            ground.createFixture({ shape });
        }

        // Spawn in a bunch of crap
        for (let i = 0; i < 5; i++) {
            const body = this.world.createBody({
                type: BodyType.Dynamic,
                // isBullet: true,
                position: {
                    x: Math.random() * 40 - 20,
                    y: Math.random() * 15 + 5,
                },
                angle: Math.random() * Math.PI,
            });

            const polygon = new PolygonShape();
            polygon.setAsBox(Math.random() * 0.5 + 1, Math.random() * 0.5 + 1);
            body.createFixture({
                density: 1,
                // Override the default friction.
                friction: 0.3,
                restitution: 0.1,
                shape: polygon,
            });

            this.bodies.push(body);
        }

        for (let i = 0; i < 5; i++) {
            const body = this.world.createBody({
                type: BodyType.Dynamic,
                // isBullet: true,
                position: {
                    x: Math.random() * 40 - 20,
                    y: Math.random() * 15 + 5,
                },
                angle: Math.random() * Math.PI,
            });

            body.createFixture({
                density: 1,
                // Override the default friction.
                friction: 0.3,
                restitution: 0.1,
                shape: new CircleShape(Math.random() * 0.5 + 1),
            });

            this.bodies.push(body);
        }

        for (let i = 0; i < 15; i++) {
            const body = this.world.createBody({
                type: BodyType.Dynamic,
                // isBullet: true,
                position: {
                    x: Math.random() * 40 - 20,
                    y: Math.random() * 15 + 5,
                },
                angle: Math.random() * Math.PI,
            });

            const polygon = new PolygonShape();
            if (Math.random() > 0.66) {
                polygon.set([
                    new Vec2(-1 - Math.random() * 1, 1 + Math.random() * 1),
                    new Vec2(-0.5 - Math.random() * 1, -1 - Math.random() * 1),
                    new Vec2(0.5 + Math.random() * 1, -1 - Math.random() * 1),
                    new Vec2(1 + Math.random() * 1, 1 + Math.random() * 1),
                ]);
            } else if (Math.random() > 0.5) {
                const array = [];
                array[0] = new Vec2(0, 1 + Math.random() * 1);
                array[2] = new Vec2(-0.5 - Math.random() * 1, -1 - Math.random() * 1);
                array[3] = new Vec2(0.5 + Math.random() * 1, -1 - Math.random() * 1);
                array[1] = new Vec2(array[0].x + array[2].x, array[0].y + array[2].y);
                array[1].scale(Math.random() / 2 + 0.8);
                array[4] = new Vec2(array[3].x + array[0].x, array[3].y + array[0].y);
                array[4].scale(Math.random() / 2 + 0.8);
                polygon.set(array);
            } else {
                polygon.set([
                    new Vec2(0, 1 + Math.random() * 1),
                    new Vec2(-0.5 - Math.random() * 1, -1 - Math.random() * 1),
                    new Vec2(0.5 + Math.random() * 1, -1 - Math.random() * 1),
                ]);
            }
            body.createFixture({
                density: 1,
                friction: 0.3,
                restitution: 0.1,
                shape: polygon,
            });

            this.bodies.push(body);
        }

        // Add some exciting bath toys
        {
            const body = this.world.createBody({
                type: BodyType.Dynamic,
                position: { x: 0, y: 40 },
                angle: 0,
            });

            const polygon = new PolygonShape();
            polygon.setAsBox(4, 1);
            body.createFixture({
                density: 3,
                shape: polygon,
            });

            this.bodies.push(body);
        }

        {
            const body = this.world.createBody({
                type: BodyType.Dynamic,
                position: {
                    x: 0,
                    y: 30,
                },
            });

            const circle = new CircleShape(0.7);
            const fd: FixtureDef = {
                density: 2,
                shape: circle,
            };
            circle.p.set(3, 0);
            body.createFixture(fd);
            circle.p.set(-3, 0);
            body.createFixture(fd);
            circle.p.set(0, 3);
            body.createFixture(fd);
            circle.p.set(0, -3);
            body.createFixture(fd);

            fd.density = 2;
            const polygon = new PolygonShape();
            fd.shape = polygon;
            polygon.setAsBox(3, 0.2);
            body.createFixture(fd);
            polygon.setAsBox(0.2, 3);
            body.createFixture(fd);

            this.bodies.push(body);
        }

        // if (DEBUG) {
        //   for (let body_i = 0; i < this.bodies.length; ++i)
        //     this.controller.addBody(this.bodies[body_i]);
        //   for (let body_i = 0; i < this.bodies.length; ++i)
        //     this.controller.removeBody(this.bodies[body_i]);
        // }
        for (const body of this.bodies) {
            this.controller.addBody(body);
        }
        // if (DEBUG) {
        //   this.world.addController(this.controller);
        //   this.world.removeController(this.controller);
        // }
        this.world.addController(this.controller);
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 10,
        };
    }
}

registerTest("Controllers", "Boyancy", BuoyancyTest);
