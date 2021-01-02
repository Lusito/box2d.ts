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

import { Body, EdgeShape, Vec2, ChainShape, PolygonShape, BodyType, CircleShape, XY, makeArray } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";

/**
 * This is a test of typical character collision scenarios. This does not
 * show how you should implement a character in your application.
 * Instead this is used to test smooth collision on edge chains.
 */
class CharacterCollisionTest extends Test {
    public character: Body;

    public constructor() {
        super();

        // Ground body
        {
            const ground = this.world.createBody();

            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-20, 0), new Vec2(20, 0));
            ground.createFixture({ shape });
        }

        // Collinear edges with no adjacency information.
        // This shows the problematic case where a box shape can hit
        // an internal vertex.
        {
            const ground = this.world.createBody();

            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-8, 1), new Vec2(-6, 1));
            ground.createFixture({ shape });
            shape.setTwoSided(new Vec2(-6, 1), new Vec2(-4, 1));
            ground.createFixture({ shape });
            shape.setTwoSided(new Vec2(-4, 1), new Vec2(-2, 1));
            ground.createFixture({ shape });
        }

        // Chain shape
        {
            const ground = this.world.createBody({
                angle: 0.25 * Math.PI,
            });

            const vs = makeArray(4, Vec2);
            vs[0].set(5, 7);
            vs[1].set(8, 7);
            vs[2].set(7, 8);
            vs[3].set(6, 8);

            const shape = new ChainShape();
            shape.createLoop(vs, 4);
            ground.createFixture({ shape });
        }

        // Square tiles. This shows that adjacency shapes may
        // have non-smooth collision. There is no solution
        // to this problem.
        {
            const ground = this.world.createBody();

            const shape = new PolygonShape();
            shape.setAsBox(1, 1, new Vec2(4, 3), 0);
            ground.createFixture({ shape });
            shape.setAsBox(1, 1, new Vec2(6, 3), 0);
            ground.createFixture({ shape });
            shape.setAsBox(1, 1, new Vec2(8, 3), 0);
            ground.createFixture({ shape });
        }

        // Square made from an edge loop. Collision should be smooth.
        {
            const ground = this.world.createBody();

            const vs = makeArray(4, Vec2);
            vs[0].set(-1, 3);
            vs[1].set(1, 3);
            vs[2].set(1, 5);
            vs[3].set(-1, 5);

            const shape = new ChainShape();
            shape.createLoop(vs, 4);
            ground.createFixture({ shape });
        }

        // Edge loop. Collision should be smooth.
        {
            const ground = this.world.createBody({
                position: { x: -10, y: 4 },
            });

            const vs = makeArray(10, Vec2);
            vs[0].set(0, 0);
            vs[1].set(6, 0);
            vs[2].set(6, 2);
            vs[3].set(4, 1);
            vs[4].set(2, 2);
            vs[5].set(0, 2);
            vs[6].set(-2, 2);
            vs[7].set(-4, 3);
            vs[8].set(-6, 2);
            vs[9].set(-6, 0);

            const shape = new ChainShape();
            shape.createLoop(vs, 10);
            ground.createFixture({ shape });
        }

        // Square character 1
        {
            const body = this.world.createBody({
                position: { x: -3, y: 8 },
                type: BodyType.Dynamic,
                fixedRotation: true,
                allowSleep: false,
            });

            const shape = new PolygonShape();
            shape.setAsBox(0.5, 0.5);

            body.createFixture({
                shape,
                density: 20,
            });
        }

        // Square character 2
        {
            const body = this.world.createBody({
                position: { x: -5, y: 5 },
                type: BodyType.Dynamic,
                fixedRotation: true,
                allowSleep: false,
            });

            const shape = new PolygonShape();
            shape.setAsBox(0.25, 0.25);

            body.createFixture({
                shape,
                density: 20,
            });
        }

        // Hexagon character
        {
            const body = this.world.createBody({
                position: { x: -5, y: 8 },
                type: BodyType.Dynamic,
                fixedRotation: true,
                allowSleep: false,
            });

            let angle = 0;
            const delta = Math.PI / 3;
            const vertices = makeArray(6, Vec2);
            for (let i = 0; i < 6; ++i) {
                vertices[i].set(0.5 * Math.cos(angle), 0.5 * Math.sin(angle));
                angle += delta;
            }

            const shape = new PolygonShape();
            shape.set(vertices, 6);

            body.createFixture({
                shape,
                density: 20,
            });
        }

        // Circle character
        {
            const body = this.world.createBody({
                position: { x: 3, y: 5 },
                type: BodyType.Dynamic,
                fixedRotation: true,
                allowSleep: false,
            });

            const shape = new CircleShape();
            shape.radius = 0.5;

            body.createFixture({
                shape,
                density: 20,
            });
        }

        // Circle character
        {
            this.character = this.world.createBody({
                position: { x: -7, y: 6 },
                type: BodyType.Dynamic,
                allowSleep: false,
            });

            const shape = new CircleShape();
            shape.radius = 0.25;

            this.character.createFixture({
                shape,
                density: 20,
                friction: 1,
            });
        }
    }

    public getDefaultViewZoom() {
        return 30;
    }

    public getCenter(): XY {
        return {
            x: -2,
            y: 0,
        };
    }

    public step(settings: Settings, timeStep: number): void {
        const v = this.character.getLinearVelocity();
        this.character.setLinearVelocity({
            x: -5,
            y: v.y,
        });

        super.step(settings, timeStep);
        this.addText("This tests various character collision shapes");
        this.addText("Limitation: square and hexagon can snag on aligned boxes.");
        this.addText("Feature: edge chains have smooth collision inside and out.");
    }
}

registerTest("Examples", "Character Collision", CharacterCollisionTest);
