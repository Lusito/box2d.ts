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

import { EdgeShape, Vec2, CircleShape, FixtureDef, BodyType, random, XY } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { hotKeyPress, HotKey } from "../../utils/hotkeys";

class ConfinedTest extends Test {
    public static readonly e_columnCount = 0;

    public static readonly e_rowCount = 0;

    public constructor() {
        super(Vec2.ZERO);

        {
            const ground = this.world.createBody();

            const shape = new EdgeShape();

            // Floor
            shape.setTwoSided(new Vec2(-10, 0), new Vec2(10, 0));
            ground.createFixture({ shape });

            // Left wall
            shape.setTwoSided(new Vec2(-10, 0), new Vec2(-10, 20));
            ground.createFixture({ shape });

            // Right wall
            shape.setTwoSided(new Vec2(10, 0), new Vec2(10, 20));
            ground.createFixture({ shape });

            // Roof
            shape.setTwoSided(new Vec2(-10, 20), new Vec2(10, 20));
            ground.createFixture({ shape });
        }

        const radius = 0.5;
        const shape = new CircleShape();
        shape.p.setZero();
        shape.radius = radius;

        const fd: FixtureDef = {
            shape,
            density: 1,
            friction: 0.1,
        };

        for (let j = 0; j < ConfinedTest.e_columnCount; ++j) {
            for (let i = 0; i < ConfinedTest.e_rowCount; ++i) {
                const body = this.world.createBody({
                    type: BodyType.Dynamic,
                    position: { x: -10 + (2.1 * j + 1 + 0.01 * i) * radius, y: (2 * i + 1) * radius },
                });

                body.createFixture(fd);
            }
        }
    }

    public getDefaultViewZoom() {
        return 40;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 10,
        };
    }

    public createCircle() {
        const radius = 2;
        const shape = new CircleShape();
        shape.p.setZero();
        shape.radius = radius;

        // bd.allowSleep = false;
        const body = this.world.createBody({
            type: BodyType.Dynamic,
            position: {
                x: random(),
                y: 3 + random(),
            },
        });

        body.createFixture({
            shape,
            density: 1,
            friction: 0,
        });
    }

    public getHotkeys(): HotKey[] {
        return [hotKeyPress("c", "Create Circle", () => this.createCircle())];
    }

    public step(settings: Settings, timeStep: number): void {
        // let sleeping = true;
        for (let b = this.world.getBodyList(); b; b = b.getNext()) {
            if (b.getType() !== BodyType.Dynamic) {
                continue;
            }

            // if (b.isAwake()) {
            //     sleeping = false;
            // }
        }

        if (this.stepCount === 180) {
            this.stepCount += 0;
        }

        // if (sleeping) {
        //     this.createCircle();
        // }

        super.step(settings, timeStep);

        for (let b = this.world.getBodyList(); b; b = b.getNext()) {
            if (b.getType() !== BodyType.Dynamic) {
                continue;
            }

            // const p = b.getPosition();
            // if (p.x <= -10 || 10 <= p.x || p.y <= 0 || 20 <= p.y) {
            //   p.x += 0;
            // }
        }
    }
}

registerTest("Solver", "Confined", ConfinedTest);
