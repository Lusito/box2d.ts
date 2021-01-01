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

import {
    Body,
    EdgeShape,
    Vec2,
    PolygonShape,
    FixtureDef,
    BodyType,
    CircleShape,
    makeNumberArray,
    setBlockSolve,
    getBlockSolve,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { hotKeyPress, HotKey } from "../../utils/hotkeys";

class BoxStackTest extends Test {
    public static readonly e_columnCount = 1;

    public static readonly e_rowCount = 15;

    public m_bullet: Body | null = null;

    public m_bodies = new Array<Body>(BoxStackTest.e_rowCount * BoxStackTest.e_columnCount);

    public m_indices = makeNumberArray(BoxStackTest.e_rowCount * BoxStackTest.e_columnCount);

    public constructor() {
        super();

        {
            const ground = this.m_world.createBody();

            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-40, 0), new Vec2(40, 0));
            ground.createFixture({ shape });

            shape.setTwoSided(new Vec2(20, 0), new Vec2(20, 20));
            ground.createFixture({ shape });
        }

        const xs = [0, -10, -5, 5, 10];

        for (let j = 0; j < BoxStackTest.e_columnCount; ++j) {
            const shape = new PolygonShape();
            shape.setAsBox(0.5, 0.5);

            const fd: FixtureDef = {
                shape,
                density: 1,
                friction: 0.3,
            };

            for (let i = 0; i < BoxStackTest.e_rowCount; ++i) {
                const n = j * BoxStackTest.e_rowCount + i;
                // DEBUG: assert(n < VerticalStack.e_rowCount * VerticalStack.e_columnCount);
                this.m_indices[n] = n;

                const x = 0;
                // const x = randomFloat(-0.02, 0.02);
                // const x = i % 2 === 0 ? -0.01 : 0.01;
                const body = this.m_world.createBody({
                    type: BodyType.Dynamic,
                    position: { x: xs[j] + x, y: 0.55 + 1.1 * i },
                    userData: this.m_indices[n],
                });

                this.m_bodies[n] = body;

                body.createFixture(fd);
            }
        }
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("Enter", "Launch a bullet", () => this.launchBullet()),
            hotKeyPress("b", "Toggle Block solving", () => setBlockSolve(!getBlockSolve())),
        ];
    }

    public destroy() {
        setBlockSolve(true);
    }

    private launchBullet() {
        if (this.m_bullet) {
            this.m_world.destroyBody(this.m_bullet);
            this.m_bullet = null;
        }

        {
            const shape = new CircleShape();
            shape.m_radius = 0.25;

            const fd: FixtureDef = {
                shape,
                density: 20,
                restitution: 0.05,
            };

            this.m_bullet = this.m_world.createBody({
                type: BodyType.Dynamic,
                bullet: true,
                position: { x: -31, y: 5 },
            });
            this.m_bullet.createFixture(fd);

            this.m_bullet.setLinearVelocity(new Vec2(400, 0));
        }
    }

    public step(settings: Settings, timeStep: number): void {
        super.step(settings, timeStep);
        this.addDebug("Blocksolve", getBlockSolve());
        // if (this.m_stepCount === 300) {
        //     if (this.m_bullet !== null) {
        //         this.m_world.destroyBody(this.m_bullet);
        //         this.m_bullet = null;
        //     }

        //     {
        //         const shape = new CircleShape();
        //         shape.m_radius = 0.25;

        //         this.m_bullet = this.m_world.createBody({
        //             type: BodyType.Dynamic,
        //             bullet: true,
        //             position: { x: -31, y: 5 },
        //         });
        //         this.m_bullet.createFixture({
        //             shape,
        //             density: 20,
        //             restitution: 0.05,
        //         });

        //         this.m_bullet.setLinearVelocity(new Vec2(400, 0));
        //     }
        // }
    }
}

registerTest("Stacking", "Box Stack", BoxStackTest);
