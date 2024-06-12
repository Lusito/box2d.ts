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

import { b2EdgeShape, b2Vec2, b2CircleShape, b2FixtureDef, b2BodyType, b2Random, XY } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { hotKeyPress, HotKey } from "../../utils/hotkeys";

class Confined extends Test {
    public static readonly e_columnCount = 0;

    public static readonly e_rowCount = 0;

    public constructor() {
        super(b2Vec2.ZERO);

        {
            const ground = this.m_world.CreateBody();

            const shape = new b2EdgeShape();

            // Floor
            shape.SetTwoSided(new b2Vec2(-10, 0), new b2Vec2(10, 0));
            ground.CreateFixture({ shape });

            // Left wall
            shape.SetTwoSided(new b2Vec2(-10, 0), new b2Vec2(-10, 20));
            ground.CreateFixture({ shape });

            // Right wall
            shape.SetTwoSided(new b2Vec2(10, 0), new b2Vec2(10, 20));
            ground.CreateFixture({ shape });

            // Roof
            shape.SetTwoSided(new b2Vec2(-10, 20), new b2Vec2(10, 20));
            ground.CreateFixture({ shape });
        }

        const radius = 0.5;
        const shape = new b2CircleShape();
        shape.m_p.SetZero();
        shape.m_radius = radius;

        const fd: b2FixtureDef = {
            shape,
            density: 1,
            friction: 0.1,
        };

        for (let j = 0; j < Confined.e_columnCount; ++j) {
            for (let i = 0; i < Confined.e_rowCount; ++i) {
                const body = this.m_world.CreateBody({
                    type: b2BodyType.b2_dynamicBody,
                    position: { x: -10 + (2.1 * j + 1 + 0.01 * i) * radius, y: (2 * i + 1) * radius },
                });

                body.CreateFixture(fd);
            }
        }
    }

    public GetDefaultViewZoom() {
        return 40;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 10,
        };
    }

    public CreateCircle() {
        const radius = 2;
        const shape = new b2CircleShape();
        shape.m_p.SetZero();
        shape.m_radius = radius;

        // bd.allowSleep = false;
        const body = this.m_world.CreateBody({
            type: b2BodyType.b2_dynamicBody,
            position: {
                x: b2Random(),
                y: 3 + b2Random(),
            },
        });

        body.CreateFixture({
            shape,
            density: 1,
            friction: 0,
        });
    }

    public getHotkeys(): HotKey[] {
        return [hotKeyPress("c", "Create Circle", () => this.CreateCircle())];
    }

    public Step(settings: Settings, timeStep: number): void {
        // let sleeping = true;
        for (let b = this.m_world.GetBodyList(); b; b = b.GetNext()) {
            if (b.GetType() !== b2BodyType.b2_dynamicBody) {
                continue;
            }

            // if (b.IsAwake()) {
            //     sleeping = false;
            // }
        }

        if (this.m_stepCount === 180) {
            this.m_stepCount += 0;
        }

        // if (sleeping) {
        //     this.CreateCircle();
        // }

        super.Step(settings, timeStep);

        for (let b = this.m_world.GetBodyList(); b; b = b.GetNext()) {
            if (b.GetType() !== b2BodyType.b2_dynamicBody) {
                continue;
            }

            // const p = b.GetPosition();
            // if (p.x <= -10 || 10 <= p.x || p.y <= 0 || 20 <= p.y) {
            //   p.x += 0;
            // }
        }
    }
}

registerTest("Solver", "Confined", Confined);
