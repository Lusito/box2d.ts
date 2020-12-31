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

import { Body, EdgeShape, Vec2, PolygonShape, BodyType, RandomFloat, Gjk, Toi } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";

function formatValueAveMax(step: number, ave: number, max: number) {
    return `${step.toFixed(0)} [${ave.toFixed(1)}] (${max.toFixed(0)})`;
}

class BulletTest extends Test {
    public m_body: Body;

    public m_bullet: Body;

    public m_x = 0;

    public constructor() {
        super();

        {
            const body = this.m_world.CreateBody();

            const edge = new EdgeShape();

            edge.SetTwoSided(new Vec2(-10, 0), new Vec2(10, 0));
            body.CreateFixture({ shape: edge });

            const shape = new PolygonShape();
            shape.SetAsBox(0.2, 1, new Vec2(0.5, 1), 0);
            body.CreateFixture({ shape });
        }

        {
            const box = new PolygonShape();
            box.SetAsBox(2, 0.1);

            this.m_body = this.m_world.CreateBody({
                type: BodyType.Dynamic,
                position: {
                    x: 0,
                    y: 4,
                },
            });
            this.m_body.CreateFixture({ shape: box, density: 1 });

            box.SetAsBox(0.25, 0.25);

            // this.m_x = RandomFloat(-1, 1);
            this.m_x = 0.20352793;

            this.m_bullet = this.m_world.CreateBody({
                type: BodyType.Dynamic,
                bullet: true,
                position: {
                    x: this.m_x,
                    y: 10,
                },
            });
            this.m_bullet.CreateFixture({ shape: box, density: 100 });

            this.m_bullet.SetLinearVelocity(new Vec2(0, -50));
        }
    }

    public Launch() {
        this.m_body.SetTransformVec(new Vec2(0, 4), 0);
        this.m_body.SetLinearVelocity(Vec2.ZERO);
        this.m_body.SetAngularVelocity(0);

        this.m_x = RandomFloat(-1, 1);
        this.m_bullet.SetTransformVec(new Vec2(this.m_x, 10), 0);
        this.m_bullet.SetLinearVelocity(new Vec2(0, -50));
        this.m_bullet.SetAngularVelocity(0);

        Gjk.reset();
        Toi.reset();
    }

    public GetDefaultViewZoom() {
        return 50;
    }

    public Step(settings: Settings, timeStep: number): void {
        super.Step(settings, timeStep);

        this.addDebug(
            "GJK Calls [ave Iters] (max Iters)",
            Gjk.calls > 0 && formatValueAveMax(Gjk.calls, Gjk.iters / Gjk.calls, Gjk.maxIters),
        );

        this.addDebug(
            "Toi Calls [ave Iters] (max Iters)",
            Toi.calls > 0 && formatValueAveMax(Toi.calls, Toi.iters / Toi.calls, Toi.maxIters),
        );

        this.addDebug(
            "Root Toi [ave Iters] (max Iters)",
            Toi.calls > 0 && `[${(Toi.rootIters / Toi.calls).toFixed(1)}] (${Toi.maxRootIters})`,
        );

        if (this.m_stepCount % 60 === 0) {
            this.Launch();
        }
    }
}

registerTest("Continuous", "Bullet", BulletTest);
