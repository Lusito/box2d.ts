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

class ContinuousTest extends Test {
    public m_body: Body;

    public m_angularVelocity = 0;

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
            const shape = new PolygonShape();
            shape.SetAsBox(2, 0.1);

            this.m_body = this.m_world.CreateBody({
                type: BodyType.Dynamic,
                position: { x: 0, y: 20 },
                // angle: 0.1,
            });
            this.m_body.CreateFixture({ shape, density: 1 });

            this.m_angularVelocity = RandomFloat(-50, 50);
            // this.m_angularVelocity = 46.661274;
            this.m_body.SetLinearVelocity(new Vec2(0, -100));
            this.m_body.SetAngularVelocity(this.m_angularVelocity);
        }
        /*
    else
    {
      const body = this.m_world.CreateBody({
        type: BodyType.Dynamic,
        position: { y: 0, y:2}
      });
      const shape = new CircleShape();
      shape.m_p.SetZero();
      shape.m_radius = 0.5;
      body.CreateFixture({ shape, density: 1 });
      body = this.m_world.CreateBody({
        type: BodyType.Dynamic,
        bullet: true,
        position: { y: 0, y:10}
      });
      body.CreateFixture({ shape, density: 1 });
      body.SetLinearVelocity(new Vec2(0, -100));
    }
    */

        Gjk.reset();
        Toi.reset();
    }

    public Launch() {
        Gjk.reset();
        Toi.reset();

        this.m_body.SetTransformVec(new Vec2(0, 20), 0);
        this.m_angularVelocity = RandomFloat(-50, 50);
        this.m_body.SetLinearVelocity(new Vec2(0, -100));
        this.m_body.SetAngularVelocity(this.m_angularVelocity);
    }

    public Step(settings: Settings, timeStep: number): void {
        super.Step(settings, timeStep);

        this.addDebug(
            "GJK Calls [ave Iters] (max Iters)",
            Gjk.calls > 0 &&
                `${Gjk.calls.toFixed(0)} [${(Gjk.iters / Gjk.calls).toFixed(1)}] (${Gjk.maxIters.toFixed(0)})`,
        );

        this.addDebug(
            "Toi Calls [ave Iters] (max Iters)",
            Toi.calls > 0 && `${Toi.calls} [${(Toi.iters / Toi.calls).toFixed(1)}] (${Toi.maxIters})`,
        );

        this.addDebug(
            "Toi Root [ave Iters] (max Iters)",
            Toi.calls > 0 && `${Toi.calls} [${(Toi.rootIters / Toi.calls).toFixed(1)}] (${Toi.maxRootIters})`,
        );

        this.addDebug(
            "Toi Time in ms [ave] (max)",
            Toi.calls > 0 && `[${((1000 * Toi.time) / Toi.calls).toFixed(1)}] (${(1000 * Toi.maxTime).toFixed(1)})`,
        );

        if (this.m_stepCount % 60 === 0) {
            this.Launch();
        }
    }

    public GetDefaultViewZoom() {
        return 50;
    }
}

registerTest("Continuous", "Continuous", ContinuousTest);
