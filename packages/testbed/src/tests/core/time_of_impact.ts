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

import { PolygonShape, Sweep, TOIInput, TOIOutput, TimeOfImpact, Transform, Vec2, Color, Toi, XY } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { g_debugDraw } from "../../utils/draw";

class TimeOfImpactTest extends Test {
    public m_shapeA = new PolygonShape();

    public m_shapeB = new PolygonShape();

    public constructor() {
        super();

        this.m_shapeA.SetAsBox(25, 5);
        this.m_shapeB.SetAsBox(2.5, 2.5);
    }

    public Step(settings: Settings, timeStep: number): void {
        super.Step(settings, timeStep);

        const sweepA = new Sweep();
        sweepA.c0.Set(0, 20 + 8 * Math.cos(Date.now() / 1000)); // (24, -60);
        sweepA.a0 = 2.95;
        sweepA.c.Copy(sweepA.c0);
        sweepA.a = sweepA.a0;
        sweepA.localCenter.SetZero();

        const sweepB = new Sweep();
        sweepB.c0.Set(20, 40); // (53.474274, -50.252514);
        sweepB.a0 = 0.1; // 513.36676; // - 162 * Math.PI;
        sweepB.c.Set(-20, 0); // (54.595478, -51.083473);
        sweepB.a = 3.1; // 513.62781; //  - 162 * Math.PI;
        sweepB.localCenter.SetZero();

        // sweepB.a0 -= 300 * Math.PI;
        // sweepB.a -= 300 * Math.PI;

        const input = new TOIInput();
        input.proxyA.SetShape(this.m_shapeA, 0);
        input.proxyB.SetShape(this.m_shapeB, 0);
        input.sweepA.Copy(sweepA);
        input.sweepB.Copy(sweepB);
        input.tMax = 1;

        const output = new TOIOutput();

        TimeOfImpact(output, input);

        this.addDebug("Toi", output.t.toFixed(3));

        this.addDebug("Max Toi Iters", Toi.maxIters);
        this.addDebug("Max Root Iters", Toi.maxRootIters);

        const vertices = [];

        const transformA = new Transform();
        sweepA.GetTransform(transformA, 0);
        for (let i = 0; i < this.m_shapeA.m_count; ++i) {
            vertices[i] = Transform.MultiplyVec2(transformA, this.m_shapeA.m_vertices[i], new Vec2());
        }
        g_debugDraw.DrawPolygon(vertices, this.m_shapeA.m_count, new Color(0.9, 0.9, 0.9));

        const transformB = new Transform();
        sweepB.GetTransform(transformB, 0);

        // Vec2 localPoint(2, -0.1 );

        for (let i = 0; i < this.m_shapeB.m_count; ++i) {
            vertices[i] = Transform.MultiplyVec2(transformB, this.m_shapeB.m_vertices[i], new Vec2());
        }
        g_debugDraw.DrawPolygon(vertices, this.m_shapeB.m_count, new Color(0.5, 0.9, 0.5));
        g_debugDraw.DrawStringWorld(transformB.p.x, transformB.p.y, `${(0).toFixed(1)}`);

        sweepB.GetTransform(transformB, output.t);
        for (let i = 0; i < this.m_shapeB.m_count; ++i) {
            vertices[i] = Transform.MultiplyVec2(transformB, this.m_shapeB.m_vertices[i], new Vec2());
        }
        g_debugDraw.DrawPolygon(vertices, this.m_shapeB.m_count, new Color(0.5, 0.7, 0.9));
        g_debugDraw.DrawStringWorld(transformB.p.x, transformB.p.y, `${output.t.toFixed(3)}`);

        sweepB.GetTransform(transformB, 1);
        for (let i = 0; i < this.m_shapeB.m_count; ++i) {
            vertices[i] = Transform.MultiplyVec2(transformB, this.m_shapeB.m_vertices[i], new Vec2());
        }
        g_debugDraw.DrawPolygon(vertices, this.m_shapeB.m_count, new Color(0.9, 0.5, 0.5));
        g_debugDraw.DrawStringWorld(transformB.p.x, transformB.p.y, `${(1).toFixed(1)}`);

        // #if 0
        for (let t = 0; t < 1; t += 0.1) {
            sweepB.GetTransform(transformB, t);
            for (let i = 0; i < this.m_shapeB.m_count; ++i) {
                vertices[i] = Transform.MultiplyVec2(transformB, this.m_shapeB.m_vertices[i], new Vec2());
            }
            g_debugDraw.DrawPolygon(vertices, this.m_shapeB.m_count, new Color(0.5, 0.5, 0.5));
            g_debugDraw.DrawStringWorld(transformB.p.x, transformB.p.y, `${t.toFixed(1)}`);
        }
        // #endif
    }

    public GetDefaultViewZoom() {
        return 20;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 10,
        };
    }
}

registerTest("Collision", "Time of Impact", TimeOfImpactTest);
