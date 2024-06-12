/*
 * Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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
    b2Vec2,
    b2_polygonRadius,
    b2Transform,
    b2ShapeCastInput,
    b2ShapeCastOutput,
    b2ShapeCast,
    b2DistanceInput,
    b2SimplexCache,
    b2DistanceOutput,
    b2Distance,
    b2Color,
    b2MakeArray,
    b2_maxPolygonVertices,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";

const colorA = new b2Color(0.9, 0.9, 0.9);
const colorB = new b2Color(0.5, 0.9, 0.5);
const colorB2 = new b2Color(0.5, 0.7, 0.9);
const colorHit = new b2Color(0.9, 0.3, 0.3);

class ShapeCast extends Test {
    public static e_vertexCount = 8;

    public m_vAs = b2MakeArray(b2_maxPolygonVertices, b2Vec2);

    public m_countA = 0;

    public m_radiusA = 0;

    public m_vBs = b2MakeArray(b2_maxPolygonVertices, b2Vec2);

    public m_countB = 0;

    public m_radiusB = 0;

    public m_transformA = new b2Transform();

    public m_transformB = new b2Transform();

    public m_translationB = new b2Vec2();

    public constructor() {
        super();

        // #if 1
        this.m_vAs[0].Set(-0.5, 1);
        this.m_vAs[1].Set(0.5, 1);
        this.m_vAs[2].Set(0, 0);
        this.m_countA = 3;
        this.m_radiusA = b2_polygonRadius;

        this.m_vBs[0].Set(-0.5, -0.5);
        this.m_vBs[1].Set(0.5, -0.5);
        this.m_vBs[2].Set(0.5, 0.5);
        this.m_vBs[3].Set(-0.5, 0.5);
        this.m_countB = 4;
        this.m_radiusB = b2_polygonRadius;

        this.m_transformA.p.Set(0, 0.25);
        this.m_transformA.q.SetIdentity();
        this.m_transformB.p.Set(-4, 0);
        this.m_transformB.q.SetIdentity();
        this.m_translationB.Set(8, 0);
        // #elif 0
        // this.m_vAs[0].Set(0, 0);
        // this.m_countA = 1;
        // this.m_radiusA = 0.5;

        // this.m_vBs[0].Set(0, 0);
        // this.m_countB = 1;
        // this.m_radiusB = 0.5;

        // this.m_transformA.p.Set(0, 0.25);
        // this.m_transformA.q.SetIdentity();
        // this.m_transformB.p.Set(-4, 0);
        // this.m_transformB.q.SetIdentity();
        // this.m_translationB.Set(8, 0);
        // #else
        // this.m_vAs[0].Set(0, 0);
        // this.m_vAs[1].Set(2, 0);
        // this.m_countA = 2;
        // this.m_radiusA = b2_polygonRadius;

        // this.m_vBs[0].Set(0, 0);
        // this.m_countB = 1;
        // this.m_radiusB = 0.25;

        // // Initial overlap
        // this.m_transformA.p.Set(0, 0);
        // this.m_transformA.q.SetIdentity();
        // this.m_transformB.p.Set(-0.244360745, 0.05999358);
        // this.m_transformB.q.SetIdentity();
        // this.m_translationB.Set(0, 0.0399999991);
        // #endif
    }

    public Step(settings: Settings, timeStep: number): void {
        super.Step(settings, timeStep);

        const input = new b2ShapeCastInput();
        input.proxyA.SetVerticesRadius(this.m_vAs, this.m_countA, this.m_radiusA);
        input.proxyB.SetVerticesRadius(this.m_vBs, this.m_countB, this.m_radiusB);
        input.transformA.Copy(this.m_transformA);
        input.transformB.Copy(this.m_transformB);
        input.translationB.Copy(this.m_translationB);

        const output = new b2ShapeCastOutput();
        const hit = b2ShapeCast(output, input);

        const transformB2 = new b2Transform();
        transformB2.q.Copy(this.m_transformB.q);
        transformB2.p.Copy(this.m_transformB.p).AddScaled(output.lambda, input.translationB);

        const distanceInput = new b2DistanceInput();
        distanceInput.proxyA.SetVerticesRadius(this.m_vAs, this.m_countA, this.m_radiusA);
        distanceInput.proxyB.SetVerticesRadius(this.m_vBs, this.m_countB, this.m_radiusB);
        distanceInput.transformA.Copy(this.m_transformA);
        distanceInput.transformB.Copy(transformB2);
        distanceInput.useRadii = false;
        const simplexCache = new b2SimplexCache();
        simplexCache.count = 0;
        const distanceOutput = new b2DistanceOutput();

        b2Distance(distanceOutput, simplexCache, distanceInput);

        this.addDebug("Hit", hit);
        this.addDebug("Iters", output.iterations);
        this.addDebug("Lambda", output.lambda.toFixed(6));
        this.addDebug("Distance", distanceOutput.distance.toFixed(6));

        const vertices = b2MakeArray(b2_maxPolygonVertices, b2Vec2);

        for (let i = 0; i < this.m_countA; ++i) {
            b2Transform.MultiplyVec2(this.m_transformA, this.m_vAs[i], vertices[i]);
        }

        const draw = settings.m_debugDraw;
        if (this.m_countA === 1) {
            draw.DrawCircle(vertices[0], this.m_radiusA, colorA);
        } else {
            draw.DrawPolygon(vertices, this.m_countA, colorA);
        }

        for (let i = 0; i < this.m_countB; ++i) {
            b2Transform.MultiplyVec2(this.m_transformB, this.m_vBs[i], vertices[i]);
        }

        if (this.m_countB === 1) {
            draw.DrawCircle(vertices[0], this.m_radiusB, colorB);
        } else {
            draw.DrawPolygon(vertices, this.m_countB, colorB);
        }

        for (let i = 0; i < this.m_countB; ++i) {
            b2Transform.MultiplyVec2(transformB2, this.m_vBs[i], vertices[i]);
        }

        if (this.m_countB === 1) {
            draw.DrawCircle(vertices[0], this.m_radiusB, colorB2);
        } else {
            draw.DrawPolygon(vertices, this.m_countB, colorB2);
        }

        if (hit) {
            const p1 = output.point;
            draw.DrawPoint(p1, 10, colorHit);
            const p2 = b2Vec2.Add(p1, output.normal, b2Vec2.s_t0);
            draw.DrawSegment(p1, p2, colorHit);
        }
    }
}

registerTest("Collision", "Shape Cast", ShapeCast);
