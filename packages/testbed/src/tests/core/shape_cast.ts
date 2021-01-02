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
    Vec2,
    POLYGON_RADIUS,
    Transform,
    ShapeCastInput,
    ShapeCastOutput,
    shapeCast,
    DistanceInput,
    SimplexCache,
    DistanceOutput,
    distance,
    Color,
    makeArray,
    MAX_POLYGON_VERTICES,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { g_debugDraw } from "../../utils/draw";

const colorA = new Color(0.9, 0.9, 0.9);
const colorB = new Color(0.5, 0.9, 0.5);
const colorB2 = new Color(0.5, 0.7, 0.9);
const colorHit = new Color(0.9, 0.3, 0.3);

class ShapeCastTest extends Test {
    public static e_vertexCount = 8;

    public vAs = makeArray(MAX_POLYGON_VERTICES, Vec2);

    public countA = 0;

    public radiusA = 0;

    public vBs = makeArray(MAX_POLYGON_VERTICES, Vec2);

    public countB = 0;

    public radiusB = 0;

    public transformA = new Transform();

    public transformB = new Transform();

    public translationB = new Vec2();

    public constructor() {
        super();

        // #if 1
        this.vAs[0].set(-0.5, 1);
        this.vAs[1].set(0.5, 1);
        this.vAs[2].set(0, 0);
        this.countA = 3;
        this.radiusA = POLYGON_RADIUS;

        this.vBs[0].set(-0.5, -0.5);
        this.vBs[1].set(0.5, -0.5);
        this.vBs[2].set(0.5, 0.5);
        this.vBs[3].set(-0.5, 0.5);
        this.countB = 4;
        this.radiusB = POLYGON_RADIUS;

        this.transformA.p.set(0, 0.25);
        this.transformA.q.setIdentity();
        this.transformB.p.set(-4, 0);
        this.transformB.q.setIdentity();
        this.translationB.set(8, 0);
        // #elif 0
        // this.vAs[0].set(0, 0);
        // this.countA = 1;
        // this.radiusA = 0.5;

        // this.vBs[0].set(0, 0);
        // this.countB = 1;
        // this.radiusB = 0.5;

        // this.transformA.p.set(0, 0.25);
        // this.transformA.q.setIdentity();
        // this.transformB.p.set(-4, 0);
        // this.transformB.q.setIdentity();
        // this.translationB.set(8, 0);
        // #else
        // this.vAs[0].set(0, 0);
        // this.vAs[1].set(2, 0);
        // this.countA = 2;
        // this.radiusA = POLYGON_RADIUS;

        // this.vBs[0].set(0, 0);
        // this.countB = 1;
        // this.radiusB = 0.25;

        // // Initial overlap
        // this.transformA.p.set(0, 0);
        // this.transformA.q.setIdentity();
        // this.transformB.p.set(-0.244360745, 0.05999358);
        // this.transformB.q.setIdentity();
        // this.translationB.set(0, 0.0399999991);
        // #endif
    }

    public step(settings: Settings, timeStep: number): void {
        super.step(settings, timeStep);

        const input = new ShapeCastInput();
        input.proxyA.setVerticesRadius(this.vAs, this.countA, this.radiusA);
        input.proxyB.setVerticesRadius(this.vBs, this.countB, this.radiusB);
        input.transformA.copy(this.transformA);
        input.transformB.copy(this.transformB);
        input.translationB.copy(this.translationB);

        const output = new ShapeCastOutput();
        const hit = shapeCast(output, input);

        const transformB2 = new Transform();
        transformB2.q.copy(this.transformB.q);
        transformB2.p.copy(this.transformB.p).addScaled(output.lambda, input.translationB);

        const distanceInput = new DistanceInput();
        distanceInput.proxyA.setVerticesRadius(this.vAs, this.countA, this.radiusA);
        distanceInput.proxyB.setVerticesRadius(this.vBs, this.countB, this.radiusB);
        distanceInput.transformA.copy(this.transformA);
        distanceInput.transformB.copy(transformB2);
        distanceInput.useRadii = false;
        const simplexCache = new SimplexCache();
        simplexCache.count = 0;
        const distanceOutput = new DistanceOutput();

        distance(distanceOutput, simplexCache, distanceInput);

        this.addDebug("Hit", hit);
        this.addDebug("Iters", output.iterations);
        this.addDebug("Lambda", output.lambda.toFixed(6));
        this.addDebug("distance", distanceOutput.distance.toFixed(6));

        const vertices = makeArray(MAX_POLYGON_VERTICES, Vec2);

        for (let i = 0; i < this.countA; ++i) {
            Transform.multiplyVec2(this.transformA, this.vAs[i], vertices[i]);
        }

        if (this.countA === 1) {
            g_debugDraw.drawCircle(vertices[0], this.radiusA, colorA);
        } else {
            g_debugDraw.drawPolygon(vertices, this.countA, colorA);
        }

        for (let i = 0; i < this.countB; ++i) {
            Transform.multiplyVec2(this.transformB, this.vBs[i], vertices[i]);
        }

        if (this.countB === 1) {
            g_debugDraw.drawCircle(vertices[0], this.radiusB, colorB);
        } else {
            g_debugDraw.drawPolygon(vertices, this.countB, colorB);
        }

        for (let i = 0; i < this.countB; ++i) {
            Transform.multiplyVec2(transformB2, this.vBs[i], vertices[i]);
        }

        if (this.countB === 1) {
            g_debugDraw.drawCircle(vertices[0], this.radiusB, colorB2);
        } else {
            g_debugDraw.drawPolygon(vertices, this.countB, colorB2);
        }

        if (hit) {
            const p1 = output.point;
            g_debugDraw.drawPoint(p1, 10, colorHit);
            const p2 = Vec2.add(p1, output.normal, Vec2.s_t0);
            g_debugDraw.drawSegment(p1, p2, colorHit);
        }
    }
}

registerTest("Collision", "Shape Cast", ShapeCastTest);
