// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// DEBUG: import { assert } from "../common/b2_common";
import { EPSILON, EPSILON_SQUARED, POLYGON_RADIUS, LINEAR_SLOP, makeArray } from "../common/b2_common";
import { Vec2, Rot, Transform } from "../common/b2_math";
import type { Shape } from "./b2_shape";

/**
 * A distance proxy is used by the GJK algorithm.
 * It encapsulates any shape.
 */
export class DistanceProxy {
    public readonly buffer = makeArray(2, Vec2);

    public vertices = this.buffer;

    public count = 0;

    public radius = 0;

    public copy(other: Readonly<DistanceProxy>) {
        if (other.vertices === other.buffer) {
            this.vertices = this.buffer;
            this.buffer[0].copy(other.buffer[0]);
            this.buffer[1].copy(other.buffer[1]);
        } else {
            this.vertices = other.vertices;
        }
        this.count = other.count;
        this.radius = other.radius;
        return this;
    }

    public reset(): DistanceProxy {
        this.vertices = this.buffer;
        this.count = 0;
        this.radius = 0;
        return this;
    }

    public setShape(shape: Shape, index: number): void {
        shape.setupDistanceProxy(this, index);
    }

    public setVerticesRadius(vertices: Vec2[], count: number, radius: number): void {
        this.vertices = vertices;
        this.count = count;
        this.radius = radius;
    }

    public getSupport(d: Vec2): number {
        let bestIndex = 0;
        let bestValue = Vec2.dot(this.vertices[0], d);
        for (let i = 1; i < this.count; ++i) {
            const value = Vec2.dot(this.vertices[i], d);
            if (value > bestValue) {
                bestIndex = i;
                bestValue = value;
            }
        }

        return bestIndex;
    }

    public getSupportVertex(d: Vec2): Vec2 {
        let bestIndex = 0;
        let bestValue = Vec2.dot(this.vertices[0], d);
        for (let i = 1; i < this.count; ++i) {
            const value = Vec2.dot(this.vertices[i], d);
            if (value > bestValue) {
                bestIndex = i;
                bestValue = value;
            }
        }

        return this.vertices[bestIndex];
    }

    public getVertexCount(): number {
        return this.count;
    }

    public getVertex(index: number): Vec2 {
        // DEBUG: assert(0 <= index && index < this.count);
        return this.vertices[index];
    }
}

/**
 * Used to warm start Distance.
 * Set count to zero on first call.
 */
export class SimplexCache {
    /** Length or area */
    public metric = 0;

    public count = 0;

    /** Vertices on shape A */
    public readonly indexA: [number, number, number] = [0, 0, 0];

    /** Vertices on shape B */
    public readonly indexB: [number, number, number] = [0, 0, 0];

    public reset(): SimplexCache {
        this.metric = 0;
        this.count = 0;
        return this;
    }
}

/**
 * Input for Distance.
 * You have to option to use the shape radii
 * in the computation. Even
 */
export class DistanceInput {
    public readonly proxyA = new DistanceProxy();

    public readonly proxyB = new DistanceProxy();

    public readonly transformA = new Transform();

    public readonly transformB = new Transform();

    public useRadii = false;

    public reset(): DistanceInput {
        this.proxyA.reset();
        this.proxyB.reset();
        this.transformA.setIdentity();
        this.transformB.setIdentity();
        this.useRadii = false;
        return this;
    }
}

/**
 * Output for Distance.
 */
export class DistanceOutput {
    /** Closest point on shapeA */
    public readonly pointA = new Vec2();

    /** Closest point on shapeB */
    public readonly pointB = new Vec2();

    public distance = 0;

    /** Number of GJK iterations used */
    public iterations = 0;

    public reset(): DistanceOutput {
        this.pointA.setZero();
        this.pointB.setZero();
        this.distance = 0;
        this.iterations = 0;
        return this;
    }
}

/**
 * Input parameters for ShapeCast
 */
export class ShapeCastInput {
    public readonly proxyA = new DistanceProxy();

    public readonly proxyB = new DistanceProxy();

    public readonly transformA = new Transform();

    public readonly transformB = new Transform();

    public readonly translationB = new Vec2();
}

/**
 * Output results for ShapeCast
 */
export class ShapeCastOutput {
    public readonly point = new Vec2();

    public readonly normal = new Vec2();

    public lambda = 0;

    public iterations = 0;
}

export const Gjk = {
    calls: 0,
    iters: 0,
    maxIters: 0,
    reset() {
        this.calls = 0;
        this.iters = 0;
        this.maxIters = 0;
    },
};

class SimplexVertex {
    public readonly wA = new Vec2(); // support point in proxyA

    public readonly wB = new Vec2(); // support point in proxyB

    public readonly w = new Vec2(); // wB - wA

    public a = 0; // barycentric coordinate for closest point

    public indexA = 0; // wA index

    public indexB = 0; // wB index

    public copy(other: SimplexVertex): SimplexVertex {
        this.wA.copy(other.wA); // support point in proxyA
        this.wB.copy(other.wB); // support point in proxyB
        this.w.copy(other.w); // wB - wA
        this.a = other.a; // barycentric coordinate for closest point
        this.indexA = other.indexA; // wA index
        this.indexB = other.indexB; // wB index
        return this;
    }
}

class Simplex {
    public readonly v1 = new SimplexVertex();

    public readonly v2 = new SimplexVertex();

    public readonly v3 = new SimplexVertex();

    public readonly vertices: [SimplexVertex, SimplexVertex, SimplexVertex];

    public count = 0;

    public constructor() {
        this.vertices = [this.v1, this.v2, this.v3];
    }

    public readCache(
        cache: SimplexCache,
        proxyA: DistanceProxy,
        transformA: Transform,
        proxyB: DistanceProxy,
        transformB: Transform,
    ): void {
        // DEBUG: assert(cache.count <= 3);

        // Copy data from cache.
        this.count = cache.count;
        const { vertices } = this;
        for (let i = 0; i < this.count; ++i) {
            const v = vertices[i];
            v.indexA = cache.indexA[i];
            v.indexB = cache.indexB[i];
            const wALocal = proxyA.getVertex(v.indexA);
            const wBLocal = proxyB.getVertex(v.indexB);
            Transform.multiplyVec2(transformA, wALocal, v.wA);
            Transform.multiplyVec2(transformB, wBLocal, v.wB);
            Vec2.subtract(v.wB, v.wA, v.w);
            v.a = 0;
        }

        // Compute the new simplex metric, if it is substantially different than
        // old metric then flush the simplex.
        if (this.count > 1) {
            const metric1 = cache.metric;
            const metric2 = this.getMetric();
            if (metric2 < 0.5 * metric1 || 2 * metric1 < metric2 || metric2 < EPSILON) {
                // Reset the simplex.
                this.count = 0;
            }
        }

        // If the cache is empty or invalid ...
        if (this.count === 0) {
            const v = vertices[0];
            v.indexA = 0;
            v.indexB = 0;
            const wALocal = proxyA.getVertex(0);
            const wBLocal = proxyB.getVertex(0);
            Transform.multiplyVec2(transformA, wALocal, v.wA);
            Transform.multiplyVec2(transformB, wBLocal, v.wB);
            Vec2.subtract(v.wB, v.wA, v.w);
            v.a = 1;
            this.count = 1;
        }
    }

    public writeCache(cache: SimplexCache): void {
        cache.metric = this.getMetric();
        cache.count = this.count;
        const { vertices } = this;
        for (let i = 0; i < this.count; ++i) {
            cache.indexA[i] = vertices[i].indexA;
            cache.indexB[i] = vertices[i].indexB;
        }
    }

    public getSearchDirection(out: Vec2): Vec2 {
        switch (this.count) {
            case 1:
                return Vec2.negate(this.v1.w, out);

            case 2: {
                const e12 = Vec2.subtract(this.v2.w, this.v1.w, out);
                const sgn = Vec2.cross(e12, Vec2.negate(this.v1.w, Vec2.s_t0));
                if (sgn > 0) {
                    // Origin is left of e12.
                    return Vec2.crossOneVec2(e12, out);
                }
                // Origin is right of e12.
                return Vec2.crossVec2One(e12, out);
            }

            default:
                // DEBUG: assert(false);
                return out.setZero();
        }
    }

    public getClosestPoint(out: Vec2): Vec2 {
        switch (this.count) {
            case 0:
                // DEBUG: assert(false);
                return out.setZero();

            case 1:
                return out.copy(this.v1.w);

            case 2:
                return out.set(
                    this.v1.a * this.v1.w.x + this.v2.a * this.v2.w.x,
                    this.v1.a * this.v1.w.y + this.v2.a * this.v2.w.y,
                );

            case 3:
                return out.setZero();

            default:
                // DEBUG: assert(false);
                return out.setZero();
        }
    }

    public getWitnessPoints(pA: Vec2, pB: Vec2): void {
        switch (this.count) {
            case 0:
                // DEBUG: assert(false);
                break;

            case 1:
                pA.copy(this.v1.wA);
                pB.copy(this.v1.wB);
                break;

            case 2:
                pA.x = this.v1.a * this.v1.wA.x + this.v2.a * this.v2.wA.x;
                pA.y = this.v1.a * this.v1.wA.y + this.v2.a * this.v2.wA.y;
                pB.x = this.v1.a * this.v1.wB.x + this.v2.a * this.v2.wB.x;
                pB.y = this.v1.a * this.v1.wB.y + this.v2.a * this.v2.wB.y;
                break;

            case 3:
                pB.x = pA.x = this.v1.a * this.v1.wA.x + this.v2.a * this.v2.wA.x + this.v3.a * this.v3.wA.x;
                pB.y = pA.y = this.v1.a * this.v1.wA.y + this.v2.a * this.v2.wA.y + this.v3.a * this.v3.wA.y;
                break;

            default:
                // DEBUG: assert(false);
                break;
        }
    }

    public getMetric(): number {
        switch (this.count) {
            case 0:
                // DEBUG: assert(false);
                return 0;

            case 1:
                return 0;

            case 2:
                return Vec2.distance(this.v1.w, this.v2.w);

            case 3:
                return Vec2.cross(
                    Vec2.subtract(this.v2.w, this.v1.w, Vec2.s_t0),
                    Vec2.subtract(this.v3.w, this.v1.w, Vec2.s_t1),
                );

            default:
                // DEBUG: assert(false);
                return 0;
        }
    }

    public solve2(): void {
        const w1 = this.v1.w;
        const w2 = this.v2.w;
        const e12 = Vec2.subtract(w2, w1, Simplex.s_e12);

        // w1 region
        const d12_2 = -Vec2.dot(w1, e12);
        if (d12_2 <= 0) {
            // a2 <= 0, so we clamp it to 0
            this.v1.a = 1;
            this.count = 1;
            return;
        }

        // w2 region
        const d12_1 = Vec2.dot(w2, e12);
        if (d12_1 <= 0) {
            // a1 <= 0, so we clamp it to 0
            this.v2.a = 1;
            this.count = 1;
            this.v1.copy(this.v2);
            return;
        }

        // Must be in e12 region.
        const inv_d12 = 1 / (d12_1 + d12_2);
        this.v1.a = d12_1 * inv_d12;
        this.v2.a = d12_2 * inv_d12;
        this.count = 2;
    }

    public solve3(): void {
        const w1 = this.v1.w;
        const w2 = this.v2.w;
        const w3 = this.v3.w;

        // Edge12
        // [1      1     ][a1] = [1]
        // [w1.e12 w2.e12][a2] = [0]
        // a3 = 0
        const e12 = Vec2.subtract(w2, w1, Simplex.s_e12);
        const w1e12 = Vec2.dot(w1, e12);
        const w2e12 = Vec2.dot(w2, e12);
        const d12_1 = w2e12;
        const d12_2 = -w1e12;

        // Edge13
        // [1      1     ][a1] = [1]
        // [w1.e13 w3.e13][a3] = [0]
        // a2 = 0
        const e13 = Vec2.subtract(w3, w1, Simplex.s_e13);
        const w1e13 = Vec2.dot(w1, e13);
        const w3e13 = Vec2.dot(w3, e13);
        const d13_1 = w3e13;
        const d13_2 = -w1e13;

        // Edge23
        // [1      1     ][a2] = [1]
        // [w2.e23 w3.e23][a3] = [0]
        // a1 = 0
        const e23 = Vec2.subtract(w3, w2, Simplex.s_e23);
        const w2e23 = Vec2.dot(w2, e23);
        const w3e23 = Vec2.dot(w3, e23);
        const d23_1 = w3e23;
        const d23_2 = -w2e23;

        // Triangle123
        const n123 = Vec2.cross(e12, e13);

        const d123_1 = n123 * Vec2.cross(w2, w3);
        const d123_2 = n123 * Vec2.cross(w3, w1);
        const d123_3 = n123 * Vec2.cross(w1, w2);

        // w1 region
        if (d12_2 <= 0 && d13_2 <= 0) {
            this.v1.a = 1;
            this.count = 1;
            return;
        }

        // e12
        if (d12_1 > 0 && d12_2 > 0 && d123_3 <= 0) {
            const inv_d12 = 1 / (d12_1 + d12_2);
            this.v1.a = d12_1 * inv_d12;
            this.v2.a = d12_2 * inv_d12;
            this.count = 2;
            return;
        }

        // e13
        if (d13_1 > 0 && d13_2 > 0 && d123_2 <= 0) {
            const inv_d13 = 1 / (d13_1 + d13_2);
            this.v1.a = d13_1 * inv_d13;
            this.v3.a = d13_2 * inv_d13;
            this.count = 2;
            this.v2.copy(this.v3);
            return;
        }

        // w2 region
        if (d12_1 <= 0 && d23_2 <= 0) {
            this.v2.a = 1;
            this.count = 1;
            this.v1.copy(this.v2);
            return;
        }

        // w3 region
        if (d13_1 <= 0 && d23_1 <= 0) {
            this.v3.a = 1;
            this.count = 1;
            this.v1.copy(this.v3);
            return;
        }

        // e23
        if (d23_1 > 0 && d23_2 > 0 && d123_1 <= 0) {
            const inv_d23 = 1 / (d23_1 + d23_2);
            this.v2.a = d23_1 * inv_d23;
            this.v3.a = d23_2 * inv_d23;
            this.count = 2;
            this.v1.copy(this.v3);
            return;
        }

        // Must be in triangle123
        const inv_d123 = 1 / (d123_1 + d123_2 + d123_3);
        this.v1.a = d123_1 * inv_d123;
        this.v2.a = d123_2 * inv_d123;
        this.v3.a = d123_3 * inv_d123;
        this.count = 3;
    }

    private static s_e12 = new Vec2();

    private static s_e13 = new Vec2();

    private static s_e23 = new Vec2();
}

const Distance_s_simplex = new Simplex();
const Distance_s_saveA: [number, number, number] = [0, 0, 0];
const Distance_s_saveB: [number, number, number] = [0, 0, 0];
const Distance_s_p = new Vec2();
const Distance_s_d = new Vec2();
const Distance_s_normal = new Vec2();
const Distance_s_supportA = new Vec2();
const Distance_s_supportB = new Vec2();
export function distance(output: DistanceOutput, cache: SimplexCache, input: DistanceInput): void {
    ++Gjk.calls;

    const { proxyA, proxyB, transformA, transformB } = input;

    // Initialize the simplex.
    const simplex = Distance_s_simplex;
    simplex.readCache(cache, proxyA, transformA, proxyB, transformB);

    // Get simplex vertices as an array.
    const { vertices } = simplex;
    const k_maxIters = 20;

    // These store the vertices of the last simplex so that we
    // can check for duplicates and prevent cycling.
    const saveA = Distance_s_saveA;
    const saveB = Distance_s_saveB;
    let saveCount = 0;

    // Main iteration loop.
    let iter = 0;
    while (iter < k_maxIters) {
        // Copy simplex so we can identify duplicates.
        saveCount = simplex.count;
        for (let i = 0; i < saveCount; ++i) {
            saveA[i] = vertices[i].indexA;
            saveB[i] = vertices[i].indexB;
        }

        switch (simplex.count) {
            case 1:
                break;

            case 2:
                simplex.solve2();
                break;

            case 3:
                simplex.solve3();
                break;

            // DEBUG: default:
            // DEBUG: assert(false);
        }

        // If we have 3 points, then the origin is in the corresponding triangle.
        if (simplex.count === 3) {
            break;
        }

        // Get search direction.
        const d = simplex.getSearchDirection(Distance_s_d);

        // Ensure the search direction is numerically fit.
        if (d.lengthSquared() < EPSILON_SQUARED) {
            // The origin is probably contained by a line segment
            // or triangle. Thus the shapes are overlapped.

            // We can't return zero here even though there may be overlap.
            // In case the simplex is a point, segment, or triangle it is difficult
            // to determine if the origin is contained in the CSO or very close to it.
            break;
        }

        // Compute a tentative new simplex vertex using support points.
        const vertex = vertices[simplex.count];
        vertex.indexA = proxyA.getSupport(
            Rot.transposeMultiplyVec2(transformA.q, Vec2.negate(d, Vec2.s_t0), Distance_s_supportA),
        );
        Transform.multiplyVec2(transformA, proxyA.getVertex(vertex.indexA), vertex.wA);
        vertex.indexB = proxyB.getSupport(Rot.transposeMultiplyVec2(transformB.q, d, Distance_s_supportB));
        Transform.multiplyVec2(transformB, proxyB.getVertex(vertex.indexB), vertex.wB);
        Vec2.subtract(vertex.wB, vertex.wA, vertex.w);

        // Iteration count is equated to the number of support point calls.
        ++iter;
        ++Gjk.iters;

        // Check for duplicate support points. This is the main termination criteria.
        let duplicate = false;
        for (let i = 0; i < saveCount; ++i) {
            if (vertex.indexA === saveA[i] && vertex.indexB === saveB[i]) {
                duplicate = true;
                break;
            }
        }

        // If we found a duplicate support point we must exit to avoid cycling.
        if (duplicate) {
            break;
        }

        // New vertex is ok and needed.
        ++simplex.count;
    }

    Gjk.maxIters = Math.max(Gjk.maxIters, iter);

    // Prepare output.
    simplex.getWitnessPoints(output.pointA, output.pointB);
    output.distance = Vec2.distance(output.pointA, output.pointB);
    output.iterations = iter;

    // Cache the simplex.
    simplex.writeCache(cache);

    // Apply radii if requested.
    if (input.useRadii) {
        const rA = proxyA.radius;
        const rB = proxyB.radius;

        if (output.distance > rA + rB && output.distance > EPSILON) {
            // Shapes are still no overlapped.
            // Move the witness points to the outer surface.
            output.distance -= rA + rB;
            const normal = Vec2.subtract(output.pointB, output.pointA, Distance_s_normal);
            normal.normalize();
            output.pointA.addScaled(rA, normal);
            output.pointB.subtractScaled(rB, normal);
        } else {
            // Shapes are overlapped when radii are considered.
            // Move the witness points to the middle.
            const p = Vec2.mid(output.pointA, output.pointB, Distance_s_p);
            output.pointA.copy(p);
            output.pointB.copy(p);
            output.distance = 0;
        }
    }
}

const ShapeCast_s_n = new Vec2();
const ShapeCast_s_simplex = new Simplex();
const ShapeCast_s_wA = new Vec2();
const ShapeCast_s_wB = new Vec2();
const ShapeCast_s_v = new Vec2();
const ShapeCast_s_p = new Vec2();
const ShapeCast_s_pointA = new Vec2();
const ShapeCast_s_pointB = new Vec2();

/**
 * Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
 * GJK-raycast
 * Algorithm by Gino van den Bergen.
 * "Smooth Mesh Contacts with GJK" in Game Physics Pearls. 2010
 */
export function shapeCast(output: ShapeCastOutput, input: ShapeCastInput): boolean {
    output.iterations = 0;
    output.lambda = 1;
    output.normal.setZero();
    output.point.setZero();

    const { proxyA, proxyB } = input;

    const radiusA = Math.max(proxyA.radius, POLYGON_RADIUS);
    const radiusB = Math.max(proxyB.radius, POLYGON_RADIUS);
    const radius = radiusA + radiusB;

    const xfA = input.transformA;
    const xfB = input.transformB;

    const r = input.translationB;
    const n = ShapeCast_s_n.setZero();
    let lambda = 0;

    // Initial simplex
    const simplex = ShapeCast_s_simplex;
    simplex.count = 0;

    // Get simplex vertices as an array.
    // SimplexVertex* vertices = &simplex.v1;
    const { vertices } = simplex;

    // Get support point in -r direction
    let indexA = proxyA.getSupport(Rot.transposeMultiplyVec2(xfA.q, Vec2.negate(r, Vec2.s_t1), Vec2.s_t0));
    let wA = Transform.multiplyVec2(xfA, proxyA.getVertex(indexA), ShapeCast_s_wA);
    let indexB = proxyB.getSupport(Rot.transposeMultiplyVec2(xfB.q, r, Vec2.s_t0));
    let wB = Transform.multiplyVec2(xfB, proxyB.getVertex(indexB), ShapeCast_s_wB);
    const v = Vec2.subtract(wA, wB, ShapeCast_s_v);

    // Sigma is the target distance between polygons
    const sigma = Math.max(POLYGON_RADIUS, radius - POLYGON_RADIUS);
    const tolerance = 0.5 * LINEAR_SLOP;

    // Main iteration loop.
    const k_maxIters = 20;
    let iter = 0;
    while (iter < k_maxIters && v.length() - sigma > tolerance) {
        // DEBUG: assert(simplex.count < 3);

        output.iterations += 1;

        // Support in direction -v (A - B)
        indexA = proxyA.getSupport(Rot.transposeMultiplyVec2(xfA.q, Vec2.negate(v, Vec2.s_t1), Vec2.s_t0));
        wA = Transform.multiplyVec2(xfA, proxyA.getVertex(indexA), ShapeCast_s_wA);
        indexB = proxyB.getSupport(Rot.transposeMultiplyVec2(xfB.q, v, Vec2.s_t0));
        wB = Transform.multiplyVec2(xfB, proxyB.getVertex(indexB), ShapeCast_s_wB);
        const p = Vec2.subtract(wA, wB, ShapeCast_s_p);

        // -v is a normal at p
        v.normalize();

        // Intersect ray with plane
        const vp = Vec2.dot(v, p);
        const vr = Vec2.dot(v, r);
        if (vp - sigma > lambda * vr) {
            if (vr <= 0) {
                return false;
            }

            lambda = (vp - sigma) / vr;
            if (lambda > 1) {
                return false;
            }

            Vec2.negate(v, n);
            simplex.count = 0;
        }

        // Reverse simplex since it works with B - A.
        // Shift by lambda * r because we want the closest point to the current clip point.
        // Note that the support point p is not shifted because we want the plane equation
        // to be formed in unshifted space.
        const vertex = vertices[simplex.count];
        vertex.indexA = indexB;
        Vec2.addScaled(wB, lambda, r, vertex.wA);
        vertex.indexB = indexA;
        vertex.wB.copy(wA);
        Vec2.subtract(vertex.wB, vertex.wA, vertex.w);
        vertex.a = 1;
        simplex.count += 1;

        switch (simplex.count) {
            case 1:
                break;

            case 2:
                simplex.solve2();
                break;

            case 3:
                simplex.solve3();
                break;

            // DEBUG: default:
            // DEBUG: assert(false);
        }

        // If we have 3 points, then the origin is in the corresponding triangle.
        if (simplex.count === 3) {
            // Overlap
            return false;
        }

        // Get search direction.
        simplex.getClosestPoint(v);

        // Iteration count is equated to the number of support point calls.
        ++iter;
    }

    if (iter === 0) {
        // Initial overlap
        return false;
    }

    // Prepare output.
    const pointA = ShapeCast_s_pointA;
    const pointB = ShapeCast_s_pointB;
    simplex.getWitnessPoints(pointA, pointB);

    if (v.lengthSquared() > 0) {
        Vec2.negate(v, n);
        n.normalize();
    }

    Vec2.addScaled(pointA, radiusA, n, output.point);
    output.normal.copy(n);
    output.lambda = lambda;
    output.iterations = iter;
    return true;
}
