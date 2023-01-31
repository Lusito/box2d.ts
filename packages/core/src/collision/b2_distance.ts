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

// DEBUG: import { b2Assert } from "../common/b2_common";
import { b2_epsilon, b2_epsilon_sq, b2_polygonRadius, b2_linearSlop, b2MakeArray } from "../common/b2_common";
import { b2Vec2, b2Rot, b2Transform } from "../common/b2_math";
import type { b2Shape } from "./b2_shape";

/**
 * A distance proxy is used by the GJK algorithm.
 * It encapsulates any shape.
 */
export class b2DistanceProxy {
    public readonly m_buffer = b2MakeArray(2, b2Vec2);

    public m_vertices = this.m_buffer;

    public m_count = 0;

    public m_radius = 0;

    public Copy(other: Readonly<b2DistanceProxy>) {
        if (other.m_vertices === other.m_buffer) {
            this.m_vertices = this.m_buffer;
            // eslint-disable-next-line prefer-destructuring
            this.m_buffer[0] = other.m_buffer[0];
            // eslint-disable-next-line prefer-destructuring
            this.m_buffer[1] = other.m_buffer[1];
        } else {
            this.m_vertices = other.m_vertices;
        }
        this.m_count = other.m_count;
        this.m_radius = other.m_radius;
        return this;
    }

    public Reset(): b2DistanceProxy {
        this.m_vertices = this.m_buffer;
        this.m_count = 0;
        this.m_radius = 0;
        return this;
    }

    public SetShape(shape: b2Shape, index: number): void {
        shape.SetupDistanceProxy(this, index);
    }

    public SetVerticesRadius(vertices: b2Vec2[], count: number, radius: number): void {
        this.m_vertices = vertices;
        this.m_count = count;
        this.m_radius = radius;
    }

    public GetSupport(d: b2Vec2): number {
        let bestIndex = 0;
        let bestValue = b2Vec2.Dot(this.m_vertices[0], d);
        for (let i = 1; i < this.m_count; ++i) {
            const value = b2Vec2.Dot(this.m_vertices[i], d);
            if (value > bestValue) {
                bestIndex = i;
                bestValue = value;
            }
        }

        return bestIndex;
    }

    public GetSupportVertex(d: b2Vec2): b2Vec2 {
        let bestIndex = 0;
        let bestValue = b2Vec2.Dot(this.m_vertices[0], d);
        for (let i = 1; i < this.m_count; ++i) {
            const value = b2Vec2.Dot(this.m_vertices[i], d);
            if (value > bestValue) {
                bestIndex = i;
                bestValue = value;
            }
        }

        return this.m_vertices[bestIndex];
    }

    public GetVertexCount(): number {
        return this.m_count;
    }

    public GetVertex(index: number): b2Vec2 {
        // DEBUG: b2Assert(0 <= index && index < this.m_count);
        return this.m_vertices[index];
    }
}

/**
 * Used to warm start b2Distance.
 * Set count to zero on first call.
 */
export class b2SimplexCache {
    /** Length or area */
    public metric = 0;

    public count = 0;

    /** Vertices on shape A */
    public readonly indexA: [number, number, number] = [0, 0, 0];

    /** Vertices on shape B */
    public readonly indexB: [number, number, number] = [0, 0, 0];

    public Reset(): b2SimplexCache {
        this.metric = 0;
        this.count = 0;
        return this;
    }
}

/**
 * Input for b2Distance.
 * You have to option to use the shape radii
 * in the computation. Even
 */
export class b2DistanceInput {
    public readonly proxyA = new b2DistanceProxy();

    public readonly proxyB = new b2DistanceProxy();

    public readonly transformA = new b2Transform();

    public readonly transformB = new b2Transform();

    public useRadii = false;

    public Reset(): b2DistanceInput {
        this.proxyA.Reset();
        this.proxyB.Reset();
        this.transformA.SetIdentity();
        this.transformB.SetIdentity();
        this.useRadii = false;
        return this;
    }
}

/**
 * Output for b2Distance.
 */
export class b2DistanceOutput {
    /** Closest point on shapeA */
    public readonly pointA = new b2Vec2();

    /** Closest point on shapeB */
    public readonly pointB = new b2Vec2();

    public distance = 0;

    /** Number of GJK iterations used */
    public iterations = 0;

    public Reset(): b2DistanceOutput {
        this.pointA.SetZero();
        this.pointB.SetZero();
        this.distance = 0;
        this.iterations = 0;
        return this;
    }
}

/**
 * Input parameters for b2ShapeCast
 */
export class b2ShapeCastInput {
    public readonly proxyA = new b2DistanceProxy();

    public readonly proxyB = new b2DistanceProxy();

    public readonly transformA = new b2Transform();

    public readonly transformB = new b2Transform();

    public readonly translationB = new b2Vec2();
}

/**
 * Output results for b2ShapeCast
 */
export class b2ShapeCastOutput {
    public readonly point = new b2Vec2();

    public readonly normal = new b2Vec2();

    public lambda = 0;

    public iterations = 0;
}

export const b2Gjk = {
    calls: 0,
    iters: 0,
    maxIters: 0,
    reset() {
        this.calls = 0;
        this.iters = 0;
        this.maxIters = 0;
    },
};

class b2SimplexVertex {
    public readonly wA = new b2Vec2(); // support point in proxyA

    public readonly wB = new b2Vec2(); // support point in proxyB

    public readonly w = new b2Vec2(); // wB - wA

    public a = 0; // barycentric coordinate for closest point

    public indexA = 0; // wA index

    public indexB = 0; // wB index

    public Copy(other: b2SimplexVertex): b2SimplexVertex {
        this.wA.Copy(other.wA); // support point in proxyA
        this.wB.Copy(other.wB); // support point in proxyB
        this.w.Copy(other.w); // wB - wA
        this.a = other.a; // barycentric coordinate for closest point
        this.indexA = other.indexA; // wA index
        this.indexB = other.indexB; // wB index
        return this;
    }
}

class b2Simplex {
    public readonly m_v1 = new b2SimplexVertex();

    public readonly m_v2 = new b2SimplexVertex();

    public readonly m_v3 = new b2SimplexVertex();

    public readonly m_vertices: [b2SimplexVertex, b2SimplexVertex, b2SimplexVertex];

    public m_count = 0;

    public constructor() {
        this.m_vertices = [this.m_v1, this.m_v2, this.m_v3];
    }

    public ReadCache(
        cache: b2SimplexCache,
        proxyA: b2DistanceProxy,
        transformA: b2Transform,
        proxyB: b2DistanceProxy,
        transformB: b2Transform,
    ): void {
        // DEBUG: b2Assert(cache.count <= 3);

        // Copy data from cache.
        this.m_count = cache.count;
        const vertices = this.m_vertices;
        for (let i = 0; i < this.m_count; ++i) {
            const v = vertices[i];
            v.indexA = cache.indexA[i];
            v.indexB = cache.indexB[i];
            const wALocal = proxyA.GetVertex(v.indexA);
            const wBLocal = proxyB.GetVertex(v.indexB);
            b2Transform.MultiplyVec2(transformA, wALocal, v.wA);
            b2Transform.MultiplyVec2(transformB, wBLocal, v.wB);
            b2Vec2.Subtract(v.wB, v.wA, v.w);
            v.a = 0;
        }

        // Compute the new simplex metric, if it is substantially different than
        // old metric then flush the simplex.
        if (this.m_count > 1) {
            const metric1 = cache.metric;
            const metric2 = this.GetMetric();
            if (metric2 < 0.5 * metric1 || 2 * metric1 < metric2 || metric2 < b2_epsilon) {
                // Reset the simplex.
                this.m_count = 0;
            }
        }

        // If the cache is empty or invalid ...
        if (this.m_count === 0) {
            const v = vertices[0];
            v.indexA = 0;
            v.indexB = 0;
            const wALocal = proxyA.GetVertex(0);
            const wBLocal = proxyB.GetVertex(0);
            b2Transform.MultiplyVec2(transformA, wALocal, v.wA);
            b2Transform.MultiplyVec2(transformB, wBLocal, v.wB);
            b2Vec2.Subtract(v.wB, v.wA, v.w);
            v.a = 1;
            this.m_count = 1;
        }
    }

    public WriteCache(cache: b2SimplexCache): void {
        cache.metric = this.GetMetric();
        cache.count = this.m_count;
        const vertices = this.m_vertices;
        for (let i = 0; i < this.m_count; ++i) {
            cache.indexA[i] = vertices[i].indexA;
            cache.indexB[i] = vertices[i].indexB;
        }
    }

    public GetSearchDirection(out: b2Vec2): b2Vec2 {
        switch (this.m_count) {
            case 1:
                return b2Vec2.Negate(this.m_v1.w, out);

            case 2: {
                const e12 = b2Vec2.Subtract(this.m_v2.w, this.m_v1.w, out);
                const sgn = b2Vec2.Cross(e12, b2Vec2.Negate(this.m_v1.w, b2Vec2.s_t0));
                if (sgn > 0) {
                    // Origin is left of e12.
                    return b2Vec2.CrossOneVec2(e12, out);
                }
                // Origin is right of e12.
                return b2Vec2.CrossVec2One(e12, out);
            }

            default:
                // DEBUG: b2Assert(false);
                return out.SetZero();
        }
    }

    public GetClosestPoint(out: b2Vec2): b2Vec2 {
        switch (this.m_count) {
            case 0:
                // DEBUG: b2Assert(false);
                return out.SetZero();

            case 1:
                return out.Copy(this.m_v1.w);

            case 2:
                return out.Set(
                    this.m_v1.a * this.m_v1.w.x + this.m_v2.a * this.m_v2.w.x,
                    this.m_v1.a * this.m_v1.w.y + this.m_v2.a * this.m_v2.w.y,
                );

            case 3:
                return out.SetZero();

            default:
                // DEBUG: b2Assert(false);
                return out.SetZero();
        }
    }

    public GetWitnessPoints(pA: b2Vec2, pB: b2Vec2): void {
        switch (this.m_count) {
            case 0:
                // DEBUG: b2Assert(false);
                break;

            case 1:
                pA.Copy(this.m_v1.wA);
                pB.Copy(this.m_v1.wB);
                break;

            case 2:
                pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x;
                pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y;
                pB.x = this.m_v1.a * this.m_v1.wB.x + this.m_v2.a * this.m_v2.wB.x;
                pB.y = this.m_v1.a * this.m_v1.wB.y + this.m_v2.a * this.m_v2.wB.y;
                break;

            case 3:
                pB.x = pA.x =
                    this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x + this.m_v3.a * this.m_v3.wA.x;
                pB.y = pA.y =
                    this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y + this.m_v3.a * this.m_v3.wA.y;
                break;

            default:
                // DEBUG: b2Assert(false);
                break;
        }
    }

    public GetMetric(): number {
        switch (this.m_count) {
            case 0:
                // DEBUG: b2Assert(false);
                return 0;

            case 1:
                return 0;

            case 2:
                return b2Vec2.Distance(this.m_v1.w, this.m_v2.w);

            case 3:
                return b2Vec2.Cross(
                    b2Vec2.Subtract(this.m_v2.w, this.m_v1.w, b2Vec2.s_t0),
                    b2Vec2.Subtract(this.m_v3.w, this.m_v1.w, b2Vec2.s_t1),
                );

            default:
                // DEBUG: b2Assert(false);
                return 0;
        }
    }

    public Solve2(): void {
        const w1 = this.m_v1.w;
        const w2 = this.m_v2.w;
        const e12 = b2Vec2.Subtract(w2, w1, b2Simplex.s_e12);

        // w1 region
        const d12_2 = -b2Vec2.Dot(w1, e12);
        if (d12_2 <= 0) {
            // a2 <= 0, so we clamp it to 0
            this.m_v1.a = 1;
            this.m_count = 1;
            return;
        }

        // w2 region
        const d12_1 = b2Vec2.Dot(w2, e12);
        if (d12_1 <= 0) {
            // a1 <= 0, so we clamp it to 0
            this.m_v2.a = 1;
            this.m_count = 1;
            this.m_v1.Copy(this.m_v2);
            return;
        }

        // Must be in e12 region.
        const inv_d12 = 1 / (d12_1 + d12_2);
        this.m_v1.a = d12_1 * inv_d12;
        this.m_v2.a = d12_2 * inv_d12;
        this.m_count = 2;
    }

    public Solve3(): void {
        const w1 = this.m_v1.w;
        const w2 = this.m_v2.w;
        const w3 = this.m_v3.w;

        // Edge12
        // [1      1     ][a1] = [1]
        // [w1.e12 w2.e12][a2] = [0]
        // a3 = 0
        const e12 = b2Vec2.Subtract(w2, w1, b2Simplex.s_e12);
        const w1e12 = b2Vec2.Dot(w1, e12);
        const w2e12 = b2Vec2.Dot(w2, e12);
        const d12_1 = w2e12;
        const d12_2 = -w1e12;

        // Edge13
        // [1      1     ][a1] = [1]
        // [w1.e13 w3.e13][a3] = [0]
        // a2 = 0
        const e13 = b2Vec2.Subtract(w3, w1, b2Simplex.s_e13);
        const w1e13 = b2Vec2.Dot(w1, e13);
        const w3e13 = b2Vec2.Dot(w3, e13);
        const d13_1 = w3e13;
        const d13_2 = -w1e13;

        // Edge23
        // [1      1     ][a2] = [1]
        // [w2.e23 w3.e23][a3] = [0]
        // a1 = 0
        const e23 = b2Vec2.Subtract(w3, w2, b2Simplex.s_e23);
        const w2e23 = b2Vec2.Dot(w2, e23);
        const w3e23 = b2Vec2.Dot(w3, e23);
        const d23_1 = w3e23;
        const d23_2 = -w2e23;

        // Triangle123
        const n123 = b2Vec2.Cross(e12, e13);

        const d123_1 = n123 * b2Vec2.Cross(w2, w3);
        const d123_2 = n123 * b2Vec2.Cross(w3, w1);
        const d123_3 = n123 * b2Vec2.Cross(w1, w2);

        // w1 region
        if (d12_2 <= 0 && d13_2 <= 0) {
            this.m_v1.a = 1;
            this.m_count = 1;
            return;
        }

        // e12
        if (d12_1 > 0 && d12_2 > 0 && d123_3 <= 0) {
            const inv_d12 = 1 / (d12_1 + d12_2);
            this.m_v1.a = d12_1 * inv_d12;
            this.m_v2.a = d12_2 * inv_d12;
            this.m_count = 2;
            return;
        }

        // e13
        if (d13_1 > 0 && d13_2 > 0 && d123_2 <= 0) {
            const inv_d13 = 1 / (d13_1 + d13_2);
            this.m_v1.a = d13_1 * inv_d13;
            this.m_v3.a = d13_2 * inv_d13;
            this.m_count = 2;
            this.m_v2.Copy(this.m_v3);
            return;
        }

        // w2 region
        if (d12_1 <= 0 && d23_2 <= 0) {
            this.m_v2.a = 1;
            this.m_count = 1;
            this.m_v1.Copy(this.m_v2);
            return;
        }

        // w3 region
        if (d13_1 <= 0 && d23_1 <= 0) {
            this.m_v3.a = 1;
            this.m_count = 1;
            this.m_v1.Copy(this.m_v3);
            return;
        }

        // e23
        if (d23_1 > 0 && d23_2 > 0 && d123_1 <= 0) {
            const inv_d23 = 1 / (d23_1 + d23_2);
            this.m_v2.a = d23_1 * inv_d23;
            this.m_v3.a = d23_2 * inv_d23;
            this.m_count = 2;
            this.m_v1.Copy(this.m_v3);
            return;
        }

        // Must be in triangle123
        const inv_d123 = 1 / (d123_1 + d123_2 + d123_3);
        this.m_v1.a = d123_1 * inv_d123;
        this.m_v2.a = d123_2 * inv_d123;
        this.m_v3.a = d123_3 * inv_d123;
        this.m_count = 3;
    }

    private static s_e12 = new b2Vec2();

    private static s_e13 = new b2Vec2();

    private static s_e23 = new b2Vec2();
}

const b2Distance_s_simplex = new b2Simplex();
const b2Distance_s_saveA: [number, number, number] = [0, 0, 0];
const b2Distance_s_saveB: [number, number, number] = [0, 0, 0];
const b2Distance_s_p = new b2Vec2();
const b2Distance_s_d = new b2Vec2();
const b2Distance_s_normal = new b2Vec2();
const b2Distance_s_supportA = new b2Vec2();
const b2Distance_s_supportB = new b2Vec2();
export function b2Distance(output: b2DistanceOutput, cache: b2SimplexCache, input: b2DistanceInput): void {
    ++b2Gjk.calls;

    const { proxyA, proxyB, transformA, transformB } = input;

    // Initialize the simplex.
    const simplex = b2Distance_s_simplex;
    simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);

    // Get simplex vertices as an array.
    const vertices = simplex.m_vertices;
    const k_maxIters = 20;

    // These store the vertices of the last simplex so that we
    // can check for duplicates and prevent cycling.
    const saveA = b2Distance_s_saveA;
    const saveB = b2Distance_s_saveB;
    let saveCount = 0;

    // Main iteration loop.
    let iter = 0;
    while (iter < k_maxIters) {
        // Copy simplex so we can identify duplicates.
        saveCount = simplex.m_count;
        for (let i = 0; i < saveCount; ++i) {
            saveA[i] = vertices[i].indexA;
            saveB[i] = vertices[i].indexB;
        }

        switch (simplex.m_count) {
            case 1:
                break;

            case 2:
                simplex.Solve2();
                break;

            case 3:
                simplex.Solve3();
                break;

            // DEBUG: default:
            // DEBUG: b2Assert(false);
        }

        // If we have 3 points, then the origin is in the corresponding triangle.
        if (simplex.m_count === 3) {
            break;
        }

        // Get search direction.
        const d = simplex.GetSearchDirection(b2Distance_s_d);

        // Ensure the search direction is numerically fit.
        if (d.LengthSquared() < b2_epsilon_sq) {
            // The origin is probably contained by a line segment
            // or triangle. Thus the shapes are overlapped.

            // We can't return zero here even though there may be overlap.
            // In case the simplex is a point, segment, or triangle it is difficult
            // to determine if the origin is contained in the CSO or very close to it.
            break;
        }

        // Compute a tentative new simplex vertex using support points.
        const vertex = vertices[simplex.m_count];
        vertex.indexA = proxyA.GetSupport(
            b2Rot.TransposeMultiplyVec2(transformA.q, b2Vec2.Negate(d, b2Vec2.s_t0), b2Distance_s_supportA),
        );
        b2Transform.MultiplyVec2(transformA, proxyA.GetVertex(vertex.indexA), vertex.wA);
        vertex.indexB = proxyB.GetSupport(b2Rot.TransposeMultiplyVec2(transformB.q, d, b2Distance_s_supportB));
        b2Transform.MultiplyVec2(transformB, proxyB.GetVertex(vertex.indexB), vertex.wB);
        b2Vec2.Subtract(vertex.wB, vertex.wA, vertex.w);

        // Iteration count is equated to the number of support point calls.
        ++iter;
        ++b2Gjk.iters;

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
        ++simplex.m_count;
    }

    b2Gjk.maxIters = Math.max(b2Gjk.maxIters, iter);

    // Prepare output.
    simplex.GetWitnessPoints(output.pointA, output.pointB);
    output.distance = b2Vec2.Distance(output.pointA, output.pointB);
    output.iterations = iter;

    // Cache the simplex.
    simplex.WriteCache(cache);

    // Apply radii if requested
    if (input.useRadii) {
        if (output.distance < b2_epsilon) {
            // Shapes are too close to safely compute normal
            const p = b2Vec2.Mid(output.pointA, output.pointB, b2Distance_s_p);
            output.pointA.Copy(p);
            output.pointB.Copy(p);
            output.distance = 0;
        } else {
            // Keep closest points on perimeter even if overlapped, this way
            // the points move smoothly.
            const rA = proxyA.m_radius;
            const rB = proxyB.m_radius;
            const normal = b2Vec2.Subtract(output.pointB, output.pointA, b2Distance_s_normal);
            normal.Normalize();
            output.distance = Math.max(0, output.distance - rA - rB);
            output.pointA.AddScaled(rA, normal);
            output.pointB.SubtractScaled(rB, normal);
        }
    }
}

const b2ShapeCast_s_n = new b2Vec2();
const b2ShapeCast_s_simplex = new b2Simplex();
const b2ShapeCast_s_wA = new b2Vec2();
const b2ShapeCast_s_wB = new b2Vec2();
const b2ShapeCast_s_v = new b2Vec2();
const b2ShapeCast_s_p = new b2Vec2();
const b2ShapeCast_s_pointA = new b2Vec2();
const b2ShapeCast_s_pointB = new b2Vec2();

/**
 * Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
 * GJK-raycast
 * Algorithm by Gino van den Bergen.
 * "Smooth Mesh Contacts with GJK" in Game Physics Pearls. 2010
 */
export function b2ShapeCast(output: b2ShapeCastOutput, input: b2ShapeCastInput): boolean {
    output.iterations = 0;
    output.lambda = 1;
    output.normal.SetZero();
    output.point.SetZero();

    const { proxyA, proxyB } = input;

    const radiusA = Math.max(proxyA.m_radius, b2_polygonRadius);
    const radiusB = Math.max(proxyB.m_radius, b2_polygonRadius);
    const radius = radiusA + radiusB;

    const xfA = input.transformA;
    const xfB = input.transformB;

    const r = input.translationB;
    const n = b2ShapeCast_s_n.SetZero();
    let lambda = 0;

    // Initial simplex
    const simplex = b2ShapeCast_s_simplex;
    simplex.m_count = 0;

    // Get simplex vertices as an array.
    // b2SimplexVertex* vertices = &simplex.m_v1;
    const vertices = simplex.m_vertices;

    // Get support point in -r direction
    let indexA = proxyA.GetSupport(b2Rot.TransposeMultiplyVec2(xfA.q, b2Vec2.Negate(r, b2Vec2.s_t1), b2Vec2.s_t0));
    let wA = b2Transform.MultiplyVec2(xfA, proxyA.GetVertex(indexA), b2ShapeCast_s_wA);
    let indexB = proxyB.GetSupport(b2Rot.TransposeMultiplyVec2(xfB.q, r, b2Vec2.s_t0));
    let wB = b2Transform.MultiplyVec2(xfB, proxyB.GetVertex(indexB), b2ShapeCast_s_wB);
    const v = b2Vec2.Subtract(wA, wB, b2ShapeCast_s_v);

    // Sigma is the target distance between polygons
    const sigma = Math.max(b2_polygonRadius, radius - b2_polygonRadius);
    const tolerance = 0.5 * b2_linearSlop;

    // Main iteration loop.
    const k_maxIters = 20;
    let iter = 0;
    while (iter < k_maxIters && v.Length() - sigma > tolerance) {
        // DEBUG: b2Assert(simplex.m_count < 3);

        output.iterations += 1;

        // Support in direction -v (A - B)
        indexA = proxyA.GetSupport(b2Rot.TransposeMultiplyVec2(xfA.q, b2Vec2.Negate(v, b2Vec2.s_t1), b2Vec2.s_t0));
        wA = b2Transform.MultiplyVec2(xfA, proxyA.GetVertex(indexA), b2ShapeCast_s_wA);
        indexB = proxyB.GetSupport(b2Rot.TransposeMultiplyVec2(xfB.q, v, b2Vec2.s_t0));
        wB = b2Transform.MultiplyVec2(xfB, proxyB.GetVertex(indexB), b2ShapeCast_s_wB);
        const p = b2Vec2.Subtract(wA, wB, b2ShapeCast_s_p);

        // -v is a normal at p
        v.Normalize();

        // Intersect ray with plane
        const vp = b2Vec2.Dot(v, p);
        const vr = b2Vec2.Dot(v, r);
        if (vp - sigma > lambda * vr) {
            if (vr <= 0) {
                return false;
            }

            lambda = (vp - sigma) / vr;
            if (lambda > 1) {
                return false;
            }

            b2Vec2.Negate(v, n);
            simplex.m_count = 0;
        }

        // Reverse simplex since it works with B - A.
        // Shift by lambda * r because we want the closest point to the current clip point.
        // Note that the support point p is not shifted because we want the plane equation
        // to be formed in unshifted space.
        const vertex = vertices[simplex.m_count];
        vertex.indexA = indexB;
        b2Vec2.AddScaled(wB, lambda, r, vertex.wA);
        vertex.indexB = indexA;
        vertex.wB.Copy(wA);
        b2Vec2.Subtract(vertex.wB, vertex.wA, vertex.w);
        vertex.a = 1;
        simplex.m_count += 1;

        switch (simplex.m_count) {
            case 1:
                break;

            case 2:
                simplex.Solve2();
                break;

            case 3:
                simplex.Solve3();
                break;

            // DEBUG: default:
            // DEBUG: b2Assert(false);
        }

        // If we have 3 points, then the origin is in the corresponding triangle.
        if (simplex.m_count === 3) {
            // Overlap
            return false;
        }

        // Get search direction.
        simplex.GetClosestPoint(v);

        // Iteration count is equated to the number of support point calls.
        ++iter;
    }

    if (iter === 0) {
        // Initial overlap
        return false;
    }

    // Prepare output.
    const pointA = b2ShapeCast_s_pointA;
    const pointB = b2ShapeCast_s_pointB;
    simplex.GetWitnessPoints(pointA, pointB);

    if (v.LengthSquared() > 0) {
        b2Vec2.Negate(v, n);
        n.Normalize();
    }

    b2Vec2.AddScaled(pointA, radiusA, n, output.point);
    output.normal.Copy(n);
    output.lambda = lambda;
    output.iterations = iter;
    return true;
}
