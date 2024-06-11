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
import { b2Assert, b2_linearSlop } from "../common/b2_common";
import { b2_maxPolygonVertices } from "../common/b2_settings";
import { b2Vec2, b2Rot, b2Transform, b2Sweep } from "../common/b2_math";
import { b2Timer } from "../common/b2_timer";
import { b2Distance, b2DistanceInput, b2DistanceOutput, b2DistanceProxy, b2SimplexCache } from "./b2_distance";

export const b2Toi = {
    time: 0,
    maxTime: 0,
    calls: 0,
    iters: 0,
    maxIters: 0,
    rootIters: 0,
    maxRootIters: 0,
    reset() {
        this.time = 0;
        this.maxTime = 0;
        this.calls = 0;
        this.iters = 0;
        this.maxIters = 0;
        this.rootIters = 0;
        this.maxRootIters = 0;
    },
};

const b2TimeOfImpact_s_xfA = new b2Transform();
const b2TimeOfImpact_s_xfB = new b2Transform();
const b2TimeOfImpact_s_pointA = new b2Vec2();
const b2TimeOfImpact_s_pointB = new b2Vec2();
const b2TimeOfImpact_s_normal = new b2Vec2();
const b2TimeOfImpact_s_axisA = new b2Vec2();
const b2TimeOfImpact_s_axisB = new b2Vec2();

/**
 * Input parameters for b2TimeOfImpact
 */
export class b2TOIInput {
    public readonly proxyA = new b2DistanceProxy();

    public readonly proxyB = new b2DistanceProxy();

    public readonly sweepA = new b2Sweep();

    public readonly sweepB = new b2Sweep();

    public tMax = 0; // defines sweep interval [0, tMax]
}

export enum b2TOIOutputState {
    e_unknown,
    e_failed,
    e_overlapped,
    e_touching,
    e_separated,
}

/**
 * Output parameters for b2TimeOfImpact.
 */
export class b2TOIOutput {
    public state = b2TOIOutputState.e_unknown;

    public t = 0;
}

enum b2SeparationFunctionType {
    e_points,
    e_faceA,
    e_faceB,
}

class b2SeparationFunction {
    public m_proxyA!: b2DistanceProxy;

    public m_proxyB!: b2DistanceProxy;

    public readonly m_sweepA = new b2Sweep();

    public readonly m_sweepB = new b2Sweep();

    public m_type = b2SeparationFunctionType.e_points;

    public readonly m_localPoint = new b2Vec2();

    public readonly m_axis = new b2Vec2();

    public Initialize(
        cache: b2SimplexCache,
        proxyA: b2DistanceProxy,
        sweepA: b2Sweep,
        proxyB: b2DistanceProxy,
        sweepB: b2Sweep,
        t1: number,
    ): number {
        this.m_proxyA = proxyA;
        this.m_proxyB = proxyB;
        const { count } = cache;
        // DEBUG: b2Assert(0 < count && count < 3);

        this.m_sweepA.Copy(sweepA);
        this.m_sweepB.Copy(sweepB);

        const xfA = this.m_sweepA.GetTransform(b2TimeOfImpact_s_xfA, t1);
        const xfB = this.m_sweepB.GetTransform(b2TimeOfImpact_s_xfB, t1);

        if (count === 1) {
            this.m_type = b2SeparationFunctionType.e_points;
            const localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
            const localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
            const pointA = b2Transform.MultiplyVec2(xfA, localPointA, b2TimeOfImpact_s_pointA);
            const pointB = b2Transform.MultiplyVec2(xfB, localPointB, b2TimeOfImpact_s_pointB);
            b2Vec2.Subtract(pointB, pointA, this.m_axis);
            const s = this.m_axis.Normalize();
            return s;
        }
        if (cache.indexA[0] === cache.indexA[1]) {
            // Two points on B and one on A.
            this.m_type = b2SeparationFunctionType.e_faceB;
            const localPointB1 = this.m_proxyB.GetVertex(cache.indexB[0]);
            const localPointB2 = this.m_proxyB.GetVertex(cache.indexB[1]);

            b2Vec2.CrossVec2One(b2Vec2.Subtract(localPointB2, localPointB1, b2Vec2.s_t0), this.m_axis).Normalize();
            const normal = b2Rot.MultiplyVec2(xfB.q, this.m_axis, b2TimeOfImpact_s_normal);

            b2Vec2.Mid(localPointB1, localPointB2, this.m_localPoint);
            const pointB = b2Transform.MultiplyVec2(xfB, this.m_localPoint, b2TimeOfImpact_s_pointB);

            const localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
            const pointA = b2Transform.MultiplyVec2(xfA, localPointA, b2TimeOfImpact_s_pointA);

            let s = b2Vec2.Dot(b2Vec2.Subtract(pointA, pointB, b2Vec2.s_t0), normal);
            if (s < 0) {
                this.m_axis.Negate();
                s = -s;
            }
            return s;
        }
        // Two points on A and one or two points on B.
        this.m_type = b2SeparationFunctionType.e_faceA;
        const localPointA1 = this.m_proxyA.GetVertex(cache.indexA[0]);
        const localPointA2 = this.m_proxyA.GetVertex(cache.indexA[1]);

        b2Vec2.CrossVec2One(b2Vec2.Subtract(localPointA2, localPointA1, b2Vec2.s_t0), this.m_axis).Normalize();
        const normal = b2Rot.MultiplyVec2(xfA.q, this.m_axis, b2TimeOfImpact_s_normal);

        b2Vec2.Mid(localPointA1, localPointA2, this.m_localPoint);
        const pointA = b2Transform.MultiplyVec2(xfA, this.m_localPoint, b2TimeOfImpact_s_pointA);

        const localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
        const pointB = b2Transform.MultiplyVec2(xfB, localPointB, b2TimeOfImpact_s_pointB);

        let s = b2Vec2.Dot(b2Vec2.Subtract(pointB, pointA, b2Vec2.s_t0), normal);
        if (s < 0) {
            this.m_axis.Negate();
            s = -s;
        }
        return s;
    }

    public FindMinSeparation(indexA: [number], indexB: [number], t: number): number {
        const xfA = this.m_sweepA.GetTransform(b2TimeOfImpact_s_xfA, t);
        const xfB = this.m_sweepB.GetTransform(b2TimeOfImpact_s_xfB, t);

        switch (this.m_type) {
            case b2SeparationFunctionType.e_points: {
                const axisA = b2Rot.TransposeMultiplyVec2(xfA.q, this.m_axis, b2TimeOfImpact_s_axisA);
                const axisB = b2Rot.TransposeMultiplyVec2(
                    xfB.q,
                    b2Vec2.Negate(this.m_axis, b2Vec2.s_t0),
                    b2TimeOfImpact_s_axisB,
                );

                indexA[0] = this.m_proxyA.GetSupport(axisA);
                indexB[0] = this.m_proxyB.GetSupport(axisB);

                const localPointA = this.m_proxyA.GetVertex(indexA[0]);
                const localPointB = this.m_proxyB.GetVertex(indexB[0]);

                const pointA = b2Transform.MultiplyVec2(xfA, localPointA, b2TimeOfImpact_s_pointA);
                const pointB = b2Transform.MultiplyVec2(xfB, localPointB, b2TimeOfImpact_s_pointB);

                const separation = b2Vec2.Dot(b2Vec2.Subtract(pointB, pointA, b2Vec2.s_t0), this.m_axis);
                return separation;
            }

            case b2SeparationFunctionType.e_faceA: {
                const normal = b2Rot.MultiplyVec2(xfA.q, this.m_axis, b2TimeOfImpact_s_normal);
                const pointA = b2Transform.MultiplyVec2(xfA, this.m_localPoint, b2TimeOfImpact_s_pointA);

                const axisB = b2Rot.TransposeMultiplyVec2(
                    xfB.q,
                    b2Vec2.Negate(normal, b2Vec2.s_t0),
                    b2TimeOfImpact_s_axisB,
                );

                indexA[0] = -1;
                indexB[0] = this.m_proxyB.GetSupport(axisB);

                const localPointB = this.m_proxyB.GetVertex(indexB[0]);
                const pointB = b2Transform.MultiplyVec2(xfB, localPointB, b2TimeOfImpact_s_pointB);

                const separation = b2Vec2.Dot(b2Vec2.Subtract(pointB, pointA, b2Vec2.s_t0), normal);
                return separation;
            }

            case b2SeparationFunctionType.e_faceB: {
                const normal = b2Rot.MultiplyVec2(xfB.q, this.m_axis, b2TimeOfImpact_s_normal);
                const pointB = b2Transform.MultiplyVec2(xfB, this.m_localPoint, b2TimeOfImpact_s_pointB);

                const axisA = b2Rot.TransposeMultiplyVec2(
                    xfA.q,
                    b2Vec2.Negate(normal, b2Vec2.s_t0),
                    b2TimeOfImpact_s_axisA,
                );

                indexB[0] = -1;
                indexA[0] = this.m_proxyA.GetSupport(axisA);

                const localPointA = this.m_proxyA.GetVertex(indexA[0]);
                const pointA = b2Transform.MultiplyVec2(xfA, localPointA, b2TimeOfImpact_s_pointA);

                const separation = b2Vec2.Dot(b2Vec2.Subtract(pointA, pointB, b2Vec2.s_t0), normal);
                return separation;
            }

            default:
                // DEBUG: b2Assert(false);
                indexA[0] = -1;
                indexB[0] = -1;
                return 0;
        }
    }

    public Evaluate(indexA: number, indexB: number, t: number): number {
        const xfA = this.m_sweepA.GetTransform(b2TimeOfImpact_s_xfA, t);
        const xfB = this.m_sweepB.GetTransform(b2TimeOfImpact_s_xfB, t);

        switch (this.m_type) {
            case b2SeparationFunctionType.e_points: {
                const localPointA = this.m_proxyA.GetVertex(indexA);
                const localPointB = this.m_proxyB.GetVertex(indexB);

                const pointA = b2Transform.MultiplyVec2(xfA, localPointA, b2TimeOfImpact_s_pointA);
                const pointB = b2Transform.MultiplyVec2(xfB, localPointB, b2TimeOfImpact_s_pointB);
                const separation = b2Vec2.Dot(b2Vec2.Subtract(pointB, pointA, b2Vec2.s_t0), this.m_axis);

                return separation;
            }

            case b2SeparationFunctionType.e_faceA: {
                const normal = b2Rot.MultiplyVec2(xfA.q, this.m_axis, b2TimeOfImpact_s_normal);
                const pointA = b2Transform.MultiplyVec2(xfA, this.m_localPoint, b2TimeOfImpact_s_pointA);

                const localPointB = this.m_proxyB.GetVertex(indexB);
                const pointB = b2Transform.MultiplyVec2(xfB, localPointB, b2TimeOfImpact_s_pointB);

                const separation = b2Vec2.Dot(b2Vec2.Subtract(pointB, pointA, b2Vec2.s_t0), normal);
                return separation;
            }

            case b2SeparationFunctionType.e_faceB: {
                const normal = b2Rot.MultiplyVec2(xfB.q, this.m_axis, b2TimeOfImpact_s_normal);
                const pointB = b2Transform.MultiplyVec2(xfB, this.m_localPoint, b2TimeOfImpact_s_pointB);

                const localPointA = this.m_proxyA.GetVertex(indexA);
                const pointA = b2Transform.MultiplyVec2(xfA, localPointA, b2TimeOfImpact_s_pointA);

                const separation = b2Vec2.Dot(b2Vec2.Subtract(pointA, pointB, b2Vec2.s_t0), normal);
                return separation;
            }

            default:
                b2Assert(false);
                return 0;
        }
    }
}

const b2TimeOfImpact_s_timer = new b2Timer();
const b2TimeOfImpact_s_cache = new b2SimplexCache();
const b2TimeOfImpact_s_distanceInput = new b2DistanceInput();
const b2TimeOfImpact_s_distanceOutput = new b2DistanceOutput();
const b2TimeOfImpact_s_fcn = new b2SeparationFunction();
const b2TimeOfImpact_s_indexA: [number] = [0];
const b2TimeOfImpact_s_indexB: [number] = [0];
const b2TimeOfImpact_s_sweepA = new b2Sweep();
const b2TimeOfImpact_s_sweepB = new b2Sweep();

/**
 * CCD via the local separating axis method. This seeks progression
 * by computing the largest time at which separation is maintained.
 * Compute the upper bound on time before two shapes penetrate. Time is represented as
 * a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
 * again.
 * Note: use b2Distance to compute the contact point and normal at the time of impact.
 */
export function b2TimeOfImpact(output: b2TOIOutput, input: b2TOIInput): void {
    const timer = b2TimeOfImpact_s_timer.Reset();

    ++b2Toi.calls;

    output.state = b2TOIOutputState.e_unknown;
    output.t = input.tMax;

    const { proxyA, proxyB, tMax } = input;
    const maxVertices = Math.max(b2_maxPolygonVertices, proxyA.m_count, proxyB.m_count);

    const sweepA = b2TimeOfImpact_s_sweepA.Copy(input.sweepA);
    const sweepB = b2TimeOfImpact_s_sweepB.Copy(input.sweepB);

    // Large rotations can make the root finder fail, so we normalize the
    // sweep angles.
    sweepA.Normalize();
    sweepB.Normalize();

    const totalRadius = proxyA.m_radius + proxyB.m_radius;
    const target = Math.max(b2_linearSlop, totalRadius - 3 * b2_linearSlop);
    const tolerance = 0.25 * b2_linearSlop;
    // DEBUG: b2Assert(target > tolerance);

    let t1 = 0;
    const k_maxIterations = 20; // TODO_ERIN b2Settings
    let iter = 0;

    // Prepare input for distance query.
    const cache = b2TimeOfImpact_s_cache;
    cache.count = 0;
    const distanceInput = b2TimeOfImpact_s_distanceInput;
    distanceInput.proxyA.Copy(input.proxyA);
    distanceInput.proxyB.Copy(input.proxyB);
    distanceInput.useRadii = false;

    // The outer loop progressively attempts to compute new separating axes.
    // This loop terminates when an axis is repeated (no progress is made).
    for (;;) {
        const xfA = sweepA.GetTransform(b2TimeOfImpact_s_xfA, t1);
        const xfB = sweepB.GetTransform(b2TimeOfImpact_s_xfB, t1);

        // Get the distance between shapes. We can also use the results
        // to get a separating axis.
        distanceInput.transformA.Copy(xfA);
        distanceInput.transformB.Copy(xfB);
        const distanceOutput = b2TimeOfImpact_s_distanceOutput;
        b2Distance(distanceOutput, cache, distanceInput);

        // If the shapes are overlapped, we give up on continuous collision.
        if (distanceOutput.distance <= 0) {
            // Failure!
            output.state = b2TOIOutputState.e_overlapped;
            output.t = 0;
            break;
        }

        if (distanceOutput.distance < target + tolerance) {
            // Victory!
            output.state = b2TOIOutputState.e_touching;
            output.t = t1;
            break;
        }

        // Initialize the separating axis.
        const fcn = b2TimeOfImpact_s_fcn;
        fcn.Initialize(cache, proxyA, sweepA, proxyB, sweepB, t1);

        // Compute the TOI on the separating axis. We do this by successively
        // resolving the deepest point. This loop is bounded by the number of vertices.
        let done = false;
        let t2 = tMax;
        let pushBackIter = 0;
        for (;;) {
            // Find the deepest point at t2. Store the witness point indices.
            const indexA = b2TimeOfImpact_s_indexA;
            const indexB = b2TimeOfImpact_s_indexB;
            let s2 = fcn.FindMinSeparation(indexA, indexB, t2);

            // Is the final configuration separated?
            if (s2 > target + tolerance) {
                // Victory!
                output.state = b2TOIOutputState.e_separated;
                output.t = tMax;
                done = true;
                break;
            }

            // Has the separation reached tolerance?
            if (s2 > target - tolerance) {
                // Advance the sweeps
                t1 = t2;
                break;
            }

            // Compute the initial separation of the witness points.
            let s1 = fcn.Evaluate(indexA[0], indexB[0], t1);

            // Check for initial overlap. This might happen if the root finder
            // runs out of iterations.
            if (s1 < target - tolerance) {
                output.state = b2TOIOutputState.e_failed;
                output.t = t1;
                done = true;
                break;
            }

            // Check for touching
            if (s1 <= target + tolerance) {
                // Victory! t1 should hold the TOI (could be 0).
                output.state = b2TOIOutputState.e_touching;
                output.t = t1;
                done = true;
                break;
            }

            // Compute 1D root of: f(x) - target = 0
            let rootIterCount = 0;
            let a1 = t1;
            let a2 = t2;
            for (;;) {
                // Use a mix of the secant rule and bisection.
                let t: number;
                if (rootIterCount & 1) {
                    // Secant rule to improve convergence.
                    t = a1 + ((target - s1) * (a2 - a1)) / (s2 - s1);
                } else {
                    // Bisection to guarantee progress.
                    t = 0.5 * (a1 + a2);
                }

                ++rootIterCount;
                ++b2Toi.rootIters;

                const s = fcn.Evaluate(indexA[0], indexB[0], t);

                if (Math.abs(s - target) < tolerance) {
                    // t2 holds a tentative value for t1
                    t2 = t;
                    break;
                }

                // Ensure we continue to bracket the root.
                if (s > target) {
                    a1 = t;
                    s1 = s;
                } else {
                    a2 = t;
                    s2 = s;
                }

                if (rootIterCount === 50) {
                    break;
                }
            }

            b2Toi.maxRootIters = Math.max(b2Toi.maxRootIters, rootIterCount);

            ++pushBackIter;

            if (pushBackIter === maxVertices) {
                break;
            }
        }

        ++iter;
        ++b2Toi.iters;

        if (done) {
            break;
        }

        if (iter === k_maxIterations) {
            // Root finder got stuck. Semi-victory.
            output.state = b2TOIOutputState.e_failed;
            output.t = t1;
            break;
        }
    }

    b2Toi.maxIters = Math.max(b2Toi.maxIters, iter);

    const time = timer.GetMilliseconds();
    b2Toi.maxTime = Math.max(b2Toi.maxTime, time);
    b2Toi.time += time;
}
