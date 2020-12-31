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

// DEBUG: import { Assert } from "../common/b2_common";
import { Assert, LINEAR_SLOP } from "../common/b2_common";
import { MAX_POLYGON_VERTICES } from "../common/b2_settings";
import { Vec2, Rot, Transform, Sweep } from "../common/b2_math";
import { Timer } from "../common/b2_timer";
import { Distance, DistanceInput, DistanceOutput, DistanceProxy, SimplexCache } from "./b2_distance";

export const Toi = {
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

const TimeOfImpact_s_xfA = new Transform();
const TimeOfImpact_s_xfB = new Transform();
const TimeOfImpact_s_pointA = new Vec2();
const TimeOfImpact_s_pointB = new Vec2();
const TimeOfImpact_s_normal = new Vec2();
const TimeOfImpact_s_axisA = new Vec2();
const TimeOfImpact_s_axisB = new Vec2();

/**
 * Input parameters for TimeOfImpact
 */
export class TOIInput {
    public readonly proxyA = new DistanceProxy();

    public readonly proxyB = new DistanceProxy();

    public readonly sweepA = new Sweep();

    public readonly sweepB = new Sweep();

    public tMax = 0; // defines sweep interval [0, tMax]
}

export enum TOIOutputState {
    Unknown,
    Failed,
    Overlapped,
    Touching,
    Separated,
}

/**
 * Output parameters for TimeOfImpact.
 */
export class TOIOutput {
    public state = TOIOutputState.Unknown;

    public t = 0;
}

enum SeparationFunctionType {
    Points,
    FaceA,
    FaceB,
}

class SeparationFunction {
    public m_proxyA!: DistanceProxy;

    public m_proxyB!: DistanceProxy;

    public readonly m_sweepA = new Sweep();

    public readonly m_sweepB = new Sweep();

    public m_type = SeparationFunctionType.Points;

    public readonly m_localPoint = new Vec2();

    public readonly m_axis = new Vec2();

    public Initialize(
        cache: SimplexCache,
        proxyA: DistanceProxy,
        sweepA: Sweep,
        proxyB: DistanceProxy,
        sweepB: Sweep,
        t1: number,
    ): number {
        this.m_proxyA = proxyA;
        this.m_proxyB = proxyB;
        const { count } = cache;
        // DEBUG: Assert(0 < count && count < 3);

        this.m_sweepA.Copy(sweepA);
        this.m_sweepB.Copy(sweepB);

        const xfA = this.m_sweepA.GetTransform(TimeOfImpact_s_xfA, t1);
        const xfB = this.m_sweepB.GetTransform(TimeOfImpact_s_xfB, t1);

        if (count === 1) {
            this.m_type = SeparationFunctionType.Points;
            const localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
            const localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
            const pointA = Transform.MultiplyVec2(xfA, localPointA, TimeOfImpact_s_pointA);
            const pointB = Transform.MultiplyVec2(xfB, localPointB, TimeOfImpact_s_pointB);
            Vec2.Subtract(pointB, pointA, this.m_axis);
            const s = this.m_axis.Normalize();
            return s;
        }
        if (cache.indexA[0] === cache.indexA[1]) {
            // Two points on B and one on A.
            this.m_type = SeparationFunctionType.FaceB;
            const localPointB1 = this.m_proxyB.GetVertex(cache.indexB[0]);
            const localPointB2 = this.m_proxyB.GetVertex(cache.indexB[1]);

            Vec2.CrossVec2One(Vec2.Subtract(localPointB2, localPointB1, Vec2.s_t0), this.m_axis).Normalize();
            const normal = Rot.MultiplyVec2(xfB.q, this.m_axis, TimeOfImpact_s_normal);

            Vec2.Mid(localPointB1, localPointB2, this.m_localPoint);
            const pointB = Transform.MultiplyVec2(xfB, this.m_localPoint, TimeOfImpact_s_pointB);

            const localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
            const pointA = Transform.MultiplyVec2(xfA, localPointA, TimeOfImpact_s_pointA);

            let s = Vec2.Dot(Vec2.Subtract(pointA, pointB, Vec2.s_t0), normal);
            if (s < 0) {
                this.m_axis.Negate();
                s = -s;
            }
            return s;
        }
        // Two points on A and one or two points on B.
        this.m_type = SeparationFunctionType.FaceA;
        const localPointA1 = this.m_proxyA.GetVertex(cache.indexA[0]);
        const localPointA2 = this.m_proxyA.GetVertex(cache.indexA[1]);

        Vec2.CrossVec2One(Vec2.Subtract(localPointA2, localPointA1, Vec2.s_t0), this.m_axis).Normalize();
        const normal = Rot.MultiplyVec2(xfA.q, this.m_axis, TimeOfImpact_s_normal);

        Vec2.Mid(localPointA1, localPointA2, this.m_localPoint);
        const pointA = Transform.MultiplyVec2(xfA, this.m_localPoint, TimeOfImpact_s_pointA);

        const localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
        const pointB = Transform.MultiplyVec2(xfB, localPointB, TimeOfImpact_s_pointB);

        let s = Vec2.Dot(Vec2.Subtract(pointB, pointA, Vec2.s_t0), normal);
        if (s < 0) {
            this.m_axis.Negate();
            s = -s;
        }
        return s;
    }

    public FindMinSeparation(indexA: [number], indexB: [number], t: number): number {
        const xfA = this.m_sweepA.GetTransform(TimeOfImpact_s_xfA, t);
        const xfB = this.m_sweepB.GetTransform(TimeOfImpact_s_xfB, t);

        switch (this.m_type) {
            case SeparationFunctionType.Points: {
                const axisA = Rot.TransposeMultiplyVec2(xfA.q, this.m_axis, TimeOfImpact_s_axisA);
                const axisB = Rot.TransposeMultiplyVec2(
                    xfB.q,
                    Vec2.Negate(this.m_axis, Vec2.s_t0),
                    TimeOfImpact_s_axisB,
                );

                indexA[0] = this.m_proxyA.GetSupport(axisA);
                indexB[0] = this.m_proxyB.GetSupport(axisB);

                const localPointA = this.m_proxyA.GetVertex(indexA[0]);
                const localPointB = this.m_proxyB.GetVertex(indexB[0]);

                const pointA = Transform.MultiplyVec2(xfA, localPointA, TimeOfImpact_s_pointA);
                const pointB = Transform.MultiplyVec2(xfB, localPointB, TimeOfImpact_s_pointB);

                const separation = Vec2.Dot(Vec2.Subtract(pointB, pointA, Vec2.s_t0), this.m_axis);
                return separation;
            }

            case SeparationFunctionType.FaceA: {
                const normal = Rot.MultiplyVec2(xfA.q, this.m_axis, TimeOfImpact_s_normal);
                const pointA = Transform.MultiplyVec2(xfA, this.m_localPoint, TimeOfImpact_s_pointA);

                const axisB = Rot.TransposeMultiplyVec2(xfB.q, Vec2.Negate(normal, Vec2.s_t0), TimeOfImpact_s_axisB);

                indexA[0] = -1;
                indexB[0] = this.m_proxyB.GetSupport(axisB);

                const localPointB = this.m_proxyB.GetVertex(indexB[0]);
                const pointB = Transform.MultiplyVec2(xfB, localPointB, TimeOfImpact_s_pointB);

                const separation = Vec2.Dot(Vec2.Subtract(pointB, pointA, Vec2.s_t0), normal);
                return separation;
            }

            case SeparationFunctionType.FaceB: {
                const normal = Rot.MultiplyVec2(xfB.q, this.m_axis, TimeOfImpact_s_normal);
                const pointB = Transform.MultiplyVec2(xfB, this.m_localPoint, TimeOfImpact_s_pointB);

                const axisA = Rot.TransposeMultiplyVec2(xfA.q, Vec2.Negate(normal, Vec2.s_t0), TimeOfImpact_s_axisA);

                indexB[0] = -1;
                indexA[0] = this.m_proxyA.GetSupport(axisA);

                const localPointA = this.m_proxyA.GetVertex(indexA[0]);
                const pointA = Transform.MultiplyVec2(xfA, localPointA, TimeOfImpact_s_pointA);

                const separation = Vec2.Dot(Vec2.Subtract(pointA, pointB, Vec2.s_t0), normal);
                return separation;
            }

            default:
                // DEBUG: Assert(false);
                indexA[0] = -1;
                indexB[0] = -1;
                return 0;
        }
    }

    public Evaluate(indexA: number, indexB: number, t: number): number {
        const xfA = this.m_sweepA.GetTransform(TimeOfImpact_s_xfA, t);
        const xfB = this.m_sweepB.GetTransform(TimeOfImpact_s_xfB, t);

        switch (this.m_type) {
            case SeparationFunctionType.Points: {
                const localPointA = this.m_proxyA.GetVertex(indexA);
                const localPointB = this.m_proxyB.GetVertex(indexB);

                const pointA = Transform.MultiplyVec2(xfA, localPointA, TimeOfImpact_s_pointA);
                const pointB = Transform.MultiplyVec2(xfB, localPointB, TimeOfImpact_s_pointB);
                const separation = Vec2.Dot(Vec2.Subtract(pointB, pointA, Vec2.s_t0), this.m_axis);

                return separation;
            }

            case SeparationFunctionType.FaceA: {
                const normal = Rot.MultiplyVec2(xfA.q, this.m_axis, TimeOfImpact_s_normal);
                const pointA = Transform.MultiplyVec2(xfA, this.m_localPoint, TimeOfImpact_s_pointA);

                const localPointB = this.m_proxyB.GetVertex(indexB);
                const pointB = Transform.MultiplyVec2(xfB, localPointB, TimeOfImpact_s_pointB);

                const separation = Vec2.Dot(Vec2.Subtract(pointB, pointA, Vec2.s_t0), normal);
                return separation;
            }

            case SeparationFunctionType.FaceB: {
                const normal = Rot.MultiplyVec2(xfB.q, this.m_axis, TimeOfImpact_s_normal);
                const pointB = Transform.MultiplyVec2(xfB, this.m_localPoint, TimeOfImpact_s_pointB);

                const localPointA = this.m_proxyA.GetVertex(indexA);
                const pointA = Transform.MultiplyVec2(xfA, localPointA, TimeOfImpact_s_pointA);

                const separation = Vec2.Dot(Vec2.Subtract(pointA, pointB, Vec2.s_t0), normal);
                return separation;
            }

            default:
                Assert(false);
                return 0;
        }
    }
}

const TimeOfImpact_s_timer = new Timer();
const TimeOfImpact_s_cache = new SimplexCache();
const TimeOfImpact_s_distanceInput = new DistanceInput();
const TimeOfImpact_s_distanceOutput = new DistanceOutput();
const TimeOfImpact_s_fcn = new SeparationFunction();
const TimeOfImpact_s_indexA: [number] = [0];
const TimeOfImpact_s_indexB: [number] = [0];
const TimeOfImpact_s_sweepA = new Sweep();
const TimeOfImpact_s_sweepB = new Sweep();
export function TimeOfImpact(output: TOIOutput, input: TOIInput): void {
    const timer = TimeOfImpact_s_timer.Reset();

    ++Toi.calls;

    output.state = TOIOutputState.Unknown;
    output.t = input.tMax;

    const { proxyA, proxyB, tMax } = input;
    const maxVertices = Math.max(MAX_POLYGON_VERTICES, proxyA.m_count, proxyB.m_count);

    const sweepA = TimeOfImpact_s_sweepA.Copy(input.sweepA);
    const sweepB = TimeOfImpact_s_sweepB.Copy(input.sweepB);

    // Large rotations can make the root finder fail, so we normalize the
    // sweep angles.
    sweepA.Normalize();
    sweepB.Normalize();

    const totalRadius = proxyA.m_radius + proxyB.m_radius;
    const target = Math.max(LINEAR_SLOP, totalRadius - 3 * LINEAR_SLOP);
    const tolerance = 0.25 * LINEAR_SLOP;
    // DEBUG: Assert(target > tolerance);

    let t1 = 0;
    const k_maxIterations = 20; // TODO_ERIN Settings
    let iter = 0;

    // Prepare input for distance query.
    const cache = TimeOfImpact_s_cache;
    cache.count = 0;
    const distanceInput = TimeOfImpact_s_distanceInput;
    distanceInput.proxyA.Copy(input.proxyA);
    distanceInput.proxyB.Copy(input.proxyB);
    distanceInput.useRadii = false;

    // The outer loop progressively attempts to compute new separating axes.
    // This loop terminates when an axis is repeated (no progress is made).
    for (;;) {
        const xfA = sweepA.GetTransform(TimeOfImpact_s_xfA, t1);
        const xfB = sweepB.GetTransform(TimeOfImpact_s_xfB, t1);

        // Get the distance between shapes. We can also use the results
        // to get a separating axis.
        distanceInput.transformA.Copy(xfA);
        distanceInput.transformB.Copy(xfB);
        const distanceOutput = TimeOfImpact_s_distanceOutput;
        Distance(distanceOutput, cache, distanceInput);

        // If the shapes are overlapped, we give up on continuous collision.
        if (distanceOutput.distance <= 0) {
            // Failure!
            output.state = TOIOutputState.Overlapped;
            output.t = 0;
            break;
        }

        if (distanceOutput.distance < target + tolerance) {
            // Victory!
            output.state = TOIOutputState.Touching;
            output.t = t1;
            break;
        }

        // Initialize the separating axis.
        const fcn = TimeOfImpact_s_fcn;
        fcn.Initialize(cache, proxyA, sweepA, proxyB, sweepB, t1);

        // Compute the TOI on the separating axis. We do this by successively
        // resolving the deepest point. This loop is bounded by the number of vertices.
        let done = false;
        let t2 = tMax;
        let pushBackIter = 0;
        for (;;) {
            // Find the deepest point at t2. Store the witness point indices.
            const indexA = TimeOfImpact_s_indexA;
            const indexB = TimeOfImpact_s_indexB;
            let s2 = fcn.FindMinSeparation(indexA, indexB, t2);

            // Is the final configuration separated?
            if (s2 > target + tolerance) {
                // Victory!
                output.state = TOIOutputState.Separated;
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
                output.state = TOIOutputState.Failed;
                output.t = t1;
                done = true;
                break;
            }

            // Check for touching
            if (s1 <= target + tolerance) {
                // Victory! t1 should hold the TOI (could be 0).
                output.state = TOIOutputState.Touching;
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
                ++Toi.rootIters;

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

            Toi.maxRootIters = Math.max(Toi.maxRootIters, rootIterCount);

            ++pushBackIter;

            if (pushBackIter === maxVertices) {
                break;
            }
        }

        ++iter;
        ++Toi.iters;

        if (done) {
            break;
        }

        if (iter === k_maxIterations) {
            // Root finder got stuck. Semi-victory.
            output.state = TOIOutputState.Failed;
            output.t = t1;
            break;
        }
    }

    Toi.maxIters = Math.max(Toi.maxIters, iter);

    const time = timer.GetMilliseconds();
    Toi.maxTime = Math.max(Toi.maxTime, time);
    Toi.time += time;
}
