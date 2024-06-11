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

// Structures and functions used for computing contact points, distance queries, and TOI queries.
// DEBUG: import { b2Assert } from "../common/b2_common";
import {
    b2_maxFloat,
    b2_epsilon,
    b2_epsilon_sq,
    b2_maxManifoldPoints,
    b2MakeNumberArray,
    b2MakeArray,
    b2_linearSlop,
    b2Assert,
} from "../common/b2_common";
import { b2Vec2, b2Rot, b2Transform, XY } from "../common/b2_math";
import type { b2Shape } from "./b2_shape";
import { b2Distance, b2DistanceInput, b2DistanceOutput, b2SimplexCache } from "./b2_distance";
import { b2_maxPolygonVertices } from "../common/b2_settings";

export enum b2ContactFeatureType {
    e_vertex = 0,
    e_face = 1,
}

/**
 * The features that intersect to form the contact point
 * This must be 4 bytes or less.
 */
export class b2ContactFeature {
    private m_key = 0;

    private m_key_invalid = false;

    /** Feature index on shapeA */
    private m_indexA = 0;

    /** Feature index on shapeB */
    private m_indexB = 0;

    /** The feature type on shapeA */
    private m_typeA = b2ContactFeatureType.e_vertex;

    /** The feature type on shapeB */
    private m_typeB = b2ContactFeatureType.e_vertex;

    public get key(): number {
        if (this.m_key_invalid) {
            this.m_key_invalid = false;
            this.m_key = this.m_indexA | (this.m_indexB << 8) | (this.m_typeA << 16) | (this.m_typeB << 24);
        }
        return this.m_key;
    }

    public set key(value: number) {
        this.m_key = value;
        this.m_key_invalid = false;
        this.m_indexA = this.m_key & 0xff;
        this.m_indexB = (this.m_key >> 8) & 0xff;
        this.m_typeA = (this.m_key >> 16) & 0xff;
        this.m_typeB = (this.m_key >> 24) & 0xff;
    }

    public get indexA(): number {
        return this.m_indexA;
    }

    public set indexA(value: number) {
        this.m_indexA = value;
        this.m_key_invalid = true;
    }

    public get indexB(): number {
        return this.m_indexB;
    }

    public set indexB(value: number) {
        this.m_indexB = value;
        this.m_key_invalid = true;
    }

    public get typeA(): number {
        return this.m_typeA;
    }

    public set typeA(value: number) {
        this.m_typeA = value;
        this.m_key_invalid = true;
    }

    public get typeB(): number {
        return this.m_typeB;
    }

    public set typeB(value: number) {
        this.m_typeB = value;
        this.m_key_invalid = true;
    }
}

/**
 * Contact ids to facilitate warm starting.
 */
export class b2ContactID {
    public readonly cf = new b2ContactFeature();

    public Copy(o: b2ContactID): b2ContactID {
        this.key = o.key;
        return this;
    }

    public Clone(): b2ContactID {
        return new b2ContactID().Copy(this);
    }

    public get key(): number {
        return this.cf.key;
    }

    public set key(value: number) {
        this.cf.key = value;
    }
}

/**
 * A manifold point is a contact point belonging to a contact
 * manifold. It holds details related to the geometry and dynamics
 * of the contact points.
 * The local point usage depends on the manifold type:
 * -e_circles: the local center of circleB
 * -e_faceA: the local center of cirlceB or the clip point of polygonB
 * -e_faceB: the clip point of polygonA
 * This structure is stored across time steps, so we keep it small.
 * Note: the impulses are used for internal caching and may not
 * provide reliable contact forces, especially for high speed collisions.
 */
export class b2ManifoldPoint {
    /** Usage depends on manifold type */
    public readonly localPoint = new b2Vec2();

    /** The non-penetration impulse */
    public normalImpulse = 0;

    /** The friction impulse */
    public tangentImpulse = 0;

    /** Uniquely identifies a contact point between two shapes */
    public readonly id = new b2ContactID();

    public Reset(): void {
        this.localPoint.SetZero();
        this.normalImpulse = 0;
        this.tangentImpulse = 0;
        this.id.key = 0;
    }

    public Copy(o: b2ManifoldPoint): b2ManifoldPoint {
        this.localPoint.Copy(o.localPoint);
        this.normalImpulse = o.normalImpulse;
        this.tangentImpulse = o.tangentImpulse;
        this.id.Copy(o.id);
        return this;
    }
}

export enum b2ManifoldType {
    e_circles,
    e_faceA,
    e_faceB,
}

/**
 * A manifold for two touching convex shapes.
 * Box2D supports multiple types of contact:
 * - clip point versus plane with radius
 * - point versus point with radius (circles)
 * The local point usage depends on the manifold type:
 * -e_circles: the local center of circleA
 * -e_faceA: the center of faceA
 * -e_faceB: the center of faceB
 * Similarly the local normal usage:
 * -e_circles: not used
 * -e_faceA: the normal on polygonA
 * -e_faceB: the normal on polygonB
 * We store contacts in this way so that position correction can
 * account for movement, which is critical for continuous physics.
 * All contact scenarios must be expressed in one of these types.
 * This structure is stored across time steps, so we keep it small.
 */
export class b2Manifold {
    /** The points of contact */
    public readonly points = b2MakeArray(b2_maxManifoldPoints, b2ManifoldPoint);

    /** Not use for Type::e_points */
    public readonly localNormal = new b2Vec2();

    /** Usage depends on manifold type */
    public readonly localPoint = new b2Vec2();

    public type = b2ManifoldType.e_circles;

    /** The number of manifold points */
    public pointCount = 0;

    public Reset(): void {
        for (let i = 0; i < b2_maxManifoldPoints; ++i) {
            // DEBUG: b2Assert(this.points[i] instanceof b2ManifoldPoint);
            this.points[i].Reset();
        }
        this.localNormal.SetZero();
        this.localPoint.SetZero();
        this.type = b2ManifoldType.e_circles;
        this.pointCount = 0;
    }

    public Copy(o: b2Manifold): b2Manifold {
        this.pointCount = o.pointCount;
        for (let i = 0; i < b2_maxManifoldPoints; ++i) {
            // DEBUG: b2Assert(this.points[i] instanceof b2ManifoldPoint);
            this.points[i].Copy(o.points[i]);
        }
        this.localNormal.Copy(o.localNormal);
        this.localPoint.Copy(o.localPoint);
        this.type = o.type;
        return this;
    }

    public Clone(): b2Manifold {
        return new b2Manifold().Copy(this);
    }
}

/**
 * This is used to compute the current state of a contact manifold.
 */
export class b2WorldManifold {
    /** World vector pointing from A to B */
    public readonly normal = new b2Vec2();

    /** World contact point (point of intersection) */
    public readonly points = b2MakeArray(b2_maxManifoldPoints, b2Vec2);

    /** A negative value indicates overlap, in meters */
    public readonly separations = b2MakeNumberArray(b2_maxManifoldPoints);

    private static Initialize_s_pointA = new b2Vec2();

    private static Initialize_s_pointB = new b2Vec2();

    private static Initialize_s_cA = new b2Vec2();

    private static Initialize_s_cB = new b2Vec2();

    private static Initialize_s_planePoint = new b2Vec2();

    private static Initialize_s_clipPoint = new b2Vec2();

    /**
     * Evaluate the manifold with supplied transforms. This assumes
     * modest motion from the original state. This does not change the
     * point count, impulses, etc. The radii must come from the shapes
     * that generated the manifold.
     */
    public Initialize(
        manifold: b2Manifold,
        xfA: b2Transform,
        radiusA: number,
        xfB: b2Transform,
        radiusB: number,
    ): void {
        if (manifold.pointCount === 0) {
            return;
        }

        switch (manifold.type) {
            case b2ManifoldType.e_circles: {
                this.normal.Set(1, 0);
                const pointA = b2Transform.MultiplyVec2(xfA, manifold.localPoint, b2WorldManifold.Initialize_s_pointA);
                const pointB = b2Transform.MultiplyVec2(
                    xfB,
                    manifold.points[0].localPoint,
                    b2WorldManifold.Initialize_s_pointB,
                );
                if (b2Vec2.DistanceSquared(pointA, pointB) > b2_epsilon_sq) {
                    b2Vec2.Subtract(pointB, pointA, this.normal).Normalize();
                }

                const cA = b2Vec2.AddScaled(pointA, radiusA, this.normal, b2WorldManifold.Initialize_s_cA);
                const cB = b2Vec2.SubtractScaled(pointB, radiusB, this.normal, b2WorldManifold.Initialize_s_cB);
                b2Vec2.Mid(cA, cB, this.points[0]);
                this.separations[0] = b2Vec2.Dot(b2Vec2.Subtract(cB, cA, b2Vec2.s_t0), this.normal);
                break;
            }

            case b2ManifoldType.e_faceA: {
                b2Rot.MultiplyVec2(xfA.q, manifold.localNormal, this.normal);
                const planePoint = b2Transform.MultiplyVec2(
                    xfA,
                    manifold.localPoint,
                    b2WorldManifold.Initialize_s_planePoint,
                );

                for (let i = 0; i < manifold.pointCount; ++i) {
                    const clipPoint = b2Transform.MultiplyVec2(
                        xfB,
                        manifold.points[i].localPoint,
                        b2WorldManifold.Initialize_s_clipPoint,
                    );
                    const s = radiusA - b2Vec2.Dot(b2Vec2.Subtract(clipPoint, planePoint, b2Vec2.s_t0), this.normal);
                    const cA = b2Vec2.AddScaled(clipPoint, s, this.normal, b2WorldManifold.Initialize_s_cA);
                    const cB = b2Vec2.SubtractScaled(clipPoint, radiusB, this.normal, b2WorldManifold.Initialize_s_cB);
                    b2Vec2.Mid(cA, cB, this.points[i]);
                    this.separations[i] = b2Vec2.Dot(b2Vec2.Subtract(cB, cA, b2Vec2.s_t0), this.normal);
                }
                break;
            }

            case b2ManifoldType.e_faceB: {
                b2Rot.MultiplyVec2(xfB.q, manifold.localNormal, this.normal);
                const planePoint = b2Transform.MultiplyVec2(
                    xfB,
                    manifold.localPoint,
                    b2WorldManifold.Initialize_s_planePoint,
                );

                for (let i = 0; i < manifold.pointCount; ++i) {
                    const clipPoint = b2Transform.MultiplyVec2(
                        xfA,
                        manifold.points[i].localPoint,
                        b2WorldManifold.Initialize_s_clipPoint,
                    );
                    const s = radiusB - b2Vec2.Dot(b2Vec2.Subtract(clipPoint, planePoint, b2Vec2.s_t0), this.normal);
                    const cB = b2Vec2.AddScaled(clipPoint, s, this.normal, b2WorldManifold.Initialize_s_cB);
                    const cA = b2Vec2.SubtractScaled(clipPoint, radiusA, this.normal, b2WorldManifold.Initialize_s_cA);
                    b2Vec2.Mid(cA, cB, this.points[i]);
                    this.separations[i] = b2Vec2.Dot(b2Vec2.Subtract(cA, cB, b2Vec2.s_t0), this.normal);
                }

                // Ensure normal points from A to B.
                this.normal.Negate();
                break;
            }
        }
    }
}

/**
 * This is used for determining the state of contact points.
 */
export enum b2PointState {
    /** Point does not exist */
    b2_nullState,
    /** Point was added in the update */
    b2_addState,
    /** Point persisted across the update */
    b2_persistState,
    /** Point was removed in the update */
    b2_removeState,
}

/**
 * Compute the point states given two manifolds. The states pertain to the transition from manifold1
 * to manifold2. So state1 is either persist or remove while state2 is either add or persist.
 */
export function b2GetPointStates(
    state1: b2PointState[],
    state2: b2PointState[],
    manifold1: b2Manifold,
    manifold2: b2Manifold,
): void {
    // Detect persists and removes.
    let i: number;
    for (i = 0; i < manifold1.pointCount; ++i) {
        const { key } = manifold1.points[i].id;

        state1[i] = b2PointState.b2_removeState;

        for (let j = 0; j < manifold2.pointCount; ++j) {
            if (manifold2.points[j].id.key === key) {
                state1[i] = b2PointState.b2_persistState;
                break;
            }
        }
    }
    for (; i < b2_maxManifoldPoints; ++i) {
        state1[i] = b2PointState.b2_nullState;
    }

    // Detect persists and adds.
    for (i = 0; i < manifold2.pointCount; ++i) {
        const { key } = manifold2.points[i].id;

        state2[i] = b2PointState.b2_addState;

        for (let j = 0; j < manifold1.pointCount; ++j) {
            if (manifold1.points[j].id.key === key) {
                state2[i] = b2PointState.b2_persistState;
                break;
            }
        }
    }
    for (; i < b2_maxManifoldPoints; ++i) {
        state2[i] = b2PointState.b2_nullState;
    }
}

/**
 * Used for computing contact manifolds.
 */
export class b2ClipVertex {
    public readonly v = new b2Vec2();

    public readonly id = new b2ContactID();

    public Copy(other: b2ClipVertex): b2ClipVertex {
        this.v.Copy(other.v);
        this.id.Copy(other.id);
        return this;
    }
}

/**
 * Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
 */
export class b2RayCastInput {
    public readonly p1 = new b2Vec2();

    public readonly p2 = new b2Vec2();

    public maxFraction = 1;

    public Copy(o: b2RayCastInput): b2RayCastInput {
        this.p1.Copy(o.p1);
        this.p2.Copy(o.p2);
        this.maxFraction = o.maxFraction;
        return this;
    }
}

/**
 * Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
 * come from b2RayCastInput.
 */
export class b2RayCastOutput {
    public readonly normal = new b2Vec2();

    public fraction = 0;

    public Copy(o: b2RayCastOutput): b2RayCastOutput {
        this.normal.Copy(o.normal);
        this.fraction = o.fraction;
        return this;
    }
}

/**
 * An axis aligned bounding box.
 */
export class b2AABB {
    /** The lower vertex */
    public readonly lowerBound = new b2Vec2();

    /** The upper vertex */
    public readonly upperBound = new b2Vec2();

    public Copy(o: b2AABB): b2AABB {
        this.lowerBound.Copy(o.lowerBound);
        this.upperBound.Copy(o.upperBound);
        return this;
    }

    /**
     * Verify that the bounds are sorted.
     */
    public IsValid(): boolean {
        return (
            this.lowerBound.IsValid() &&
            this.upperBound.IsValid() &&
            this.upperBound.x >= this.lowerBound.x &&
            this.upperBound.y >= this.lowerBound.y
        );
    }

    /**
     * Get the center of the AABB.
     */
    public GetCenter(out: XY) {
        return b2Vec2.Mid(this.lowerBound, this.upperBound, out);
    }

    /**
     * Get the extents of the AABB (half-widths).
     */
    public GetExtents(out: XY) {
        return b2Vec2.Extents(this.lowerBound, this.upperBound, out);
    }

    /**
     * Get the perimeter length
     */
    public GetPerimeter(): number {
        const wx = this.upperBound.x - this.lowerBound.x;
        const wy = this.upperBound.y - this.lowerBound.y;
        return 2 * (wx + wy);
    }

    /**
     * Combine an AABB into this one.
     */
    public Combine1(aabb: b2AABB): b2AABB {
        this.lowerBound.x = Math.min(this.lowerBound.x, aabb.lowerBound.x);
        this.lowerBound.y = Math.min(this.lowerBound.y, aabb.lowerBound.y);
        this.upperBound.x = Math.max(this.upperBound.x, aabb.upperBound.x);
        this.upperBound.y = Math.max(this.upperBound.y, aabb.upperBound.y);
        return this;
    }

    /**
     * Combine two AABBs into this one.
     */
    public Combine2(aabb1: b2AABB, aabb2: b2AABB): b2AABB {
        this.lowerBound.x = Math.min(aabb1.lowerBound.x, aabb2.lowerBound.x);
        this.lowerBound.y = Math.min(aabb1.lowerBound.y, aabb2.lowerBound.y);
        this.upperBound.x = Math.max(aabb1.upperBound.x, aabb2.upperBound.x);
        this.upperBound.y = Math.max(aabb1.upperBound.y, aabb2.upperBound.y);
        return this;
    }

    public static Combine(aabb1: b2AABB, aabb2: b2AABB, out: b2AABB): b2AABB {
        out.Combine2(aabb1, aabb2);
        return out;
    }

    /**
     * Does this aabb contain the provided AABB.
     */
    public Contains(aabb: b2AABB): boolean {
        return (
            this.lowerBound.x <= aabb.lowerBound.x &&
            this.lowerBound.y <= aabb.lowerBound.y &&
            aabb.upperBound.x <= this.upperBound.x &&
            aabb.upperBound.y <= this.upperBound.y
        );
    }

    // From Real-time Collision Detection, p179.
    public RayCast(output: b2RayCastOutput, input: b2RayCastInput): boolean {
        let tmin = -b2_maxFloat;
        let tmax = b2_maxFloat;

        const p_x = input.p1.x;
        const p_y = input.p1.y;
        const d_x = input.p2.x - input.p1.x;
        const d_y = input.p2.y - input.p1.y;
        const absD_x = Math.abs(d_x);
        const absD_y = Math.abs(d_y);

        const { normal } = output;

        if (absD_x < b2_epsilon) {
            // Parallel.
            if (p_x < this.lowerBound.x || this.upperBound.x < p_x) {
                return false;
            }
        } else {
            const inv_d = 1 / d_x;
            let t1 = (this.lowerBound.x - p_x) * inv_d;
            let t2 = (this.upperBound.x - p_x) * inv_d;

            // Sign of the normal vector.
            let s = -1;

            if (t1 > t2) {
                const t3 = t1;
                t1 = t2;
                t2 = t3;
                s = 1;
            }

            // Push the min up
            if (t1 > tmin) {
                normal.x = s;
                normal.y = 0;
                tmin = t1;
            }

            // Pull the max down
            tmax = Math.min(tmax, t2);

            if (tmin > tmax) {
                return false;
            }
        }

        if (absD_y < b2_epsilon) {
            // Parallel.
            if (p_y < this.lowerBound.y || this.upperBound.y < p_y) {
                return false;
            }
        } else {
            const inv_d = 1 / d_y;
            let t1 = (this.lowerBound.y - p_y) * inv_d;
            let t2 = (this.upperBound.y - p_y) * inv_d;

            // Sign of the normal vector.
            let s = -1;

            if (t1 > t2) {
                const t3 = t1;
                t1 = t2;
                t2 = t3;
                s = 1;
            }

            // Push the min up
            if (t1 > tmin) {
                normal.x = 0;
                normal.y = s;
                tmin = t1;
            }

            // Pull the max down
            tmax = Math.min(tmax, t2);

            if (tmin > tmax) {
                return false;
            }
        }

        // Does the ray start inside the box?
        // Does the ray intersect beyond the max fraction?
        if (tmin < 0 || input.maxFraction < tmin) {
            return false;
        }

        // Intersection.
        output.fraction = tmin;

        return true;
    }

    public TestContain(point: XY): boolean {
        if (point.x < this.lowerBound.x || this.upperBound.x < point.x) {
            return false;
        }
        if (point.y < this.lowerBound.y || this.upperBound.y < point.y) {
            return false;
        }
        return true;
    }

    public TestOverlap(other: b2AABB): boolean {
        if (this.upperBound.x < other.lowerBound.x) {
            return false;
        }
        if (this.upperBound.y < other.lowerBound.y) {
            return false;
        }
        if (other.upperBound.x < this.lowerBound.x) {
            return false;
        }
        if (other.upperBound.y < this.lowerBound.y) {
            return false;
        }
        return true;
    }
}

/**
 * Clipping for contact manifolds.
 * Sutherland-Hodgman clipping.
 */
export function b2ClipSegmentToLine(
    vOut: readonly [b2ClipVertex, b2ClipVertex],
    [vIn0, vIn1]: readonly [b2ClipVertex, b2ClipVertex],
    normal: b2Vec2,
    offset: number,
    vertexIndexA: number,
): number {
    // Start with no output points
    let count = 0;

    // Calculate the distance of end points to the line
    const distance0 = b2Vec2.Dot(normal, vIn0.v) - offset;
    const distance1 = b2Vec2.Dot(normal, vIn1.v) - offset;

    // If the points are behind the plane
    if (distance0 <= 0) vOut[count++].Copy(vIn0);
    if (distance1 <= 0) vOut[count++].Copy(vIn1);

    // If the points are on different sides of the plane
    if (distance0 * distance1 < 0) {
        // Find intersection point of edge and plane
        const interp = distance0 / (distance0 - distance1);
        const { v, id } = vOut[count];
        v.x = vIn0.v.x + interp * (vIn1.v.x - vIn0.v.x);
        v.y = vIn0.v.y + interp * (vIn1.v.y - vIn0.v.y);

        // VertexA is hitting edgeB.
        id.cf.indexA = vertexIndexA;
        id.cf.indexB = vIn0.id.cf.indexB;
        id.cf.typeA = b2ContactFeatureType.e_vertex;
        id.cf.typeB = b2ContactFeatureType.e_face;
        ++count;

        // b2Assert(count === 2);
    }

    return count;
}

const b2TestOverlap_s_input = new b2DistanceInput();
const b2TestOverlap_s_simplexCache = new b2SimplexCache();
const b2TestOverlap_s_output = new b2DistanceOutput();
/**
 * Determine if two generic shapes overlap.
 */
export function b2TestOverlap(
    shapeA: b2Shape,
    indexA: number,
    shapeB: b2Shape,
    indexB: number,
    xfA: b2Transform,
    xfB: b2Transform,
): boolean {
    const input = b2TestOverlap_s_input.Reset();
    input.proxyA.SetShape(shapeA, indexA);
    input.proxyB.SetShape(shapeB, indexB);
    input.transformA.Copy(xfA);
    input.transformB.Copy(xfB);
    input.useRadii = true;

    const simplexCache = b2TestOverlap_s_simplexCache.Reset();
    simplexCache.count = 0;

    const output = b2TestOverlap_s_output.Reset();

    b2Distance(output, simplexCache, input);

    return output.distance < 10 * b2_epsilon;
}

/** Convex hull used for polygon collision */
export type b2Hull = Array<Readonly<XY>>;

const b2RecurseHull_s_e = new b2Vec2();
const b2RecurseHull_s_c = new b2Vec2();
const emptyHull: Readonly<b2Hull> = [];

/** quickhull recursion */
function b2RecurseHull(p1: Readonly<XY>, p2: Readonly<XY>, ps: Array<Readonly<XY>>): Readonly<b2Hull> {
    if (ps.length === 0) {
        return emptyHull;
    }

    // create an edge vector pointing from p1 to p2
    const e = b2Vec2.Subtract(p2, p1, b2RecurseHull_s_e);
    e.Normalize();

    // discard points left of e and find point furthest to the right of e
    const rightPoints: Array<Readonly<XY>> = [];

    let bestIndex = 0;
    let bestDistance = b2Vec2.Cross(b2Vec2.Subtract(ps[bestIndex], p1, b2RecurseHull_s_c), e);
    if (bestDistance > 0) {
        rightPoints.push(ps[bestIndex]);
    }

    for (let i = 1; i < ps.length; ++i) {
        const distance = b2Vec2.Cross(b2Vec2.Subtract(ps[i], p1, b2RecurseHull_s_c), e);
        if (distance > bestDistance) {
            bestIndex = i;
            bestDistance = distance;
        }

        if (distance > 0) {
            rightPoints.push(ps[i]);
        }
    }

    if (bestDistance < 2 * b2_linearSlop) {
        return emptyHull;
    }

    const bestPoint = ps[bestIndex];

    // compute hull to the right of p1-bestPoint
    const hull1 = b2RecurseHull(p1, bestPoint, rightPoints);

    // compute hull to the right of bestPoint-p2
    const hull2 = b2RecurseHull(bestPoint, p2, rightPoints);

    // stich together hulls
    const hull = [...hull1, bestPoint, ...hull2];

    b2Assert(hull.length < b2_maxPolygonVertices);

    return hull;
}

const b2ComputeHull_s_e = new b2Vec2();
const b2ComputeHull_s_v = new b2Vec2();
const b2ComputeHull_s_c = new b2Vec2();
const b2ComputeHull_s_d = new b2Vec2();
const b2ComputeHull_s_aabb = new b2AABB();

/**
 * Compute the convex hull of a set of points.
 * quickhull algorithm
 * - merges vertices based on b2_linearSlop
 * - removes collinear points using b2_linearSlop
 * - returns an empty hull if it fails
 *
 * Some failure cases:
 * - all points very close together
 * - all points on a line
 * - less than 3 points
 * - more than b2_maxPolygonVertices points
 *
 * This welds close points and removes collinear points.
 *
 * @returns an empty hull if it fails.
 */
export function b2ComputeHull(points: ReadonlyArray<Readonly<XY>>, count: number): Readonly<b2Hull> {
    if (count < 3 || count > b2_maxPolygonVertices) {
        // check your data
        return emptyHull;
    }

    count = Math.min(count, b2_maxPolygonVertices);

    const aabb = b2ComputeHull_s_aabb;
    aabb.lowerBound.Set(b2_maxFloat, b2_maxFloat);
    aabb.upperBound.Set(-b2_maxFloat, -b2_maxFloat);

    // Perform aggressive point welding. First point always remains.
    // Also compute the bounding box for later.
    const ps: Array<Readonly<XY>> = [];
    const tolSqr = 16 * b2_linearSlop * b2_linearSlop;
    for (let i = 0; i < count; ++i) {
        b2Vec2.Min(aabb.lowerBound, points[i], aabb.lowerBound);
        b2Vec2.Max(aabb.upperBound, points[i], aabb.upperBound);

        const vi = points[i];

        let unique = true;
        for (let j = 0; j < i; ++j) {
            const vj = points[j];

            const distSqr = b2Vec2.DistanceSquared(vi, vj);
            if (distSqr < tolSqr) {
                unique = false;
                break;
            }
        }

        if (unique) {
            ps.push(vi);
        }
    }

    let n = ps.length;

    if (n < 3) {
        // all points very close together, check your data and check your scale
        return emptyHull;
    }

    // Find an extreme point as the first point on the hull
    const c = aabb.GetCenter(b2ComputeHull_s_c);
    let i1 = 0;
    let dsq1 = b2Vec2.DistanceSquared(c, ps[i1]);
    for (let i = 1; i < n; ++i) {
        const dsq = b2Vec2.DistanceSquared(c, ps[i]);
        if (dsq > dsq1) {
            i1 = i;
            dsq1 = dsq;
        }
    }

    // remove p1 from working set
    const p1 = ps[i1];
    ps[i1] = ps[n - 1];
    n -= 1;

    let i2 = 0;
    let dsq2 = b2Vec2.DistanceSquared(p1, ps[i2]);
    for (let i = 1; i < n; ++i) {
        const dsq = b2Vec2.DistanceSquared(p1, ps[i]);
        if (dsq > dsq2) {
            i2 = i;
            dsq2 = dsq;
        }
    }

    // remove p2 from working set
    const p2 = ps[i2];
    ps[i2] = ps[n - 1];
    n -= 1;

    // split the points into points that are left and right of the line p1-p2.
    const rightPoints: Array<Readonly<XY>> = [];
    const leftPoints: Array<Readonly<XY>> = [];

    const e = b2Vec2.Subtract(p2, p1, b2ComputeHull_s_e);
    e.Normalize();

    for (let i = 0; i < n; ++i) {
        const d = b2Vec2.Cross(b2Vec2.Subtract(ps[i], p1, b2ComputeHull_s_d), e);

        // slop used here to skip points that are very close to the line p1-p2
        if (d >= 2 * b2_linearSlop) {
            rightPoints.push(ps[i]);
        } else if (d <= -2 * b2_linearSlop) {
            leftPoints.push(ps[i]);
        }
    }

    // compute hulls on right and left
    const hull1 = b2RecurseHull(p1, p2, rightPoints);
    const hull2 = b2RecurseHull(p2, p1, leftPoints);

    if (hull1.length === 0 && hull2.length === 0) {
        // all points collinear
        return emptyHull;
    }

    // stitch hulls together, preserving CCW winding order
    const hull = [p1, ...hull1, p2, ...hull2];

    b2Assert(hull.length <= b2_maxPolygonVertices);

    // merge collinear
    let searching = true;
    while (searching && hull.length > 2) {
        searching = false;

        for (let i = 0; i < hull.length; ++i) {
            const i1b = i;
            const i2b = (i + 1) % hull.length;
            const i3b = (i + 2) % hull.length;

            const p1b = hull[i1b];
            const p2b = hull[i2b];
            const p3b = hull[i3b];

            const eb = b2Vec2.Subtract(p3b, p1b, b2ComputeHull_s_e);
            eb.Normalize();

            const v = b2Vec2.Subtract(p2b, p1b, b2ComputeHull_s_v);
            const distance = b2Vec2.Cross(v, eb);
            if (distance <= 2 * b2_linearSlop) {
                // remove midpoint from hull
                hull.splice(i2b, 1);

                // continue searching for collinear points
                searching = true;

                break;
            }
        }
    }

    if (hull.length < 3) {
        // all points collinear, shouldn't be reached since this was validated above
        hull.length = 0;
    }

    return hull;
}

const b2ValidateHull_s_e = new b2Vec2();
const b2ValidateHull_s_d = new b2Vec2();
/**
 * This determines if a hull is valid. Checks for:
 * - convexity
 * - collinear points
 * This is expensive and should not be called at runtime.
 */
export function b2ValidateHull(hull: Readonly<b2Hull>, count: number): boolean {
    if (count < 3 || b2_maxPolygonVertices < count) {
        return false;
    }

    // test that every point is behind every edge
    for (let i = 0; i < count; ++i) {
        // create an edge vector
        const i1 = i;
        const i2 = i < count - 1 ? i1 + 1 : 0;
        const p = hull[i1];
        const e = b2Vec2.Subtract(hull[i2], p, b2ValidateHull_s_e);
        e.Normalize();

        for (let j = 0; j < count; ++j) {
            // skip points that subtend the current edge
            if (j === i1 || j === i2) {
                continue;
            }

            const distance = b2Vec2.Cross(b2Vec2.Subtract(hull[j], p, b2ValidateHull_s_d), e);
            if (distance >= 0) {
                return false;
            }
        }
    }

    // test for collinear points
    for (let i = 0; i < count; ++i) {
        const i1 = i;
        const i2 = (i + 1) % count;
        const i3 = (i + 2) % count;

        const p1 = hull[i1];
        const p2 = hull[i2];
        const p3 = hull[i3];

        const e = b2Vec2.Subtract(p3, p1, b2ValidateHull_s_e);
        e.Normalize();

        const distance = b2Vec2.Cross(b2Vec2.Subtract(p2, p1, b2ValidateHull_s_d), e);
        if (distance <= b2_linearSlop) {
            // p1-p2-p3 are collinear
            return false;
        }
    }

    return true;
}
