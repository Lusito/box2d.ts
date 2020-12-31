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
// DEBUG: import { Assert } from "../common/b2_common";
import {
    MAX_FLOAT,
    EPSILON,
    EPSILON_SQUARED,
    MAX_MANIFOLD_POINTS,
    MakeNumberArray,
    MakeArray,
} from "../common/b2_common";
import { Vec2, Rot, Transform, XY } from "../common/b2_math";
import type { Shape } from "./b2_shape";
import { Distance, DistanceInput, DistanceOutput, SimplexCache } from "./b2_distance";

export enum ContactFeatureType {
    Vertex = 0,
    Face = 1,
}

/**
 * The features that intersect to form the contact point
 * This must be 4 bytes or less.
 */
export class ContactFeature {
    private m_key = 0;

    private m_key_invalid = false;

    /** Feature index on shapeA */
    private m_indexA = 0;

    /** Feature index on shapeB */
    private m_indexB = 0;

    /** The feature type on shapeA */
    private m_typeA = ContactFeatureType.Vertex;

    /** The feature type on shapeB */
    private m_typeB = ContactFeatureType.Vertex;

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
export class ContactID {
    public readonly cf = new ContactFeature();

    public Copy(o: ContactID): ContactID {
        this.key = o.key;
        return this;
    }

    public Clone(): ContactID {
        return new ContactID().Copy(this);
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
 * -Circles: the local center of circleB
 * -FaceA: the local center of cirlceB or the clip point of polygonB
 * -FaceB: the clip point of polygonA
 * This structure is stored across time steps, so we keep it small.
 * Note: the impulses are used for internal caching and may not
 * provide reliable contact forces, especially for high speed collisions.
 */
export class ManifoldPoint {
    /** Usage depends on manifold type */
    public readonly localPoint = new Vec2();

    /** The non-penetration impulse */
    public normalImpulse = 0;

    /** The friction impulse */
    public tangentImpulse = 0;

    /** Uniquely identifies a contact point between two shapes */
    public readonly id = new ContactID();

    public Reset(): void {
        this.localPoint.SetZero();
        this.normalImpulse = 0;
        this.tangentImpulse = 0;
        this.id.key = 0;
    }

    public Copy(o: ManifoldPoint): ManifoldPoint {
        this.localPoint.Copy(o.localPoint);
        this.normalImpulse = o.normalImpulse;
        this.tangentImpulse = o.tangentImpulse;
        this.id.Copy(o.id);
        return this;
    }
}

export enum ManifoldType {
    Circles,
    FaceA,
    FaceB,
}

/**
 * A manifold for two touching convex shapes.
 * Box2D supports multiple types of contact:
 * - clip point versus plane with radius
 * - point versus point with radius (circles)
 * The local point usage depends on the manifold type:
 * -Circles: the local center of circleA
 * -FaceA: the center of faceA
 * -FaceB: the center of faceB
 * Similarly the local normal usage:
 * -Circles: not used
 * -FaceA: the normal on polygonA
 * -FaceB: the normal on polygonB
 * We store contacts in this way so that position correction can
 * account for movement, which is critical for continuous physics.
 * All contact scenarios must be expressed in one of these types.
 * This structure is stored across time steps, so we keep it small.
 */
export class Manifold {
    /** The points of contact */
    public readonly points = MakeArray(MAX_MANIFOLD_POINTS, ManifoldPoint);

    /** Not use for Type::Points */
    public readonly localNormal = new Vec2();

    /** Usage depends on manifold type */
    public readonly localPoint = new Vec2();

    public type = ManifoldType.Circles;

    /** The number of manifold points */
    public pointCount = 0;

    public Reset(): void {
        for (let i = 0; i < MAX_MANIFOLD_POINTS; ++i) {
            // DEBUG: Assert(this.points[i] instanceof ManifoldPoint);
            this.points[i].Reset();
        }
        this.localNormal.SetZero();
        this.localPoint.SetZero();
        this.type = ManifoldType.Circles;
        this.pointCount = 0;
    }

    public Copy(o: Manifold): Manifold {
        this.pointCount = o.pointCount;
        for (let i = 0; i < MAX_MANIFOLD_POINTS; ++i) {
            // DEBUG: Assert(this.points[i] instanceof ManifoldPoint);
            this.points[i].Copy(o.points[i]);
        }
        this.localNormal.Copy(o.localNormal);
        this.localPoint.Copy(o.localPoint);
        this.type = o.type;
        return this;
    }

    public Clone(): Manifold {
        return new Manifold().Copy(this);
    }
}

/**
 * This is used to compute the current state of a contact manifold.
 */
export class WorldManifold {
    /** World vector pointing from A to B */
    public readonly normal = new Vec2();

    /** World contact point (point of intersection) */
    public readonly points = MakeArray(MAX_MANIFOLD_POINTS, Vec2);

    /** A negative value indicates overlap, in meters */
    public readonly separations = MakeNumberArray(MAX_MANIFOLD_POINTS);

    private static Initialize_s_pointA = new Vec2();

    private static Initialize_s_pointB = new Vec2();

    private static Initialize_s_cA = new Vec2();

    private static Initialize_s_cB = new Vec2();

    private static Initialize_s_planePoint = new Vec2();

    private static Initialize_s_clipPoint = new Vec2();

    public Initialize(manifold: Manifold, xfA: Transform, radiusA: number, xfB: Transform, radiusB: number): void {
        if (manifold.pointCount === 0) {
            return;
        }

        switch (manifold.type) {
            case ManifoldType.Circles: {
                this.normal.Set(1, 0);
                const pointA = Transform.MultiplyVec2(xfA, manifold.localPoint, WorldManifold.Initialize_s_pointA);
                const pointB = Transform.MultiplyVec2(
                    xfB,
                    manifold.points[0].localPoint,
                    WorldManifold.Initialize_s_pointB,
                );
                if (Vec2.DistanceSquared(pointA, pointB) > EPSILON_SQUARED) {
                    Vec2.Subtract(pointB, pointA, this.normal).Normalize();
                }

                const cA = Vec2.AddScaled(pointA, radiusA, this.normal, WorldManifold.Initialize_s_cA);
                const cB = Vec2.SubtractScaled(pointB, radiusB, this.normal, WorldManifold.Initialize_s_cB);
                Vec2.Mid(cA, cB, this.points[0]);
                this.separations[0] = Vec2.Dot(Vec2.Subtract(cB, cA, Vec2.s_t0), this.normal);
                break;
            }

            case ManifoldType.FaceA: {
                Rot.MultiplyVec2(xfA.q, manifold.localNormal, this.normal);
                const planePoint = Transform.MultiplyVec2(
                    xfA,
                    manifold.localPoint,
                    WorldManifold.Initialize_s_planePoint,
                );

                for (let i = 0; i < manifold.pointCount; ++i) {
                    const clipPoint = Transform.MultiplyVec2(
                        xfB,
                        manifold.points[i].localPoint,
                        WorldManifold.Initialize_s_clipPoint,
                    );
                    const s = radiusA - Vec2.Dot(Vec2.Subtract(clipPoint, planePoint, Vec2.s_t0), this.normal);
                    const cA = Vec2.AddScaled(clipPoint, s, this.normal, WorldManifold.Initialize_s_cA);
                    const cB = Vec2.SubtractScaled(clipPoint, radiusB, this.normal, WorldManifold.Initialize_s_cB);
                    Vec2.Mid(cA, cB, this.points[i]);
                    this.separations[i] = Vec2.Dot(Vec2.Subtract(cB, cA, Vec2.s_t0), this.normal);
                }
                break;
            }

            case ManifoldType.FaceB: {
                Rot.MultiplyVec2(xfB.q, manifold.localNormal, this.normal);
                const planePoint = Transform.MultiplyVec2(
                    xfB,
                    manifold.localPoint,
                    WorldManifold.Initialize_s_planePoint,
                );

                for (let i = 0; i < manifold.pointCount; ++i) {
                    const clipPoint = Transform.MultiplyVec2(
                        xfA,
                        manifold.points[i].localPoint,
                        WorldManifold.Initialize_s_clipPoint,
                    );
                    const s = radiusB - Vec2.Dot(Vec2.Subtract(clipPoint, planePoint, Vec2.s_t0), this.normal);
                    const cB = Vec2.AddScaled(clipPoint, s, this.normal, WorldManifold.Initialize_s_cB);
                    const cA = Vec2.SubtractScaled(clipPoint, radiusA, this.normal, WorldManifold.Initialize_s_cA);
                    Vec2.Mid(cA, cB, this.points[i]);
                    this.separations[i] = Vec2.Dot(Vec2.Subtract(cA, cB, Vec2.s_t0), this.normal);
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
export enum PointState {
    /** Point does not exist */
    Null,
    /** Point was added in the update */
    Add,
    /** Point persisted across the update */
    Persist,
    /** Point was removed in the update */
    Remove,
}

/**
 * Compute the point states given two manifolds. The states pertain to the transition from manifold1
 * to manifold2. So state1 is either persist or remove while state2 is either add or persist.
 */
export function GetPointStates(
    state1: PointState[],
    state2: PointState[],
    manifold1: Manifold,
    manifold2: Manifold,
): void {
    // Detect persists and removes.
    let i: number;
    for (i = 0; i < manifold1.pointCount; ++i) {
        const { key } = manifold1.points[i].id;

        state1[i] = PointState.Remove;

        for (let j = 0; j < manifold2.pointCount; ++j) {
            if (manifold2.points[j].id.key === key) {
                state1[i] = PointState.Persist;
                break;
            }
        }
    }
    for (; i < MAX_MANIFOLD_POINTS; ++i) {
        state1[i] = PointState.Null;
    }

    // Detect persists and adds.
    for (i = 0; i < manifold2.pointCount; ++i) {
        const { key } = manifold2.points[i].id;

        state2[i] = PointState.Add;

        for (let j = 0; j < manifold1.pointCount; ++j) {
            if (manifold1.points[j].id.key === key) {
                state2[i] = PointState.Persist;
                break;
            }
        }
    }
    for (; i < MAX_MANIFOLD_POINTS; ++i) {
        state2[i] = PointState.Null;
    }
}

/**
 * Used for computing contact manifolds.
 */
export class ClipVertex {
    public readonly v = new Vec2();

    public readonly id = new ContactID();

    public Copy(other: ClipVertex): ClipVertex {
        this.v.Copy(other.v);
        this.id.Copy(other.id);
        return this;
    }
}

/**
 * Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
 */
export class RayCastInput {
    public readonly p1 = new Vec2();

    public readonly p2 = new Vec2();

    public maxFraction = 1;

    public Copy(o: RayCastInput): RayCastInput {
        this.p1.Copy(o.p1);
        this.p2.Copy(o.p2);
        this.maxFraction = o.maxFraction;
        return this;
    }
}

/**
 * Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
 * come from RayCastInput.
 */
export class RayCastOutput {
    public readonly normal = new Vec2();

    public fraction = 0;

    public Copy(o: RayCastOutput): RayCastOutput {
        this.normal.Copy(o.normal);
        this.fraction = o.fraction;
        return this;
    }
}

/**
 * An axis aligned bounding box.
 */
export class AABB {
    /** The lower vertex */
    public readonly lowerBound = new Vec2();

    /** The upper vertex */
    public readonly upperBound = new Vec2();

    public Copy(o: AABB): AABB {
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
        return Vec2.Mid(this.lowerBound, this.upperBound, out);
    }

    /**
     * Get the extents of the AABB (half-widths).
     */
    public GetExtents(out: XY) {
        return Vec2.Extents(this.lowerBound, this.upperBound, out);
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
    public Combine1(aabb: AABB): AABB {
        this.lowerBound.x = Math.min(this.lowerBound.x, aabb.lowerBound.x);
        this.lowerBound.y = Math.min(this.lowerBound.y, aabb.lowerBound.y);
        this.upperBound.x = Math.max(this.upperBound.x, aabb.upperBound.x);
        this.upperBound.y = Math.max(this.upperBound.y, aabb.upperBound.y);
        return this;
    }

    /**
     * Combine two AABBs into this one.
     */
    public Combine2(aabb1: AABB, aabb2: AABB): AABB {
        this.lowerBound.x = Math.min(aabb1.lowerBound.x, aabb2.lowerBound.x);
        this.lowerBound.y = Math.min(aabb1.lowerBound.y, aabb2.lowerBound.y);
        this.upperBound.x = Math.max(aabb1.upperBound.x, aabb2.upperBound.x);
        this.upperBound.y = Math.max(aabb1.upperBound.y, aabb2.upperBound.y);
        return this;
    }

    public static Combine(aabb1: AABB, aabb2: AABB, out: AABB): AABB {
        out.Combine2(aabb1, aabb2);
        return out;
    }

    /**
     * Does this aabb contain the provided AABB.
     */
    public Contains(aabb: AABB): boolean {
        return (
            this.lowerBound.x <= aabb.lowerBound.x &&
            this.lowerBound.y <= aabb.lowerBound.y &&
            aabb.upperBound.x <= this.upperBound.x &&
            aabb.upperBound.y <= this.upperBound.y
        );
    }

    // From Real-time Collision Detection, p179.
    public RayCast(output: RayCastOutput, input: RayCastInput): boolean {
        let tmin = -MAX_FLOAT;
        let tmax = MAX_FLOAT;

        const p_x = input.p1.x;
        const p_y = input.p1.y;
        const d_x = input.p2.x - input.p1.x;
        const d_y = input.p2.y - input.p1.y;
        const absD_x = Math.abs(d_x);
        const absD_y = Math.abs(d_y);

        const { normal } = output;

        if (absD_x < EPSILON) {
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

        if (absD_y < EPSILON) {
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

    public TestOverlap(other: AABB): boolean {
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
 */
export function ClipSegmentToLine(
    vOut: readonly [ClipVertex, ClipVertex],
    [vIn0, vIn1]: readonly [ClipVertex, ClipVertex],
    normal: Vec2,
    offset: number,
    vertexIndexA: number,
): number {
    // Start with no output points
    let count = 0;

    // Calculate the distance of end points to the line
    const distance0 = Vec2.Dot(normal, vIn0.v) - offset;
    const distance1 = Vec2.Dot(normal, vIn1.v) - offset;

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
        id.cf.typeA = ContactFeatureType.Vertex;
        id.cf.typeB = ContactFeatureType.Face;
        ++count;

        // Assert(count === 2);
    }

    return count;
}

const TestOverlap_s_input = new DistanceInput();
const TestOverlap_s_simplexCache = new SimplexCache();
const TestOverlap_s_output = new DistanceOutput();
/**
 * Determine if two generic shapes overlap.
 */
export function TestOverlap(
    shapeA: Shape,
    indexA: number,
    shapeB: Shape,
    indexB: number,
    xfA: Transform,
    xfB: Transform,
): boolean {
    const input = TestOverlap_s_input.Reset();
    input.proxyA.SetShape(shapeA, indexA);
    input.proxyB.SetShape(shapeB, indexB);
    input.transformA.Copy(xfA);
    input.transformB.Copy(xfB);
    input.useRadii = true;

    const simplexCache = TestOverlap_s_simplexCache.Reset();
    simplexCache.count = 0;

    const output = TestOverlap_s_output.Reset();

    Distance(output, simplexCache, input);

    return output.distance < 10 * EPSILON;
}
