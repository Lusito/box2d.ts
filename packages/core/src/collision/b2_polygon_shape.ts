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

// DEBUG: import { b2Assert, b2_epsilon_sq } from "../common/b2_common";
import { b2Assert, b2MakeArray, b2_polygonRadius } from "../common/b2_common";
import { b2Color, b2Draw } from "../common/b2_draw";
import { b2Vec2, b2Rot, b2Transform, XY } from "../common/b2_math";
import { b2_maxPolygonVertices } from "../common/b2_settings";
import { b2AABB, b2ComputeHull, b2Hull, b2RayCastInput, b2RayCastOutput, b2ValidateHull } from "./b2_collision";
import { b2DistanceProxy } from "./b2_distance";
import { b2MassData, b2Shape, b2ShapeType } from "./b2_shape";

const temp = {
    ComputeCentroid: {
        s: new b2Vec2(),
        p1: new b2Vec2(),
        p2: new b2Vec2(),
        p3: new b2Vec2(),
        e1: new b2Vec2(),
        e2: new b2Vec2(),
    },
    TestPoint: {
        pLocal: new b2Vec2(),
    },
    ComputeAABB: {
        v: new b2Vec2(),
    },
    ComputeMass: {
        center: new b2Vec2(),
        s: new b2Vec2(),
        e1: new b2Vec2(),
        e2: new b2Vec2(),
    },
    Validate: {
        e: new b2Vec2(),
        v: new b2Vec2(),
    },
    Set: {
        r: new b2Vec2(),
        v: new b2Vec2(),
    },
    RayCast: {
        p1: new b2Vec2(),
        p2: new b2Vec2(),
        d: new b2Vec2(),
    },
    SetAsBox: {
        xf: new b2Transform(),
    },
};

function ComputeCentroid(vs: b2Vec2[], count: number, out: b2Vec2): b2Vec2 {
    // DEBUG: b2Assert(count >= 3);

    const c = out;
    c.SetZero();
    let area = 0;

    const { s, p1, p2, p3, e1, e2 } = temp.ComputeCentroid;

    // Get a reference point for forming triangles.
    // Use the first vertex to reduce round-off errors.
    s.Copy(vs[0]);

    const inv3 = 1 / 3;

    for (let i = 0; i < count; ++i) {
        // Triangle vertices.
        b2Vec2.Subtract(vs[0], s, p1);
        b2Vec2.Subtract(vs[i], s, p2);
        b2Vec2.Subtract(vs[i + 1 < count ? i + 1 : 0], s, p3);

        b2Vec2.Subtract(p2, p1, e1);
        b2Vec2.Subtract(p3, p1, e2);

        const D = b2Vec2.Cross(e1, e2);

        const triangleArea = 0.5 * D;
        area += triangleArea;

        // Area weighted centroid
        c.x += triangleArea * inv3 * (p1.x + p2.x + p3.x);
        c.y += triangleArea * inv3 * (p1.y + p2.y + p3.y);
    }

    // Centroid
    // DEBUG: b2Assert(area > b2_epsilon);
    const f = 1 / area;
    c.x = f * c.x + s.x;
    c.y = f * c.y + s.y;
    return c;
}

const setHull_s_edge = new b2Vec2();
/**
 * A solid convex polygon. It is assumed that the interior of the polygon is to
 * the left of each edge.
 * Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
 * In most cases you should not need many vertices for a convex polygon.
 */
export class b2PolygonShape extends b2Shape {
    public readonly m_centroid = new b2Vec2();

    public m_vertices: b2Vec2[] = [];

    public m_normals: b2Vec2[] = [];

    public m_count = 0;

    public constructor() {
        super(b2ShapeType.e_polygon, b2_polygonRadius);
    }

    /**
     * Implement b2Shape.
     */
    public Clone(): b2PolygonShape {
        return new b2PolygonShape().Copy(this);
    }

    public Copy(other: b2PolygonShape): b2PolygonShape {
        super.Copy(other);

        // DEBUG: b2Assert(other instanceof b2PolygonShape);

        this.m_centroid.Copy(other.m_centroid);
        this.m_count = other.m_count;
        this.m_vertices = b2MakeArray(this.m_count, b2Vec2);
        this.m_normals = b2MakeArray(this.m_count, b2Vec2);
        for (let i = 0; i < this.m_count; ++i) {
            this.m_vertices[i].Copy(other.m_vertices[i]);
            this.m_normals[i].Copy(other.m_normals[i]);
        }
        return this;
    }

    /**
     * @see b2Shape::GetChildCount
     */
    public GetChildCount(): number {
        return 1;
    }

    /**
     * Create a convex hull from the given array of local points.
     * The count must be in the range [3, b2_maxPolygonVertices].
     *
     * @warning the points may be re-ordered, even if they form a convex polygon
     * @warning if this fails then the polygon is invalid
     * @returns true if valid
     */
    public Set(vertices: XY[], count = vertices.length): boolean {
        const hull = b2ComputeHull(vertices, count);

        if (hull.length < 3) {
            return false;
        }

        this.SetHull(hull, hull.length);

        return true;
    }

    /**
     * Create a polygon from a given convex hull (see b2ComputeHull).
     * @warning the hull must be valid or this will crash or have unexpected behavior
     */
    public SetHull(hull: Readonly<b2Hull>, count: number): b2PolygonShape {
        b2Assert(count >= 3);

        this.m_count = count;

        // Copy vertices
        this.m_vertices = b2MakeArray(count, b2Vec2);
        for (let i = 0; i < count; ++i) {
            this.m_vertices[i].Copy(hull[i]);
        }

        // Compute normals. Ensure the edges have non-zero length.
        this.m_normals = b2MakeArray(count, b2Vec2);
        for (let i = 0; i < this.m_count; ++i) {
            const i1 = i;
            const i2 = i + 1 < this.m_count ? i + 1 : 0;
            const edge = b2Vec2.Subtract(this.m_vertices[i2], this.m_vertices[i1], setHull_s_edge);
            // DEBUG: b2Assert(edge.LengthSquared() > b2_epsilon * b2_epsilon);
            b2Vec2.CrossVec2Scalar(edge, 1, this.m_normals[i]);
            this.m_normals[i].Normalize();
        }

        // Compute the polygon centroid.
        ComputeCentroid(this.m_vertices, this.m_count, this.m_centroid);

        return this;
    }

    /**
     * Build vertices to represent an axis-aligned box centered on the local origin.
     *
     * @param hx The half-width.
     * @param hy The half-height.
     * @param center The center of the box in local coordinates.
     * @param angle The rotation of the box in local coordinates.
     */
    public SetAsBox(hx: number, hy: number, center?: XY, angle = 0): b2PolygonShape {
        this.m_count = 4;
        this.m_vertices = b2MakeArray(this.m_count, b2Vec2);
        this.m_normals = b2MakeArray(this.m_count, b2Vec2);
        this.m_vertices[0].Set(-hx, -hy);
        this.m_vertices[1].Set(hx, -hy);
        this.m_vertices[2].Set(hx, hy);
        this.m_vertices[3].Set(-hx, hy);
        this.m_normals[0].Set(0, -1);
        this.m_normals[1].Set(1, 0);
        this.m_normals[2].Set(0, 1);
        this.m_normals[3].Set(-1, 0);

        if (center) {
            this.m_centroid.Copy(center);

            const { xf } = temp.SetAsBox;
            xf.SetPosition(center);
            xf.SetRotationAngle(angle);

            // Transform vertices and normals.
            for (let i = 0; i < this.m_count; ++i) {
                b2Transform.MultiplyVec2(xf, this.m_vertices[i], this.m_vertices[i]);
                b2Rot.MultiplyVec2(xf.q, this.m_normals[i], this.m_normals[i]);
            }
        } else {
            this.m_centroid.SetZero();
        }

        return this;
    }

    /**
     * @see b2Shape::TestPoint
     */
    public TestPoint(xf: b2Transform, p: XY): boolean {
        const pLocal = b2Transform.TransposeMultiplyVec2(xf, p, temp.TestPoint.pLocal);

        for (let i = 0; i < this.m_count; ++i) {
            const dot = b2Vec2.Dot(this.m_normals[i], b2Vec2.Subtract(pLocal, this.m_vertices[i], b2Vec2.s_t0));
            if (dot > 0) {
                return false;
            }
        }

        return true;
    }

    /**
     * Implement b2Shape.
     *
     * @note because the polygon is solid, rays that start inside do not hit because the normal is
     * not defined.
     */
    public RayCast(output: b2RayCastOutput, input: b2RayCastInput, xf: b2Transform, _childIndex: number): boolean {
        // Put the ray into the polygon's frame of reference.
        const p1 = b2Transform.TransposeMultiplyVec2(xf, input.p1, temp.RayCast.p1);
        const p2 = b2Transform.TransposeMultiplyVec2(xf, input.p2, temp.RayCast.p2);
        const d = b2Vec2.Subtract(p2, p1, temp.RayCast.d);

        let lower = 0;
        let upper = input.maxFraction;

        let index = -1;

        for (let i = 0; i < this.m_count; ++i) {
            // p = p1 + a * d
            // dot(normal, p - v) = 0
            // dot(normal, p1 - v) + a * dot(normal, d) = 0
            const numerator = b2Vec2.Dot(this.m_normals[i], b2Vec2.Subtract(this.m_vertices[i], p1, b2Vec2.s_t0));
            const denominator = b2Vec2.Dot(this.m_normals[i], d);

            if (denominator === 0) {
                if (numerator < 0) {
                    return false;
                }
                // Note: we want this predicate without division:
                // lower < numerator / denominator, where denominator < 0
                // Since denominator < 0, we have to flip the inequality:
                // lower < numerator / denominator <==> denominator * lower > numerator.
            } else if (denominator < 0 && numerator < lower * denominator) {
                // Increase lower.
                // The segment enters this half-space.
                lower = numerator / denominator;
                index = i;
            } else if (denominator > 0 && numerator < upper * denominator) {
                // Decrease upper.
                // The segment exits this half-space.
                upper = numerator / denominator;
            }

            // The use of epsilon here causes the assert on lower to trip
            // in some cases. Apparently the use of epsilon was to make edge
            // shapes work, but now those are handled separately.
            // if (upper < lower - b2_epsilon)
            if (upper < lower) {
                return false;
            }
        }

        // DEBUG: b2Assert(0 <= lower && lower <= input.maxFraction);

        if (index >= 0) {
            output.fraction = lower;
            b2Rot.MultiplyVec2(xf.q, this.m_normals[index], output.normal);
            return true;
        }

        return false;
    }

    /**
     * @see b2Shape::ComputeAABB
     */
    public ComputeAABB(aabb: b2AABB, xf: b2Transform, _childIndex: number): void {
        const lower = b2Transform.MultiplyVec2(xf, this.m_vertices[0], aabb.lowerBound);
        const upper = aabb.upperBound.Copy(lower);

        for (let i = 1; i < this.m_count; ++i) {
            const v = b2Transform.MultiplyVec2(xf, this.m_vertices[i], temp.ComputeAABB.v);
            b2Vec2.Min(lower, v, lower);
            b2Vec2.Max(upper, v, upper);
        }

        const r = this.m_radius;
        lower.SubtractXY(r, r);
        upper.AddXY(r, r);
    }

    /**
     * @see b2Shape::ComputeMass
     */
    public ComputeMass(massData: b2MassData, density: number): void {
        // Polygon mass, centroid, and inertia.
        // Let rho be the polygon density in mass per unit area.
        // Then:
        // mass = rho * int(dA)
        // centroid.x = (1/mass) * rho * int(x * dA)
        // centroid.y = (1/mass) * rho * int(y * dA)
        // I = rho * int((x*x + y*y) * dA)
        //
        // We can compute these integrals by summing all the integrals
        // for each triangle of the polygon. To evaluate the integral
        // for a single triangle, we make a change of variables to
        // the (u,v) coordinates of the triangle:
        // x = x0 + e1x * u + e2x * v
        // y = y0 + e1y * u + e2y * v
        // where 0 <= u && 0 <= v && u + v <= 1.
        //
        // We integrate u from [0,1-v] and then v from [0,1].
        // We also need to use the Jacobian of the transformation:
        // D = cross(e1, e2)
        //
        // Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
        //
        // The rest of the derivation is handled by computer algebra.

        // DEBUG: b2Assert(this.m_count >= 3);

        const center = temp.ComputeMass.center.SetZero();
        let area = 0;
        let I = 0;

        // Get a reference point for forming triangles.
        // Use the first vertex to reduce round-off errors.
        const s = temp.ComputeMass.s.Copy(this.m_vertices[0]);

        const k_inv3 = 1 / 3;

        for (let i = 0; i < this.m_count; ++i) {
            // Triangle vertices.
            const e1 = b2Vec2.Subtract(this.m_vertices[i], s, temp.ComputeMass.e1);
            const e2 = b2Vec2.Subtract(this.m_vertices[i + 1 < this.m_count ? i + 1 : 0], s, temp.ComputeMass.e2);

            const D = b2Vec2.Cross(e1, e2);

            const triangleArea = 0.5 * D;
            area += triangleArea;

            // Area weighted centroid
            center.AddScaled(triangleArea * k_inv3, b2Vec2.Add(e1, e2, b2Vec2.s_t0));

            const ex1 = e1.x;
            const ey1 = e1.y;
            const ex2 = e2.x;
            const ey2 = e2.y;

            const intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
            const inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

            I += 0.25 * k_inv3 * D * (intx2 + inty2);
        }

        // Total mass
        massData.mass = density * area;

        // Center of mass
        // DEBUG: b2Assert(area > b2_epsilon);
        center.Scale(1 / area);
        b2Vec2.Add(center, s, massData.center);

        // Inertia tensor relative to the local origin (point s).
        massData.I = density * I;

        // Shift to center of mass then to original body origin.
        massData.I += massData.mass * (b2Vec2.Dot(massData.center, massData.center) - b2Vec2.Dot(center, center));
    }

    /**
     * Validate convexity. This is a very time consuming operation.
     * @returns true if valid
     */
    public Validate(): boolean {
        if (this.m_count < 3 || b2_maxPolygonVertices < this.m_count) {
            return false;
        }

        return b2ValidateHull(this.m_vertices, this.m_count);
    }

    public SetupDistanceProxy(proxy: b2DistanceProxy, _index: number): void {
        proxy.m_vertices = this.m_vertices;
        proxy.m_count = this.m_count;
        proxy.m_radius = this.m_radius;
    }

    public Draw(draw: b2Draw, color: b2Color): void {
        const vertexCount = this.m_count;
        const vertices = this.m_vertices;
        draw.DrawSolidPolygon(vertices, vertexCount, color);
    }
}
