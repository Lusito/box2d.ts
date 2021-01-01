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

// DEBUG: import { assert, EPSILON_SQUARED } from "../common/b2_common";
import { assert, makeArray, LINEAR_SLOP, POLYGON_RADIUS } from "../common/b2_common";
import { Color, Draw } from "../common/b2_draw";
import { Vec2, Rot, Transform, XY } from "../common/b2_math";
import { MAX_POLYGON_VERTICES } from "../common/b2_settings";
import { AABB, RayCastInput, RayCastOutput } from "./b2_collision";
import { DistanceProxy } from "./b2_distance";
import { MassData, Shape, ShapeType } from "./b2_shape";

const temp = {
    ComputeCentroid: {
        s: new Vec2(),
        p1: new Vec2(),
        p2: new Vec2(),
        p3: new Vec2(),
        e1: new Vec2(),
        e2: new Vec2(),
    },
    TestPoint: {
        pLocal: new Vec2(),
    },
    ComputeAABB: {
        v: new Vec2(),
    },
    ComputeMass: {
        center: new Vec2(),
        s: new Vec2(),
        e1: new Vec2(),
        e2: new Vec2(),
    },
    Validate: {
        e: new Vec2(),
        v: new Vec2(),
    },
    Set: {
        r: new Vec2(),
        v: new Vec2(),
    },
    RayCast: {
        p1: new Vec2(),
        p2: new Vec2(),
        d: new Vec2(),
    },
    SetAsBox: {
        xf: new Transform(),
    },
};

const weldingDistanceSquared = (0.5 * LINEAR_SLOP) ** 2;

function computeCentroid(vs: Vec2[], count: number, out: Vec2): Vec2 {
    // DEBUG: assert(count >= 3);

    const c = out;
    c.setZero();
    let area = 0;

    const { s, p1, p2, p3, e1, e2 } = temp.ComputeCentroid;

    // Get a reference point for forming triangles.
    // Use the first vertex to reduce round-off errors.
    s.copy(vs[0]);

    const inv3 = 1 / 3;

    for (let i = 0; i < count; ++i) {
        // Triangle vertices.
        Vec2.subtract(vs[0], s, p1);
        Vec2.subtract(vs[i], s, p2);
        Vec2.subtract(vs[i + 1 < count ? i + 1 : 0], s, p3);

        Vec2.subtract(p2, p1, e1);
        Vec2.subtract(p3, p1, e2);

        const D = Vec2.cross(e1, e2);

        const triangleArea = 0.5 * D;
        area += triangleArea;

        // Area weighted centroid
        c.x += triangleArea * inv3 * (p1.x + p2.x + p3.x);
        c.y += triangleArea * inv3 * (p1.y + p2.y + p3.y);
    }

    // Centroid
    // DEBUG: assert(area > EPSILON);
    const f = 1 / area;
    c.x = f * c.x + s.x;
    c.y = f * c.y + s.y;
    return c;
}

/**
 * A solid convex polygon. It is assumed that the interior of the polygon is to
 * the left of each edge.
 * Polygons have a maximum number of vertices equal to MAX_POLYGON_VERTICES.
 * In most cases you should not need many vertices for a convex polygon.
 */
export class PolygonShape extends Shape {
    public readonly m_centroid = new Vec2();

    public m_vertices: Vec2[] = [];

    public m_normals: Vec2[] = [];

    public m_count = 0;

    public constructor() {
        super(ShapeType.Polygon, POLYGON_RADIUS);
    }

    /**
     * Implement Shape.
     */
    public clone(): PolygonShape {
        return new PolygonShape().copy(this);
    }

    public copy(other: PolygonShape): PolygonShape {
        super.copy(other);

        // DEBUG: assert(other instanceof PolygonShape);

        this.m_centroid.copy(other.m_centroid);
        this.m_count = other.m_count;
        this.m_vertices = makeArray(this.m_count, Vec2);
        this.m_normals = makeArray(this.m_count, Vec2);
        for (let i = 0; i < this.m_count; ++i) {
            this.m_vertices[i].copy(other.m_vertices[i]);
            this.m_normals[i].copy(other.m_normals[i]);
        }
        return this;
    }

    /**
     * @see Shape::GetChildCount
     */
    public getChildCount(): number {
        return 1;
    }

    /**
     * Create a convex hull from the given array of points.
     *
     * @warning the points may be re-ordered, even if they form a convex polygon
     * @warning collinear points are handled but not removed. Collinear points
     * may lead to poor stacking behavior.
     */
    public set(vertices: XY[], count = vertices.length): PolygonShape {
        // DEBUG: assert(3 <= count && count <= MAX_POLYGON_VERTICES);
        if (count < 3) {
            return this.setAsBox(1, 1);
        }

        let n = Math.min(count, MAX_POLYGON_VERTICES);

        // Perform welding and copy vertices into local buffer.
        const ps: XY[] = [];
        for (let i = 0; i < n; ++i) {
            const v = vertices[i];

            const unique = ps.every((p) => Vec2.distanceSquared(v, p) >= weldingDistanceSquared);
            if (unique) {
                ps.push(v);
            }
        }

        n = ps.length;
        if (n < 3) {
            // Polygon is degenerate.
            // DEBUG: assert(false);
            return this.setAsBox(1, 1);
        }

        // Create the convex hull using the Gift wrapping algorithm
        // http://en.wikipedia.org/wiki/Gift_wrapping_algorithm

        // Find the right most point on the hull
        let i0 = 0;
        let x0 = ps[0].x;
        for (let i = 1; i < n; ++i) {
            const { x } = ps[i];
            if (x > x0 || (x === x0 && ps[i].y < ps[i0].y)) {
                i0 = i;
                x0 = x;
            }
        }

        const hull: number[] = [];
        let m = 0;
        let ih = i0;

        for (;;) {
            // DEBUG: assert(m < MAX_POLYGON_VERTICES);
            hull[m] = ih;

            let ie = 0;
            for (let j = 1; j < n; ++j) {
                if (ie === ih) {
                    ie = j;
                    continue;
                }

                const r = Vec2.subtract(ps[ie], ps[hull[m]], temp.Set.r);
                const v = Vec2.subtract(ps[j], ps[hull[m]], temp.Set.v);
                const c = Vec2.cross(r, v);
                if (c < 0) {
                    ie = j;
                }

                // Collinearity check
                if (c === 0 && v.lengthSquared() > r.lengthSquared()) {
                    ie = j;
                }
            }

            ++m;
            ih = ie;

            if (ie === i0) {
                break;
            }
        }

        assert(m >= 3, "Polygon is degenerate");

        this.m_count = m;
        this.m_vertices = makeArray(this.m_count, Vec2);
        this.m_normals = makeArray(this.m_count, Vec2);

        // Copy vertices.
        for (let i = 0; i < m; ++i) {
            this.m_vertices[i].copy(ps[hull[i]]);
        }

        // Compute normals. Ensure the edges have non-zero length.
        for (let i = 0; i < m; ++i) {
            const i1 = i;
            const i2 = i + 1 < m ? i + 1 : 0;
            const edge = Vec2.subtract(this.m_vertices[i2], this.m_vertices[i1], Vec2.s_t0);
            // DEBUG: assert(edge.LengthSquared() > EPSILON_SQUARED);
            Vec2.crossVec2One(edge, this.m_normals[i]).normalize();
        }

        // Compute the polygon centroid.
        computeCentroid(this.m_vertices, m, this.m_centroid);

        return this;
    }

    /**
     * Build vertices to represent an axis-aligned box or an oriented box.
     *
     * @param hx The half-width.
     * @param hy The half-height.
     * @param center The center of the box in local coordinates.
     * @param angle The rotation of the box in local coordinates.
     */
    public setAsBox(hx: number, hy: number, center?: XY, angle = 0): PolygonShape {
        this.m_count = 4;
        this.m_vertices = makeArray(this.m_count, Vec2);
        this.m_normals = makeArray(this.m_count, Vec2);
        this.m_vertices[0].set(-hx, -hy);
        this.m_vertices[1].set(hx, -hy);
        this.m_vertices[2].set(hx, hy);
        this.m_vertices[3].set(-hx, hy);
        this.m_normals[0].set(0, -1);
        this.m_normals[1].set(1, 0);
        this.m_normals[2].set(0, 1);
        this.m_normals[3].set(-1, 0);

        if (center) {
            this.m_centroid.copy(center);

            const { xf } = temp.SetAsBox;
            xf.setPosition(center);
            xf.setRotationAngle(angle);

            // Transform vertices and normals.
            for (let i = 0; i < this.m_count; ++i) {
                Transform.multiplyVec2(xf, this.m_vertices[i], this.m_vertices[i]);
                Rot.multiplyVec2(xf.q, this.m_normals[i], this.m_normals[i]);
            }
        } else {
            this.m_centroid.setZero();
        }

        return this;
    }

    /**
     * @see Shape::TestPoint
     */
    public testPoint(xf: Transform, p: XY): boolean {
        const pLocal = Transform.transposeMultiplyVec2(xf, p, temp.TestPoint.pLocal);

        for (let i = 0; i < this.m_count; ++i) {
            const dot = Vec2.dot(this.m_normals[i], Vec2.subtract(pLocal, this.m_vertices[i], Vec2.s_t0));
            if (dot > 0) {
                return false;
            }
        }

        return true;
    }

    /**
     * Implement Shape.
     *
     * @note because the polygon is solid, rays that start inside do not hit because the normal is
     * not defined.
     */
    public rayCast(output: RayCastOutput, input: RayCastInput, xf: Transform, _childIndex: number): boolean {
        // Put the ray into the polygon's frame of reference.
        const p1 = Transform.transposeMultiplyVec2(xf, input.p1, temp.RayCast.p1);
        const p2 = Transform.transposeMultiplyVec2(xf, input.p2, temp.RayCast.p2);
        const d = Vec2.subtract(p2, p1, temp.RayCast.d);

        let lower = 0;
        let upper = input.maxFraction;

        let index = -1;

        for (let i = 0; i < this.m_count; ++i) {
            // p = p1 + a * d
            // dot(normal, p - v) = 0
            // dot(normal, p1 - v) + a * dot(normal, d) = 0
            const numerator = Vec2.dot(this.m_normals[i], Vec2.subtract(this.m_vertices[i], p1, Vec2.s_t0));
            const denominator = Vec2.dot(this.m_normals[i], d);

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
            // if (upper < lower - EPSILON)
            if (upper < lower) {
                return false;
            }
        }

        // DEBUG: assert(0 <= lower && lower <= input.maxFraction);

        if (index >= 0) {
            output.fraction = lower;
            Rot.multiplyVec2(xf.q, this.m_normals[index], output.normal);
            return true;
        }

        return false;
    }

    /**
     * @see Shape::ComputeAABB
     */
    public computeAABB(aabb: AABB, xf: Transform, _childIndex: number): void {
        const lower = Transform.multiplyVec2(xf, this.m_vertices[0], aabb.lowerBound);
        const upper = aabb.upperBound.copy(lower);

        for (let i = 1; i < this.m_count; ++i) {
            const v = Transform.multiplyVec2(xf, this.m_vertices[i], temp.ComputeAABB.v);
            Vec2.min(lower, v, lower);
            Vec2.max(upper, v, upper);
        }

        const r = this.m_radius;
        lower.subtractXY(r, r);
        upper.addXY(r, r);
    }

    /**
     * @see Shape::ComputeMass
     */
    public computeMass(massData: MassData, density: number): void {
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

        // DEBUG: assert(this.m_count >= 3);

        const center = temp.ComputeMass.center.setZero();
        let area = 0;
        let I = 0;

        // Get a reference point for forming triangles.
        // Use the first vertex to reduce round-off errors.
        const s = temp.ComputeMass.s.copy(this.m_vertices[0]);

        const k_inv3 = 1 / 3;

        for (let i = 0; i < this.m_count; ++i) {
            // Triangle vertices.
            const e1 = Vec2.subtract(this.m_vertices[i], s, temp.ComputeMass.e1);
            const e2 = Vec2.subtract(this.m_vertices[i + 1 < this.m_count ? i + 1 : 0], s, temp.ComputeMass.e2);

            const D = Vec2.cross(e1, e2);

            const triangleArea = 0.5 * D;
            area += triangleArea;

            // Area weighted centroid
            center.addScaled(triangleArea * k_inv3, Vec2.add(e1, e2, Vec2.s_t0));

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
        // DEBUG: assert(area > EPSILON);
        center.scale(1 / area);
        Vec2.add(center, s, massData.center);

        // Inertia tensor relative to the local origin (point s).
        massData.I = density * I;

        // Shift to center of mass then to original body origin.
        massData.I += massData.mass * (Vec2.dot(massData.center, massData.center) - Vec2.dot(center, center));
    }

    public validate(): boolean {
        const { e, v } = temp.Validate;
        for (let i = 0; i < this.m_count; ++i) {
            const i1 = i;
            const i2 = i < this.m_count - 1 ? i1 + 1 : 0;
            const p = this.m_vertices[i1];
            Vec2.subtract(this.m_vertices[i2], p, e);

            for (let j = 0; j < this.m_count; ++j) {
                if (j === i1 || j === i2) {
                    continue;
                }

                Vec2.subtract(this.m_vertices[j], p, v);
                const c = Vec2.cross(e, v);
                if (c < 0) {
                    return false;
                }
            }
        }

        return true;
    }

    public setupDistanceProxy(proxy: DistanceProxy, _index: number): void {
        proxy.m_vertices = this.m_vertices;
        proxy.m_count = this.m_count;
        proxy.m_radius = this.m_radius;
    }

    public draw(draw: Draw, color: Color): void {
        const vertexCount = this.m_count;
        const vertices = this.m_vertices;
        draw.drawSolidPolygon(vertices, vertexCount, color);
    }
}
