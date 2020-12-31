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
import { POLYGON_RADIUS } from "../common/b2_common";
import { Color, Draw } from "../common/b2_draw";
import { Vec2, Rot, Transform, XY } from "../common/b2_math";
import { AABB, RayCastInput, RayCastOutput } from "./b2_collision";
import { DistanceProxy } from "./b2_distance";
import { MassData, Shape, ShapeType } from "./b2_shape";

/**
 * A line segment (edge) shape. These can be connected in chains or loops
 * to other edge shapes. Edges created independently are two-sided and do
 * no provide smooth movement across junctions.
 */
export class EdgeShape extends Shape {
    /** These are the edge vertices */
    public readonly m_vertex1 = new Vec2();

    public readonly m_vertex2 = new Vec2();

    /** Optional adjacent vertices. These are used for smooth collision. */
    public readonly m_vertex0 = new Vec2();

    public readonly m_vertex3 = new Vec2();

    /** Uses m_vertex0 and m_vertex3 to create smooth collision. */
    public m_oneSided = false;

    public constructor() {
        super(ShapeType.Edge, POLYGON_RADIUS);
    }

    /**
     * Set this as a part of a sequence. Vertex v0 precedes the edge and vertex v3
     * follows. These extra vertices are used to provide smooth movement
     * across junctions. This also makes the collision one-sided. The edge
     * normal points to the right looking from v1 to v2.
     */
    public SetOneSided(v0: XY, v1: XY, v2: XY, v3: XY): EdgeShape {
        this.m_vertex0.Copy(v0);
        this.m_vertex1.Copy(v1);
        this.m_vertex2.Copy(v2);
        this.m_vertex3.Copy(v3);
        this.m_oneSided = true;
        return this;
    }

    /**
     * Set this as an isolated edge. Collision is two-sided.
     */
    public SetTwoSided(v1: XY, v2: XY): EdgeShape {
        this.m_vertex1.Copy(v1);
        this.m_vertex2.Copy(v2);
        this.m_oneSided = false;
        return this;
    }

    /**
     * Implement Shape.
     */
    public Clone(): EdgeShape {
        return new EdgeShape().Copy(this);
    }

    public Copy(other: EdgeShape): EdgeShape {
        super.Copy(other);

        // DEBUG: Assert(other instanceof EdgeShape);

        this.m_vertex1.Copy(other.m_vertex1);
        this.m_vertex2.Copy(other.m_vertex2);
        this.m_vertex0.Copy(other.m_vertex0);
        this.m_vertex3.Copy(other.m_vertex3);
        this.m_oneSided = other.m_oneSided;

        return this;
    }

    /**
     * @see Shape::GetChildCount
     */
    public GetChildCount(): number {
        return 1;
    }

    /**
     * @see Shape::TestPoint
     */
    public TestPoint(_xf: Transform, _p: XY): boolean {
        return false;
    }

    private static RayCast_s_p1 = new Vec2();

    private static RayCast_s_p2 = new Vec2();

    private static RayCast_s_d = new Vec2();

    private static RayCast_s_e = new Vec2();

    private static RayCast_s_q = new Vec2();

    private static RayCast_s_r = new Vec2();

    /**
     * Implement Shape.
     *
     * p = p1 + t * d
     * v = v1 + s * e
     * p1 + t * d = v1 + s * e
     * s * e - t * d = p1 - v1
     */
    public RayCast(output: RayCastOutput, input: RayCastInput, xf: Transform, _childIndex: number): boolean {
        // Put the ray into the edge's frame of reference.
        const p1 = Transform.TransposeMultiplyVec2(xf, input.p1, EdgeShape.RayCast_s_p1);
        const p2 = Transform.TransposeMultiplyVec2(xf, input.p2, EdgeShape.RayCast_s_p2);
        const d = Vec2.Subtract(p2, p1, EdgeShape.RayCast_s_d);

        const v1 = this.m_vertex1;
        const v2 = this.m_vertex2;
        const e = Vec2.Subtract(v2, v1, EdgeShape.RayCast_s_e);

        // Normal points to the right, looking from v1 at v2
        const { normal } = output;
        normal.Set(e.y, -e.x).Normalize();

        // q = p1 + t * d
        // dot(normal, q - v1) = 0
        // dot(normal, p1 - v1) + t * dot(normal, d) = 0
        const numerator = Vec2.Dot(normal, Vec2.Subtract(v1, p1, Vec2.s_t0));
        if (this.m_oneSided && numerator > 0) {
            return false;
        }

        const denominator = Vec2.Dot(normal, d);

        if (denominator === 0) {
            return false;
        }

        const t = numerator / denominator;
        if (t < 0 || input.maxFraction < t) {
            return false;
        }

        const q = Vec2.AddScaled(p1, t, d, EdgeShape.RayCast_s_q);

        // q = v1 + s * r
        // s = dot(q - v1, r) / dot(r, r)
        const r = Vec2.Subtract(v2, v1, EdgeShape.RayCast_s_r);
        const rr = Vec2.Dot(r, r);
        if (rr === 0) {
            return false;
        }

        const s = Vec2.Dot(Vec2.Subtract(q, v1, Vec2.s_t0), r) / rr;
        if (s < 0 || s > 1) {
            return false;
        }

        output.fraction = t;
        Rot.MultiplyVec2(xf.q, output.normal, output.normal);
        if (numerator > 0) {
            output.normal.Negate();
        }
        return true;
    }

    private static ComputeAABB_s_v1 = new Vec2();

    private static ComputeAABB_s_v2 = new Vec2();

    /**
     * @see Shape::ComputeAABB
     */
    public ComputeAABB(aabb: AABB, xf: Transform, _childIndex: number): void {
        const v1 = Transform.MultiplyVec2(xf, this.m_vertex1, EdgeShape.ComputeAABB_s_v1);
        const v2 = Transform.MultiplyVec2(xf, this.m_vertex2, EdgeShape.ComputeAABB_s_v2);

        Vec2.Min(v1, v2, aabb.lowerBound);
        Vec2.Max(v1, v2, aabb.upperBound);

        const r = this.m_radius;
        aabb.lowerBound.SubtractXY(r, r);
        aabb.upperBound.AddXY(r, r);
    }

    /**
     * @see Shape::ComputeMass
     */
    public ComputeMass(massData: MassData, _density: number): void {
        massData.mass = 0;
        Vec2.Mid(this.m_vertex1, this.m_vertex2, massData.center);
        massData.I = 0;
    }

    public SetupDistanceProxy(proxy: DistanceProxy, _index: number): void {
        proxy.m_vertices = proxy.m_buffer;
        proxy.m_vertices[0].Copy(this.m_vertex1);
        proxy.m_vertices[1].Copy(this.m_vertex2);
        proxy.m_count = 2;
        proxy.m_radius = this.m_radius;
    }

    public Draw(draw: Draw, color: Color): void {
        const v1 = this.m_vertex1;
        const v2 = this.m_vertex2;
        draw.DrawSegment(v1, v2, color);

        if (this.m_oneSided === false) {
            draw.DrawPoint(v1, 4, color);
            draw.DrawPoint(v2, 4, color);
        }
    }
}
