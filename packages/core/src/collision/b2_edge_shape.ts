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
import { b2_polygonRadius } from "../common/b2_common";
import { b2Color, b2Draw } from "../common/b2_draw";
import { b2Vec2, b2Rot, b2Transform, XY } from "../common/b2_math";
import { b2Readonly } from "../common/b2_readonly";
import { b2AABB, b2RayCastInput, b2RayCastOutput } from "./b2_collision";
import { b2DistanceProxy } from "./b2_distance";
import { b2MassData, b2Shape, b2ShapeType } from "./b2_shape";

/**
 * A line segment (edge) shape. These can be connected in chains or loops
 * to other edge shapes. Edges created independently are two-sided and do
 * no provide smooth movement across junctions.
 */
export class b2EdgeShape extends b2Shape {
    /** These are the edge vertices */
    public readonly m_vertex1 = new b2Vec2();

    public readonly m_vertex2 = new b2Vec2();

    /** Optional adjacent vertices. These are used for smooth collision. */
    public readonly m_vertex0 = new b2Vec2();

    public readonly m_vertex3 = new b2Vec2();

    /** Uses m_vertex0 and m_vertex3 to create smooth collision. */
    public m_oneSided = false;

    public constructor() {
        super(b2ShapeType.e_edge, b2_polygonRadius);
    }

    /**
     * Set this as a part of a sequence. Vertex v0 precedes the edge and vertex v3
     * follows. These extra vertices are used to provide smooth movement
     * across junctions. This also makes the collision one-sided. The edge
     * normal points to the right looking from v1 to v2.
     */
    public SetOneSided(v0: XY, v1: XY, v2: XY, v3: XY): b2EdgeShape {
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
    public SetTwoSided(v1: XY, v2: XY): b2EdgeShape {
        this.m_vertex1.Copy(v1);
        this.m_vertex2.Copy(v2);
        this.m_oneSided = false;
        return this;
    }

    /**
     * Implement b2Shape.
     */
    public Clone(): b2EdgeShape {
        return new b2EdgeShape().Copy(this);
    }

    public override Copy(other: b2EdgeShape): b2EdgeShape {
        super.Copy(other);

        // DEBUG: b2Assert(other instanceof b2EdgeShape);

        this.m_vertex1.Copy(other.m_vertex1);
        this.m_vertex2.Copy(other.m_vertex2);
        this.m_vertex0.Copy(other.m_vertex0);
        this.m_vertex3.Copy(other.m_vertex3);
        this.m_oneSided = other.m_oneSided;

        return this;
    }

    /**
     * @see b2Shape::GetChildCount
     */
    public GetChildCount(): number {
        return 1;
    }

    /**
     * @see b2Shape::TestPoint
     */
    public TestPoint(_xf: b2Readonly<b2Transform>, _p: XY): boolean {
        return false;
    }

    private static RayCast_s_p1 = new b2Vec2();

    private static RayCast_s_p2 = new b2Vec2();

    private static RayCast_s_d = new b2Vec2();

    private static RayCast_s_e = new b2Vec2();

    private static RayCast_s_q = new b2Vec2();

    private static RayCast_s_r = new b2Vec2();

    /**
     * Implement b2Shape.
     *
     * p = p1 + t * d
     * v = v1 + s * e
     * p1 + t * d = v1 + s * e
     * s * e - t * d = p1 - v1
     */
    public RayCast(
        output: b2RayCastOutput,
        input: b2RayCastInput,
        xf: b2Readonly<b2Transform>,
        _childIndex: number,
    ): boolean {
        // Put the ray into the edge's frame of reference.
        const p1 = b2Transform.TransposeMultiplyVec2(xf, input.p1, b2EdgeShape.RayCast_s_p1);
        const p2 = b2Transform.TransposeMultiplyVec2(xf, input.p2, b2EdgeShape.RayCast_s_p2);
        const d = b2Vec2.Subtract(p2, p1, b2EdgeShape.RayCast_s_d);

        const v1 = this.m_vertex1;
        const v2 = this.m_vertex2;
        const e = b2Vec2.Subtract(v2, v1, b2EdgeShape.RayCast_s_e);

        // Normal points to the right, looking from v1 at v2
        const { normal } = output;
        normal.Set(e.y, -e.x).Normalize();

        // q = p1 + t * d
        // dot(normal, q - v1) = 0
        // dot(normal, p1 - v1) + t * dot(normal, d) = 0
        const numerator = b2Vec2.Dot(normal, b2Vec2.Subtract(v1, p1, b2Vec2.s_t0));
        if (this.m_oneSided && numerator > 0) {
            return false;
        }

        const denominator = b2Vec2.Dot(normal, d);

        if (denominator === 0) {
            return false;
        }

        const t = numerator / denominator;
        if (t < 0 || input.maxFraction < t) {
            return false;
        }

        const q = b2Vec2.AddScaled(p1, t, d, b2EdgeShape.RayCast_s_q);

        // q = v1 + s * r
        // s = dot(q - v1, r) / dot(r, r)
        const r = b2Vec2.Subtract(v2, v1, b2EdgeShape.RayCast_s_r);
        const rr = b2Vec2.Dot(r, r);
        if (rr === 0) {
            return false;
        }

        const s = b2Vec2.Dot(b2Vec2.Subtract(q, v1, b2Vec2.s_t0), r) / rr;
        if (s < 0 || s > 1) {
            return false;
        }

        output.fraction = t;
        b2Rot.MultiplyVec2(xf.q, output.normal, output.normal);
        if (numerator > 0) {
            output.normal.Negate();
        }
        return true;
    }

    private static ComputeAABB_s_v1 = new b2Vec2();

    private static ComputeAABB_s_v2 = new b2Vec2();

    /**
     * @see b2Shape::ComputeAABB
     */
    public ComputeAABB(aabb: b2AABB, xf: b2Readonly<b2Transform>, _childIndex: number): void {
        const v1 = b2Transform.MultiplyVec2(xf, this.m_vertex1, b2EdgeShape.ComputeAABB_s_v1);
        const v2 = b2Transform.MultiplyVec2(xf, this.m_vertex2, b2EdgeShape.ComputeAABB_s_v2);

        b2Vec2.Min(v1, v2, aabb.lowerBound);
        b2Vec2.Max(v1, v2, aabb.upperBound);

        const r = this.m_radius;
        aabb.lowerBound.SubtractXY(r, r);
        aabb.upperBound.AddXY(r, r);
    }

    /**
     * @see b2Shape::ComputeMass
     */
    public ComputeMass(massData: b2MassData, _density: number): void {
        massData.mass = 0;
        b2Vec2.Mid(this.m_vertex1, this.m_vertex2, massData.center);
        massData.I = 0;
    }

    public SetupDistanceProxy(proxy: b2DistanceProxy, _index: number): void {
        proxy.m_vertices = proxy.m_buffer;
        proxy.m_vertices[0] = this.m_vertex1;
        proxy.m_vertices[1] = this.m_vertex2;
        proxy.m_count = 2;
        proxy.m_radius = this.m_radius;
    }

    public Draw(draw: b2Draw, color: b2Color): void {
        const v1 = this.m_vertex1;
        const v2 = this.m_vertex2;
        draw.DrawSegment(v1, v2, color);

        if (this.m_oneSided === false) {
            draw.DrawPoint(v1, 4, color);
            draw.DrawPoint(v2, 4, color);
        }
    }
}
