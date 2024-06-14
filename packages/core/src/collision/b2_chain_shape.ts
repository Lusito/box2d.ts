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

// DEBUG: import { b2Assert, b2_linearSlop } from "../common/b2_common";
import { b2_polygonRadius } from "../common/b2_common";
import { b2Vec2, b2Transform, XY } from "../common/b2_math";
import { b2AABB, b2RayCastInput, b2RayCastOutput } from "./b2_collision";
import { b2DistanceProxy } from "./b2_distance";
import { b2MassData, b2Shape, b2ShapeType } from "./b2_shape";
import { b2EdgeShape } from "./b2_edge_shape";
import { b2Color, b2Draw } from "../common/b2_draw";
import { b2Readonly } from "../common/b2_readonly";

/**
 * A chain shape is a free form sequence of line segments.
 * The chain has one-sided collision, with the surface normal pointing to the right of the edge.
 * This provides a counter-clockwise winding like the polygon shape.
 * Connectivity information is used to create smooth collisions.
 *
 * @warning the chain will not collide properly if there are self-intersections.
 */
export class b2ChainShape extends b2Shape {
    public m_vertices: b2Vec2[] = [];

    public readonly m_prevVertex = new b2Vec2();

    public readonly m_nextVertex = new b2Vec2();

    public constructor() {
        super(b2ShapeType.e_chain, b2_polygonRadius);
    }

    /**
     * Create a loop. This automatically adjusts connectivity.
     *
     * @param vertices An array of vertices, these are copied
     * @param count The vertex count
     */
    public CreateLoop(vertices: XY[], count = vertices.length): b2ChainShape {
        // DEBUG: b2Assert(count >= 3);
        if (count < 3) {
            return this;
        }
        // DEBUG: for (let i =  1; i < count; ++i) {
        // DEBUG:   const v1 = vertices[i - 1];
        // DEBUG:   const v2 = vertices[i];
        // DEBUG:   // If the code crashes here, it means your vertices are too close together.
        // DEBUG:   b2Assert(b2Vec2.DistanceSquared(v1, v2) > b2_linearSlop * b2_linearSlop);
        // DEBUG: }

        this.m_vertices.length = count + 1;
        for (let i = 0; i < count; ++i) {
            const { x, y } = vertices[i];
            this.m_vertices[i] = new b2Vec2(x, y);
        }

        this.m_vertices[count] = this.m_vertices[0].Clone();
        this.m_prevVertex.Copy(this.m_vertices[this.m_vertices.length - 2]);
        this.m_nextVertex.Copy(this.m_vertices[1]);
        return this;
    }

    /**
     * Create a chain with ghost vertices to connect multiple chains together.
     *
     * @param vertices An array of vertices, these are copied
     * @param count The vertex count
     * @param prevVertex Previous vertex from chain that connects to the start
     * @param nextVertex Next vertex from chain that connects to the end
     */
    public CreateChain(
        vertices: XY[],
        count: number,
        prevVertex: Readonly<XY>,
        nextVertex: Readonly<XY>,
    ): b2ChainShape {
        // DEBUG: b2Assert(count >= 2);
        // DEBUG: for (let i =  1; i < count; ++i) {
        // DEBUG:   // If the code crashes here, it means your vertices are too close together.
        // DEBUG:   b2Assert(b2Vec2.DistanceSquared(vertices[i-1], vertices[i]) > b2_linearSlop * b2_linearSlop);
        // DEBUG: }

        this.m_vertices.length = count;
        for (let i = 0; i < count; ++i) {
            const { x, y } = vertices[i];
            this.m_vertices[i] = new b2Vec2(x, y);
        }

        this.m_prevVertex.Copy(prevVertex);
        this.m_nextVertex.Copy(nextVertex);

        return this;
    }

    /**
     * Implement b2Shape. Vertices are cloned using b2Alloc.
     */
    public Clone(): b2ChainShape {
        return new b2ChainShape().Copy(this);
    }

    public Copy(other: b2ChainShape): b2ChainShape {
        super.Copy(other);

        // DEBUG: b2Assert(other instanceof b2ChainShape);

        return this.CreateChain(other.m_vertices, other.m_vertices.length, other.m_prevVertex, other.m_nextVertex);
    }

    /**
     * @see b2Shape::GetChildCount
     */
    public GetChildCount(): number {
        // edge count = vertex count - 1
        return this.m_vertices.length - 1;
    }

    /**
     * Get a child edge.
     */
    public GetChildEdge(edge: b2EdgeShape, index: number): void {
        // DEBUG: b2Assert(0 <= index && index < this.m_vertices.length - 1);
        edge.m_radius = this.m_radius;

        edge.m_vertex1.Copy(this.m_vertices[index]);
        edge.m_vertex2.Copy(this.m_vertices[index + 1]);
        edge.m_oneSided = true;

        if (index > 0) {
            edge.m_vertex0.Copy(this.m_vertices[index - 1]);
        } else {
            edge.m_vertex0.Copy(this.m_prevVertex);
        }

        if (index < this.m_vertices.length - 2) {
            edge.m_vertex3.Copy(this.m_vertices[index + 2]);
        } else {
            edge.m_vertex3.Copy(this.m_nextVertex);
        }
    }

    /**
     * This always return false.
     *
     * @see b2Shape::TestPoint
     */
    public TestPoint(_xf: b2Readonly<b2Transform>, _p: XY): boolean {
        return false;
    }

    private static RayCast_s_edgeShape = new b2EdgeShape();

    /**
     * Implement b2Shape.
     */
    public RayCast(output: b2RayCastOutput, input: b2RayCastInput, xf: b2Readonly<b2Transform>, childIndex: number): boolean {
        // DEBUG: b2Assert(childIndex < this.m_vertices.length);

        const edgeShape = b2ChainShape.RayCast_s_edgeShape;

        const i1 = childIndex;
        let i2 = childIndex + 1;
        if (i2 === this.m_vertices.length) {
            i2 = 0;
        }

        edgeShape.m_vertex1.Copy(this.m_vertices[i1]);
        edgeShape.m_vertex2.Copy(this.m_vertices[i2]);

        return edgeShape.RayCast(output, input, xf, 0);
    }

    private static ComputeAABB_s_v1 = new b2Vec2();

    private static ComputeAABB_s_v2 = new b2Vec2();

    private static ComputeAABB_s_lower = new b2Vec2();

    private static ComputeAABB_s_upper = new b2Vec2();

    /**
     * @see b2Shape::ComputeAABB
     */
    public ComputeAABB(aabb: b2AABB, xf: b2Readonly<b2Transform>, childIndex: number): void {
        // DEBUG: b2Assert(childIndex < this.m_vertices.length);

        const i1 = childIndex;
        let i2 = childIndex + 1;
        if (i2 === this.m_vertices.length) {
            i2 = 0;
        }

        const v1 = b2Transform.MultiplyVec2(xf, this.m_vertices[i1], b2ChainShape.ComputeAABB_s_v1);
        const v2 = b2Transform.MultiplyVec2(xf, this.m_vertices[i2], b2ChainShape.ComputeAABB_s_v2);

        const lower = b2Vec2.Min(v1, v2, b2ChainShape.ComputeAABB_s_lower);
        const upper = b2Vec2.Max(v1, v2, b2ChainShape.ComputeAABB_s_upper);

        aabb.lowerBound.x = lower.x - this.m_radius;
        aabb.lowerBound.y = lower.y - this.m_radius;
        aabb.upperBound.x = upper.x + this.m_radius;
        aabb.upperBound.y = upper.y + this.m_radius;
    }

    /**
     * Chains have zero mass.
     *
     * @see b2Shape::ComputeMass
     */
    public ComputeMass(massData: b2MassData, _density: number): void {
        massData.mass = 0;
        massData.center.SetZero();
        massData.I = 0;
    }

    public SetupDistanceProxy(proxy: b2DistanceProxy, index: number): void {
        // DEBUG: b2Assert(0 <= index && index < this.m_vertices.length);

        proxy.m_vertices = proxy.m_buffer;
        proxy.m_vertices[0] = this.m_vertices[index];
        const secondIndex = index + 1 < this.m_vertices.length ? index + 1 : 0;
        proxy.m_vertices[1] = this.m_vertices[secondIndex];
        proxy.m_count = 2;
        proxy.m_radius = this.m_radius;
    }

    public Draw(draw: b2Draw, color: b2Color): void {
        const vertices = this.m_vertices;
        let v1 = vertices[0];
        for (let i = 1; i < vertices.length; ++i) {
            const v2 = vertices[i];
            draw.DrawSegment(v1, v2, color);
            v1 = v2;
        }
    }
}
