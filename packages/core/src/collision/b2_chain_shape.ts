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

// DEBUG: import { assert, LINEAR_SLOP } from "../common/b2_common";
import { POLYGON_RADIUS } from "../common/b2_common";
import { Vec2, Transform, XY } from "../common/b2_math";
import { AABB, RayCastInput, RayCastOutput } from "./b2_collision";
import { DistanceProxy } from "./b2_distance";
import { MassData, Shape, ShapeType } from "./b2_shape";
import { EdgeShape } from "./b2_edge_shape";
import { Color, Draw } from "../common/b2_draw";

/**
 * A chain shape is a free form sequence of line segments.
 * The chain has one-sided collision, with the surface normal pointing to the right of the edge.
 * This provides a counter-clockwise winding like the polygon shape.
 * Connectivity information is used to create smooth collisions.
 *
 * @warning the chain will not collide properly if there are self-intersections.
 */
export class ChainShape extends Shape {
    public m_vertices: Vec2[] = [];

    public readonly m_prevVertex = new Vec2();

    public readonly m_nextVertex = new Vec2();

    public constructor() {
        super(ShapeType.Chain, POLYGON_RADIUS);
    }

    /**
     * Create a loop. This automatically adjusts connectivity.
     *
     * @param vertices An array of vertices, these are copied
     * @param count The vertex count
     */
    public createLoop(vertices: XY[], count = vertices.length): ChainShape {
        // DEBUG: assert(count >= 3);
        if (count < 3) {
            return this;
        }
        // DEBUG: for (let i =  1; i < count; ++i) {
        // DEBUG:   const v1 = vertices[i - 1];
        // DEBUG:   const v2 = vertices[i];
        // DEBUG:   // If the code crashes here, it means your vertices are too close together.
        // DEBUG:   assert(Vec2.DistanceSquared(v1, v2) > LINEAR_SLOP * LINEAR_SLOP);
        // DEBUG: }

        this.m_vertices.length = count + 1;
        for (let i = 0; i < count; ++i) {
            const { x, y } = vertices[i];
            this.m_vertices[i] = new Vec2(x, y);
        }

        this.m_vertices[count] = this.m_vertices[0].clone();
        this.m_prevVertex.copy(this.m_vertices[this.m_vertices.length - 2]);
        this.m_nextVertex.copy(this.m_vertices[1]);
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
    public createChain(vertices: XY[], count: number, prevVertex: Readonly<XY>, nextVertex: Readonly<XY>): ChainShape {
        // DEBUG: assert(count >= 2);
        // DEBUG: for (let i =  1; i < count; ++i) {
        // DEBUG:   // If the code crashes here, it means your vertices are too close together.
        // DEBUG:   assert(Vec2.DistanceSquared(vertices[i-1], vertices[i]) > LINEAR_SLOP * LINEAR_SLOP);
        // DEBUG: }

        this.m_vertices.length = count;
        for (let i = 0; i < count; ++i) {
            const { x, y } = vertices[i];
            this.m_vertices[i] = new Vec2(x, y);
        }

        this.m_prevVertex.copy(prevVertex);
        this.m_nextVertex.copy(nextVertex);

        return this;
    }

    /**
     * Implement Shape. Vertices are cloned using Alloc.
     */
    public clone(): ChainShape {
        return new ChainShape().copy(this);
    }

    public copy(other: ChainShape): ChainShape {
        super.copy(other);

        // DEBUG: assert(other instanceof ChainShape);

        return this.createChain(other.m_vertices, other.m_vertices.length, other.m_prevVertex, other.m_nextVertex);
    }

    /**
     * @see Shape::GetChildCount
     */
    public getChildCount(): number {
        // edge count = vertex count - 1
        return this.m_vertices.length - 1;
    }

    /**
     * Get a child edge.
     */
    public getChildEdge(edge: EdgeShape, index: number): void {
        // DEBUG: assert(0 <= index && index < this.m_vertices.length - 1);
        edge.m_radius = this.m_radius;

        edge.m_vertex1.copy(this.m_vertices[index]);
        edge.m_vertex2.copy(this.m_vertices[index + 1]);
        edge.m_oneSided = true;

        if (index > 0) {
            edge.m_vertex0.copy(this.m_vertices[index - 1]);
        } else {
            edge.m_vertex0.copy(this.m_prevVertex);
        }

        if (index < this.m_vertices.length - 2) {
            edge.m_vertex3.copy(this.m_vertices[index + 2]);
        } else {
            edge.m_vertex3.copy(this.m_nextVertex);
        }
    }

    /**
     * This always return false.
     *
     * @see Shape::TestPoint
     */
    public testPoint(_xf: Transform, _p: XY): boolean {
        return false;
    }

    private static RayCast_s_edgeShape = new EdgeShape();

    /**
     * Implement Shape.
     */
    public rayCast(output: RayCastOutput, input: RayCastInput, xf: Transform, childIndex: number): boolean {
        // DEBUG: assert(childIndex < this.m_vertices.length);

        const edgeShape = ChainShape.RayCast_s_edgeShape;

        const i1 = childIndex;
        let i2 = childIndex + 1;
        if (i2 === this.m_vertices.length) {
            i2 = 0;
        }

        edgeShape.m_vertex1.copy(this.m_vertices[i1]);
        edgeShape.m_vertex2.copy(this.m_vertices[i2]);

        return edgeShape.rayCast(output, input, xf, 0);
    }

    private static ComputeAABB_s_v1 = new Vec2();

    private static ComputeAABB_s_v2 = new Vec2();

    private static ComputeAABB_s_lower = new Vec2();

    private static ComputeAABB_s_upper = new Vec2();

    /**
     * @see Shape::ComputeAABB
     */
    public computeAABB(aabb: AABB, xf: Transform, childIndex: number): void {
        // DEBUG: assert(childIndex < this.m_vertices.length);

        const i1 = childIndex;
        let i2 = childIndex + 1;
        if (i2 === this.m_vertices.length) {
            i2 = 0;
        }

        const v1 = Transform.multiplyVec2(xf, this.m_vertices[i1], ChainShape.ComputeAABB_s_v1);
        const v2 = Transform.multiplyVec2(xf, this.m_vertices[i2], ChainShape.ComputeAABB_s_v2);

        const lower = Vec2.min(v1, v2, ChainShape.ComputeAABB_s_lower);
        const upper = Vec2.max(v1, v2, ChainShape.ComputeAABB_s_upper);

        aabb.lowerBound.x = lower.x - this.m_radius;
        aabb.lowerBound.y = lower.y - this.m_radius;
        aabb.upperBound.x = upper.x + this.m_radius;
        aabb.upperBound.y = upper.y + this.m_radius;
    }

    /**
     * Chains have zero mass.
     *
     * @see Shape::ComputeMass
     */
    public computeMass(massData: MassData, _density: number): void {
        massData.mass = 0;
        massData.center.setZero();
        massData.I = 0;
    }

    public setupDistanceProxy(proxy: DistanceProxy, index: number): void {
        // DEBUG: assert(0 <= index && index < this.m_vertices.length);

        proxy.m_vertices = proxy.m_buffer;
        proxy.m_vertices[0].copy(this.m_vertices[index]);
        if (index + 1 < this.m_vertices.length) {
            proxy.m_vertices[1].copy(this.m_vertices[index + 1]);
        } else {
            proxy.m_vertices[1].copy(this.m_vertices[0]);
        }
        proxy.m_count = 2;
        proxy.m_radius = this.m_radius;
    }

    public draw(draw: Draw, color: Color): void {
        const vertices = this.m_vertices;
        let v1 = vertices[0];
        for (let i = 1; i < vertices.length; ++i) {
            const v2 = vertices[i];
            draw.drawSegment(v1, v2, color);
            v1 = v2;
        }
    }
}
