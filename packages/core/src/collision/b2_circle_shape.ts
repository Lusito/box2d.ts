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
import { EPSILON } from "../common/b2_common";
import { Color, Draw } from "../common/b2_draw";
import { Vec2, Transform, XY } from "../common/b2_math";
import { AABB, RayCastInput, RayCastOutput } from "./b2_collision";
import { DistanceProxy } from "./b2_distance";
import { MassData, Shape, ShapeType } from "./b2_shape";

/**
 * A solid circle shape
 */
export class CircleShape extends Shape {
    /** Position */
    public readonly m_p = new Vec2();

    public constructor(radius = 0) {
        super(ShapeType.Circle, radius);
    }

    public Set(position: XY, radius = this.m_radius) {
        this.m_p.Copy(position);
        this.m_radius = radius;
        return this;
    }

    /**
     * Implement Shape.
     */
    public Clone(): CircleShape {
        return new CircleShape().Copy(this);
    }

    public Copy(other: CircleShape): CircleShape {
        super.Copy(other);

        // DEBUG: Assert(other instanceof CircleShape);

        this.m_p.Copy(other.m_p);
        return this;
    }

    /**
     * @see Shape::GetChildCount
     */
    public GetChildCount(): number {
        return 1;
    }

    private static TestPoint_s_center = new Vec2();

    private static TestPoint_s_d = new Vec2();

    /**
     * Implement Shape.
     */
    public TestPoint(transform: Transform, p: XY): boolean {
        const center = Transform.MultiplyVec2(transform, this.m_p, CircleShape.TestPoint_s_center);
        const d = Vec2.Subtract(p, center, CircleShape.TestPoint_s_d);
        return Vec2.Dot(d, d) <= this.m_radius ** 2;
    }

    private static RayCast_s_position = new Vec2();

    private static RayCast_s_s = new Vec2();

    private static RayCast_s_r = new Vec2();

    /**
     * Implement Shape.
     *
     * @note because the circle is solid, rays that start inside do not hit because the normal is
     * not defined. Collision Detection in Interactive 3D Environments by Gino van den Bergen
     * From Section 3.1.2
     * x = s + a * r
     * norm(x) = radius
     */
    public RayCast(output: RayCastOutput, input: RayCastInput, transform: Transform, _childIndex: number): boolean {
        const position = Transform.MultiplyVec2(transform, this.m_p, CircleShape.RayCast_s_position);
        const s = Vec2.Subtract(input.p1, position, CircleShape.RayCast_s_s);
        const b = Vec2.Dot(s, s) - this.m_radius ** 2;

        // Solve quadratic equation.
        const r = Vec2.Subtract(input.p2, input.p1, CircleShape.RayCast_s_r);
        const c = Vec2.Dot(s, r);
        const rr = Vec2.Dot(r, r);
        const sigma = c * c - rr * b;

        // Check for negative discriminant and short segment.
        if (sigma < 0 || rr < EPSILON) {
            return false;
        }

        // Find the point of intersection of the line with the circle.
        let a = -(c + Math.sqrt(sigma));

        // Is the intersection point on the segment?
        if (a >= 0 && a <= input.maxFraction * rr) {
            a /= rr;
            output.fraction = a;
            Vec2.AddScaled(s, a, r, output.normal).Normalize();
            return true;
        }

        return false;
    }

    private static ComputeAABB_s_p = new Vec2();

    /**
     * @see Shape::ComputeAABB
     */
    public ComputeAABB(aabb: AABB, transform: Transform, _childIndex: number): void {
        const p = Transform.MultiplyVec2(transform, this.m_p, CircleShape.ComputeAABB_s_p);
        aabb.lowerBound.Set(p.x - this.m_radius, p.y - this.m_radius);
        aabb.upperBound.Set(p.x + this.m_radius, p.y + this.m_radius);
    }

    /**
     * @see Shape::ComputeMass
     */
    public ComputeMass(massData: MassData, density: number): void {
        const radius_sq = this.m_radius ** 2;
        massData.mass = density * Math.PI * radius_sq;
        massData.center.Copy(this.m_p);

        // inertia about the local origin
        massData.I = massData.mass * (0.5 * radius_sq + Vec2.Dot(this.m_p, this.m_p));
    }

    public SetupDistanceProxy(proxy: DistanceProxy, _index: number): void {
        proxy.m_vertices = proxy.m_buffer;
        proxy.m_vertices[0].Copy(this.m_p);
        proxy.m_count = 1;
        proxy.m_radius = this.m_radius;
    }

    public Draw(draw: Draw, color: Color): void {
        const center = this.m_p;
        const radius = this.m_radius;
        const axis = Vec2.UNITX;
        draw.DrawSolidCircle(center, radius, axis, color);
    }
}
