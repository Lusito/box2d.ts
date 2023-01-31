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
import { b2Color, b2Draw } from "../common/b2_draw";
import { b2Vec2, b2Transform, XY } from "../common/b2_math";
import { b2AABB, b2RayCastInput, b2RayCastOutput } from "./b2_collision";
import { b2DistanceProxy } from "./b2_distance";

/**
 * This holds the mass data computed for a shape.
 */
export class b2MassData {
    /** The mass of the shape, usually in kilograms. */
    public mass = 0;

    /** The position of the shape's centroid relative to the shape's origin. */
    public readonly center = new b2Vec2();

    /** The rotational inertia of the shape about the local origin. */
    public I = 0;
}

export enum b2ShapeType {
    e_unknown = -1,
    e_circle = 0,
    e_edge = 1,
    e_polygon = 2,
    e_chain = 3,
    e_typeCount = 4,
}

/**
 * A shape is used for collision detection. You can create a shape however you like.
 * Shapes used for simulation in b2World are created automatically when a b2Fixture
 * is created. Shapes may encapsulate a one or more child shapes.
 */
export abstract class b2Shape {
    public readonly m_type: b2ShapeType;

    /**
     * Radius of a shape. For polygonal shapes this must be b2_polygonRadius. There is no support for
     * making rounded polygons.
     */
    public m_radius = 0;

    public constructor(type: b2ShapeType, radius: number) {
        this.m_type = type;
        this.m_radius = radius;
    }

    /**
     * Clone the concrete shape.
     */
    public abstract Clone(): b2Shape;

    public Copy(other: b2Shape): b2Shape {
        // DEBUG: b2Assert(this.m_type === other.m_type);
        this.m_radius = other.m_radius;
        return this;
    }

    /**
     * Get the type of this shape. You can use this to down cast to the concrete shape.
     *
     * @returns The shape type.
     */
    public GetType(): b2ShapeType {
        return this.m_type;
    }

    /**
     * Get the number of child primitives.
     */
    public abstract GetChildCount(): number;

    /**
     * Test a point for containment in this shape. This only works for convex shapes.
     *
     * @param xf The shape world transform.
     * @param p A point in world coordinates.
     */
    public abstract TestPoint(xf: b2Transform, p: XY): boolean;

    /**
     * Cast a ray against a child shape.
     *
     * @param output The ray-cast results.
     * @param input The ray-cast input parameters.
     * @param transform The transform to be applied to the shape.
     * @param childIndex The child shape index
     */
    public abstract RayCast(
        output: b2RayCastOutput,
        input: b2RayCastInput,
        transform: b2Transform,
        childIndex: number,
    ): boolean;

    /**
     * Given a transform, compute the associated axis aligned bounding box for a child shape.
     *
     * @param aabb Returns the axis aligned box.
     * @param xf The world transform of the shape.
     * @param childIndex The child shape
     */
    public abstract ComputeAABB(aabb: b2AABB, xf: b2Transform, childIndex: number): void;

    /**
     * Compute the mass properties of this shape using its dimensions and density.
     * The inertia tensor is computed about the local origin.
     *
     * @param massData Returns the mass data for this shape.
     * @param density The density in kilograms per meter squared.
     */
    public abstract ComputeMass(massData: b2MassData, density: number): void;

    public abstract SetupDistanceProxy(proxy: b2DistanceProxy, index: number): void;

    public abstract Draw(draw: b2Draw, color: b2Color): void;
}
