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

// DEBUG: import { assert } from "../common/b2_common";
import { Color, Draw } from "../common/b2_draw";
import { Vec2, Transform, XY } from "../common/b2_math";
import { AABB, RayCastInput, RayCastOutput } from "./b2_collision";
import { DistanceProxy } from "./b2_distance";

/**
 * This holds the mass data computed for a shape.
 */
export class MassData {
    /** The mass of the shape, usually in kilograms. */
    public mass = 0;

    /** The position of the shape's centroid relative to the shape's origin. */
    public readonly center = new Vec2();

    /** The rotational inertia of the shape about the local origin. */
    public I = 0;
}

export enum ShapeType {
    Unknown = -1,
    Circle = 0,
    Edge = 1,
    Polygon = 2,
    Chain = 3,
    TypeCount = 4,
}

/**
 * A shape is used for collision detection. You can create a shape however you like.
 * Shapes used for simulation in World are created automatically when a Fixture
 * is created. Shapes may encapsulate a one or more child shapes.
 */
export abstract class Shape {
    public readonly m_type: ShapeType;

    /**
     * Radius of a shape. For polygonal shapes this must be POLYGON_RADIUS. There is no support for
     * making rounded polygons.
     */
    public m_radius = 0;

    public constructor(type: ShapeType, radius: number) {
        this.m_type = type;
        this.m_radius = radius;
    }

    /**
     * Clone the concrete shape.
     */
    public abstract clone(): Shape;

    public copy(other: Shape): Shape {
        // DEBUG: assert(this.m_type === other.m_type);
        this.m_radius = other.m_radius;
        return this;
    }

    /**
     * Get the type of this shape. You can use this to down cast to the concrete shape.
     *
     * @returns The shape type.
     */
    public getType(): ShapeType {
        return this.m_type;
    }

    /**
     * Get the number of child primitives.
     */
    public abstract getChildCount(): number;

    /**
     * Test a point for containment in this shape. This only works for convex shapes.
     *
     * @param xf The shape world transform.
     * @param p A point in world coordinates.
     */
    public abstract testPoint(xf: Transform, p: XY): boolean;

    /**
     * Cast a ray against a child shape.
     *
     * @param output The ray-cast results.
     * @param input The ray-cast input parameters.
     * @param transform The transform to be applied to the shape.
     * @param childIndex The child shape index
     */
    public abstract rayCast(
        output: RayCastOutput,
        input: RayCastInput,
        transform: Transform,
        childIndex: number,
    ): boolean;

    /**
     * Given a transform, compute the associated axis aligned bounding box for a child shape.
     *
     * @param aabb Returns the axis aligned box.
     * @param xf The world transform of the shape.
     * @param childIndex The child shape
     */
    public abstract computeAABB(aabb: AABB, xf: Transform, childIndex: number): void;

    /**
     * Compute the mass properties of this shape using its dimensions and density.
     * The inertia tensor is computed about the local origin.
     *
     * @param massData Returns the mass data for this shape.
     * @param density The density in kilograms per meter squared.
     */
    public abstract computeMass(massData: MassData, density: number): void;

    // Fixme: check the logic of the implementations. Seems strange
    public abstract setupDistanceProxy(proxy: DistanceProxy, index: number): void;

    public abstract draw(draw: Draw, color: Color): void;
}
