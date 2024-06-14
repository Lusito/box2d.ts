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
import { b2Vec2, b2Transform, XY } from "../common/b2_math";
import { b2AABB, b2RayCastInput, b2RayCastOutput } from "../collision/b2_collision";
import { b2TreeNode } from "../collision/b2_dynamic_tree";
import { b2Shape, b2ShapeType, b2MassData } from "../collision/b2_shape";
import type { b2Body } from "./b2_body";
import { b2Assert } from "../common/b2_common";
import { b2Readonly } from "../common/b2_readonly";
import { b2_lengthUnitsPerMeter } from "../common/b2_settings";
import { b2BroadPhase } from "../collision/b2_broad_phase";
import type { b2FixtureUserData } from "..";

const temp = {
    c1: new b2Vec2(),
    c2: new b2Vec2(),
};

/**
 * This holds contact filtering data.
 */
export interface b2Filter {
    /** The collision category bits. Normally you would just set one bit. */
    categoryBits: number;

    /**
     * The collision mask bits. This states the categories that this
     * shape would accept for collision.
     */
    maskBits: number;

    /**
     * Collision groups allow a certain group of objects to never collide (negative)
     * or always collide (positive). Zero means no collision group. Non-zero group
     * filtering always wins against the mask bits.
     */
    groupIndex: number;
}

export const b2DefaultFilter: Readonly<b2Filter> = {
    categoryBits: 0x0001,
    maskBits: 0xffff,
    groupIndex: 0,
};

/**
 * A fixture definition is used to create a fixture. This class defines an
 * abstract fixture definition. You can reuse fixture definitions safely.
 */
export interface b2FixtureDef {
    /**
     * The shape, this must be set. The shape will be cloned, so you
     * can create the shape on the stack.
     */
    shape: b2Shape;

    /** Use this to store application specific fixture data. */
    userData?: b2FixtureUserData;

    /** The friction coefficient, usually in the range [0,1]. */
    friction?: number;

    /** The restitution (elasticity) usually in the range [0,1]. */
    restitution?: number;

    /**
     * Restitution velocity threshold, usually in m/s. Collisions above this
     * speed have restitution applied (will bounce).
     */
    restitutionThreshold?: number;

    /** The density, usually in kg/m^2. */
    density?: number;

    /**
     * A sensor shape collects contact information but never generates a collision
     * response.
     */
    isSensor?: boolean;

    /** Contact filtering data. */
    filter?: Partial<b2Filter>;
}

/**
 * This proxy is used internally to connect fixtures to the broad-phase.
 */
export class b2FixtureProxy {
    public readonly aabb = new b2AABB();

    public readonly fixture: b2Fixture;

    public readonly childIndex: number;

    public readonly treeNode: b2TreeNode<b2FixtureProxy>;

    public constructor(
        fixture: b2Fixture,
        broadPhase: b2BroadPhase<b2FixtureProxy>,
        xf: b2Readonly<b2Transform>,
        childIndex: number,
    ) {
        this.fixture = fixture;
        this.childIndex = childIndex;
        fixture.m_shape.ComputeAABB(this.aabb, xf, childIndex);
        this.treeNode = broadPhase.CreateProxy(this.aabb, this);
    }
}

const Synchronize_s_aabb1 = new b2AABB();
const Synchronize_s_aabb2 = new b2AABB();
const Synchronize_s_displacement = new b2Vec2();

/**
 * A fixture is used to attach a shape to a body for collision detection. A fixture
 * inherits its transform from its parent. Fixtures hold additional non-geometric data
 * such as friction, collision filters, etc.
 * Fixtures are created via b2Body::CreateFixture.
 *
 * @warning you cannot reuse fixtures.
 */
export class b2Fixture {
    /** @internal protected */
    public m_density = 0;

    /** @internal protected */
    public m_next: b2Fixture | null = null;

    /** @internal protected */
    public readonly m_body: b2Body;

    /** @internal protected */
    public readonly m_shape: b2Shape;

    /** @internal protected */
    public m_friction = 0;

    /** @internal protected */
    public m_restitution = 0;

    /** @internal protected */
    public m_restitutionThreshold = 0;

    /** @internal protected */
    public readonly m_proxies: b2FixtureProxy[] = [];

    /** @internal protected */
    public get m_proxyCount(): number {
        return this.m_proxies.length;
    }

    protected readonly m_filter: b2Filter;

    /** @internal protected */
    public m_isSensor = false;

    protected readonly m_userData: b2FixtureUserData = {};

    /** @internal protected */
    public constructor(body: b2Body, def: b2FixtureDef) {
        this.m_body = body;
        this.m_shape = def.shape.Clone();
        if (def.userData) this.SetUserData(def.userData);
        this.m_friction = def.friction ?? 0.2;
        this.m_restitution = def.restitution ?? 0;
        this.m_restitutionThreshold = def.restitutionThreshold ?? b2_lengthUnitsPerMeter;
        this.m_filter = {
            ...b2DefaultFilter,
            ...def.filter,
        };
        this.m_isSensor = def.isSensor ?? false;
        this.m_density = def.density ?? 0;
    }

    /**
     * Get the type of the child shape. You can use this to down cast to the concrete shape.
     *
     * @returns The shape type.
     */
    public GetType(): b2ShapeType {
        return this.m_shape.GetType();
    }

    /**
     * Get the child shape. You can modify the child shape, however you should not change the
     * number of vertices because this will crash some collision caching mechanisms.
     * Manipulating the shape may lead to non-physical behavior.
     */
    public GetShape(): b2Shape {
        return this.m_shape;
    }

    /**
     * Set if this fixture is a sensor.
     */
    public SetSensor(sensor: boolean): void {
        if (sensor !== this.m_isSensor) {
            this.m_body.SetAwake(true);
            this.m_isSensor = sensor;
        }
    }

    /**
     * Is this fixture a sensor (non-solid)?
     *
     * @returns The true if the shape is a sensor.
     */
    public IsSensor(): boolean {
        return this.m_isSensor;
    }

    /**
     * Set the contact filtering data. This will not update contacts until the next time
     * step when either parent body is active and awake.
     * This automatically calls Refilter.
     */
    public SetFilterData(filter: Readonly<Partial<b2Filter>>): void {
        this.m_filter.categoryBits = filter.categoryBits ?? b2DefaultFilter.categoryBits;
        this.m_filter.groupIndex = filter.groupIndex ?? b2DefaultFilter.groupIndex;
        this.m_filter.maskBits = filter.maskBits ?? b2DefaultFilter.maskBits;

        this.Refilter();
    }

    /**
     * Get the contact filtering data.
     */
    public GetFilterData(): Readonly<b2Filter> {
        return this.m_filter;
    }

    /**
     * Call this if you want to establish collision that was previously disabled by b2ContactFilter::ShouldCollide.
     */
    public Refilter(): void {
        // Flag associated contacts for filtering.
        let edge = this.m_body.GetContactList();

        while (edge) {
            const { contact } = edge;
            const fixtureA = contact.GetFixtureA();
            const fixtureB = contact.GetFixtureB();
            if (fixtureA === this || fixtureB === this) {
                contact.FlagForFiltering();
            }

            edge = edge.next;
        }

        const world = this.m_body.GetWorld();

        // Touch each proxy so that new pairs may be created
        const broadPhase = world.m_contactManager.m_broadPhase;
        for (const proxy of this.m_proxies) {
            broadPhase.TouchProxy(proxy.treeNode);
        }
    }

    /**
     * Get the parent body of this fixture.
     *
     * @returns The parent body.
     */
    public GetBody(): b2Body {
        return this.m_body;
    }

    /**
     * Get the next fixture in the parent body's fixture list.
     *
     * @returns The next shape.
     */
    public GetNext(): b2Fixture | null {
        return this.m_next;
    }

    /**
     * Get the user data that was assigned in the fixture definition. Use this to
     * store your application specific data.
     */
    public GetUserData(): b2FixtureUserData {
        return this.m_userData;
    }

    /**
     * Set the user data. Use this to store your application specific data.
     * This is a merge operation. Only specified keys will be overridden.
     */
    public SetUserData(data: b2FixtureUserData): void {
        Object.assign(this.m_userData, data);
    }

    /**
     * Test a point for containment in this fixture.
     *
     * @param p A point in world coordinates.
     */
    public TestPoint(p: XY): boolean {
        return this.m_shape.TestPoint(this.m_body.GetTransform(), p);
    }

    /**
     * Cast a ray against this shape.
     *
     * @param output The ray-cast results.
     * @param input The ray-cast input parameters.
     */
    public RayCast(output: b2RayCastOutput, input: b2RayCastInput, childIndex: number): boolean {
        return this.m_shape.RayCast(output, input, this.m_body.GetTransform(), childIndex);
    }

    /**
     * Get the mass data for this fixture. The mass data is based on the density and
     * the shape. The rotational inertia is about the shape's origin. This operation
     * may be expensive.
     */
    public GetMassData(massData = new b2MassData()): b2MassData {
        this.m_shape.ComputeMass(massData, this.m_density);

        return massData;
    }

    /**
     * Set the density of this fixture. This will _not_ automatically adjust the mass
     * of the body. You must call b2Body::ResetMassData to update the body's mass.
     */
    public SetDensity(density: number): void {
        // DEBUG: b2Assert(Number.isFinite(density) && density >= 0);
        this.m_density = density;
    }

    /**
     * Get the density of this fixture.
     */
    public GetDensity(): number {
        return this.m_density;
    }

    /**
     * Get the coefficient of friction.
     */
    public GetFriction(): number {
        return this.m_friction;
    }

    /**
     * Set the coefficient of friction. This will _not_ change the friction of
     * existing contacts.
     */
    public SetFriction(friction: number): void {
        this.m_friction = friction;
    }

    /**
     * Get the coefficient of restitution.
     */
    public GetRestitution(): number {
        return this.m_restitution;
    }

    /**
     * Set the coefficient of restitution. This will _not_ change the restitution of
     * existing contacts.
     */
    public SetRestitution(restitution: number): void {
        this.m_restitution = restitution;
    }

    /** Get the restitution velocity threshold. */
    public GetRestitutionThreshold(): number {
        return this.m_restitutionThreshold;
    }

    /**
     * Set the restitution threshold. This will _not_ change the restitution threshold of
     * existing contacts.
     */
    public SetRestitutionThreshold(threshold: number): void {
        this.m_restitutionThreshold = threshold;
    }

    /**
     * Get the fixture's AABB. This AABB may be enlarge and/or stale.
     * If you need a more accurate AABB, compute it using the shape and
     * the body transform.
     */
    public GetAABB(childIndex: number): Readonly<b2AABB> {
        // DEBUG: b2Assert(0 <= childIndex && childIndex < this.m_proxyCount);
        return this.m_proxies[childIndex].aabb;
    }

    /**
     * These support body activation/deactivation.
     *
     * @internal protected
     */
    public CreateProxies(broadPhase: b2BroadPhase<b2FixtureProxy>, xf: b2Readonly<b2Transform>): void {
        b2Assert(this.m_proxies.length === 0);
        // Create proxies in the broad-phase.
        this.m_proxies.length = this.m_shape.GetChildCount();
        for (let i = 0; i < this.m_proxies.length; ++i) {
            this.m_proxies[i] = new b2FixtureProxy(this, broadPhase, xf, i);
        }
    }

    /** @internal protected */
    public DestroyProxies(broadPhase: b2BroadPhase<b2FixtureProxy>): void {
        // Destroy proxies in the broad-phase.
        for (const proxy of this.m_proxies) {
            broadPhase.DestroyProxy(proxy.treeNode);
        }
        this.m_proxies.length = 0;
    }

    /** @internal protected */
    public Synchronize(broadPhase: b2BroadPhase<b2FixtureProxy>, transform1: b2Readonly<b2Transform>, transform2: b2Readonly<b2Transform>) {
        const { c1, c2 } = temp;
        const displacement = Synchronize_s_displacement;
        for (const proxy of this.m_proxies) {
            // Compute an AABB that covers the swept shape (may miss some rotation effect).
            const aabb1 = Synchronize_s_aabb1;
            const aabb2 = Synchronize_s_aabb2;
            this.m_shape.ComputeAABB(aabb1, transform1, proxy.childIndex);
            this.m_shape.ComputeAABB(aabb2, transform2, proxy.childIndex);

            proxy.aabb.Combine2(aabb1, aabb2);

            b2Vec2.Subtract(aabb2.GetCenter(c2), aabb1.GetCenter(c1), displacement);

            broadPhase.MoveProxy(proxy.treeNode, proxy.aabb, displacement);
        }
    }
}
