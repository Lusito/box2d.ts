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
import { Vec2, Transform, XY } from "../common/b2_math";
import { AABB, RayCastInput, RayCastOutput } from "../collision/b2_collision";
import { TreeNode } from "../collision/b2_dynamic_tree";
import { Shape, ShapeType, MassData } from "../collision/b2_shape";
import type { Body } from "./b2_body";
import { assert } from "../common/b2_common";
import { LENGTH_UNITS_PER_METER } from "../common/b2_settings";
import { BroadPhase } from "../collision/b2_broad_phase";

const temp = {
    c1: new Vec2(),
    c2: new Vec2(),
};

/**
 * This holds contact filtering data.
 */
export interface Filter {
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

export const DefaultFilter: Readonly<Filter> = {
    categoryBits: 0x0001,
    maskBits: 0xffff,
    groupIndex: 0,
};

/**
 * A fixture definition is used to create a fixture. This class defines an
 * abstract fixture definition. You can reuse fixture definitions safely.
 */
export interface FixtureDef {
    /**
     * The shape, this must be set. The shape will be cloned, so you
     * can create the shape on the stack.
     */
    shape: Shape;

    /** Use this to store application specific fixture data. */
    userData?: any;

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
    filter?: Partial<Filter>;
}

/**
 * This proxy is used internally to connect fixtures to the broad-phase.
 */
export class FixtureProxy {
    public readonly aabb = new AABB();

    public readonly fixture: Fixture;

    public readonly childIndex: number;

    public readonly treeNode: TreeNode<FixtureProxy>;

    public constructor(fixture: Fixture, broadPhase: BroadPhase<FixtureProxy>, xf: Transform, childIndex: number) {
        this.fixture = fixture;
        this.childIndex = childIndex;
        fixture.shape.computeAABB(this.aabb, xf, childIndex);
        this.treeNode = broadPhase.createProxy(this.aabb, this);
    }
}

const Synchronize_s_aabb1 = new AABB();
const Synchronize_s_aabb2 = new AABB();
const Synchronize_s_displacement = new Vec2();

/**
 * A fixture is used to attach a shape to a body for collision detection. A fixture
 * inherits its transform from its parent. Fixtures hold additional non-geometric data
 * such as friction, collision filters, etc.
 * Fixtures are created via Body::CreateFixture.
 *
 * @warning you cannot reuse fixtures.
 */
export class Fixture {
    /** @internal protected */
    public density = 0;

    /** @internal protected */
    public next: Fixture | null = null;

    /** @internal protected */
    public readonly body: Body;

    /** @internal protected */
    public readonly shape: Shape;

    /** @internal protected */
    public friction = 0;

    /** @internal protected */
    public restitution = 0;

    /** @internal protected */
    public restitutionThreshold = 0;

    /** @internal protected */
    public readonly proxies: FixtureProxy[] = [];

    protected readonly filter: Filter;

    /** @internal protected */
    public m_isSensor = false;

    protected userData: any = null;

    /** @internal protected */
    public constructor(body: Body, def: FixtureDef) {
        this.body = body;
        this.shape = def.shape.clone();
        this.userData = def.userData;
        this.friction = def.friction ?? 0.2;
        this.restitution = def.restitution ?? 0;
        this.restitutionThreshold = def.restitutionThreshold ?? LENGTH_UNITS_PER_METER;
        this.filter = {
            ...DefaultFilter,
            ...def.filter,
        };
        this.m_isSensor = def.isSensor ?? false;
        this.density = def.density ?? 0;
    }

    /**
     * Get the type of the child shape. You can use this to down cast to the concrete shape.
     *
     * @returns The shape type.
     */
    public getType(): ShapeType {
        return this.shape.getType();
    }

    /**
     * Get the child shape. You can modify the child shape, however you should not change the
     * number of vertices because this will crash some collision caching mechanisms.
     * Manipulating the shape may lead to non-physical behavior.
     */
    public getShape(): Shape {
        return this.shape;
    }

    /**
     * Set if this fixture is a sensor.
     */
    public setSensor(sensor: boolean): void {
        if (sensor !== this.m_isSensor) {
            this.body.setAwake(true);
            this.m_isSensor = sensor;
        }
    }

    /**
     * Is this fixture a sensor (non-solid)?
     *
     * @returns The true if the shape is a sensor.
     */
    public isSensor(): boolean {
        return this.m_isSensor;
    }

    /**
     * Set the contact filtering data. This will not update contacts until the next time
     * step when either parent body is active and awake.
     * This automatically calls Refilter.
     */
    public setFilterData(filter: Readonly<Partial<Filter>>): void {
        this.filter.categoryBits = filter.categoryBits ?? DefaultFilter.categoryBits;
        this.filter.groupIndex = filter.groupIndex ?? DefaultFilter.groupIndex;
        this.filter.maskBits = filter.maskBits ?? DefaultFilter.maskBits;

        this.refilter();
    }

    /**
     * Get the contact filtering data.
     */
    public getFilterData(): Readonly<Filter> {
        return this.filter;
    }

    /**
     * Call this if you want to establish collision that was previously disabled by ContactFilter::ShouldCollide.
     */
    public refilter(): void {
        // Flag associated contacts for filtering.
        let edge = this.body.getContactList();

        while (edge) {
            const { contact } = edge;
            const fixtureA = contact.getFixtureA();
            const fixtureB = contact.getFixtureB();
            if (fixtureA === this || fixtureB === this) {
                contact.flagForFiltering();
            }

            edge = edge.next;
        }

        const world = this.body.getWorld();

        // Touch each proxy so that new pairs may be created
        const { broadPhase } = world.contactManager;
        for (const proxy of this.proxies) {
            broadPhase.touchProxy(proxy.treeNode);
        }
    }

    /**
     * Get the parent body of this fixture. This is NULL if the fixture is not attached.
     *
     * @returns The parent body.
     */
    public getBody(): Body {
        return this.body;
    }

    /**
     * Get the next fixture in the parent body's fixture list.
     *
     * @returns The next shape.
     */
    public getNext(): Fixture | null {
        return this.next;
    }

    /**
     * Get the user data that was assigned in the fixture definition. Use this to
     * store your application specific data.
     */
    public getUserData(): any {
        return this.userData;
    }

    /**
     * Set the user data. Use this to store your application specific data.
     */
    public setUserData(data: any): void {
        this.userData = data;
    }

    /**
     * Test a point for containment in this fixture.
     *
     * @param p A point in world coordinates.
     */
    public testPoint(p: XY): boolean {
        return this.shape.testPoint(this.body.getTransform(), p);
    }

    /**
     * Cast a ray against this shape.
     *
     * @param output The ray-cast results.
     * @param input The ray-cast input parameters.
     */
    public rayCast(output: RayCastOutput, input: RayCastInput, childIndex: number): boolean {
        return this.shape.rayCast(output, input, this.body.getTransform(), childIndex);
    }

    /**
     * Get the mass data for this fixture. The mass data is based on the density and
     * the shape. The rotational inertia is about the shape's origin. This operation
     * may be expensive.
     */
    public getMassData(massData = new MassData()): MassData {
        this.shape.computeMass(massData, this.density);

        return massData;
    }

    /**
     * Set the density of this fixture. This will _not_ automatically adjust the mass
     * of the body. You must call Body::ResetMassData to update the body's mass.
     */
    public setDensity(density: number): void {
        // DEBUG: assert(Number.isFinite(density) && density >= 0);
        this.density = density;
    }

    /**
     * Get the density of this fixture.
     */
    public getDensity(): number {
        return this.density;
    }

    /**
     * Get the coefficient of friction.
     */
    public getFriction(): number {
        return this.friction;
    }

    /**
     * Set the coefficient of friction. This will _not_ change the friction of
     * existing contacts.
     */
    public setFriction(friction: number): void {
        this.friction = friction;
    }

    /**
     * Get the coefficient of restitution.
     */
    public getRestitution(): number {
        return this.restitution;
    }

    /**
     * Set the coefficient of restitution. This will _not_ change the restitution of
     * existing contacts.
     */
    public setRestitution(restitution: number): void {
        this.restitution = restitution;
    }

    public setRestitutionThreshold(threshold: number): void {
        this.restitutionThreshold = threshold;
    }

    /**
     * Get the fixture's AABB. This AABB may be enlarge and/or stale.
     * If you need a more accurate AABB, compute it using the shape and
     * the body transform.
     */
    public getAABB(childIndex: number): Readonly<AABB> {
        // DEBUG: assert(0 <= childIndex && childIndex < this.proxyCount);
        return this.proxies[childIndex].aabb;
    }

    /**
     * These support body activation/deactivation.
     *
     * @internal protected
     */
    public createProxies(broadPhase: BroadPhase<FixtureProxy>, xf: Transform): void {
        assert(this.proxies.length === 0);
        // Create proxies in the broad-phase.
        this.proxies.length = this.shape.getChildCount();
        for (let i = 0; i < this.proxies.length; ++i) {
            this.proxies[i] = new FixtureProxy(this, broadPhase, xf, i);
        }
    }

    /** @internal protected */
    public destroyProxies(broadPhase: BroadPhase<FixtureProxy>): void {
        // Destroy proxies in the broad-phase.
        for (const proxy of this.proxies) {
            broadPhase.destroyProxy(proxy.treeNode);
        }
        this.proxies.length = 0;
    }

    /** @internal protected */
    public synchronize(broadPhase: BroadPhase<FixtureProxy>, transform1: Transform, transform2: Transform) {
        const { c1, c2 } = temp;
        const displacement = Synchronize_s_displacement;
        for (const proxy of this.proxies) {
            // Compute an AABB that covers the swept shape (may miss some rotation effect).
            const aabb1 = Synchronize_s_aabb1;
            const aabb2 = Synchronize_s_aabb2;
            this.shape.computeAABB(aabb1, transform1, proxy.childIndex);
            this.shape.computeAABB(aabb2, transform2, proxy.childIndex);

            proxy.aabb.combine2(aabb1, aabb2);

            Vec2.subtract(aabb2.getCenter(c2), aabb1.getCenter(c1), displacement);

            broadPhase.moveProxy(proxy.treeNode, proxy.aabb, displacement);
        }
    }
}
