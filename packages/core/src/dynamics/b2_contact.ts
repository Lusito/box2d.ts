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

import { b2Assert } from "../common/b2_common";
import { b2Transform } from "../common/b2_math";
import { b2Manifold, b2WorldManifold, b2TestOverlap } from "../collision/b2_collision";
import { b2Body } from "./b2_body";
import { b2Fixture } from "./b2_fixture";
import { b2Shape } from "../collision/b2_shape";
import type { b2ContactListener } from "./b2_world_callbacks";
import { b2Readonly } from "../common/b2_readonly";

/**
 * Friction mixing law. The idea is to allow either fixture to drive the friction to zero.
 * For example, anything slides on ice.
 */
export function b2MixFriction(friction1: number, friction2: number): number {
    return Math.sqrt(friction1 * friction2);
}

/**
 * Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
 * For example, a superball bounces on anything.
 */
export function b2MixRestitution(restitution1: number, restitution2: number): number {
    return restitution1 > restitution2 ? restitution1 : restitution2;
}

/**
 * Restitution mixing law. This picks the lowest value.
 */
export function b2MixRestitutionThreshold(threshold1: number, threshold2: number) {
    return threshold1 < threshold2 ? threshold1 : threshold2;
}

/**
 * A contact edge is used to connect bodies and contacts together
 * in a contact graph where each body is a node and each contact
 * is an edge. A contact edge belongs to a doubly linked list
 * maintained in each attached body. Each contact has two contact
 * nodes, one for each attached body.
 */
export class b2ContactEdge {
    /** Provides quick access to the other body attached. */
    private m_other: b2Body | null = null;

    public get other(): b2Body {
        b2Assert(this.m_other !== null);
        return this.m_other;
    }

    public set other(value: b2Body) {
        b2Assert(this.m_other === null);
        this.m_other = value;
    }

    /** The contact */
    public readonly contact: b2Contact;

    /** The previous contact edge in the body's contact list */
    public prev: b2ContactEdge | null = null;

    /** The next contact edge in the body's contact list */
    public next: b2ContactEdge | null = null;

    public constructor(contact: b2Contact) {
        this.contact = contact;
    }

    public Reset(): void {
        this.m_other = null;
        this.prev = null;
        this.next = null;
    }
}

/**
 * The class manages contact between two shapes. A contact exists for each overlapping
 * AABB in the broad-phase (except if filtered). Therefore a contact object may exist
 * that has no contact points.
 */
export abstract class b2Contact<A extends b2Shape = b2Shape, B extends b2Shape = b2Shape> {
    /**
     * Used when crawling contact graph when forming islands.
     *
     * @internal protected
     */
    public m_islandFlag = false;

    /**
     * Set when the shapes are touching.
     *
     * @internal protected
     */
    public m_touchingFlag = false;

    /**
     * This contact can be disabled (by user)
     *
     * @internal protected
     */
    public m_enabledFlag = false;

    /**
     * This contact needs filtering because a fixture filter was changed.
     *
     * @internal protected
     */
    public m_filterFlag = false;

    /**
     * This bullet contact had a TOI event
     *
     * @internal protected
     */
    public m_bulletHitFlag = false;

    /**
     * This contact has a valid TOI in m_toi
     *
     * @internal protected
     */
    public m_toiFlag = false;

    /**
     * World pool and list pointers.
     *
     * @internal protected
     */
    public m_prev: b2Contact | null = null;

    /** @internal protected */
    public m_next: b2Contact | null = null;

    /**
     * Nodes for connecting bodies.
     *
     * @internal protected
     */
    public readonly m_nodeA: b2ContactEdge = new b2ContactEdge(this);

    /** @internal protected */
    public readonly m_nodeB: b2ContactEdge = new b2ContactEdge(this);

    /** @internal protected */
    public m_fixtureA!: b2Fixture;

    /** @internal protected */
    public m_fixtureB!: b2Fixture;

    /** @internal protected */
    public m_indexA = 0;

    /** @internal protected */
    public m_indexB = 0;

    /** @internal protected */
    public m_manifold = new b2Manifold(); // TODO: readonly

    /** @internal protected */
    public m_toiCount = 0;

    /** @internal protected */
    public m_toi = 0;

    /** @internal protected */
    public m_friction = 0;

    /** @internal protected */
    public m_restitution = 0;

    /** @internal protected */
    public m_restitutionThreshold = 0;

    /** @internal protected */
    public m_tangentSpeed = 0;

    protected m_oldManifold = new b2Manifold(); // TODO: readonly

    /**
     * Get the contact manifold.
     * Do not modify the manifold unless you understand the internals of Box2D.
     */
    public GetManifold() {
        return this.m_manifold;
    }

    /** Get the world manifold. */
    public GetWorldManifold(worldManifold: b2WorldManifold): void {
        const bodyA = this.m_fixtureA.GetBody();
        const bodyB = this.m_fixtureB.GetBody();
        const shapeA = this.GetShapeA();
        const shapeB = this.GetShapeB();
        worldManifold.Initialize(
            this.m_manifold,
            bodyA.GetTransform(),
            shapeA.m_radius,
            bodyB.GetTransform(),
            shapeB.m_radius,
        );
    }

    /** Is this contact touching? */
    public IsTouching(): boolean {
        return this.m_touchingFlag;
    }

    /**
     * Enable/disable this contact. This can be used inside the pre-solve
     * contact listener. The contact is only disabled for the current
     * time step (or sub-step in continuous collisions).
     */
    public SetEnabled(flag: boolean): void {
        this.m_enabledFlag = flag;
    }

    /** Has this contact been disabled? */
    public IsEnabled(): boolean {
        return this.m_enabledFlag;
    }

    /** Get the next contact in the world's contact list. */
    public GetNext(): b2Contact | null {
        return this.m_next;
    }

    /** Get fixture A in this contact. */
    public GetFixtureA(): b2Fixture {
        return this.m_fixtureA;
    }

    /** Get the child primitive index for fixture A. */
    public GetChildIndexA(): number {
        return this.m_indexA;
    }

    /** Get fixture A in this contact. */
    public GetShapeA(): A {
        return this.m_fixtureA.GetShape() as A;
    }

    /** Get fixture B in this contact. */
    public GetFixtureB(): b2Fixture {
        return this.m_fixtureB;
    }

    /** Get the child primitive index for fixture B. */
    public GetChildIndexB(): number {
        return this.m_indexB;
    }

    public GetShapeB(): B {
        return this.m_fixtureB.GetShape() as B;
    }

    /** Evaluate this contact with your own manifold and transforms. */
    public abstract Evaluate(manifold: b2Manifold, xfA: b2Readonly<b2Transform>, xfB: b2Readonly<b2Transform>): void;

    /**
     * Flag this contact for filtering. Filtering will occur the next time step.
     *
     * @internal protected
     */
    public FlagForFiltering(): void {
        this.m_filterFlag = true;
    }

    /**
     * Override the default friction mixture.
     * You can call this in b2ContactListener::PreSolve.
     * This value persists until set or reset.
     */
    public SetFriction(friction: number): void {
        this.m_friction = friction;
    }

    /** Get the friction. */
    public GetFriction(): number {
        return this.m_friction;
    }

    /** Reset the friction mixture to the default value. */
    public ResetFriction(): void {
        this.m_friction = b2MixFriction(this.m_fixtureA.m_friction, this.m_fixtureB.m_friction);
    }

    /**
     * Override the default restitution mixture.
     * You can call this in b2ContactListener::PreSolve.
     * The value persists until you set or reset.
     */
    public SetRestitution(restitution: number): void {
        this.m_restitution = restitution;
    }

    /** Get the restitution. */
    public GetRestitution(): number {
        return this.m_restitution;
    }

    /** Reset the restitution to the default value. */
    public ResetRestitution(): void {
        this.m_restitution = b2MixRestitution(this.m_fixtureA.m_restitution, this.m_fixtureB.m_restitution);
    }

    /**
     * Override the default restitution velocity threshold mixture. You can call this in b2ContactListener::PreSolve.
     * The value persists until you set or reset.
     */
    public SetRestitutionThreshold(threshold: number) {
        this.m_restitutionThreshold = threshold;
    }

    /**
     * Get the restitution threshold.
     */
    public GetRestitutionThreshold() {
        return this.m_restitutionThreshold;
    }

    /**
     * Reset the restitution threshold to the default value.
     */
    public ResetRestitutionThreshold() {
        this.m_restitutionThreshold = b2MixRestitutionThreshold(
            this.m_fixtureA.m_restitutionThreshold,
            this.m_fixtureB.m_restitutionThreshold,
        );
    }

    /** Set the desired tangent speed for a conveyor belt behavior. In meters per second. */
    public SetTangentSpeed(speed: number): void {
        this.m_tangentSpeed = speed;
    }

    /** Get the desired tangent speed. In meters per second. */
    public GetTangentSpeed(): number {
        return this.m_tangentSpeed;
    }

    public Reset(fixtureA: b2Fixture, indexA: number, fixtureB: b2Fixture, indexB: number): void {
        this.m_islandFlag = false;
        this.m_touchingFlag = false;
        this.m_enabledFlag = true;
        this.m_filterFlag = false;
        this.m_bulletHitFlag = false;
        this.m_toiFlag = false;

        this.m_fixtureA = fixtureA;
        this.m_fixtureB = fixtureB;

        this.m_indexA = indexA;
        this.m_indexB = indexB;

        this.m_manifold.pointCount = 0;

        this.m_prev = null;
        this.m_next = null;

        this.m_nodeA.Reset();
        this.m_nodeB.Reset();

        this.m_toiCount = 0;

        this.m_friction = b2MixFriction(this.m_fixtureA.m_friction, this.m_fixtureB.m_friction);
        this.m_restitution = b2MixRestitution(this.m_fixtureA.m_restitution, this.m_fixtureB.m_restitution);
        this.m_restitutionThreshold = b2MixRestitutionThreshold(
            this.m_fixtureA.m_restitutionThreshold,
            this.m_fixtureB.m_restitutionThreshold,
        );
    }

    /**
     * Update the contact manifold and touching status.
     * Note: do not assume the fixture AABBs are overlapping or are valid.
     *
     * @internal protected
     */
    public Update(listener: b2ContactListener): void {
        const tManifold = this.m_oldManifold;
        this.m_oldManifold = this.m_manifold;
        this.m_manifold = tManifold;

        // Re-enable this contact.
        this.m_enabledFlag = true;

        let touching = false;
        const wasTouching = this.m_touchingFlag;

        const sensorA = this.m_fixtureA.IsSensor();
        const sensorB = this.m_fixtureB.IsSensor();
        const sensor = sensorA || sensorB;

        const bodyA = this.m_fixtureA.GetBody();
        const bodyB = this.m_fixtureB.GetBody();
        const xfA = bodyA.GetTransform();
        const xfB = bodyB.GetTransform();

        // Is this contact a sensor?
        if (sensor) {
            const shapeA = this.GetShapeA();
            const shapeB = this.GetShapeB();
            touching = b2TestOverlap(shapeA, this.m_indexA, shapeB, this.m_indexB, xfA, xfB);

            // Sensors don't generate manifolds.
            this.m_manifold.pointCount = 0;
        } else {
            this.Evaluate(this.m_manifold, xfA, xfB);
            touching = this.m_manifold.pointCount > 0;

            // Match old contact ids to new contact ids and copy the
            // stored impulses to warm start the solver.
            for (let i = 0; i < this.m_manifold.pointCount; ++i) {
                const mp2 = this.m_manifold.points[i];
                mp2.normalImpulse = 0;
                mp2.tangentImpulse = 0;
                const id2 = mp2.id;

                for (let j = 0; j < this.m_oldManifold.pointCount; ++j) {
                    const mp1 = this.m_oldManifold.points[j];

                    if (mp1.id.key === id2.key) {
                        mp2.normalImpulse = mp1.normalImpulse;
                        mp2.tangentImpulse = mp1.tangentImpulse;
                        break;
                    }
                }
            }

            if (touching !== wasTouching) {
                bodyA.SetAwake(true);
                bodyB.SetAwake(true);
            }
        }

        this.m_touchingFlag = touching;

        if (!wasTouching && touching && listener) {
            listener.BeginContact(this);
        }

        if (wasTouching && !touching && listener) {
            listener.EndContact(this);
        }

        if (!sensor && touching && listener) {
            listener.PreSolve(this, this.m_oldManifold);
        }
    }
}
