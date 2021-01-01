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

import { assert } from "../common/b2_common";
import { Transform } from "../common/b2_math";
import { Manifold, WorldManifold, testOverlap } from "../collision/b2_collision";
import { Body } from "./b2_body";
import { Fixture } from "./b2_fixture";
import { Shape } from "../collision/b2_shape";
import type { ContactListener } from "./b2_world_callbacks";

/**
 * Friction mixing law. The idea is to allow either fixture to drive the friction to zero.
 * For example, anything slides on ice.
 */
export function mixFriction(friction1: number, friction2: number): number {
    return Math.sqrt(friction1 * friction2);
}

/**
 * Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
 * For example, a superball bounces on anything.
 */
export function mixRestitution(restitution1: number, restitution2: number): number {
    return restitution1 > restitution2 ? restitution1 : restitution2;
}

/**
 * Restitution mixing law. This picks the lowest value.
 */
export function mixRestitutionThreshold(threshold1: number, threshold2: number) {
    return threshold1 < threshold2 ? threshold1 : threshold2;
}

/**
 * A contact edge is used to connect bodies and contacts together
 * in a contact graph where each body is a node and each contact
 * is an edge. A contact edge belongs to a doubly linked list
 * maintained in each attached body. Each contact has two contact
 * nodes, one for each attached body.
 */
export class ContactEdge {
    /** Provides quick access to the other body attached. */
    private m_other: Body | null = null;

    public get other(): Body {
        assert(this.m_other !== null);
        return this.m_other;
    }

    public set other(value: Body) {
        assert(this.m_other === null);
        this.m_other = value;
    }

    /** The contact */
    public readonly contact: Contact;

    /** The previous contact edge in the body's contact list */
    public prev: ContactEdge | null = null;

    /** The next contact edge in the body's contact list */
    public next: ContactEdge | null = null;

    public constructor(contact: Contact) {
        this.contact = contact;
    }

    public reset(): void {
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
export abstract class Contact<A extends Shape = Shape, B extends Shape = Shape> {
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
    public m_prev: Contact | null = null;

    /** @internal protected */
    public m_next: Contact | null = null;

    /**
     * Nodes for connecting bodies.
     *
     * @internal protected
     */
    public readonly m_nodeA: ContactEdge = new ContactEdge(this);

    /** @internal protected */
    public readonly m_nodeB: ContactEdge = new ContactEdge(this);

    /** @internal protected */
    public m_fixtureA!: Fixture;

    /** @internal protected */
    public m_fixtureB!: Fixture;

    /** @internal protected */
    public m_indexA = 0;

    /** @internal protected */
    public m_indexB = 0;

    /** @internal protected */
    public m_manifold = new Manifold(); // TODO: readonly

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

    protected m_oldManifold = new Manifold(); // TODO: readonly

    public getManifold() {
        return this.m_manifold;
    }

    public getWorldManifold(worldManifold: WorldManifold): void {
        const bodyA = this.m_fixtureA.getBody();
        const bodyB = this.m_fixtureB.getBody();
        const shapeA = this.getShapeA();
        const shapeB = this.getShapeB();
        worldManifold.initialize(
            this.m_manifold,
            bodyA.getTransform(),
            shapeA.m_radius,
            bodyB.getTransform(),
            shapeB.m_radius,
        );
    }

    public isTouching(): boolean {
        return this.m_touchingFlag;
    }

    public setEnabled(flag: boolean): void {
        this.m_enabledFlag = flag;
    }

    public isEnabled(): boolean {
        return this.m_enabledFlag;
    }

    public getNext(): Contact | null {
        return this.m_next;
    }

    public getFixtureA(): Fixture {
        return this.m_fixtureA;
    }

    public getChildIndexA(): number {
        return this.m_indexA;
    }

    public getShapeA(): A {
        return this.m_fixtureA.getShape() as A;
    }

    public getFixtureB(): Fixture {
        return this.m_fixtureB;
    }

    public getChildIndexB(): number {
        return this.m_indexB;
    }

    public getShapeB(): B {
        return this.m_fixtureB.getShape() as B;
    }

    public abstract evaluate(manifold: Manifold, xfA: Transform, xfB: Transform): void;

    /** @internal protected */
    public flagForFiltering(): void {
        this.m_filterFlag = true;
    }

    public setFriction(friction: number): void {
        this.m_friction = friction;
    }

    public getFriction(): number {
        return this.m_friction;
    }

    public resetFriction(): void {
        this.m_friction = mixFriction(this.m_fixtureA.m_friction, this.m_fixtureB.m_friction);
    }

    public setRestitution(restitution: number): void {
        this.m_restitution = restitution;
    }

    public getRestitution(): number {
        return this.m_restitution;
    }

    public resetRestitution(): void {
        this.m_restitution = mixRestitution(this.m_fixtureA.m_restitution, this.m_fixtureB.m_restitution);
    }

    /**
     * Override the default restitution velocity threshold mixture. You can call this in ContactListener::PreSolve.
     * The value persists until you set or reset.
     */
    public setRestitutionThreshold(threshold: number) {
        this.m_restitutionThreshold = threshold;
    }

    /**
     * Get the restitution threshold.
     */
    public getRestitutionThreshold() {
        return this.m_restitutionThreshold;
    }

    /**
     * Reset the restitution threshold to the default value.
     */
    public resetRestitutionThreshold() {
        this.m_restitutionThreshold = mixRestitutionThreshold(
            this.m_fixtureA.m_restitutionThreshold,
            this.m_fixtureB.m_restitutionThreshold,
        );
    }

    public setTangentSpeed(speed: number): void {
        this.m_tangentSpeed = speed;
    }

    public getTangentSpeed(): number {
        return this.m_tangentSpeed;
    }

    public reset(fixtureA: Fixture, indexA: number, fixtureB: Fixture, indexB: number): void {
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

        this.m_nodeA.reset();
        this.m_nodeB.reset();

        this.m_toiCount = 0;

        this.m_friction = mixFriction(this.m_fixtureA.m_friction, this.m_fixtureB.m_friction);
        this.m_restitution = mixRestitution(this.m_fixtureA.m_restitution, this.m_fixtureB.m_restitution);
        this.m_restitutionThreshold = mixRestitutionThreshold(
            this.m_fixtureA.m_restitutionThreshold,
            this.m_fixtureB.m_restitutionThreshold,
        );
    }

    /** @internal protected */
    public update(listener: ContactListener): void {
        const tManifold = this.m_oldManifold;
        this.m_oldManifold = this.m_manifold;
        this.m_manifold = tManifold;

        // Re-enable this contact.
        this.m_enabledFlag = true;

        let touching = false;
        const wasTouching = this.m_touchingFlag;

        const sensorA = this.m_fixtureA.isSensor();
        const sensorB = this.m_fixtureB.isSensor();
        const sensor = sensorA || sensorB;

        const bodyA = this.m_fixtureA.getBody();
        const bodyB = this.m_fixtureB.getBody();
        const xfA = bodyA.getTransform();
        const xfB = bodyB.getTransform();

        // Is this contact a sensor?
        if (sensor) {
            const shapeA = this.getShapeA();
            const shapeB = this.getShapeB();
            touching = testOverlap(shapeA, this.m_indexA, shapeB, this.m_indexB, xfA, xfB);

            // Sensors don't generate manifolds.
            this.m_manifold.pointCount = 0;
        } else {
            this.evaluate(this.m_manifold, xfA, xfB);
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
                bodyA.setAwake(true);
                bodyB.setAwake(true);
            }
        }

        this.m_touchingFlag = touching;

        if (!wasTouching && touching && listener) {
            listener.beginContact(this);
        }

        if (wasTouching && !touching && listener) {
            listener.endContact(this);
        }

        if (!sensor && touching && listener) {
            listener.preSolve(this, this.m_oldManifold);
        }
    }
}
