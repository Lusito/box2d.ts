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
    public islandFlag = false;

    /**
     * Set when the shapes are touching.
     *
     * @internal protected
     */
    public touchingFlag = false;

    /**
     * This contact can be disabled (by user)
     *
     * @internal protected
     */
    public enabledFlag = false;

    /**
     * This contact needs filtering because a fixture filter was changed.
     *
     * @internal protected
     */
    public filterFlag = false;

    /**
     * This bullet contact had a TOI event
     *
     * @internal protected
     */
    public bulletHitFlag = false;

    /**
     * This contact has a valid TOI in toi
     *
     * @internal protected
     */
    public toiFlag = false;

    /**
     * World pool and list pointers.
     *
     * @internal protected
     */
    public prev: Contact | null = null;

    /** @internal protected */
    public next: Contact | null = null;

    /**
     * Nodes for connecting bodies.
     *
     * @internal protected
     */
    public readonly nodeA: ContactEdge = new ContactEdge(this);

    /** @internal protected */
    public readonly nodeB: ContactEdge = new ContactEdge(this);

    /** @internal protected */
    public fixtureA!: Fixture;

    /** @internal protected */
    public fixtureB!: Fixture;

    /** @internal protected */
    public indexA = 0;

    /** @internal protected */
    public indexB = 0;

    /** @internal protected */
    public manifold = new Manifold(); // TODO: readonly

    /** @internal protected */
    public toiCount = 0;

    /** @internal protected */
    public toi = 0;

    /** @internal protected */
    public friction = 0;

    /** @internal protected */
    public restitution = 0;

    /** @internal protected */
    public restitutionThreshold = 0;

    /** @internal protected */
    public tangentSpeed = 0;

    protected oldManifold = new Manifold(); // TODO: readonly

    public getManifold() {
        return this.manifold;
    }

    public getWorldManifold(worldManifold: WorldManifold): void {
        const bodyA = this.fixtureA.getBody();
        const bodyB = this.fixtureB.getBody();
        const shapeA = this.getShapeA();
        const shapeB = this.getShapeB();
        worldManifold.initialize(
            this.manifold,
            bodyA.getTransform(),
            shapeA.radius,
            bodyB.getTransform(),
            shapeB.radius,
        );
    }

    public isTouching(): boolean {
        return this.touchingFlag;
    }

    public setEnabled(flag: boolean): void {
        this.enabledFlag = flag;
    }

    public isEnabled(): boolean {
        return this.enabledFlag;
    }

    public getNext(): Contact | null {
        return this.next;
    }

    public getFixtureA(): Fixture {
        return this.fixtureA;
    }

    public getChildIndexA(): number {
        return this.indexA;
    }

    public getShapeA(): A {
        return this.fixtureA.getShape() as A;
    }

    public getFixtureB(): Fixture {
        return this.fixtureB;
    }

    public getChildIndexB(): number {
        return this.indexB;
    }

    public getShapeB(): B {
        return this.fixtureB.getShape() as B;
    }

    public abstract evaluate(manifold: Manifold, xfA: Transform, xfB: Transform): void;

    /** @internal protected */
    public flagForFiltering(): void {
        this.filterFlag = true;
    }

    public setFriction(friction: number): void {
        this.friction = friction;
    }

    public getFriction(): number {
        return this.friction;
    }

    public resetFriction(): void {
        this.friction = mixFriction(this.fixtureA.friction, this.fixtureB.friction);
    }

    public setRestitution(restitution: number): void {
        this.restitution = restitution;
    }

    public getRestitution(): number {
        return this.restitution;
    }

    public resetRestitution(): void {
        this.restitution = mixRestitution(this.fixtureA.restitution, this.fixtureB.restitution);
    }

    /**
     * Override the default restitution velocity threshold mixture. You can call this in ContactListener::PreSolve.
     * The value persists until you set or reset.
     */
    public setRestitutionThreshold(threshold: number) {
        this.restitutionThreshold = threshold;
    }

    /**
     * Get the restitution threshold.
     */
    public getRestitutionThreshold() {
        return this.restitutionThreshold;
    }

    /**
     * Reset the restitution threshold to the default value.
     */
    public resetRestitutionThreshold() {
        this.restitutionThreshold = mixRestitutionThreshold(
            this.fixtureA.restitutionThreshold,
            this.fixtureB.restitutionThreshold,
        );
    }

    public setTangentSpeed(speed: number): void {
        this.tangentSpeed = speed;
    }

    public getTangentSpeed(): number {
        return this.tangentSpeed;
    }

    public reset(fixtureA: Fixture, indexA: number, fixtureB: Fixture, indexB: number): void {
        this.islandFlag = false;
        this.touchingFlag = false;
        this.enabledFlag = true;
        this.filterFlag = false;
        this.bulletHitFlag = false;
        this.toiFlag = false;

        this.fixtureA = fixtureA;
        this.fixtureB = fixtureB;

        this.indexA = indexA;
        this.indexB = indexB;

        this.manifold.pointCount = 0;

        this.prev = null;
        this.next = null;

        this.nodeA.reset();
        this.nodeB.reset();

        this.toiCount = 0;

        this.friction = mixFriction(this.fixtureA.friction, this.fixtureB.friction);
        this.restitution = mixRestitution(this.fixtureA.restitution, this.fixtureB.restitution);
        this.restitutionThreshold = mixRestitutionThreshold(
            this.fixtureA.restitutionThreshold,
            this.fixtureB.restitutionThreshold,
        );
    }

    /** @internal protected */
    public update(listener: ContactListener): void {
        const tManifold = this.oldManifold;
        this.oldManifold = this.manifold;
        this.manifold = tManifold;

        // Re-enable this contact.
        this.enabledFlag = true;

        let touching = false;
        const wasTouching = this.touchingFlag;

        const sensorA = this.fixtureA.isSensor();
        const sensorB = this.fixtureB.isSensor();
        const sensor = sensorA || sensorB;

        const bodyA = this.fixtureA.getBody();
        const bodyB = this.fixtureB.getBody();
        const xfA = bodyA.getTransform();
        const xfB = bodyB.getTransform();

        // Is this contact a sensor?
        if (sensor) {
            const shapeA = this.getShapeA();
            const shapeB = this.getShapeB();
            touching = testOverlap(shapeA, this.indexA, shapeB, this.indexB, xfA, xfB);

            // Sensors don't generate manifolds.
            this.manifold.pointCount = 0;
        } else {
            this.evaluate(this.manifold, xfA, xfB);
            touching = this.manifold.pointCount > 0;

            // Match old contact ids to new contact ids and copy the
            // stored impulses to warm start the solver.
            for (let i = 0; i < this.manifold.pointCount; ++i) {
                const mp2 = this.manifold.points[i];
                mp2.normalImpulse = 0;
                mp2.tangentImpulse = 0;
                const id2 = mp2.id;

                for (let j = 0; j < this.oldManifold.pointCount; ++j) {
                    const mp1 = this.oldManifold.points[j];

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

        this.touchingFlag = touching;

        if (!wasTouching && touching && listener) {
            listener.beginContact(this);
        }

        if (wasTouching && !touching && listener) {
            listener.endContact(this);
        }

        if (!sensor && touching && listener) {
            listener.preSolve(this, this.oldManifold);
        }
    }
}
