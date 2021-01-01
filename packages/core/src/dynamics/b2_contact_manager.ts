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
import { BroadPhase } from "../collision/b2_broad_phase";
import { Contact, ContactEdge } from "./b2_contact";
import { ContactFactory } from "./b2_contact_factory";
import { BodyType } from "./b2_body";
import { FixtureProxy } from "./b2_fixture";
import { ContactFilter, ContactListener } from "./b2_world_callbacks";

/** Delegate of World. */
export class ContactManager {
    public readonly m_broadPhase = new BroadPhase<FixtureProxy>();

    public m_contactList: Contact | null = null;

    public m_contactCount = 0;

    public m_contactFilter = ContactFilter.defaultFilter;

    public m_contactListener = ContactListener.defaultListener;

    public readonly m_contactFactory = new ContactFactory();

    /** Broad-phase callback. */
    public addPair = (proxyA: FixtureProxy, proxyB: FixtureProxy): void => {
        // DEBUG: assert(proxyA instanceof FixtureProxy);
        // DEBUG: assert(proxyB instanceof FixtureProxy);

        let fixtureA = proxyA.fixture;
        let fixtureB = proxyB.fixture;

        let indexA = proxyA.childIndex;
        let indexB = proxyB.childIndex;

        let bodyA = fixtureA.getBody();
        let bodyB = fixtureB.getBody();

        // Are the fixtures on the same body?
        if (bodyA === bodyB) {
            return;
        }

        // TODO_ERIN use a hash table to remove a potential bottleneck when both
        // bodies have a lot of contacts.
        // Does a contact already exist?
        let edge: ContactEdge | null = bodyB.getContactList();
        while (edge) {
            if (edge.other === bodyA) {
                const fA = edge.contact.getFixtureA();
                const fB = edge.contact.getFixtureB();
                const iA = edge.contact.getChildIndexA();
                const iB = edge.contact.getChildIndexB();

                if (fA === fixtureA && fB === fixtureB && iA === indexA && iB === indexB) {
                    // A contact already exists.
                    return;
                }

                if (fA === fixtureB && fB === fixtureA && iA === indexB && iB === indexA) {
                    // A contact already exists.
                    return;
                }
            }

            edge = edge.next;
        }

        // Does a joint override collision? Is at least one body dynamic?
        if (bodyB.shouldCollide(bodyA) === false) {
            return;
        }

        // Check user filtering.
        if (this.m_contactFilter && !this.m_contactFilter.shouldCollide(fixtureA, fixtureB)) {
            return;
        }

        // Call the factory.
        const c: Contact | null = this.m_contactFactory.create(fixtureA, indexA, fixtureB, indexB);
        if (c === null) {
            return;
        }

        // Contact creation may swap fixtures.
        fixtureA = c.getFixtureA();
        fixtureB = c.getFixtureB();
        indexA = c.getChildIndexA();
        indexB = c.getChildIndexB();
        bodyA = fixtureA.m_body;
        bodyB = fixtureB.m_body;

        // Insert into the world.
        c.m_prev = null;
        c.m_next = this.m_contactList;
        if (this.m_contactList !== null) {
            this.m_contactList.m_prev = c;
        }
        this.m_contactList = c;

        // Connect to island graph.

        // Connect to body A
        c.m_nodeA.other = bodyB;

        c.m_nodeA.prev = null;
        c.m_nodeA.next = bodyA.m_contactList;
        if (bodyA.m_contactList !== null) {
            bodyA.m_contactList.prev = c.m_nodeA;
        }
        bodyA.m_contactList = c.m_nodeA;

        // Connect to body B
        c.m_nodeB.other = bodyA;

        c.m_nodeB.prev = null;
        c.m_nodeB.next = bodyB.m_contactList;
        if (bodyB.m_contactList !== null) {
            bodyB.m_contactList.prev = c.m_nodeB;
        }
        bodyB.m_contactList = c.m_nodeB;

        ++this.m_contactCount;
    };

    public findNewContacts(): void {
        this.m_broadPhase.updatePairs(this.addPair);
    }

    public destroy(c: Contact): void {
        const fixtureA = c.getFixtureA();
        const fixtureB = c.getFixtureB();
        const bodyA = fixtureA.getBody();
        const bodyB = fixtureB.getBody();

        if (this.m_contactListener && c.isTouching()) {
            this.m_contactListener.endContact(c);
        }

        // Remove from the world.
        if (c.m_prev) {
            c.m_prev.m_next = c.m_next;
        }

        if (c.m_next) {
            c.m_next.m_prev = c.m_prev;
        }

        if (c === this.m_contactList) {
            this.m_contactList = c.m_next;
        }

        // Remove from body 1
        if (c.m_nodeA.prev) {
            c.m_nodeA.prev.next = c.m_nodeA.next;
        }

        if (c.m_nodeA.next) {
            c.m_nodeA.next.prev = c.m_nodeA.prev;
        }

        if (c.m_nodeA === bodyA.m_contactList) {
            bodyA.m_contactList = c.m_nodeA.next;
        }

        // Remove from body 2
        if (c.m_nodeB.prev) {
            c.m_nodeB.prev.next = c.m_nodeB.next;
        }

        if (c.m_nodeB.next) {
            c.m_nodeB.next.prev = c.m_nodeB.prev;
        }

        if (c.m_nodeB === bodyB.m_contactList) {
            bodyB.m_contactList = c.m_nodeB.next;
        }

        // moved this from ContactFactory:Destroy
        if (c.m_manifold.pointCount > 0 && !fixtureA.isSensor() && !fixtureB.isSensor()) {
            fixtureA.getBody().setAwake(true);
            fixtureB.getBody().setAwake(true);
        }

        // Call the factory.
        this.m_contactFactory.destroy(c);
        --this.m_contactCount;
    }

    /**
     * This is the top level collision call for the time step. Here
     * all the narrow phase collision is processed for the world
     * contact list.
     */
    public collide(): void {
        // Update awake contacts.
        let c: Contact | null = this.m_contactList;
        while (c) {
            const fixtureA = c.getFixtureA();
            const fixtureB = c.getFixtureB();
            const indexA = c.getChildIndexA();
            const indexB = c.getChildIndexB();
            const bodyA = fixtureA.getBody();
            const bodyB = fixtureB.getBody();

            // Is this contact flagged for filtering?
            if (c.m_filterFlag) {
                if (
                    // Should these bodies collide?
                    !bodyB.shouldCollide(bodyA) ||
                    // Check user filtering.
                    (this.m_contactFilter && !this.m_contactFilter.shouldCollide(fixtureA, fixtureB))
                ) {
                    const cNuke = c;
                    c = cNuke.m_next;
                    this.destroy(cNuke);
                    continue;
                }

                // Clear the filtering flag.
                c.m_filterFlag = false;
            }

            const activeA = bodyA.isAwake() && bodyA.m_type !== BodyType.Static;
            const activeB = bodyB.isAwake() && bodyB.m_type !== BodyType.Static;

            // At least one body must be awake and it must be dynamic or kinematic.
            if (!activeA && !activeB) {
                c = c.m_next;
                continue;
            }

            const treeNodeA = fixtureA.m_proxies[indexA].treeNode;
            const treeNodeB = fixtureB.m_proxies[indexB].treeNode;
            const overlap = treeNodeA.aabb.testOverlap(treeNodeB.aabb);

            // Here we destroy contacts that cease to overlap in the broad-phase.
            if (!overlap) {
                const cNuke = c;
                c = cNuke.m_next;
                this.destroy(cNuke);
                continue;
            }

            // The contact persists.
            c.update(this.m_contactListener);
            c = c.m_next;
        }
    }
}
