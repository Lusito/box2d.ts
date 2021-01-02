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
    public readonly broadPhase = new BroadPhase<FixtureProxy>();

    public contactList: Contact | null = null;

    public contactCount = 0;

    public contactFilter = ContactFilter.defaultFilter;

    public contactListener = ContactListener.defaultListener;

    public readonly contactFactory = new ContactFactory();

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
        if (this.contactFilter && !this.contactFilter.shouldCollide(fixtureA, fixtureB)) {
            return;
        }

        // Call the factory.
        const c: Contact | null = this.contactFactory.create(fixtureA, indexA, fixtureB, indexB);
        if (c === null) {
            return;
        }

        // Contact creation may swap fixtures.
        fixtureA = c.getFixtureA();
        fixtureB = c.getFixtureB();
        indexA = c.getChildIndexA();
        indexB = c.getChildIndexB();
        bodyA = fixtureA.body;
        bodyB = fixtureB.body;

        // Insert into the world.
        c.prev = null;
        c.next = this.contactList;
        if (this.contactList !== null) {
            this.contactList.prev = c;
        }
        this.contactList = c;

        // Connect to island graph.

        // Connect to body A
        c.nodeA.other = bodyB;

        c.nodeA.prev = null;
        c.nodeA.next = bodyA.contactList;
        if (bodyA.contactList !== null) {
            bodyA.contactList.prev = c.nodeA;
        }
        bodyA.contactList = c.nodeA;

        // Connect to body B
        c.nodeB.other = bodyA;

        c.nodeB.prev = null;
        c.nodeB.next = bodyB.contactList;
        if (bodyB.contactList !== null) {
            bodyB.contactList.prev = c.nodeB;
        }
        bodyB.contactList = c.nodeB;

        ++this.contactCount;
    };

    public findNewContacts(): void {
        this.broadPhase.updatePairs(this.addPair);
    }

    public destroy(c: Contact): void {
        const fixtureA = c.getFixtureA();
        const fixtureB = c.getFixtureB();
        const bodyA = fixtureA.getBody();
        const bodyB = fixtureB.getBody();

        if (this.contactListener && c.isTouching()) {
            this.contactListener.endContact(c);
        }

        // Remove from the world.
        if (c.prev) {
            c.prev.next = c.next;
        }

        if (c.next) {
            c.next.prev = c.prev;
        }

        if (c === this.contactList) {
            this.contactList = c.next;
        }

        // Remove from body 1
        if (c.nodeA.prev) {
            c.nodeA.prev.next = c.nodeA.next;
        }

        if (c.nodeA.next) {
            c.nodeA.next.prev = c.nodeA.prev;
        }

        if (c.nodeA === bodyA.contactList) {
            bodyA.contactList = c.nodeA.next;
        }

        // Remove from body 2
        if (c.nodeB.prev) {
            c.nodeB.prev.next = c.nodeB.next;
        }

        if (c.nodeB.next) {
            c.nodeB.next.prev = c.nodeB.prev;
        }

        if (c.nodeB === bodyB.contactList) {
            bodyB.contactList = c.nodeB.next;
        }

        // moved this from ContactFactory:Destroy
        if (c.manifold.pointCount > 0 && !fixtureA.isSensor() && !fixtureB.isSensor()) {
            fixtureA.getBody().setAwake(true);
            fixtureB.getBody().setAwake(true);
        }

        // Call the factory.
        this.contactFactory.destroy(c);
        --this.contactCount;
    }

    /**
     * This is the top level collision call for the time step. Here
     * all the narrow phase collision is processed for the world
     * contact list.
     */
    public collide(): void {
        // Update awake contacts.
        let c: Contact | null = this.contactList;
        while (c) {
            const fixtureA = c.getFixtureA();
            const fixtureB = c.getFixtureB();
            const indexA = c.getChildIndexA();
            const indexB = c.getChildIndexB();
            const bodyA = fixtureA.getBody();
            const bodyB = fixtureB.getBody();

            // Is this contact flagged for filtering?
            if (c.filterFlag) {
                if (
                    // Should these bodies collide?
                    !bodyB.shouldCollide(bodyA) ||
                    // Check user filtering.
                    (this.contactFilter && !this.contactFilter.shouldCollide(fixtureA, fixtureB))
                ) {
                    const cNuke = c;
                    c = cNuke.next;
                    this.destroy(cNuke);
                    continue;
                }

                // Clear the filtering flag.
                c.filterFlag = false;
            }

            const activeA = bodyA.isAwake() && bodyA.type !== BodyType.Static;
            const activeB = bodyB.isAwake() && bodyB.type !== BodyType.Static;

            // At least one body must be awake and it must be dynamic or kinematic.
            if (!activeA && !activeB) {
                c = c.next;
                continue;
            }

            const treeNodeA = fixtureA.proxies[indexA].treeNode;
            const treeNodeB = fixtureB.proxies[indexB].treeNode;
            const overlap = treeNodeA.aabb.testOverlap(treeNodeB.aabb);

            // Here we destroy contacts that cease to overlap in the broad-phase.
            if (!overlap) {
                const cNuke = c;
                c = cNuke.next;
                this.destroy(cNuke);
                continue;
            }

            // The contact persists.
            c.update(this.contactListener);
            c = c.next;
        }
    }
}
