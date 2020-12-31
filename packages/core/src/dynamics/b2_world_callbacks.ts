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

import { MAX_MANIFOLD_POINTS, MakeNumberArray } from "../common/b2_common";
import { Vec2 } from "../common/b2_math";
import { Manifold } from "../collision/b2_collision";
import { Contact } from "./b2_contact";
import { Joint } from "./b2_joint";
import { Fixture } from "./b2_fixture";

/**
 * Joints and fixtures are destroyed when their associated
 * body is destroyed. Implement this listener so that you
 * may nullify references to these joints and shapes.
 */
export class DestructionListener {
    /**
     * Called when any joint is about to be destroyed due
     * to the destruction of one of its attached bodies.
     */
    public SayGoodbyeJoint(_joint: Joint): void {}

    /**
     * Called when any fixture is about to be destroyed due
     * to the destruction of its parent body.
     */
    public SayGoodbyeFixture(_fixture: Fixture): void {}
}

/**
 * Implement this class to provide collision filtering. In other words, you can implement
 * this class if you want finer control over contact creation.
 */
export class ContactFilter {
    /**
     * Return true if contact calculations should be performed between these two shapes.
     *
     * @warning for performance reasons this is only called when the AABBs begin to overlap.
     */
    public ShouldCollide(fixtureA: Fixture, fixtureB: Fixture): boolean {
        const filterA = fixtureA.GetFilterData();
        const filterB = fixtureB.GetFilterData();

        if (filterA.groupIndex === filterB.groupIndex && filterA.groupIndex !== 0) {
            return filterA.groupIndex > 0;
        }

        return (filterA.maskBits & filterB.categoryBits) !== 0 && (filterA.categoryBits & filterB.maskBits) !== 0;
    }

    public static readonly defaultFilter = new ContactFilter();
}

/**
 * Contact impulses for reporting. Impulses are used instead of forces because
 * sub-step forces may approach infinity for rigid body collisions. These
 * match up one-to-one with the contact points in Manifold.
 */
export class ContactImpulse {
    public normalImpulses = MakeNumberArray(MAX_MANIFOLD_POINTS);

    public tangentImpulses = MakeNumberArray(MAX_MANIFOLD_POINTS);

    public count = 0;
}

/**
 * Implement this class to get contact information. You can use these results for
 * things like sounds and game logic. You can also get contact results by
 * traversing the contact lists after the time step. However, you might miss
 * some contacts because continuous physics leads to sub-stepping.
 * Additionally you may receive multiple callbacks for the same contact in a
 * single time step.
 * You should strive to make your callbacks efficient because there may be
 * many callbacks per time step.
 *
 * @warning You cannot create/destroy Box2D entities inside these callbacks.
 */
export class ContactListener {
    /**
     * Called when two fixtures begin to touch.
     */
    public BeginContact(_contact: Contact): void {}

    /**
     * Called when two fixtures cease to touch.
     */
    public EndContact(_contact: Contact): void {}

    /**
     * This is called after a contact is updated. This allows you to inspect a
     * contact before it goes to the solver. If you are careful, you can modify the
     * contact manifold (e.g. disable contact).
     * A copy of the old manifold is provided so that you can detect changes.
     * Note: this is called only for awake bodies.
     * Note: this is called even when the number of contact points is zero.
     * Note: this is not called for sensors.
     * Note: if you set the number of contact points to zero, you will not
     * get an EndContact callback. However, you may get a BeginContact callback
     * the next step.
     */
    public PreSolve(_contact: Contact, _oldManifold: Manifold): void {}

    /**
     * This lets you inspect a contact after the solver is finished. This is useful
     * for inspecting impulses.
     * Note: the contact manifold does not include time of impact impulses, which can be
     * arbitrarily large if the sub-step is small. Hence the impulse is provided explicitly
     * in a separate data structure.
     * Note: this is only called for contacts that are touching, solid, and awake.
     */
    public PostSolve(_contact: Contact, _impulse: ContactImpulse): void {}

    public static readonly defaultListener = new ContactListener();
}

/**
 * Callback class for AABB queries
 * See World::Query
 */
export type QueryCallback = (fixture: Fixture) => boolean;

/**
 * Callback class for ray casts.
 * See World::RayCast
 * Called for each fixture found in the query. You control how the ray cast
 * proceeds by returning a float:
 * return -1: ignore this fixture and continue
 * return 0: terminate the ray cast
 * return fraction: clip the ray to this point
 * return 1: don't clip the ray and continue
 *
 * @param fixture The fixture hit by the ray
 * @param point The point of initial intersection
 * @param normal The normal vector at the point of intersection
 * @returns -1 to filter, 0 to terminate, fraction to clip the ray for
 * closest hit, 1 to continue
 */
export type RayCastCallback = (fixture: Fixture, point: Vec2, normal: Vec2, fraction: number) => number;
