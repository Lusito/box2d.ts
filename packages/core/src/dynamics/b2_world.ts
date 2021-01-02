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
import { assert, verify, EPSILON, MAX_SUB_STEPS, MAX_TOI_CONTACTS } from "../common/b2_common";
import { Vec2, Transform, Sweep, XY } from "../common/b2_math";
import { Timer } from "../common/b2_timer";
import { AABB, RayCastInput, RayCastOutput, testOverlap } from "../collision/b2_collision";
import { timeOfImpact, TOIInput, TOIOutput, TOIOutputState } from "../collision/b2_time_of_impact";
import { Shape } from "../collision/b2_shape";
import { Contact } from "./b2_contact";
import { Joint, IJointDef, JointType } from "./b2_joint";
import { AreaJoint, IAreaJointDef } from "./b2_area_joint";
import { DistanceJoint, IDistanceJointDef } from "./b2_distance_joint";
import { FrictionJoint, IFrictionJointDef } from "./b2_friction_joint";
import { GearJoint, IGearJointDef } from "./b2_gear_joint";
import { MotorJoint, IMotorJointDef } from "./b2_motor_joint";
import { MouseJoint, IMouseJointDef } from "./b2_mouse_joint";
import { PrismaticJoint, IPrismaticJointDef } from "./b2_prismatic_joint";
import { PulleyJoint, IPulleyJointDef } from "./b2_pulley_joint";
import { RevoluteJoint, IRevoluteJointDef } from "./b2_revolute_joint";
import { WeldJoint, IWeldJointDef } from "./b2_weld_joint";
import { WheelJoint, IWheelJointDef } from "./b2_wheel_joint";
import { Body, BodyDef, BodyType } from "./b2_body";
import { ContactManager } from "./b2_contact_manager";
import { Fixture } from "./b2_fixture";
import { Island } from "./b2_island";
import { Profile, TimeStep, StepConfig } from "./b2_time_step";
import {
    ContactFilter,
    ContactListener,
    DestructionListener,
    QueryCallback,
    RayCastCallback,
} from "./b2_world_callbacks";

/**
 * The world class manages all physics entities, dynamic simulation,
 * and asynchronous queries. The world also contains efficient memory
 * management facilities.
 */
export class World {
    /** @internal */
    public readonly contactManager = new ContactManager();

    private bodyList: Body | null = null;

    private jointList: Joint | null = null;

    private bodyCount = 0;

    private jointCount = 0;

    private readonly gravity = new Vec2();

    private allowSleep = true;

    private destructionListener: DestructionListener | null = null;

    /**
     * This is used to compute the time step ratio to
     * support a variable time step.
     */
    private inv_dt0 = 0;

    /** @internal */
    public newContacts = false;

    private locked = false;

    private autoClearForces = true;

    // These are for debugging the solver.
    private warmStarting = true;

    private continuousPhysics = true;

    private subStepping = false;

    private stepComplete = true;

    private readonly profile = new Profile();

    private readonly island = new Island(
        2 * MAX_TOI_CONTACTS,
        MAX_TOI_CONTACTS,
        0,
        this.contactManager.contactListener,
    );

    private readonly s_stack: Array<Body | null> = [];

    private constructor(gravity: XY) {
        this.gravity.copy(gravity);
    }

    /**
     * Construct a world object.
     *
     * @param gravity The world gravity vector.
     */
    public static create(gravity: XY) {
        return new World(gravity);
    }

    /**
     * Register a destruction listener. The listener is owned by you and must
     * remain in scope.
     */
    public setDestructionListener(listener: DestructionListener | null): void {
        this.destructionListener = listener;
    }

    /**
     * Get the current destruction listener
     */
    public getDestructionListener() {
        return this.destructionListener;
    }

    /**
     * Register a contact filter to provide specific control over collision.
     * Otherwise the default filter is used (b2_defaultFilter). The listener is
     * owned by you and must remain in scope.
     */
    public setContactFilter(filter: ContactFilter): void {
        this.contactManager.contactFilter = filter;
    }

    /**
     * Register a contact event listener. The listener is owned by you and must
     * remain in scope.
     */
    public setContactListener(listener: ContactListener): void {
        this.contactManager.contactListener = listener;
        this.island.listener = listener;
    }

    /**
     * Create a rigid body given a definition. No reference to the definition
     * is retained.
     *
     * @warning This function is locked during callbacks.
     */
    public createBody(def: BodyDef = {}): Body {
        assert(!this.isLocked());

        const b = new Body(def, this);

        // Add to world doubly linked list.
        b.prev = null;
        b.next = this.bodyList;
        if (this.bodyList) {
            this.bodyList.prev = b;
        }
        this.bodyList = b;
        ++this.bodyCount;

        return b;
    }

    /**
     * Destroy a rigid body given a definition. No reference to the definition
     * is retained. This function is locked during callbacks.
     *
     * @warning This automatically deletes all associated shapes and joints.
     * @warning This function is locked during callbacks.
     */
    public destroyBody(b: Body): void {
        // DEBUG: assert(this.bodyCount > 0);
        assert(!this.isLocked());

        // Delete the attached joints.
        let je = b.jointList;
        while (je) {
            const je0 = je;
            je = je.next;

            this.destructionListener?.sayGoodbyeJoint(je0.joint);

            this.destroyJoint(je0.joint);

            b.jointList = je;
        }
        b.jointList = null;

        // Delete the attached contacts.
        let ce = b.contactList;
        while (ce) {
            const ce0 = ce;
            ce = ce.next;
            this.contactManager.destroy(ce0.contact);
        }
        b.contactList = null;

        // Delete the attached fixtures. This destroys broad-phase proxies.
        const { broadPhase } = this.contactManager;
        let f = b.fixtureList;
        while (f) {
            const f0 = f;
            f = f.next;

            this.destructionListener?.sayGoodbyeFixture(f0);

            f0.destroyProxies(broadPhase);

            b.fixtureList = f;
            b.fixtureCount -= 1;
        }
        b.fixtureList = null;
        b.fixtureCount = 0;

        // Remove world body list.
        if (b.prev) {
            b.prev.next = b.next;
        }

        if (b.next) {
            b.next.prev = b.prev;
        }

        if (b === this.bodyList) {
            this.bodyList = b.next;
        }

        --this.bodyCount;
    }

    private static joint_Create(def: IJointDef): Joint {
        switch (def.type) {
            case JointType.Distance:
                return new DistanceJoint(def as IDistanceJointDef);
            case JointType.Mouse:
                return new MouseJoint(def as IMouseJointDef);
            case JointType.Prismatic:
                return new PrismaticJoint(def as IPrismaticJointDef);
            case JointType.Revolute:
                return new RevoluteJoint(def as IRevoluteJointDef);
            case JointType.Pulley:
                return new PulleyJoint(def as IPulleyJointDef);
            case JointType.Gear:
                return new GearJoint(def as IGearJointDef);
            case JointType.Wheel:
                return new WheelJoint(def as IWheelJointDef);
            case JointType.Weld:
                return new WeldJoint(def as IWeldJointDef);
            case JointType.Friction:
                return new FrictionJoint(def as IFrictionJointDef);
            case JointType.Motor:
                return new MotorJoint(def as IMotorJointDef);
            case JointType.Area:
                return new AreaJoint(def as IAreaJointDef);
        }
        throw new Error();
    }

    /**
     * Create a joint to constrain bodies together. No reference to the definition
     * is retained. This may cause the connected bodies to cease colliding.
     *
     * @warning This function is locked during callbacks.
     */
    public createJoint(def: IAreaJointDef): AreaJoint;

    public createJoint(def: IDistanceJointDef): DistanceJoint;

    public createJoint(def: IFrictionJointDef): FrictionJoint;

    public createJoint(def: IGearJointDef): GearJoint;

    public createJoint(def: IMotorJointDef): MotorJoint;

    public createJoint(def: IMouseJointDef): MouseJoint;

    public createJoint(def: IPrismaticJointDef): PrismaticJoint;

    public createJoint(def: IPulleyJointDef): PulleyJoint;

    public createJoint(def: IRevoluteJointDef): RevoluteJoint;

    public createJoint(def: IWeldJointDef): WeldJoint;

    public createJoint(def: IWheelJointDef): WheelJoint;

    public createJoint(def: IJointDef): Joint {
        assert(!this.isLocked());

        const j = World.joint_Create(def);

        // Connect to the world list.
        j.prev = null;
        j.next = this.jointList;
        if (this.jointList) {
            this.jointList.prev = j;
        }
        this.jointList = j;
        ++this.jointCount;

        // Connect to the bodies' doubly linked lists.
        j.edgeA.prev = null;
        j.edgeA.next = j.bodyA.jointList;
        if (j.bodyA.jointList) j.bodyA.jointList.prev = j.edgeA;
        j.bodyA.jointList = j.edgeA;

        j.edgeB.prev = null;
        j.edgeB.next = j.bodyB.jointList;
        if (j.bodyB.jointList) j.bodyB.jointList.prev = j.edgeB;
        j.bodyB.jointList = j.edgeB;

        const { bodyA, bodyB } = j;

        // If the joint prevents collisions, then flag any contacts for filtering.
        if (!def.collideConnected) {
            let edge = bodyB.getContactList();
            while (edge) {
                if (edge.other === bodyA) {
                    // Flag the contact for filtering at the next time step (where either
                    // body is awake).
                    edge.contact.flagForFiltering();
                }

                edge = edge.next;
            }
        }

        // Note: creating a joint doesn't wake the bodies.

        return j;
    }

    /**
     * Destroy a joint. This may cause the connected bodies to begin colliding.
     *
     * @warning This function is locked during callbacks.
     */
    public destroyJoint(j: Joint): void {
        assert(!this.isLocked());

        // Remove from the doubly linked list.
        if (j.prev) {
            j.prev.next = j.next;
        }

        if (j.next) {
            j.next.prev = j.prev;
        }

        if (j === this.jointList) {
            this.jointList = j.next;
        }

        // Disconnect from island graph.
        const { bodyA, bodyB, collideConnected } = j;

        // Wake up connected bodies.
        bodyA.setAwake(true);
        bodyB.setAwake(true);

        // Remove from body 1.
        if (j.edgeA.prev) {
            j.edgeA.prev.next = j.edgeA.next;
        }

        if (j.edgeA.next) {
            j.edgeA.next.prev = j.edgeA.prev;
        }

        if (j.edgeA === bodyA.jointList) {
            bodyA.jointList = j.edgeA.next;
        }

        j.edgeA.prev = null;
        j.edgeA.next = null;

        // Remove from body 2
        if (j.edgeB.prev) {
            j.edgeB.prev.next = j.edgeB.next;
        }

        if (j.edgeB.next) {
            j.edgeB.next.prev = j.edgeB.prev;
        }

        if (j.edgeB === bodyB.jointList) {
            bodyB.jointList = j.edgeB.next;
        }

        j.edgeB.prev = null;
        j.edgeB.next = null;

        // DEBUG: assert(this.jointCount > 0);
        --this.jointCount;

        // If the joint prevents collisions, then flag any contacts for filtering.
        if (!collideConnected) {
            let edge = bodyB.getContactList();
            while (edge) {
                if (edge.other === bodyA) {
                    // Flag the contact for filtering at the next time step (where either
                    // body is awake).
                    edge.contact.flagForFiltering();
                }

                edge = edge.next;
            }
        }
    }

    private static Step_s_step = TimeStep.create();

    private static Step_s_stepTimer = new Timer();

    private static Step_s_timer = new Timer();

    /**
     * Take a time step. This performs collision detection, integration,
     * and constraint solution.
     *
     * @param timeStep The amount of time to simulate, this should not vary.
     * @param velocityIterations For the velocity constraint solver.
     * @param positionIterations For the position constraint solver.
     */
    public step(dt: number, iterations: StepConfig): void {
        const stepTimer = World.Step_s_stepTimer.reset();

        // If new fixtures were added, we need to find the new contacts.
        if (this.newContacts) {
            this.contactManager.findNewContacts();
            this.newContacts = false;
        }

        this.locked = true;

        const step = World.Step_s_step;
        step.dt = dt;
        step.config = {
            ...iterations,
        };
        if (dt > 0) {
            step.inv_dt = 1 / dt;
        } else {
            step.inv_dt = 0;
        }

        step.dtRatio = this.inv_dt0 * dt;

        step.warmStarting = this.warmStarting;

        // Update contacts. This is where some contacts are destroyed.
        {
            const timer = World.Step_s_timer.reset();
            this.contactManager.collide();
            this.profile.collide = timer.getMilliseconds();
        }

        // Integrate velocities, solve velocity constraints, and integrate positions.
        if (this.stepComplete && step.dt > 0) {
            const timer = World.Step_s_timer.reset();
            this.solve(step);
            this.profile.solve = timer.getMilliseconds();
        }

        // Handle TOI events.
        if (this.continuousPhysics && step.dt > 0) {
            const timer = World.Step_s_timer.reset();
            this.solveTOI(step);
            this.profile.solveTOI = timer.getMilliseconds();
        }

        if (step.dt > 0) {
            this.inv_dt0 = step.inv_dt;
        }

        if (this.autoClearForces) {
            this.clearForces();
        }

        this.locked = false;

        this.profile.step = stepTimer.getMilliseconds();
    }

    /**
     * Manually clear the force buffer on all bodies. By default, forces are cleared automatically
     * after each call to Step. The default behavior is modified by calling SetAutoClearForces.
     * The purpose of this function is to support sub-stepping. Sub-stepping is often used to maintain
     * a fixed sized time step under a variable frame-rate.
     * When you perform sub-stepping you will disable auto clearing of forces and instead call
     * ClearForces after all sub-steps are complete in one pass of your game loop.
     *
     * @see SetAutoClearForces
     */
    public clearForces(): void {
        for (let body = this.bodyList; body; body = body.getNext()) {
            body.force.setZero();
            body.torque = 0;
        }
    }

    /**
     * Query the world for all fixtures that potentially overlap the
     * provided AABB.
     *
     * @param aabb The query box.
     * @param callback A user implemented callback class or function.
     */
    public queryAABB(aabb: AABB, callback: QueryCallback): void {
        this.contactManager.broadPhase.query(aabb, (proxy) => {
            const fixture_proxy = verify(proxy.userData);
            // DEBUG: assert(fixture_proxy instanceof FixtureProxy);
            return callback(fixture_proxy.fixture);
        });
    }

    public queryAllAABB(aabb: AABB, out: Fixture[] = []): Fixture[] {
        this.queryAABB(aabb, (fixture) => {
            out.push(fixture);
            return true;
        });
        return out;
    }

    /**
     * Query the world for all fixtures that potentially overlap the
     * provided point.
     *
     * @param point The query point.
     * @param callback A user implemented callback class or function.
     */
    public queryPointAABB(point: XY, callback: QueryCallback): void {
        this.contactManager.broadPhase.queryPoint(point, (proxy) => {
            const fixture_proxy = verify(proxy.userData);
            // DEBUG: assert(fixture_proxy instanceof FixtureProxy);
            return callback(fixture_proxy.fixture);
        });
    }

    public queryAllPointAABB(point: XY, out: Fixture[] = []): Fixture[] {
        this.queryPointAABB(point, (fixture) => {
            out.push(fixture);
            return true;
        });
        return out;
    }

    private static QueryFixtureShape_s_aabb = new AABB();

    public queryFixtureShape(shape: Shape, index: number, transform: Transform, callback: QueryCallback): void {
        const aabb = World.QueryFixtureShape_s_aabb;
        shape.computeAABB(aabb, transform, index);
        this.contactManager.broadPhase.query(aabb, (proxy) => {
            const fixture_proxy = verify(proxy.userData);
            // DEBUG: assert(fixture_proxy instanceof FixtureProxy);
            const { fixture } = fixture_proxy;
            const overlap = testOverlap(
                shape,
                index,
                fixture.getShape(),
                fixture_proxy.childIndex,
                transform,
                fixture.getBody().getTransform(),
            );
            return !overlap || callback(fixture);
        });
    }

    public queryAllFixtureShape(shape: Shape, index: number, transform: Transform, out: Fixture[] = []): Fixture[] {
        this.queryFixtureShape(shape, index, transform, (fixture) => {
            out.push(fixture);
            return true;
        });
        return out;
    }

    public queryFixturePoint(point: XY, callback: QueryCallback): void {
        this.contactManager.broadPhase.queryPoint(point, (proxy) => {
            const fixture_proxy = verify(proxy.userData);
            // DEBUG: assert(fixture_proxy instanceof FixtureProxy);
            const { fixture } = fixture_proxy;
            const overlap = fixture.testPoint(point);
            return !overlap || callback(fixture);
        });
    }

    public queryAllFixturePoint(point: XY, out: Fixture[] = []): Fixture[] {
        this.queryFixturePoint(point, (fixture) => {
            out.push(fixture);
            return true;
        });
        return out;
    }

    private static RayCast_s_input = new RayCastInput();

    private static RayCast_s_output = new RayCastOutput();

    private static RayCast_s_point = new Vec2();

    /**
     * Ray-cast the world for all fixtures in the path of the ray. Your callback
     * controls whether you get the closest point, any point, or n-points.
     * The ray-cast ignores shapes that contain the starting point.
     *
     * @param point1 The ray starting point
     * @param point2 The ray ending point
     * @param callback A user implemented callback class or function.
     */
    public rayCast(point1: XY, point2: XY, callback: RayCastCallback): void {
        const input = World.RayCast_s_input;
        input.maxFraction = 1;
        input.p1.copy(point1);
        input.p2.copy(point2);
        this.contactManager.broadPhase.rayCast(input, (input2, proxy) => {
            const fixture_proxy = verify(proxy.userData);
            // DEBUG: assert(fixture_proxy instanceof FixtureProxy);
            const { fixture } = fixture_proxy;
            const index = fixture_proxy.childIndex;
            const output = World.RayCast_s_output;
            const hit = fixture.rayCast(output, input2, index);
            if (hit) {
                const { fraction } = output;
                const point = World.RayCast_s_point;
                point.set(
                    (1 - fraction) * point1.x + fraction * point2.x,
                    (1 - fraction) * point1.y + fraction * point2.y,
                );
                return callback(fixture, point, output.normal, fraction);
            }
            return input2.maxFraction;
        });
    }

    public rayCastOne(point1: XY, point2: XY): Fixture | null {
        let result: Fixture | null = null;
        let min_fraction = 1;
        this.rayCast(point1, point2, (fixture, _point, _normal, fraction) => {
            if (fraction < min_fraction) {
                min_fraction = fraction;
                result = fixture;
            }
            return min_fraction;
        });
        return result;
    }

    public rayCastAll(point1: XY, point2: XY, out: Fixture[] = []): Fixture[] {
        this.rayCast(point1, point2, (fixture) => {
            out.push(fixture);
            return 1;
        });
        return out;
    }

    /**
     * Get the world body list. With the returned body, use Body::GetNext to get
     * the next body in the world list. A NULL body indicates the end of the list.
     *
     * @returns The head of the world body list.
     */
    public getBodyList(): Body | null {
        return this.bodyList;
    }

    /**
     * Get the world joint list. With the returned joint, use Joint::GetNext to get
     * the next joint in the world list. A NULL joint indicates the end of the list.
     *
     * @returns The head of the world joint list.
     */
    public getJointList(): Joint | null {
        return this.jointList;
    }

    /**
     * Get the world contact list. With the returned contact, use Contact::GetNext to get
     * the next contact in the world list. A NULL contact indicates the end of the list.
     *
     * @returns The head of the world contact list.
     * @warning contacts are created and destroyed in the middle of a time step.
     * Use ContactListener to avoid missing contacts.
     */
    public getContactList(): Contact | null {
        return this.contactManager.contactList;
    }

    /**
     * Enable/disable sleep.
     */
    public setAllowSleeping(flag: boolean): void {
        if (flag === this.allowSleep) {
            return;
        }

        this.allowSleep = flag;
        if (!this.allowSleep) {
            for (let b = this.bodyList; b; b = b.next) {
                b.setAwake(true);
            }
        }
    }

    public getAllowSleeping(): boolean {
        return this.allowSleep;
    }

    /**
     * Enable/disable warm starting. For testing.
     */
    public setWarmStarting(flag: boolean): void {
        this.warmStarting = flag;
    }

    public getWarmStarting(): boolean {
        return this.warmStarting;
    }

    /**
     * Enable/disable continuous physics. For testing.
     */
    public setContinuousPhysics(flag: boolean): void {
        this.continuousPhysics = flag;
    }

    public getContinuousPhysics(): boolean {
        return this.continuousPhysics;
    }

    /**
     * Enable/disable single stepped continuous physics. For testing.
     */
    public setSubStepping(flag: boolean): void {
        this.subStepping = flag;
    }

    public getSubStepping(): boolean {
        return this.subStepping;
    }

    /**
     * Get the number of broad-phase proxies.
     */
    public getProxyCount(): number {
        return this.contactManager.broadPhase.getProxyCount();
    }

    /**
     * Get the number of bodies.
     */
    public getBodyCount(): number {
        return this.bodyCount;
    }

    /**
     * Get the number of joints.
     */
    public getJointCount(): number {
        return this.jointCount;
    }

    /**
     * Get the number of contacts (each may have 0 or more contact points).
     */
    public getContactCount(): number {
        return this.contactManager.contactCount;
    }

    /**
     * Get the height of the dynamic tree.
     */
    public getTreeHeight(): number {
        return this.contactManager.broadPhase.getTreeHeight();
    }

    /**
     * Get the balance of the dynamic tree.
     */
    public getTreeBalance(): number {
        return this.contactManager.broadPhase.getTreeBalance();
    }

    /**
     * Get the quality metric of the dynamic tree. The smaller the better.
     * The minimum is 1.
     */
    public getTreeQuality(): number {
        return this.contactManager.broadPhase.getTreeQuality();
    }

    /**
     * Change the global gravity vector.
     */
    public setGravity(gravity: XY) {
        this.gravity.copy(gravity);
    }

    /**
     * Get the global gravity vector.
     */
    public getGravity(): Readonly<Vec2> {
        return this.gravity;
    }

    /**
     * Is the world locked (in the middle of a time step).
     */
    public isLocked(): boolean {
        return this.locked;
    }

    /**
     * Set flag to control automatic clearing of forces after each time step.
     */
    public setAutoClearForces(flag: boolean): void {
        this.autoClearForces = flag;
    }

    /**
     * Get the flag that controls automatic clearing of forces after each time step.
     */
    public getAutoClearForces(): boolean {
        return this.autoClearForces;
    }

    /**
     * Shift the world origin. Useful for large worlds.
     * The body shift formula is: position -= newOrigin
     *
     * @param newOrigin The new origin with respect to the old origin
     */
    public shiftOrigin(newOrigin: XY): void {
        assert(!this.isLocked());

        for (let b = this.bodyList; b; b = b.next) {
            b.xf.p.subtract(newOrigin);
            b.sweep.c0.subtract(newOrigin);
            b.sweep.c.subtract(newOrigin);
        }

        for (let j = this.jointList; j; j = j.next) {
            j.shiftOrigin(newOrigin);
        }

        this.contactManager.broadPhase.shiftOrigin(newOrigin);
    }

    /**
     * Get the contact manager for testing.
     */
    public getContactManager(): ContactManager {
        return this.contactManager;
    }

    /**
     * Get the current profile.
     */
    public getProfile(): Profile {
        return this.profile;
    }

    private solve(step: TimeStep): void {
        this.profile.solveInit = 0;
        this.profile.solveVelocity = 0;
        this.profile.solvePosition = 0;

        // Size the island for the worst case.
        const { island } = this;
        island.resize(this.bodyCount);
        island.clear();

        // Clear all the island flags.
        for (let b = this.bodyList; b; b = b.next) {
            b.islandFlag = false;
        }
        for (let c = this.contactManager.contactList; c; c = c.next) {
            c.islandFlag = false;
        }
        for (let j = this.jointList; j; j = j.next) {
            j.islandFlag = false;
        }

        // Build and simulate all awake islands.
        // DEBUG: const stackSize = this.bodyCount;
        const stack = this.s_stack;
        for (let seed = this.bodyList; seed; seed = seed.next) {
            if (seed.islandFlag) {
                continue;
            }

            if (!seed.isAwake() || !seed.isEnabled()) {
                continue;
            }

            // The seed can be dynamic or kinematic.
            if (seed.getType() === BodyType.Static) {
                continue;
            }

            // Reset island and stack.
            island.clear();
            let stackCount = 0;
            stack[stackCount++] = seed;
            seed.islandFlag = true;

            // Perform a depth first search (DFS) on the constraint graph.
            while (stackCount > 0) {
                // Grab the next body off the stack and add it to the island.
                const b = stack[--stackCount];
                assert(b !== null);
                // DEBUG: assert(b.IsEnabled());
                island.addBody(b);

                // To keep islands as small as possible, we don't
                // propagate islands across static bodies.
                if (b.getType() === BodyType.Static) {
                    continue;
                }

                // Make sure the body is awake (without resetting sleep timer).
                b.awakeFlag = true;

                // Search all contacts connected to this body.
                for (let ce = b.contactList; ce; ce = ce.next) {
                    const { contact } = ce;

                    // Has this contact already been added to an island?
                    if (contact.islandFlag) {
                        continue;
                    }

                    // Is this contact solid and touching?
                    if (!contact.isEnabled() || !contact.isTouching()) {
                        continue;
                    }

                    // Skip sensors.
                    const sensorA = contact.fixtureA.m_isSensor;
                    const sensorB = contact.fixtureB.m_isSensor;
                    if (sensorA || sensorB) {
                        continue;
                    }

                    island.addContact(contact);
                    contact.islandFlag = true;

                    const { other } = ce;

                    // Was the other body already added to this island?
                    if (other.islandFlag) {
                        continue;
                    }

                    // DEBUG: assert(stackCount < stackSize);
                    stack[stackCount++] = other;
                    other.islandFlag = true;
                }

                // Search all joints connect to this body.
                for (let je = b.jointList; je; je = je.next) {
                    if (je.joint.islandFlag) {
                        continue;
                    }

                    const { other } = je;

                    // Don't simulate joints connected to disabled bodies.
                    if (!other.isEnabled()) {
                        continue;
                    }

                    island.addJoint(je.joint);
                    je.joint.islandFlag = true;

                    if (other.islandFlag) {
                        continue;
                    }

                    // DEBUG: assert(stackCount < stackSize);
                    stack[stackCount++] = other;
                    other.islandFlag = true;
                }
            }

            const profile = new Profile();
            island.solve(profile, step, this.gravity, this.allowSleep);
            this.profile.solveInit += profile.solveInit;
            this.profile.solveVelocity += profile.solveVelocity;
            this.profile.solvePosition += profile.solvePosition;

            // Post solve cleanup.
            for (let i = 0; i < island.bodyCount; ++i) {
                // Allow static bodies to participate in other islands.
                const b = island.bodies[i];
                if (b.getType() === BodyType.Static) {
                    b.islandFlag = false;
                }
            }
        }

        for (let i = 0; i < stack.length; ++i) {
            if (!stack[i]) {
                break;
            }
            stack[i] = null;
        }

        const timer = new Timer();

        // Synchronize fixtures, check for out of range bodies.
        for (let b = this.bodyList; b; b = b.next) {
            // If a body was not in an island then it did not move.
            if (!b.islandFlag) {
                continue;
            }

            if (b.getType() === BodyType.Static) {
                continue;
            }

            // Update fixtures (for broad-phase).
            b.synchronizeFixtures();
        }

        // Look for new contacts.
        this.contactManager.findNewContacts();
        this.profile.broadphase = timer.getMilliseconds();
    }

    private static SolveTOI_s_subStep = TimeStep.create();

    private static SolveTOI_s_backup = new Sweep();

    private static SolveTOI_s_backup1 = new Sweep();

    private static SolveTOI_s_backup2 = new Sweep();

    private static SolveTOI_s_toi_input = new TOIInput();

    private static SolveTOI_s_toi_output = new TOIOutput();

    /** @internal */
    public solveTOI(step: TimeStep): void {
        const { island } = this;
        island.clear();

        if (this.stepComplete) {
            for (let b = this.bodyList; b; b = b.next) {
                b.islandFlag = false;
                b.sweep.alpha0 = 0;
            }

            for (let c = this.contactManager.contactList; c; c = c.next) {
                // Invalidate TOI
                c.toiFlag = false;
                c.islandFlag = false;
                c.toiCount = 0;
                c.toi = 1;
            }
        }

        // Find TOI events and solve them.
        for (;;) {
            // Find the first TOI.
            let minContact = null;
            let minAlpha = 1;

            for (let c = this.contactManager.contactList; c; c = c.next) {
                // Is this contact disabled?
                if (!c.isEnabled()) {
                    continue;
                }

                // Prevent excessive sub-stepping.
                if (c.toiCount > MAX_SUB_STEPS) {
                    continue;
                }

                let alpha = 1;
                if (c.toiFlag) {
                    // This contact has a valid cached TOI.
                    alpha = c.toi;
                } else {
                    const fA = c.getFixtureA();
                    const fB = c.getFixtureB();

                    // Is there a sensor?
                    if (fA.isSensor() || fB.isSensor()) {
                        continue;
                    }

                    const bA = fA.getBody();
                    const bB = fB.getBody();

                    const typeA = bA.type;
                    const typeB = bB.type;
                    // DEBUG: assert(typeA === BodyType.Dynamic || typeB === BodyType.Dynamic);

                    const activeA = bA.isAwake() && typeA !== BodyType.Static;
                    const activeB = bB.isAwake() && typeB !== BodyType.Static;

                    // Is at least one body active (awake and dynamic or kinematic)?
                    if (!activeA && !activeB) {
                        continue;
                    }

                    const collideA = bA.isBullet() || typeA !== BodyType.Dynamic;
                    const collideB = bB.isBullet() || typeB !== BodyType.Dynamic;

                    // Are these two non-bullet dynamic bodies?
                    if (!collideA && !collideB) {
                        continue;
                    }

                    // Compute the TOI for this contact.
                    // Put the sweeps onto the same time interval.
                    let { alpha0 } = bA.sweep;

                    if (bA.sweep.alpha0 < bB.sweep.alpha0) {
                        alpha0 = bB.sweep.alpha0;
                        bA.sweep.advance(alpha0);
                    } else if (bB.sweep.alpha0 < bA.sweep.alpha0) {
                        alpha0 = bA.sweep.alpha0;
                        bB.sweep.advance(alpha0);
                    }

                    // DEBUG: assert(alpha0 < 1);

                    const indexA = c.getChildIndexA();
                    const indexB = c.getChildIndexB();

                    // Compute the time of impact in interval [0, minTOI]
                    const input = World.SolveTOI_s_toi_input;
                    input.proxyA.setShape(fA.getShape(), indexA);
                    input.proxyB.setShape(fB.getShape(), indexB);
                    input.sweepA.copy(bA.sweep);
                    input.sweepB.copy(bB.sweep);
                    input.tMax = 1;

                    const output = World.SolveTOI_s_toi_output;
                    timeOfImpact(output, input);

                    // Beta is the fraction of the remaining portion of the .
                    const beta = output.t;
                    if (output.state === TOIOutputState.Touching) {
                        alpha = Math.min(alpha0 + (1 - alpha0) * beta, 1);
                    } else {
                        alpha = 1;
                    }

                    c.toi = alpha;
                    c.toiFlag = true;
                }

                if (alpha < minAlpha) {
                    // This is the minimum TOI found so far.
                    minContact = c;
                    minAlpha = alpha;
                }
            }

            if (minContact === null || 1 - 10 * EPSILON < minAlpha) {
                // No more TOI events. Done!
                this.stepComplete = true;
                break;
            }

            // Advance the bodies to the TOI.
            const fA = minContact.getFixtureA();
            const fB = minContact.getFixtureB();
            const bA = fA.getBody();
            const bB = fB.getBody();

            const backup1 = World.SolveTOI_s_backup1.copy(bA.sweep);
            const backup2 = World.SolveTOI_s_backup2.copy(bB.sweep);

            bA.advance(minAlpha);
            bB.advance(minAlpha);

            // The TOI contact likely has some new contact points.
            minContact.update(this.contactManager.contactListener);
            minContact.toiFlag = false;
            ++minContact.toiCount;

            // Is the contact solid?
            if (!minContact.isEnabled() || !minContact.isTouching()) {
                // Restore the sweeps.
                minContact.setEnabled(false);
                bA.sweep.copy(backup1);
                bB.sweep.copy(backup2);
                bA.synchronizeTransform();
                bB.synchronizeTransform();
                continue;
            }

            bA.setAwake(true);
            bB.setAwake(true);

            // Build the island
            island.clear();
            island.addBody(bA);
            island.addBody(bB);
            island.addContact(minContact);

            bA.islandFlag = true;
            bB.islandFlag = true;
            minContact.islandFlag = true;

            // Get contacts on bodyA and bodyB.
            for (let i = 0; i < 2; ++i) {
                const body = i === 0 ? bA : bB;
                if (body.type === BodyType.Dynamic) {
                    for (let ce = body.contactList; ce; ce = ce.next) {
                        if (island.bodyCount === island.bodyCapacity) {
                            break;
                        }

                        if (island.contactCount === MAX_TOI_CONTACTS) {
                            break;
                        }

                        const { contact } = ce;

                        // Has this contact already been added to the island?
                        if (contact.islandFlag) {
                            continue;
                        }

                        // Only add static, kinematic, or bullet bodies.
                        const { other } = ce;
                        if (other.type === BodyType.Dynamic && !body.isBullet() && !other.isBullet()) {
                            continue;
                        }

                        // Skip sensors.
                        const sensorA = contact.fixtureA.m_isSensor;
                        const sensorB = contact.fixtureB.m_isSensor;
                        if (sensorA || sensorB) {
                            continue;
                        }

                        // Tentatively advance the body to the TOI.
                        const backup = World.SolveTOI_s_backup.copy(other.sweep);
                        if (!other.islandFlag) {
                            other.advance(minAlpha);
                        }

                        // Update the contact points
                        contact.update(this.contactManager.contactListener);

                        // Was the contact disabled by the user?
                        if (!contact.isEnabled()) {
                            other.sweep.copy(backup);
                            other.synchronizeTransform();
                            continue;
                        }

                        // Are there contact points?
                        if (!contact.isTouching()) {
                            other.sweep.copy(backup);
                            other.synchronizeTransform();
                            continue;
                        }

                        // Add the contact to the island
                        contact.islandFlag = true;
                        island.addContact(contact);

                        // Has the other body already been added to the island?
                        if (other.islandFlag) {
                            continue;
                        }

                        // Add the other body to the island.
                        other.islandFlag = true;

                        if (other.type !== BodyType.Static) {
                            other.setAwake(true);
                        }

                        island.addBody(other);
                    }
                }
            }

            const subStep = World.SolveTOI_s_subStep;
            subStep.dt = (1 - minAlpha) * step.dt;
            subStep.inv_dt = 1 / subStep.dt;
            subStep.dtRatio = 1;
            subStep.config = {
                ...step.config,
                positionIterations: 20,
            };
            subStep.warmStarting = false;
            island.solveTOI(subStep, bA.islandIndex, bB.islandIndex);

            // Reset island flags and synchronize broad-phase proxies.
            for (let i = 0; i < island.bodyCount; ++i) {
                const body = island.bodies[i];
                body.islandFlag = false;

                if (body.type !== BodyType.Dynamic) {
                    continue;
                }

                body.synchronizeFixtures();

                // Invalidate all contact TOIs on this displaced body.
                for (let ce = body.contactList; ce; ce = ce.next) {
                    ce.contact.toiFlag = false;
                    ce.contact.islandFlag = false;
                }
            }

            // Commit fixture proxy movements to the broad-phase so that new contacts are created.
            // Also, some contacts can be destroyed.
            this.contactManager.findNewContacts();

            if (this.subStepping) {
                this.stepComplete = false;
                break;
            }
        }
    }
}
