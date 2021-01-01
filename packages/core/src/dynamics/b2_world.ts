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
    public readonly m_contactManager = new ContactManager();

    private m_bodyList: Body | null = null;

    private m_jointList: Joint | null = null;

    private m_bodyCount = 0;

    private m_jointCount = 0;

    private readonly m_gravity = new Vec2();

    private m_allowSleep = true;

    private m_destructionListener: DestructionListener | null = null;

    /**
     * This is used to compute the time step ratio to
     * support a variable time step.
     */
    private m_inv_dt0 = 0;

    /** @internal */
    public m_newContacts = false;

    private m_locked = false;

    private m_clearForces = true;

    // These are for debugging the solver.
    private m_warmStarting = true;

    private m_continuousPhysics = true;

    private m_subStepping = false;

    private m_stepComplete = true;

    private readonly m_profile = new Profile();

    private readonly m_island = new Island(
        2 * MAX_TOI_CONTACTS,
        MAX_TOI_CONTACTS,
        0,
        this.m_contactManager.m_contactListener,
    );

    private readonly s_stack: Array<Body | null> = [];

    private constructor(gravity: XY) {
        this.m_gravity.copy(gravity);
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
        this.m_destructionListener = listener;
    }

    /**
     * Get the current destruction listener
     */
    public getDestructionListener() {
        return this.m_destructionListener;
    }

    /**
     * Register a contact filter to provide specific control over collision.
     * Otherwise the default filter is used (b2_defaultFilter). The listener is
     * owned by you and must remain in scope.
     */
    public setContactFilter(filter: ContactFilter): void {
        this.m_contactManager.m_contactFilter = filter;
    }

    /**
     * Register a contact event listener. The listener is owned by you and must
     * remain in scope.
     */
    public setContactListener(listener: ContactListener): void {
        this.m_contactManager.m_contactListener = listener;
        this.m_island.m_listener = listener;
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
        b.m_prev = null;
        b.m_next = this.m_bodyList;
        if (this.m_bodyList) {
            this.m_bodyList.m_prev = b;
        }
        this.m_bodyList = b;
        ++this.m_bodyCount;

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
        // DEBUG: assert(this.m_bodyCount > 0);
        assert(!this.isLocked());

        // Delete the attached joints.
        let je = b.m_jointList;
        while (je) {
            const je0 = je;
            je = je.next;

            this.m_destructionListener?.sayGoodbyeJoint(je0.joint);

            this.destroyJoint(je0.joint);

            b.m_jointList = je;
        }
        b.m_jointList = null;

        // Delete the attached contacts.
        let ce = b.m_contactList;
        while (ce) {
            const ce0 = ce;
            ce = ce.next;
            this.m_contactManager.destroy(ce0.contact);
        }
        b.m_contactList = null;

        // Delete the attached fixtures. This destroys broad-phase proxies.
        const broadPhase = this.m_contactManager.m_broadPhase;
        let f = b.m_fixtureList;
        while (f) {
            const f0 = f;
            f = f.m_next;

            this.m_destructionListener?.sayGoodbyeFixture(f0);

            f0.destroyProxies(broadPhase);

            b.m_fixtureList = f;
            b.m_fixtureCount -= 1;
        }
        b.m_fixtureList = null;
        b.m_fixtureCount = 0;

        // Remove world body list.
        if (b.m_prev) {
            b.m_prev.m_next = b.m_next;
        }

        if (b.m_next) {
            b.m_next.m_prev = b.m_prev;
        }

        if (b === this.m_bodyList) {
            this.m_bodyList = b.m_next;
        }

        --this.m_bodyCount;
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
        j.m_prev = null;
        j.m_next = this.m_jointList;
        if (this.m_jointList) {
            this.m_jointList.m_prev = j;
        }
        this.m_jointList = j;
        ++this.m_jointCount;

        // Connect to the bodies' doubly linked lists.
        j.m_edgeA.prev = null;
        j.m_edgeA.next = j.m_bodyA.m_jointList;
        if (j.m_bodyA.m_jointList) j.m_bodyA.m_jointList.prev = j.m_edgeA;
        j.m_bodyA.m_jointList = j.m_edgeA;

        j.m_edgeB.prev = null;
        j.m_edgeB.next = j.m_bodyB.m_jointList;
        if (j.m_bodyB.m_jointList) j.m_bodyB.m_jointList.prev = j.m_edgeB;
        j.m_bodyB.m_jointList = j.m_edgeB;

        const bodyA = j.m_bodyA;
        const bodyB = j.m_bodyB;

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
        if (j.m_prev) {
            j.m_prev.m_next = j.m_next;
        }

        if (j.m_next) {
            j.m_next.m_prev = j.m_prev;
        }

        if (j === this.m_jointList) {
            this.m_jointList = j.m_next;
        }

        // Disconnect from island graph.
        const bodyA = j.m_bodyA;
        const bodyB = j.m_bodyB;
        const collideConnected = j.m_collideConnected;

        // Wake up connected bodies.
        bodyA.setAwake(true);
        bodyB.setAwake(true);

        // Remove from body 1.
        if (j.m_edgeA.prev) {
            j.m_edgeA.prev.next = j.m_edgeA.next;
        }

        if (j.m_edgeA.next) {
            j.m_edgeA.next.prev = j.m_edgeA.prev;
        }

        if (j.m_edgeA === bodyA.m_jointList) {
            bodyA.m_jointList = j.m_edgeA.next;
        }

        j.m_edgeA.prev = null;
        j.m_edgeA.next = null;

        // Remove from body 2
        if (j.m_edgeB.prev) {
            j.m_edgeB.prev.next = j.m_edgeB.next;
        }

        if (j.m_edgeB.next) {
            j.m_edgeB.next.prev = j.m_edgeB.prev;
        }

        if (j.m_edgeB === bodyB.m_jointList) {
            bodyB.m_jointList = j.m_edgeB.next;
        }

        j.m_edgeB.prev = null;
        j.m_edgeB.next = null;

        // DEBUG: assert(this.m_jointCount > 0);
        --this.m_jointCount;

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
        if (this.m_newContacts) {
            this.m_contactManager.findNewContacts();
            this.m_newContacts = false;
        }

        this.m_locked = true;

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

        step.dtRatio = this.m_inv_dt0 * dt;

        step.warmStarting = this.m_warmStarting;

        // Update contacts. This is where some contacts are destroyed.
        {
            const timer = World.Step_s_timer.reset();
            this.m_contactManager.collide();
            this.m_profile.collide = timer.getMilliseconds();
        }

        // Integrate velocities, solve velocity constraints, and integrate positions.
        if (this.m_stepComplete && step.dt > 0) {
            const timer = World.Step_s_timer.reset();
            this.solve(step);
            this.m_profile.solve = timer.getMilliseconds();
        }

        // Handle TOI events.
        if (this.m_continuousPhysics && step.dt > 0) {
            const timer = World.Step_s_timer.reset();
            this.solveTOI(step);
            this.m_profile.solveTOI = timer.getMilliseconds();
        }

        if (step.dt > 0) {
            this.m_inv_dt0 = step.inv_dt;
        }

        if (this.m_clearForces) {
            this.clearForces();
        }

        this.m_locked = false;

        this.m_profile.step = stepTimer.getMilliseconds();
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
        for (let body = this.m_bodyList; body; body = body.getNext()) {
            body.m_force.setZero();
            body.m_torque = 0;
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
        this.m_contactManager.m_broadPhase.query(aabb, (proxy) => {
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
        this.m_contactManager.m_broadPhase.queryPoint(point, (proxy) => {
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
        this.m_contactManager.m_broadPhase.query(aabb, (proxy) => {
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
        this.m_contactManager.m_broadPhase.queryPoint(point, (proxy) => {
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
        this.m_contactManager.m_broadPhase.rayCast(input, (input2, proxy) => {
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
        return this.m_bodyList;
    }

    /**
     * Get the world joint list. With the returned joint, use Joint::GetNext to get
     * the next joint in the world list. A NULL joint indicates the end of the list.
     *
     * @returns The head of the world joint list.
     */
    public getJointList(): Joint | null {
        return this.m_jointList;
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
        return this.m_contactManager.m_contactList;
    }

    /**
     * Enable/disable sleep.
     */
    public setAllowSleeping(flag: boolean): void {
        if (flag === this.m_allowSleep) {
            return;
        }

        this.m_allowSleep = flag;
        if (!this.m_allowSleep) {
            for (let b = this.m_bodyList; b; b = b.m_next) {
                b.setAwake(true);
            }
        }
    }

    public getAllowSleeping(): boolean {
        return this.m_allowSleep;
    }

    /**
     * Enable/disable warm starting. For testing.
     */
    public setWarmStarting(flag: boolean): void {
        this.m_warmStarting = flag;
    }

    public getWarmStarting(): boolean {
        return this.m_warmStarting;
    }

    /**
     * Enable/disable continuous physics. For testing.
     */
    public setContinuousPhysics(flag: boolean): void {
        this.m_continuousPhysics = flag;
    }

    public getContinuousPhysics(): boolean {
        return this.m_continuousPhysics;
    }

    /**
     * Enable/disable single stepped continuous physics. For testing.
     */
    public setSubStepping(flag: boolean): void {
        this.m_subStepping = flag;
    }

    public getSubStepping(): boolean {
        return this.m_subStepping;
    }

    /**
     * Get the number of broad-phase proxies.
     */
    public getProxyCount(): number {
        return this.m_contactManager.m_broadPhase.getProxyCount();
    }

    /**
     * Get the number of bodies.
     */
    public getBodyCount(): number {
        return this.m_bodyCount;
    }

    /**
     * Get the number of joints.
     */
    public getJointCount(): number {
        return this.m_jointCount;
    }

    /**
     * Get the number of contacts (each may have 0 or more contact points).
     */
    public getContactCount(): number {
        return this.m_contactManager.m_contactCount;
    }

    /**
     * Get the height of the dynamic tree.
     */
    public getTreeHeight(): number {
        return this.m_contactManager.m_broadPhase.getTreeHeight();
    }

    /**
     * Get the balance of the dynamic tree.
     */
    public getTreeBalance(): number {
        return this.m_contactManager.m_broadPhase.getTreeBalance();
    }

    /**
     * Get the quality metric of the dynamic tree. The smaller the better.
     * The minimum is 1.
     */
    public getTreeQuality(): number {
        return this.m_contactManager.m_broadPhase.getTreeQuality();
    }

    /**
     * Change the global gravity vector.
     */
    public setGravity(gravity: XY) {
        this.m_gravity.copy(gravity);
    }

    /**
     * Get the global gravity vector.
     */
    public getGravity(): Readonly<Vec2> {
        return this.m_gravity;
    }

    /**
     * Is the world locked (in the middle of a time step).
     */
    public isLocked(): boolean {
        return this.m_locked;
    }

    /**
     * Set flag to control automatic clearing of forces after each time step.
     */
    public setAutoClearForces(flag: boolean): void {
        this.m_clearForces = flag;
    }

    /**
     * Get the flag that controls automatic clearing of forces after each time step.
     */
    public getAutoClearForces(): boolean {
        return this.m_clearForces;
    }

    /**
     * Shift the world origin. Useful for large worlds.
     * The body shift formula is: position -= newOrigin
     *
     * @param newOrigin The new origin with respect to the old origin
     */
    public shiftOrigin(newOrigin: XY): void {
        assert(!this.isLocked());

        for (let b = this.m_bodyList; b; b = b.m_next) {
            b.m_xf.p.subtract(newOrigin);
            b.m_sweep.c0.subtract(newOrigin);
            b.m_sweep.c.subtract(newOrigin);
        }

        for (let j = this.m_jointList; j; j = j.m_next) {
            j.shiftOrigin(newOrigin);
        }

        this.m_contactManager.m_broadPhase.shiftOrigin(newOrigin);
    }

    /**
     * Get the contact manager for testing.
     */
    public getContactManager(): ContactManager {
        return this.m_contactManager;
    }

    /**
     * Get the current profile.
     */
    public getProfile(): Profile {
        return this.m_profile;
    }

    private solve(step: TimeStep): void {
        this.m_profile.solveInit = 0;
        this.m_profile.solveVelocity = 0;
        this.m_profile.solvePosition = 0;

        // Size the island for the worst case.
        const island = this.m_island;
        island.resize(this.m_bodyCount);
        island.clear();

        // Clear all the island flags.
        for (let b = this.m_bodyList; b; b = b.m_next) {
            b.m_islandFlag = false;
        }
        for (let c = this.m_contactManager.m_contactList; c; c = c.m_next) {
            c.m_islandFlag = false;
        }
        for (let j = this.m_jointList; j; j = j.m_next) {
            j.m_islandFlag = false;
        }

        // Build and simulate all awake islands.
        // DEBUG: const stackSize = this.m_bodyCount;
        const stack = this.s_stack;
        for (let seed = this.m_bodyList; seed; seed = seed.m_next) {
            if (seed.m_islandFlag) {
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
            seed.m_islandFlag = true;

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
                b.m_awakeFlag = true;

                // Search all contacts connected to this body.
                for (let ce = b.m_contactList; ce; ce = ce.next) {
                    const { contact } = ce;

                    // Has this contact already been added to an island?
                    if (contact.m_islandFlag) {
                        continue;
                    }

                    // Is this contact solid and touching?
                    if (!contact.isEnabled() || !contact.isTouching()) {
                        continue;
                    }

                    // Skip sensors.
                    const sensorA = contact.m_fixtureA.m_isSensor;
                    const sensorB = contact.m_fixtureB.m_isSensor;
                    if (sensorA || sensorB) {
                        continue;
                    }

                    island.addContact(contact);
                    contact.m_islandFlag = true;

                    const { other } = ce;

                    // Was the other body already added to this island?
                    if (other.m_islandFlag) {
                        continue;
                    }

                    // DEBUG: assert(stackCount < stackSize);
                    stack[stackCount++] = other;
                    other.m_islandFlag = true;
                }

                // Search all joints connect to this body.
                for (let je = b.m_jointList; je; je = je.next) {
                    if (je.joint.m_islandFlag) {
                        continue;
                    }

                    const { other } = je;

                    // Don't simulate joints connected to disabled bodies.
                    if (!other.isEnabled()) {
                        continue;
                    }

                    island.addJoint(je.joint);
                    je.joint.m_islandFlag = true;

                    if (other.m_islandFlag) {
                        continue;
                    }

                    // DEBUG: assert(stackCount < stackSize);
                    stack[stackCount++] = other;
                    other.m_islandFlag = true;
                }
            }

            const profile = new Profile();
            island.solve(profile, step, this.m_gravity, this.m_allowSleep);
            this.m_profile.solveInit += profile.solveInit;
            this.m_profile.solveVelocity += profile.solveVelocity;
            this.m_profile.solvePosition += profile.solvePosition;

            // Post solve cleanup.
            for (let i = 0; i < island.m_bodyCount; ++i) {
                // Allow static bodies to participate in other islands.
                const b = island.m_bodies[i];
                if (b.getType() === BodyType.Static) {
                    b.m_islandFlag = false;
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
        for (let b = this.m_bodyList; b; b = b.m_next) {
            // If a body was not in an island then it did not move.
            if (!b.m_islandFlag) {
                continue;
            }

            if (b.getType() === BodyType.Static) {
                continue;
            }

            // Update fixtures (for broad-phase).
            b.synchronizeFixtures();
        }

        // Look for new contacts.
        this.m_contactManager.findNewContacts();
        this.m_profile.broadphase = timer.getMilliseconds();
    }

    private static SolveTOI_s_subStep = TimeStep.create();

    private static SolveTOI_s_backup = new Sweep();

    private static SolveTOI_s_backup1 = new Sweep();

    private static SolveTOI_s_backup2 = new Sweep();

    private static SolveTOI_s_toi_input = new TOIInput();

    private static SolveTOI_s_toi_output = new TOIOutput();

    /** @internal */
    public solveTOI(step: TimeStep): void {
        const island = this.m_island;
        island.clear();

        if (this.m_stepComplete) {
            for (let b = this.m_bodyList; b; b = b.m_next) {
                b.m_islandFlag = false;
                b.m_sweep.alpha0 = 0;
            }

            for (let c = this.m_contactManager.m_contactList; c; c = c.m_next) {
                // Invalidate TOI
                c.m_toiFlag = false;
                c.m_islandFlag = false;
                c.m_toiCount = 0;
                c.m_toi = 1;
            }
        }

        // Find TOI events and solve them.
        for (;;) {
            // Find the first TOI.
            let minContact = null;
            let minAlpha = 1;

            for (let c = this.m_contactManager.m_contactList; c; c = c.m_next) {
                // Is this contact disabled?
                if (!c.isEnabled()) {
                    continue;
                }

                // Prevent excessive sub-stepping.
                if (c.m_toiCount > MAX_SUB_STEPS) {
                    continue;
                }

                let alpha = 1;
                if (c.m_toiFlag) {
                    // This contact has a valid cached TOI.
                    alpha = c.m_toi;
                } else {
                    const fA = c.getFixtureA();
                    const fB = c.getFixtureB();

                    // Is there a sensor?
                    if (fA.isSensor() || fB.isSensor()) {
                        continue;
                    }

                    const bA = fA.getBody();
                    const bB = fB.getBody();

                    const typeA = bA.m_type;
                    const typeB = bB.m_type;
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
                    let { alpha0 } = bA.m_sweep;

                    if (bA.m_sweep.alpha0 < bB.m_sweep.alpha0) {
                        alpha0 = bB.m_sweep.alpha0;
                        bA.m_sweep.advance(alpha0);
                    } else if (bB.m_sweep.alpha0 < bA.m_sweep.alpha0) {
                        alpha0 = bA.m_sweep.alpha0;
                        bB.m_sweep.advance(alpha0);
                    }

                    // DEBUG: assert(alpha0 < 1);

                    const indexA = c.getChildIndexA();
                    const indexB = c.getChildIndexB();

                    // Compute the time of impact in interval [0, minTOI]
                    const input = World.SolveTOI_s_toi_input;
                    input.proxyA.setShape(fA.getShape(), indexA);
                    input.proxyB.setShape(fB.getShape(), indexB);
                    input.sweepA.copy(bA.m_sweep);
                    input.sweepB.copy(bB.m_sweep);
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

                    c.m_toi = alpha;
                    c.m_toiFlag = true;
                }

                if (alpha < minAlpha) {
                    // This is the minimum TOI found so far.
                    minContact = c;
                    minAlpha = alpha;
                }
            }

            if (minContact === null || 1 - 10 * EPSILON < minAlpha) {
                // No more TOI events. Done!
                this.m_stepComplete = true;
                break;
            }

            // Advance the bodies to the TOI.
            const fA = minContact.getFixtureA();
            const fB = minContact.getFixtureB();
            const bA = fA.getBody();
            const bB = fB.getBody();

            const backup1 = World.SolveTOI_s_backup1.copy(bA.m_sweep);
            const backup2 = World.SolveTOI_s_backup2.copy(bB.m_sweep);

            bA.advance(minAlpha);
            bB.advance(minAlpha);

            // The TOI contact likely has some new contact points.
            minContact.update(this.m_contactManager.m_contactListener);
            minContact.m_toiFlag = false;
            ++minContact.m_toiCount;

            // Is the contact solid?
            if (!minContact.isEnabled() || !minContact.isTouching()) {
                // Restore the sweeps.
                minContact.setEnabled(false);
                bA.m_sweep.copy(backup1);
                bB.m_sweep.copy(backup2);
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

            bA.m_islandFlag = true;
            bB.m_islandFlag = true;
            minContact.m_islandFlag = true;

            // Get contacts on bodyA and bodyB.
            for (let i = 0; i < 2; ++i) {
                const body = i === 0 ? bA : bB;
                if (body.m_type === BodyType.Dynamic) {
                    for (let ce = body.m_contactList; ce; ce = ce.next) {
                        if (island.m_bodyCount === island.m_bodyCapacity) {
                            break;
                        }

                        if (island.m_contactCount === MAX_TOI_CONTACTS) {
                            break;
                        }

                        const { contact } = ce;

                        // Has this contact already been added to the island?
                        if (contact.m_islandFlag) {
                            continue;
                        }

                        // Only add static, kinematic, or bullet bodies.
                        const { other } = ce;
                        if (other.m_type === BodyType.Dynamic && !body.isBullet() && !other.isBullet()) {
                            continue;
                        }

                        // Skip sensors.
                        const sensorA = contact.m_fixtureA.m_isSensor;
                        const sensorB = contact.m_fixtureB.m_isSensor;
                        if (sensorA || sensorB) {
                            continue;
                        }

                        // Tentatively advance the body to the TOI.
                        const backup = World.SolveTOI_s_backup.copy(other.m_sweep);
                        if (!other.m_islandFlag) {
                            other.advance(minAlpha);
                        }

                        // Update the contact points
                        contact.update(this.m_contactManager.m_contactListener);

                        // Was the contact disabled by the user?
                        if (!contact.isEnabled()) {
                            other.m_sweep.copy(backup);
                            other.synchronizeTransform();
                            continue;
                        }

                        // Are there contact points?
                        if (!contact.isTouching()) {
                            other.m_sweep.copy(backup);
                            other.synchronizeTransform();
                            continue;
                        }

                        // Add the contact to the island
                        contact.m_islandFlag = true;
                        island.addContact(contact);

                        // Has the other body already been added to the island?
                        if (other.m_islandFlag) {
                            continue;
                        }

                        // Add the other body to the island.
                        other.m_islandFlag = true;

                        if (other.m_type !== BodyType.Static) {
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
            island.solveTOI(subStep, bA.m_islandIndex, bB.m_islandIndex);

            // Reset island flags and synchronize broad-phase proxies.
            for (let i = 0; i < island.m_bodyCount; ++i) {
                const body = island.m_bodies[i];
                body.m_islandFlag = false;

                if (body.m_type !== BodyType.Dynamic) {
                    continue;
                }

                body.synchronizeFixtures();

                // Invalidate all contact TOIs on this displaced body.
                for (let ce = body.m_contactList; ce; ce = ce.next) {
                    ce.contact.m_toiFlag = false;
                    ce.contact.m_islandFlag = false;
                }
            }

            // Commit fixture proxy movements to the broad-phase so that new contacts are created.
            // Also, some contacts can be destroyed.
            this.m_contactManager.findNewContacts();

            if (this.m_subStepping) {
                this.m_stepComplete = false;
                break;
            }
        }
    }
}
