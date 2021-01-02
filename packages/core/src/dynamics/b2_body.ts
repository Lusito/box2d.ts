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
import { Vec2, Rot, Transform, Sweep, XY } from "../common/b2_math";
import { MassData } from "../collision/b2_shape";
import type { ContactEdge } from "./b2_contact";
import { JointEdge } from "./b2_joint";
import { Fixture, FixtureDef } from "./b2_fixture";
import type { World } from "./b2_world";
import { assert } from "../common/b2_common";

/**
 * The body type.
 * static: zero mass, zero velocity, may be manually moved
 * kinematic: zero mass, non-zero velocity set by user, moved by solver
 * dynamic: positive mass, non-zero velocity determined by forces, moved by solver
 */
export enum BodyType {
    Static,
    Kinematic,
    Dynamic,
}

/**
 * A body definition holds all the data needed to construct a rigid body.
 * You can safely re-use body definitions. Shapes are added to a body after construction.
 */
export interface BodyDef {
    /**
     * The body type: static, kinematic, or dynamic.
     * Note: if a dynamic body would have zero mass, the mass is set to one.
     */
    type?: BodyType;

    /**
     * The world position of the body. Avoid creating bodies at the origin
     * since this can lead to many overlapping shapes.
     */
    position?: XY;

    /** The world angle of the body in radians. */
    angle?: number;

    /** The linear velocity of the body's origin in world co-ordinates. */
    linearVelocity?: XY;

    /** The angular velocity of the body. */
    angularVelocity?: number;

    /**
     * Linear damping is use to reduce the linear velocity. The damping parameter
     * can be larger than 1   but the damping effect becomes sensitive to the
     * time step when the damping parameter is large.
     * Units are 1/time
     */
    linearDamping?: number;

    /**
     * Angular damping is use to reduce the angular velocity. The damping parameter
     * can be larger than 1   but the damping effect becomes sensitive to the
     * time step when the damping parameter is large.
     * Units are 1/time
     */
    angularDamping?: number;

    /**
     * Set this flag to false if this body should never fall asleep. Note that
     * this increases CPU usage.
     */
    allowSleep?: boolean;

    /** Is this body initially awake or sleeping? */
    awake?: boolean;

    /** Should this body be prevented from rotating? Useful for characters. */
    fixedRotation?: boolean;

    /**
     * Is this a fast moving body that should be prevented from tunneling through
     * other moving bodies? Note that all bodies are prevented from tunneling through
     * kinematic and static bodies. This setting is only considered on dynamic bodies.
     *
     * @warning You should use this flag sparingly since it increases processing time.
     */
    bullet?: boolean;

    /** Does this body start out enabled? */
    enabled?: boolean;

    /** Use this to store application specific body data. */
    userData?: any;

    /** Scale the gravity applied to this body. */
    gravityScale?: number;
}

/**
 * A rigid body. These are created via World::CreateBody.
 */
export class Body {
    /** @internal */
    public type = BodyType.Static;

    /** @internal */
    public islandFlag = false;

    /** @internal */
    public awakeFlag = false;

    /** @internal */
    public autoSleepFlag = false;

    /** @internal */
    public bulletFlag = false;

    /** @internal */
    public fixedRotationFlag = false;

    /** @internal */
    public enabledFlag = false;

    /** @internal */
    public toiFlag = false;

    /** @internal */
    public islandIndex = 0;

    /** @internal */
    public readonly xf = new Transform(); // the body origin transform

    /** @internal */
    public readonly sweep = new Sweep(); // the swept motion for CCD

    /** @internal */
    public readonly linearVelocity = new Vec2();

    /** @internal */
    public angularVelocity = 0;

    /** @internal */
    public readonly force = new Vec2();

    /** @internal */
    public torque = 0;

    /** @internal */
    public readonly world: World;

    /** @internal */
    public prev: Body | null = null;

    /** @internal */
    public next: Body | null = null;

    /** @internal */
    public fixtureList: Fixture | null = null;

    /** @internal */
    public fixtureCount = 0;

    /** @internal */
    public jointList: JointEdge | null = null;

    /** @internal */
    public contactList: ContactEdge | null = null;

    /** @internal */
    public mass = 1;

    /** @internal */
    public invMass = 1;

    /**
     * Rotational inertia about the center of mass.
     * @internal
     */
    public I = 0;

    /** @internal */
    public invI = 0;

    /** @internal */
    public linearDamping = 0;

    /** @internal */
    public angularDamping = 0;

    /** @internal */
    public gravityScale = 1;

    /** @internal */
    public sleepTime = 0;

    /** @internal */
    public userData: any = null;

    /** @internal */
    public constructor(bd: BodyDef, world: World) {
        this.bulletFlag = bd.bullet ?? false;
        this.fixedRotationFlag = bd.fixedRotation ?? false;
        this.autoSleepFlag = bd.allowSleep ?? true;
        if ((bd.awake ?? true) && (bd.type ?? BodyType.Static) !== BodyType.Static) {
            this.awakeFlag = true;
        }
        this.enabledFlag = bd.enabled ?? true;

        this.world = world;

        this.xf.p.copy(bd.position ?? Vec2.ZERO);
        this.xf.q.set(bd.angle ?? 0);

        this.sweep.localCenter.setZero();
        this.sweep.c0.copy(this.xf.p);
        this.sweep.c.copy(this.xf.p);
        this.sweep.a0 = this.sweep.a = this.xf.q.getAngle();
        this.sweep.alpha0 = 0;

        this.linearVelocity.copy(bd.linearVelocity ?? Vec2.ZERO);
        this.angularVelocity = bd.angularVelocity ?? 0;

        this.linearDamping = bd.linearDamping ?? 0;
        this.angularDamping = bd.angularDamping ?? 0;
        this.gravityScale = bd.gravityScale ?? 1;

        this.force.setZero();
        this.torque = 0;

        this.sleepTime = 0;

        this.type = bd.type ?? BodyType.Static;

        this.mass = 0;
        this.invMass = 0;

        this.I = 0;
        this.invI = 0;

        this.userData = bd.userData;

        this.fixtureList = null;
        this.fixtureCount = 0;
    }

    /**
     * Creates a fixture and attach it to this body. Use this function if you need
     * to set some fixture parameters, like friction. Otherwise you can create the
     * fixture directly from a shape.
     * If the density is non-zero, this function automatically updates the mass of the body.
     * Contacts are not created until the next time step.
     *
     * @param def The fixture definition.
     * @warning This function is locked during callbacks.
     */
    public createFixture(def: FixtureDef): Fixture {
        assert(!this.world.isLocked());

        const fixture = new Fixture(this, def);

        if (this.enabledFlag) {
            const { broadPhase } = this.world.contactManager;
            fixture.createProxies(broadPhase, this.xf);
        }

        fixture.next = this.fixtureList;
        this.fixtureList = fixture;
        ++this.fixtureCount;

        // Adjust mass properties if needed.
        if (fixture.density > 0) {
            this.resetMassData();
        }

        // Let the world know we have a new fixture. This will cause new contacts
        // to be created at the beginning of the next time step.
        this.world.newContacts = true;

        return fixture;
    }

    /**
     * Destroy a fixture. This removes the fixture from the broad-phase and
     * destroys all contacts associated with this fixture. This will
     * automatically adjust the mass of the body if the body is dynamic and the
     * fixture has positive density.
     * All fixtures attached to a body are implicitly destroyed when the body is destroyed.
     *
     * @param fixture The fixture to be removed.
     * @warning This function is locked during callbacks.
     */
    public destroyFixture(fixture: Fixture): void {
        assert(!this.world.isLocked());

        // DEBUG: assert(fixture.body === this);

        // Remove the fixture from this body's singly linked list.
        // DEBUG: assert(this.fixtureCount > 0);
        let node: Fixture | null = this.fixtureList;
        let ppF: Fixture | null = null;
        // DEBUG: let found = false;
        while (node !== null) {
            if (node === fixture) {
                if (ppF) {
                    ppF.next = fixture.next;
                } else {
                    this.fixtureList = fixture.next;
                }
                // DEBUG: found = true;
                break;
            }

            ppF = node;
            node = node.next;
        }

        // You tried to remove a shape that is not attached to this body.
        // DEBUG: assert(found);

        // Destroy any contacts associated with the fixture.
        let edge: ContactEdge | null = this.contactList;
        while (edge) {
            const c = edge.contact;
            edge = edge.next;

            const fixtureA = c.getFixtureA();
            const fixtureB = c.getFixtureB();

            if (fixture === fixtureA || fixture === fixtureB) {
                // This destroys the contact and removes it from
                // this body's contact list.
                this.world.contactManager.destroy(c);
            }
        }

        if (this.enabledFlag) {
            const { broadPhase } = this.world.contactManager;
            fixture.destroyProxies(broadPhase);
        }

        // fixture.body = null;
        fixture.next = null;

        --this.fixtureCount;

        // Reset the mass data.
        this.resetMassData();
    }

    /**
     * Set the position of the body's origin and rotation.
     * This breaks any contacts and wakes the other bodies.
     * Manipulating a body's transform may cause non-physical behavior.
     *
     * @param position The world position of the body's local origin.
     * @param angle The world rotation in radians.
     */
    public setTransformVec(position: XY, angle: number): void {
        this.setTransformXY(position.x, position.y, angle);
    }

    public setTransformXY(x: number, y: number, angle: number): void {
        assert(!this.world.isLocked());

        this.xf.q.set(angle);
        this.xf.p.set(x, y);

        Transform.multiplyVec2(this.xf, this.sweep.localCenter, this.sweep.c);
        this.sweep.a = angle;

        this.sweep.c0.copy(this.sweep.c);
        this.sweep.a0 = angle;

        const { broadPhase } = this.world.contactManager;
        for (let f: Fixture | null = this.fixtureList; f; f = f.next) {
            f.synchronize(broadPhase, this.xf, this.xf);
        }

        // Check for new contacts the next step
        this.world.newContacts = true;
    }

    public setTransform(xf: Transform): void {
        this.setTransformVec(xf.p, xf.getAngle());
    }

    /**
     * Get the body transform for the body's origin.
     *
     * @returns The world transform of the body's origin.
     */
    public getTransform(): Readonly<Transform> {
        return this.xf;
    }

    /**
     * Get the world body origin position.
     *
     * @returns The world position of the body's origin.
     */
    public getPosition(): Readonly<Vec2> {
        return this.xf.p;
    }

    /**
     * Get the angle in radians.
     *
     * @returns The current world rotation angle in radians.
     */
    public getAngle(): number {
        return this.sweep.a;
    }

    public setAngle(angle: number): void {
        this.setTransformVec(this.getPosition(), angle);
    }

    /**
     * Get the world position of the center of mass.
     */
    public getWorldCenter(): Readonly<Vec2> {
        return this.sweep.c;
    }

    /**
     * Get the local position of the center of mass.
     */
    public getLocalCenter(): Readonly<Vec2> {
        return this.sweep.localCenter;
    }

    /**
     * Set the linear velocity of the center of mass.
     *
     * @param v The new linear velocity of the center of mass.
     */
    public setLinearVelocity(v: XY): void {
        if (this.type === BodyType.Static) {
            return;
        }

        if (Vec2.dot(v, v) > 0) {
            this.setAwake(true);
        }

        this.linearVelocity.copy(v);
    }

    /**
     * Get the linear velocity of the center of mass.
     *
     * @returns The linear velocity of the center of mass.
     */
    public getLinearVelocity(): Readonly<Vec2> {
        return this.linearVelocity;
    }

    /**
     * Set the angular velocity.
     *
     * @param omega The new angular velocity in radians/second.
     */
    public setAngularVelocity(w: number): void {
        if (this.type === BodyType.Static) {
            return;
        }

        if (w * w > 0) {
            this.setAwake(true);
        }

        this.angularVelocity = w;
    }

    /**
     * Get the angular velocity.
     *
     * @returns The angular velocity in radians/second.
     */
    public getAngularVelocity(): number {
        return this.angularVelocity;
    }

    /**
     * Apply a force at a world point. If the force is not
     * applied at the center of mass, it will generate a torque and
     * affect the angular velocity. This wakes up the body.
     *
     * @param force The world force vector, usually in Newtons (N).
     * @param point The world position of the point of application.
     * @param wake Also wake up the body
     */
    public applyForce(force: XY, point: XY, wake = true): void {
        if (this.type !== BodyType.Dynamic) {
            return;
        }

        if (wake && !this.awakeFlag) {
            this.setAwake(true);
        }

        // Don't accumulate a force if the body is sleeping
        if (this.awakeFlag) {
            this.force.x += force.x;
            this.force.y += force.y;
            this.torque += (point.x - this.sweep.c.x) * force.y - (point.y - this.sweep.c.y) * force.x;
        }
    }

    /**
     * Apply a force to the center of mass. This wakes up the body.
     *
     * @param force The world force vector, usually in Newtons (N).
     * @param wake Also wake up the body
     */
    public applyForceToCenter(force: XY, wake = true): void {
        if (this.type !== BodyType.Dynamic) {
            return;
        }

        if (wake && !this.awakeFlag) {
            this.setAwake(true);
        }

        // Don't accumulate a force if the body is sleeping
        if (this.awakeFlag) {
            this.force.x += force.x;
            this.force.y += force.y;
        }
    }

    /**
     * Apply a torque. This affects the angular velocity
     * without affecting the linear velocity of the center of mass.
     *
     * @param torque About the z-axis (out of the screen), usually in N-m.
     * @param wake Also wake up the body
     */
    public applyTorque(torque: number, wake = true): void {
        if (this.type !== BodyType.Dynamic) {
            return;
        }

        if (wake && !this.awakeFlag) {
            this.setAwake(true);
        }

        // Don't accumulate a force if the body is sleeping
        if (this.awakeFlag) {
            this.torque += torque;
        }
    }

    /**
     * Apply an impulse at a point. This immediately modifies the velocity.
     * It also modifies the angular velocity if the point of application
     * is not at the center of mass. This wakes up the body.
     *
     * @param impulse The world impulse vector, usually in N-seconds or kg-m/s.
     * @param point The world position of the point of application.
     * @param wake Also wake up the body
     */
    public applyLinearImpulse(impulse: XY, point: XY, wake = true): void {
        if (this.type !== BodyType.Dynamic) {
            return;
        }

        if (wake && !this.awakeFlag) {
            this.setAwake(true);
        }

        // Don't accumulate velocity if the body is sleeping
        if (this.awakeFlag) {
            this.linearVelocity.x += this.invMass * impulse.x;
            this.linearVelocity.y += this.invMass * impulse.y;
            this.angularVelocity +=
                this.invI * ((point.x - this.sweep.c.x) * impulse.y - (point.y - this.sweep.c.y) * impulse.x);
        }
    }

    /**
     * Apply an impulse at the center of gravity. This immediately modifies the velocity.
     *
     * @param impulse The world impulse vector, usually in N-seconds or kg-m/s.
     * @param wake Also wake up the body
     */
    public applyLinearImpulseToCenter(impulse: XY, wake = true): void {
        if (this.type !== BodyType.Dynamic) {
            return;
        }

        if (wake && !this.awakeFlag) {
            this.setAwake(true);
        }

        // Don't accumulate velocity if the body is sleeping
        if (this.awakeFlag) {
            this.linearVelocity.x += this.invMass * impulse.x;
            this.linearVelocity.y += this.invMass * impulse.y;
        }
    }

    /**
     * Apply an angular impulse.
     *
     * @param impulse The angular impulse in units of kg*m*m/s
     * @param wake Also wake up the body
     */
    public applyAngularImpulse(impulse: number, wake = true): void {
        if (this.type !== BodyType.Dynamic) {
            return;
        }

        if (wake && !this.awakeFlag) {
            this.setAwake(true);
        }

        // Don't accumulate velocity if the body is sleeping
        if (this.awakeFlag) {
            this.angularVelocity += this.invI * impulse;
        }
    }

    /**
     * Get the total mass of the body.
     *
     * @returns The mass, usually in kilograms (kg).
     */
    public getMass(): number {
        return this.mass;
    }

    /**
     * Get the rotational inertia of the body about the local origin.
     *
     * @returns The rotational inertia, usually in kg-m^2.
     */
    public getInertia(): number {
        return this.I + this.mass * Vec2.dot(this.sweep.localCenter, this.sweep.localCenter);
    }

    /**
     * Get the mass data of the body.
     *
     * @returns A struct containing the mass, inertia and center of the body.
     */
    public getMassData(data: MassData): MassData {
        data.mass = this.mass;
        data.I = this.I + this.mass * Vec2.dot(this.sweep.localCenter, this.sweep.localCenter);
        data.center.copy(this.sweep.localCenter);
        return data;
    }

    private static SetMassData_s_oldCenter = new Vec2();

    /**
     * Set the mass properties to override the mass properties of the fixtures.
     * Note that this changes the center of mass position.
     * Note that creating or destroying fixtures can also alter the mass.
     * This function has no effect if the body isn't dynamic.
     *
     * @param massData The mass properties.
     */
    public setMassData(massData: MassData): void {
        assert(!this.world.isLocked());

        if (this.type !== BodyType.Dynamic) {
            return;
        }

        this.invMass = 0;
        this.I = 0;
        this.invI = 0;

        this.mass = massData.mass;
        if (this.mass <= 0) {
            this.mass = 1;
        }

        this.invMass = 1 / this.mass;

        if (massData.I > 0 && !this.fixedRotationFlag) {
            this.I = massData.I - this.mass * Vec2.dot(massData.center, massData.center);
            // DEBUG: assert(this.I > 0);
            this.invI = 1 / this.I;
        }

        // Move center of mass.
        const oldCenter = Body.SetMassData_s_oldCenter.copy(this.sweep.c);
        this.sweep.localCenter.copy(massData.center);
        Transform.multiplyVec2(this.xf, this.sweep.localCenter, this.sweep.c);
        this.sweep.c0.copy(this.sweep.c);

        // Update center of mass velocity.
        Vec2.addCrossScalarVec2(
            this.linearVelocity,
            this.angularVelocity,
            Vec2.subtract(this.sweep.c, oldCenter, Vec2.s_t0),
            this.linearVelocity,
        );
    }

    private static ResetMassData_s_localCenter = new Vec2();

    private static ResetMassData_s_oldCenter = new Vec2();

    private static ResetMassData_s_massData = new MassData();

    /**
     * This resets the mass properties to the sum of the mass properties of the fixtures.
     * This normally does not need to be called unless you called SetMassData to override
     * the mass and you later want to reset the mass.
     */
    public resetMassData(): void {
        // Compute mass data from shapes. Each shape has its own density.
        this.mass = 0;
        this.invMass = 0;
        this.I = 0;
        this.invI = 0;
        this.sweep.localCenter.setZero();

        // Static and kinematic bodies have zero mass.
        if (this.type === BodyType.Static || this.type === BodyType.Kinematic) {
            this.sweep.c0.copy(this.xf.p);
            this.sweep.c.copy(this.xf.p);
            this.sweep.a0 = this.sweep.a;
            return;
        }

        // DEBUG: assert(this.type === BodyType.Dynamic);

        // Accumulate mass over all fixtures.
        const localCenter = Body.ResetMassData_s_localCenter.setZero();
        for (let f: Fixture | null = this.fixtureList; f; f = f.next) {
            if (f.density === 0) {
                continue;
            }

            const massData = f.getMassData(Body.ResetMassData_s_massData);
            this.mass += massData.mass;
            localCenter.addScaled(massData.mass, massData.center);
            this.I += massData.I;
        }

        // Compute center of mass.
        if (this.mass > 0) {
            this.invMass = 1 / this.mass;
            localCenter.scale(this.invMass);
        }

        if (this.I > 0 && !this.fixedRotationFlag) {
            // Center the inertia about the center of mass.
            this.I -= this.mass * Vec2.dot(localCenter, localCenter);
            // DEBUG: assert(this.I > 0);
            this.invI = 1 / this.I;
        } else {
            this.I = 0;
            this.invI = 0;
        }

        // Move center of mass.
        const oldCenter = Body.ResetMassData_s_oldCenter.copy(this.sweep.c);
        this.sweep.localCenter.copy(localCenter);
        Transform.multiplyVec2(this.xf, this.sweep.localCenter, this.sweep.c);
        this.sweep.c0.copy(this.sweep.c);

        // Update center of mass velocity.
        Vec2.addCrossScalarVec2(
            this.linearVelocity,
            this.angularVelocity,
            Vec2.subtract(this.sweep.c, oldCenter, Vec2.s_t0),
            this.linearVelocity,
        );
    }

    /**
     * Get the world coordinates of a point given the local coordinates.
     *
     * @param localPoint A point on the body measured relative the the body's origin.
     * @returns The same point expressed in world coordinates.
     */
    public getWorldPoint<T extends XY>(localPoint: Readonly<XY>, out: T): T {
        return Transform.multiplyVec2(this.xf, localPoint, out);
    }

    /**
     * Get the world coordinates of a vector given the local coordinates.
     *
     * @param localVector A vector fixed in the body.
     * @returns The same vector expressed in world coordinates.
     */
    public getWorldVector<T extends XY>(localVector: Readonly<XY>, out: T): T {
        return Rot.multiplyVec2(this.xf.q, localVector, out);
    }

    /**
     * Gets a local point relative to the body's origin given a world point.
     *
     * @param a Point in world coordinates.
     * @returns The corresponding local point relative to the body's origin.
     */
    public getLocalPoint<T extends XY>(worldPoint: Readonly<XY>, out: T): T {
        return Transform.transposeMultiplyVec2(this.xf, worldPoint, out);
    }

    /**
     * Gets a local vector given a world vector.
     *
     * @param a Vector in world coordinates.
     * @returns The corresponding local vector.
     */
    public getLocalVector<T extends XY>(worldVector: Readonly<XY>, out: T): T {
        return Rot.transposeMultiplyVec2(this.xf.q, worldVector, out);
    }

    /**
     * Get the world linear velocity of a world point attached to this body.
     *
     * @param a Point in world coordinates.
     * @returns The world velocity of a point.
     */
    public getLinearVelocityFromWorldPoint<T extends XY>(worldPoint: Readonly<XY>, out: T): T {
        return Vec2.addCrossScalarVec2(
            this.linearVelocity,
            this.angularVelocity,
            Vec2.subtract(worldPoint, this.sweep.c, Vec2.s_t0),
            out,
        );
    }

    /**
     * Get the world velocity of a local point.
     *
     * @param a Point in local coordinates.
     * @returns The world velocity of a point.
     */
    public getLinearVelocityFromLocalPoint<T extends XY>(localPoint: Readonly<XY>, out: T): T {
        return this.getLinearVelocityFromWorldPoint(this.getWorldPoint(localPoint, out), out);
    }

    /**
     * Get the linear damping of the body.
     */
    public getLinearDamping(): number {
        return this.linearDamping;
    }

    /**
     * Set the linear damping of the body.
     */
    public setLinearDamping(linearDamping: number): void {
        this.linearDamping = linearDamping;
    }

    /**
     * Get the angular damping of the body.
     */
    public getAngularDamping(): number {
        return this.angularDamping;
    }

    /**
     * Set the angular damping of the body.
     */
    public setAngularDamping(angularDamping: number): void {
        this.angularDamping = angularDamping;
    }

    /**
     * Get the gravity scale of the body.
     */
    public getGravityScale(): number {
        return this.gravityScale;
    }

    /**
     * Set the gravity scale of the body.
     */
    public setGravityScale(scale: number): void {
        this.gravityScale = scale;
    }

    /**
     * Set the type of this body. This may alter the mass and velocity.
     */
    public setType(type: BodyType): void {
        assert(!this.world.isLocked());

        if (this.type === type) {
            return;
        }

        this.type = type;

        this.resetMassData();

        if (this.type === BodyType.Static) {
            this.linearVelocity.setZero();
            this.angularVelocity = 0;
            this.sweep.a0 = this.sweep.a;
            this.sweep.c0.copy(this.sweep.c);
            this.awakeFlag = false;
            this.synchronizeFixtures();
        }

        this.setAwake(true);

        this.force.setZero();
        this.torque = 0;

        // Delete the attached contacts.
        let ce: ContactEdge | null = this.contactList;
        while (ce) {
            const ce0 = ce;
            ce = ce.next;
            this.world.contactManager.destroy(ce0.contact);
        }
        this.contactList = null;

        // Touch the proxies so that new contacts will be created (when appropriate)
        const { broadPhase } = this.world.contactManager;
        for (let f: Fixture | null = this.fixtureList; f; f = f.next) {
            for (const proxy of f.proxies) {
                broadPhase.touchProxy(proxy.treeNode);
            }
        }
    }

    /**
     * Get the type of this body.
     */
    public getType(): BodyType {
        return this.type;
    }

    /**
     * Should this body be treated like a bullet for continuous collision detection?
     */
    public setBullet(flag: boolean): void {
        this.bulletFlag = flag;
    }

    /**
     * Is this body treated like a bullet for continuous collision detection?
     */
    public isBullet(): boolean {
        return this.bulletFlag;
    }

    /**
     * You can disable sleeping on this body. If you disable sleeping, the
     * body will be woken.
     */
    public setSleepingAllowed(flag: boolean): void {
        this.autoSleepFlag = flag;
        if (!flag) {
            this.setAwake(true);
        }
    }

    /**
     * Is this body allowed to sleep
     */
    public isSleepingAllowed(): boolean {
        return this.autoSleepFlag;
    }

    /**
     * Set the sleep state of the body. A sleeping body has very
     * low CPU cost.
     *
     * @param flag Set to true to wake the body, false to put it to sleep.
     */
    public setAwake(flag: boolean): void {
        if (this.type === BodyType.Static) {
            return;
        }
        if (flag) {
            this.awakeFlag = true;
            this.sleepTime = 0;
        } else {
            this.awakeFlag = false;
            this.sleepTime = 0;
            this.linearVelocity.setZero();
            this.angularVelocity = 0;
            this.force.setZero();
            this.torque = 0;
        }
    }

    /**
     * Get the sleeping state of this body.
     *
     * @returns true if the body is sleeping.
     */
    public isAwake(): boolean {
        return this.awakeFlag;
    }

    /**
     * Allow a body to be disabled. A disabled body is not simulated and cannot
     * be collided with or woken up.
     * If you pass a flag of true, all fixtures will be added to the broad-phase.
     * If you pass a flag of false, all fixtures will be removed from the
     * broad-phase and all contacts will be destroyed.
     * Fixtures and joints are otherwise unaffected. You may continue
     * to create/destroy fixtures and joints on disabled bodies.
     * Fixtures on a disabled body are implicitly disabled and will
     * not participate in collisions, ray-casts, or queries.
     * Joints connected to a disabled body are implicitly disabled.
     * An disabled body is still owned by a World object and remains
     * in the body list.
     */
    public setEnabled(flag: boolean): void {
        assert(!this.world.isLocked());

        if (flag === this.isEnabled()) {
            return;
        }

        this.enabledFlag = flag;

        const { broadPhase } = this.world.contactManager;
        if (flag) {
            // Create all proxies.
            for (let f: Fixture | null = this.fixtureList; f; f = f.next) {
                f.createProxies(broadPhase, this.xf);
            }
            // Contacts are created at the beginning of the next
            this.world.newContacts = true;
        } else {
            // Destroy all proxies.
            for (let f: Fixture | null = this.fixtureList; f; f = f.next) {
                f.destroyProxies(broadPhase);
            }
            // Destroy the attached contacts.
            let ce: ContactEdge | null = this.contactList;
            while (ce) {
                const ce0 = ce;
                ce = ce.next;
                this.world.contactManager.destroy(ce0.contact);
            }
            this.contactList = null;
        }
    }

    /**
     * Get the active state of the body.
     */
    public isEnabled(): boolean {
        return this.enabledFlag;
    }

    /**
     * Set this body to have fixed rotation. This causes the mass
     * to be reset.
     */
    public setFixedRotation(flag: boolean): void {
        if (this.fixedRotationFlag === flag) {
            return;
        }

        this.fixedRotationFlag = flag;

        this.angularVelocity = 0;

        this.resetMassData();
    }

    /**
     * Does this body have fixed rotation?
     */
    public isFixedRotation(): boolean {
        return this.fixedRotationFlag;
    }

    /**
     * Get the list of all fixtures attached to this body.
     */
    public getFixtureList(): Fixture | null {
        return this.fixtureList;
    }

    /**
     * Get the list of all joints attached to this body.
     */
    public getJointList(): JointEdge | null {
        return this.jointList;
    }

    /**
     * Get the list of all contacts attached to this body.
     *
     * @warning this list changes during the time step and you may
     * miss some collisions if you don't use ContactListener.
     */
    public getContactList(): ContactEdge | null {
        return this.contactList;
    }

    /**
     * Get the next body in the world's body list.
     */
    public getNext(): Body | null {
        return this.next;
    }

    /**
     * Get the user data pointer that was provided in the body definition.
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
     * Get the parent world of this body.
     */
    public getWorld(): World {
        return this.world;
    }

    private static SynchronizeFixtures_s_xf1 = new Transform();

    /** @internal */
    public synchronizeFixtures(): void {
        const { broadPhase } = this.world.contactManager;
        if (this.awakeFlag) {
            const xf1 = Body.SynchronizeFixtures_s_xf1;
            xf1.q.set(this.sweep.a0);
            Rot.multiplyVec2(xf1.q, this.sweep.localCenter, xf1.p);
            Vec2.subtract(this.sweep.c0, xf1.p, xf1.p);

            for (let f: Fixture | null = this.fixtureList; f; f = f.next) {
                f.synchronize(broadPhase, xf1, this.xf);
            }
        } else {
            for (let f: Fixture | null = this.fixtureList; f; f = f.next) {
                f.synchronize(broadPhase, this.xf, this.xf);
            }
        }
    }

    /** @internal */
    public synchronizeTransform(): void {
        this.xf.q.set(this.sweep.a);
        Rot.multiplyVec2(this.xf.q, this.sweep.localCenter, this.xf.p);
        Vec2.subtract(this.sweep.c, this.xf.p, this.xf.p);
    }

    /**
     * This is used to prevent connected bodies from colliding.
     * It may lie, depending on the collideConnected flag.
     *
     * @internal
     */
    public shouldCollide(other: Body): boolean {
        // At least one body should be dynamic.
        if (this.type !== BodyType.Dynamic && other.type !== BodyType.Dynamic) {
            return false;
        }
        return this.shouldCollideConnected(other);
    }

    private shouldCollideConnected(other: Body): boolean {
        // Does a joint prevent collision?
        for (let jn: JointEdge | null = this.jointList; jn; jn = jn.next) {
            if (jn.other === other) {
                if (!jn.joint.collideConnected) {
                    return false;
                }
            }
        }

        return true;
    }

    /** @internal */
    public advance(alpha: number): void {
        // Advance to the new safe time. This doesn't sync the broad-phase.
        this.sweep.advance(alpha);
        this.sweep.c.copy(this.sweep.c0);
        this.sweep.a = this.sweep.a0;
        this.xf.q.set(this.sweep.a);
        Rot.multiplyVec2(this.xf.q, this.sweep.localCenter, this.xf.p);
        Vec2.subtract(this.sweep.c, this.xf.p, this.xf.p);
    }
}
