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

// DEBUG: import { Assert } from "../common/b2_common";
import { Vec2, Rot, Transform, Sweep, XY } from "../common/b2_math";
import { MassData } from "../collision/b2_shape";
import type { ContactEdge } from "./b2_contact";
import { JointEdge } from "./b2_joint";
import { Fixture, FixtureDef } from "./b2_fixture";
import type { World } from "./b2_world";
import { Assert } from "../common/b2_common";

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
    public m_type = BodyType.Static;

    /** @internal */
    public m_islandFlag = false;

    /** @internal */
    public m_awakeFlag = false;

    /** @internal */
    public m_autoSleepFlag = false;

    /** @internal */
    public m_bulletFlag = false;

    /** @internal */
    public m_fixedRotationFlag = false;

    /** @internal */
    public m_enabledFlag = false;

    /** @internal */
    public m_toiFlag = false;

    /** @internal */
    public m_islandIndex = 0;

    /** @internal */
    public readonly m_xf = new Transform(); // the body origin transform

    /** @internal */
    public readonly m_sweep = new Sweep(); // the swept motion for CCD

    /** @internal */
    public readonly m_linearVelocity = new Vec2();

    /** @internal */
    public m_angularVelocity = 0;

    /** @internal */
    public readonly m_force = new Vec2();

    /** @internal */
    public m_torque = 0;

    /** @internal */
    public readonly m_world: World;

    /** @internal */
    public m_prev: Body | null = null;

    /** @internal */
    public m_next: Body | null = null;

    /** @internal */
    public m_fixtureList: Fixture | null = null;

    /** @internal */
    public m_fixtureCount = 0;

    /** @internal */
    public m_jointList: JointEdge | null = null;

    /** @internal */
    public m_contactList: ContactEdge | null = null;

    /** @internal */
    public m_mass = 1;

    /** @internal */
    public m_invMass = 1;

    /**
     * Rotational inertia about the center of mass.
     * @internal
     */
    public m_I = 0;

    /** @internal */
    public m_invI = 0;

    /** @internal */
    public m_linearDamping = 0;

    /** @internal */
    public m_angularDamping = 0;

    /** @internal */
    public m_gravityScale = 1;

    /** @internal */
    public m_sleepTime = 0;

    /** @internal */
    public m_userData: any = null;

    /** @internal */
    public constructor(bd: BodyDef, world: World) {
        this.m_bulletFlag = bd.bullet ?? false;
        this.m_fixedRotationFlag = bd.fixedRotation ?? false;
        this.m_autoSleepFlag = bd.allowSleep ?? true;
        if ((bd.awake ?? true) && (bd.type ?? BodyType.Static) !== BodyType.Static) {
            this.m_awakeFlag = true;
        }
        this.m_enabledFlag = bd.enabled ?? true;

        this.m_world = world;

        this.m_xf.p.Copy(bd.position ?? Vec2.ZERO);
        this.m_xf.q.Set(bd.angle ?? 0);

        this.m_sweep.localCenter.SetZero();
        this.m_sweep.c0.Copy(this.m_xf.p);
        this.m_sweep.c.Copy(this.m_xf.p);
        this.m_sweep.a0 = this.m_sweep.a = this.m_xf.q.GetAngle();
        this.m_sweep.alpha0 = 0;

        this.m_linearVelocity.Copy(bd.linearVelocity ?? Vec2.ZERO);
        this.m_angularVelocity = bd.angularVelocity ?? 0;

        this.m_linearDamping = bd.linearDamping ?? 0;
        this.m_angularDamping = bd.angularDamping ?? 0;
        this.m_gravityScale = bd.gravityScale ?? 1;

        this.m_force.SetZero();
        this.m_torque = 0;

        this.m_sleepTime = 0;

        this.m_type = bd.type ?? BodyType.Static;

        this.m_mass = 0;
        this.m_invMass = 0;

        this.m_I = 0;
        this.m_invI = 0;

        this.m_userData = bd.userData;

        this.m_fixtureList = null;
        this.m_fixtureCount = 0;
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
    public CreateFixture(def: FixtureDef): Fixture {
        Assert(!this.m_world.IsLocked());

        const fixture = new Fixture(this, def);

        if (this.m_enabledFlag) {
            const broadPhase = this.m_world.m_contactManager.m_broadPhase;
            fixture.CreateProxies(broadPhase, this.m_xf);
        }

        fixture.m_next = this.m_fixtureList;
        this.m_fixtureList = fixture;
        ++this.m_fixtureCount;

        // Adjust mass properties if needed.
        if (fixture.m_density > 0) {
            this.ResetMassData();
        }

        // Let the world know we have a new fixture. This will cause new contacts
        // to be created at the beginning of the next time step.
        this.m_world.m_newContacts = true;

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
    public DestroyFixture(fixture: Fixture): void {
        Assert(!this.m_world.IsLocked());

        // DEBUG: Assert(fixture.m_body === this);

        // Remove the fixture from this body's singly linked list.
        // DEBUG: Assert(this.m_fixtureCount > 0);
        let node: Fixture | null = this.m_fixtureList;
        let ppF: Fixture | null = null;
        // DEBUG: let found = false;
        while (node !== null) {
            if (node === fixture) {
                if (ppF) {
                    ppF.m_next = fixture.m_next;
                } else {
                    this.m_fixtureList = fixture.m_next;
                }
                // DEBUG: found = true;
                break;
            }

            ppF = node;
            node = node.m_next;
        }

        // You tried to remove a shape that is not attached to this body.
        // DEBUG: Assert(found);

        // Destroy any contacts associated with the fixture.
        let edge: ContactEdge | null = this.m_contactList;
        while (edge) {
            const c = edge.contact;
            edge = edge.next;

            const fixtureA = c.GetFixtureA();
            const fixtureB = c.GetFixtureB();

            if (fixture === fixtureA || fixture === fixtureB) {
                // This destroys the contact and removes it from
                // this body's contact list.
                this.m_world.m_contactManager.Destroy(c);
            }
        }

        if (this.m_enabledFlag) {
            const broadPhase = this.m_world.m_contactManager.m_broadPhase;
            fixture.DestroyProxies(broadPhase);
        }

        // fixture.m_body = null;
        fixture.m_next = null;

        --this.m_fixtureCount;

        // Reset the mass data.
        this.ResetMassData();
    }

    /**
     * Set the position of the body's origin and rotation.
     * This breaks any contacts and wakes the other bodies.
     * Manipulating a body's transform may cause non-physical behavior.
     *
     * @param position The world position of the body's local origin.
     * @param angle The world rotation in radians.
     */
    public SetTransformVec(position: XY, angle: number): void {
        this.SetTransformXY(position.x, position.y, angle);
    }

    public SetTransformXY(x: number, y: number, angle: number): void {
        Assert(!this.m_world.IsLocked());

        this.m_xf.q.Set(angle);
        this.m_xf.p.Set(x, y);

        Transform.MultiplyVec2(this.m_xf, this.m_sweep.localCenter, this.m_sweep.c);
        this.m_sweep.a = angle;

        this.m_sweep.c0.Copy(this.m_sweep.c);
        this.m_sweep.a0 = angle;

        const broadPhase = this.m_world.m_contactManager.m_broadPhase;
        for (let f: Fixture | null = this.m_fixtureList; f; f = f.m_next) {
            f.Synchronize(broadPhase, this.m_xf, this.m_xf);
        }

        // Check for new contacts the next step
        this.m_world.m_newContacts = true;
    }

    public SetTransform(xf: Transform): void {
        this.SetTransformVec(xf.p, xf.GetAngle());
    }

    /**
     * Get the body transform for the body's origin.
     *
     * @returns The world transform of the body's origin.
     */
    public GetTransform(): Readonly<Transform> {
        return this.m_xf;
    }

    /**
     * Get the world body origin position.
     *
     * @returns The world position of the body's origin.
     */
    public GetPosition(): Readonly<Vec2> {
        return this.m_xf.p;
    }

    /**
     * Get the angle in radians.
     *
     * @returns The current world rotation angle in radians.
     */
    public GetAngle(): number {
        return this.m_sweep.a;
    }

    public SetAngle(angle: number): void {
        this.SetTransformVec(this.GetPosition(), angle);
    }

    /**
     * Get the world position of the center of mass.
     */
    public GetWorldCenter(): Readonly<Vec2> {
        return this.m_sweep.c;
    }

    /**
     * Get the local position of the center of mass.
     */
    public GetLocalCenter(): Readonly<Vec2> {
        return this.m_sweep.localCenter;
    }

    /**
     * Set the linear velocity of the center of mass.
     *
     * @param v The new linear velocity of the center of mass.
     */
    public SetLinearVelocity(v: XY): void {
        if (this.m_type === BodyType.Static) {
            return;
        }

        if (Vec2.Dot(v, v) > 0) {
            this.SetAwake(true);
        }

        this.m_linearVelocity.Copy(v);
    }

    /**
     * Get the linear velocity of the center of mass.
     *
     * @returns The linear velocity of the center of mass.
     */
    public GetLinearVelocity(): Readonly<Vec2> {
        return this.m_linearVelocity;
    }

    /**
     * Set the angular velocity.
     *
     * @param omega The new angular velocity in radians/second.
     */
    public SetAngularVelocity(w: number): void {
        if (this.m_type === BodyType.Static) {
            return;
        }

        if (w * w > 0) {
            this.SetAwake(true);
        }

        this.m_angularVelocity = w;
    }

    /**
     * Get the angular velocity.
     *
     * @returns The angular velocity in radians/second.
     */
    public GetAngularVelocity(): number {
        return this.m_angularVelocity;
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
    public ApplyForce(force: XY, point: XY, wake = true): void {
        if (this.m_type !== BodyType.Dynamic) {
            return;
        }

        if (wake && !this.m_awakeFlag) {
            this.SetAwake(true);
        }

        // Don't accumulate a force if the body is sleeping
        if (this.m_awakeFlag) {
            this.m_force.x += force.x;
            this.m_force.y += force.y;
            this.m_torque += (point.x - this.m_sweep.c.x) * force.y - (point.y - this.m_sweep.c.y) * force.x;
        }
    }

    /**
     * Apply a force to the center of mass. This wakes up the body.
     *
     * @param force The world force vector, usually in Newtons (N).
     * @param wake Also wake up the body
     */
    public ApplyForceToCenter(force: XY, wake = true): void {
        if (this.m_type !== BodyType.Dynamic) {
            return;
        }

        if (wake && !this.m_awakeFlag) {
            this.SetAwake(true);
        }

        // Don't accumulate a force if the body is sleeping
        if (this.m_awakeFlag) {
            this.m_force.x += force.x;
            this.m_force.y += force.y;
        }
    }

    /**
     * Apply a torque. This affects the angular velocity
     * without affecting the linear velocity of the center of mass.
     *
     * @param torque About the z-axis (out of the screen), usually in N-m.
     * @param wake Also wake up the body
     */
    public ApplyTorque(torque: number, wake = true): void {
        if (this.m_type !== BodyType.Dynamic) {
            return;
        }

        if (wake && !this.m_awakeFlag) {
            this.SetAwake(true);
        }

        // Don't accumulate a force if the body is sleeping
        if (this.m_awakeFlag) {
            this.m_torque += torque;
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
    public ApplyLinearImpulse(impulse: XY, point: XY, wake = true): void {
        if (this.m_type !== BodyType.Dynamic) {
            return;
        }

        if (wake && !this.m_awakeFlag) {
            this.SetAwake(true);
        }

        // Don't accumulate velocity if the body is sleeping
        if (this.m_awakeFlag) {
            this.m_linearVelocity.x += this.m_invMass * impulse.x;
            this.m_linearVelocity.y += this.m_invMass * impulse.y;
            this.m_angularVelocity +=
                this.m_invI * ((point.x - this.m_sweep.c.x) * impulse.y - (point.y - this.m_sweep.c.y) * impulse.x);
        }
    }

    /**
     * Apply an impulse at the center of gravity. This immediately modifies the velocity.
     *
     * @param impulse The world impulse vector, usually in N-seconds or kg-m/s.
     * @param wake Also wake up the body
     */
    public ApplyLinearImpulseToCenter(impulse: XY, wake = true): void {
        if (this.m_type !== BodyType.Dynamic) {
            return;
        }

        if (wake && !this.m_awakeFlag) {
            this.SetAwake(true);
        }

        // Don't accumulate velocity if the body is sleeping
        if (this.m_awakeFlag) {
            this.m_linearVelocity.x += this.m_invMass * impulse.x;
            this.m_linearVelocity.y += this.m_invMass * impulse.y;
        }
    }

    /**
     * Apply an angular impulse.
     *
     * @param impulse The angular impulse in units of kg*m*m/s
     * @param wake Also wake up the body
     */
    public ApplyAngularImpulse(impulse: number, wake = true): void {
        if (this.m_type !== BodyType.Dynamic) {
            return;
        }

        if (wake && !this.m_awakeFlag) {
            this.SetAwake(true);
        }

        // Don't accumulate velocity if the body is sleeping
        if (this.m_awakeFlag) {
            this.m_angularVelocity += this.m_invI * impulse;
        }
    }

    /**
     * Get the total mass of the body.
     *
     * @returns The mass, usually in kilograms (kg).
     */
    public GetMass(): number {
        return this.m_mass;
    }

    /**
     * Get the rotational inertia of the body about the local origin.
     *
     * @returns The rotational inertia, usually in kg-m^2.
     */
    public GetInertia(): number {
        return this.m_I + this.m_mass * Vec2.Dot(this.m_sweep.localCenter, this.m_sweep.localCenter);
    }

    /**
     * Get the mass data of the body.
     *
     * @returns A struct containing the mass, inertia and center of the body.
     */
    public GetMassData(data: MassData): MassData {
        data.mass = this.m_mass;
        data.I = this.m_I + this.m_mass * Vec2.Dot(this.m_sweep.localCenter, this.m_sweep.localCenter);
        data.center.Copy(this.m_sweep.localCenter);
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
    public SetMassData(massData: MassData): void {
        Assert(!this.m_world.IsLocked());

        if (this.m_type !== BodyType.Dynamic) {
            return;
        }

        this.m_invMass = 0;
        this.m_I = 0;
        this.m_invI = 0;

        this.m_mass = massData.mass;
        if (this.m_mass <= 0) {
            this.m_mass = 1;
        }

        this.m_invMass = 1 / this.m_mass;

        if (massData.I > 0 && !this.m_fixedRotationFlag) {
            this.m_I = massData.I - this.m_mass * Vec2.Dot(massData.center, massData.center);
            // DEBUG: Assert(this.m_I > 0);
            this.m_invI = 1 / this.m_I;
        }

        // Move center of mass.
        const oldCenter = Body.SetMassData_s_oldCenter.Copy(this.m_sweep.c);
        this.m_sweep.localCenter.Copy(massData.center);
        Transform.MultiplyVec2(this.m_xf, this.m_sweep.localCenter, this.m_sweep.c);
        this.m_sweep.c0.Copy(this.m_sweep.c);

        // Update center of mass velocity.
        Vec2.AddCrossScalarVec2(
            this.m_linearVelocity,
            this.m_angularVelocity,
            Vec2.Subtract(this.m_sweep.c, oldCenter, Vec2.s_t0),
            this.m_linearVelocity,
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
    public ResetMassData(): void {
        // Compute mass data from shapes. Each shape has its own density.
        this.m_mass = 0;
        this.m_invMass = 0;
        this.m_I = 0;
        this.m_invI = 0;
        this.m_sweep.localCenter.SetZero();

        // Static and kinematic bodies have zero mass.
        if (this.m_type === BodyType.Static || this.m_type === BodyType.Kinematic) {
            this.m_sweep.c0.Copy(this.m_xf.p);
            this.m_sweep.c.Copy(this.m_xf.p);
            this.m_sweep.a0 = this.m_sweep.a;
            return;
        }

        // DEBUG: Assert(this.m_type === BodyType.Dynamic);

        // Accumulate mass over all fixtures.
        const localCenter = Body.ResetMassData_s_localCenter.SetZero();
        for (let f: Fixture | null = this.m_fixtureList; f; f = f.m_next) {
            if (f.m_density === 0) {
                continue;
            }

            const massData = f.GetMassData(Body.ResetMassData_s_massData);
            this.m_mass += massData.mass;
            localCenter.AddScaled(massData.mass, massData.center);
            this.m_I += massData.I;
        }

        // Compute center of mass.
        if (this.m_mass > 0) {
            this.m_invMass = 1 / this.m_mass;
            localCenter.Scale(this.m_invMass);
        }

        if (this.m_I > 0 && !this.m_fixedRotationFlag) {
            // Center the inertia about the center of mass.
            this.m_I -= this.m_mass * Vec2.Dot(localCenter, localCenter);
            // DEBUG: Assert(this.m_I > 0);
            this.m_invI = 1 / this.m_I;
        } else {
            this.m_I = 0;
            this.m_invI = 0;
        }

        // Move center of mass.
        const oldCenter = Body.ResetMassData_s_oldCenter.Copy(this.m_sweep.c);
        this.m_sweep.localCenter.Copy(localCenter);
        Transform.MultiplyVec2(this.m_xf, this.m_sweep.localCenter, this.m_sweep.c);
        this.m_sweep.c0.Copy(this.m_sweep.c);

        // Update center of mass velocity.
        Vec2.AddCrossScalarVec2(
            this.m_linearVelocity,
            this.m_angularVelocity,
            Vec2.Subtract(this.m_sweep.c, oldCenter, Vec2.s_t0),
            this.m_linearVelocity,
        );
    }

    /**
     * Get the world coordinates of a point given the local coordinates.
     *
     * @param localPoint A point on the body measured relative the the body's origin.
     * @returns The same point expressed in world coordinates.
     */
    public GetWorldPoint<T extends XY>(localPoint: Readonly<XY>, out: T): T {
        return Transform.MultiplyVec2(this.m_xf, localPoint, out);
    }

    /**
     * Get the world coordinates of a vector given the local coordinates.
     *
     * @param localVector A vector fixed in the body.
     * @returns The same vector expressed in world coordinates.
     */
    public GetWorldVector<T extends XY>(localVector: Readonly<XY>, out: T): T {
        return Rot.MultiplyVec2(this.m_xf.q, localVector, out);
    }

    /**
     * Gets a local point relative to the body's origin given a world point.
     *
     * @param a Point in world coordinates.
     * @returns The corresponding local point relative to the body's origin.
     */
    public GetLocalPoint<T extends XY>(worldPoint: Readonly<XY>, out: T): T {
        return Transform.TransposeMultiplyVec2(this.m_xf, worldPoint, out);
    }

    /**
     * Gets a local vector given a world vector.
     *
     * @param a Vector in world coordinates.
     * @returns The corresponding local vector.
     */
    public GetLocalVector<T extends XY>(worldVector: Readonly<XY>, out: T): T {
        return Rot.TransposeMultiplyVec2(this.m_xf.q, worldVector, out);
    }

    /**
     * Get the world linear velocity of a world point attached to this body.
     *
     * @param a Point in world coordinates.
     * @returns The world velocity of a point.
     */
    public GetLinearVelocityFromWorldPoint<T extends XY>(worldPoint: Readonly<XY>, out: T): T {
        return Vec2.AddCrossScalarVec2(
            this.m_linearVelocity,
            this.m_angularVelocity,
            Vec2.Subtract(worldPoint, this.m_sweep.c, Vec2.s_t0),
            out,
        );
    }

    /**
     * Get the world velocity of a local point.
     *
     * @param a Point in local coordinates.
     * @returns The world velocity of a point.
     */
    public GetLinearVelocityFromLocalPoint<T extends XY>(localPoint: Readonly<XY>, out: T): T {
        return this.GetLinearVelocityFromWorldPoint(this.GetWorldPoint(localPoint, out), out);
    }

    /**
     * Get the linear damping of the body.
     */
    public GetLinearDamping(): number {
        return this.m_linearDamping;
    }

    /**
     * Set the linear damping of the body.
     */
    public SetLinearDamping(linearDamping: number): void {
        this.m_linearDamping = linearDamping;
    }

    /**
     * Get the angular damping of the body.
     */
    public GetAngularDamping(): number {
        return this.m_angularDamping;
    }

    /**
     * Set the angular damping of the body.
     */
    public SetAngularDamping(angularDamping: number): void {
        this.m_angularDamping = angularDamping;
    }

    /**
     * Get the gravity scale of the body.
     */
    public GetGravityScale(): number {
        return this.m_gravityScale;
    }

    /**
     * Set the gravity scale of the body.
     */
    public SetGravityScale(scale: number): void {
        this.m_gravityScale = scale;
    }

    /**
     * Set the type of this body. This may alter the mass and velocity.
     */
    public SetType(type: BodyType): void {
        Assert(!this.m_world.IsLocked());

        if (this.m_type === type) {
            return;
        }

        this.m_type = type;

        this.ResetMassData();

        if (this.m_type === BodyType.Static) {
            this.m_linearVelocity.SetZero();
            this.m_angularVelocity = 0;
            this.m_sweep.a0 = this.m_sweep.a;
            this.m_sweep.c0.Copy(this.m_sweep.c);
            this.m_awakeFlag = false;
            this.SynchronizeFixtures();
        }

        this.SetAwake(true);

        this.m_force.SetZero();
        this.m_torque = 0;

        // Delete the attached contacts.
        let ce: ContactEdge | null = this.m_contactList;
        while (ce) {
            const ce0 = ce;
            ce = ce.next;
            this.m_world.m_contactManager.Destroy(ce0.contact);
        }
        this.m_contactList = null;

        // Touch the proxies so that new contacts will be created (when appropriate)
        const broadPhase = this.m_world.m_contactManager.m_broadPhase;
        for (let f: Fixture | null = this.m_fixtureList; f; f = f.m_next) {
            for (const proxy of f.m_proxies) {
                broadPhase.TouchProxy(proxy.treeNode);
            }
        }
    }

    /**
     * Get the type of this body.
     */
    public GetType(): BodyType {
        return this.m_type;
    }

    /**
     * Should this body be treated like a bullet for continuous collision detection?
     */
    public SetBullet(flag: boolean): void {
        this.m_bulletFlag = flag;
    }

    /**
     * Is this body treated like a bullet for continuous collision detection?
     */
    public IsBullet(): boolean {
        return this.m_bulletFlag;
    }

    /**
     * You can disable sleeping on this body. If you disable sleeping, the
     * body will be woken.
     */
    public SetSleepingAllowed(flag: boolean): void {
        this.m_autoSleepFlag = flag;
        if (!flag) {
            this.SetAwake(true);
        }
    }

    /**
     * Is this body allowed to sleep
     */
    public IsSleepingAllowed(): boolean {
        return this.m_autoSleepFlag;
    }

    /**
     * Set the sleep state of the body. A sleeping body has very
     * low CPU cost.
     *
     * @param flag Set to true to wake the body, false to put it to sleep.
     */
    public SetAwake(flag: boolean): void {
        if (this.m_type === BodyType.Static) {
            return;
        }
        if (flag) {
            this.m_awakeFlag = true;
            this.m_sleepTime = 0;
        } else {
            this.m_awakeFlag = false;
            this.m_sleepTime = 0;
            this.m_linearVelocity.SetZero();
            this.m_angularVelocity = 0;
            this.m_force.SetZero();
            this.m_torque = 0;
        }
    }

    /**
     * Get the sleeping state of this body.
     *
     * @returns true if the body is sleeping.
     */
    public IsAwake(): boolean {
        return this.m_awakeFlag;
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
    public SetEnabled(flag: boolean): void {
        Assert(!this.m_world.IsLocked());

        if (flag === this.IsEnabled()) {
            return;
        }

        this.m_enabledFlag = flag;

        const broadPhase = this.m_world.m_contactManager.m_broadPhase;
        if (flag) {
            // Create all proxies.
            for (let f: Fixture | null = this.m_fixtureList; f; f = f.m_next) {
                f.CreateProxies(broadPhase, this.m_xf);
            }
            // Contacts are created at the beginning of the next
            this.m_world.m_newContacts = true;
        } else {
            // Destroy all proxies.
            for (let f: Fixture | null = this.m_fixtureList; f; f = f.m_next) {
                f.DestroyProxies(broadPhase);
            }
            // Destroy the attached contacts.
            let ce: ContactEdge | null = this.m_contactList;
            while (ce) {
                const ce0 = ce;
                ce = ce.next;
                this.m_world.m_contactManager.Destroy(ce0.contact);
            }
            this.m_contactList = null;
        }
    }

    /**
     * Get the active state of the body.
     */
    public IsEnabled(): boolean {
        return this.m_enabledFlag;
    }

    /**
     * Set this body to have fixed rotation. This causes the mass
     * to be reset.
     */
    public SetFixedRotation(flag: boolean): void {
        if (this.m_fixedRotationFlag === flag) {
            return;
        }

        this.m_fixedRotationFlag = flag;

        this.m_angularVelocity = 0;

        this.ResetMassData();
    }

    /**
     * Does this body have fixed rotation?
     */
    public IsFixedRotation(): boolean {
        return this.m_fixedRotationFlag;
    }

    /**
     * Get the list of all fixtures attached to this body.
     */
    public GetFixtureList(): Fixture | null {
        return this.m_fixtureList;
    }

    /**
     * Get the list of all joints attached to this body.
     */
    public GetJointList(): JointEdge | null {
        return this.m_jointList;
    }

    /**
     * Get the list of all contacts attached to this body.
     *
     * @warning this list changes during the time step and you may
     * miss some collisions if you don't use ContactListener.
     */
    public GetContactList(): ContactEdge | null {
        return this.m_contactList;
    }

    /**
     * Get the next body in the world's body list.
     */
    public GetNext(): Body | null {
        return this.m_next;
    }

    /**
     * Get the user data pointer that was provided in the body definition.
     */
    public GetUserData(): any {
        return this.m_userData;
    }

    /**
     * Set the user data. Use this to store your application specific data.
     */
    public SetUserData(data: any): void {
        this.m_userData = data;
    }

    /**
     * Get the parent world of this body.
     */
    public GetWorld(): World {
        return this.m_world;
    }

    private static SynchronizeFixtures_s_xf1 = new Transform();

    /** @internal */
    public SynchronizeFixtures(): void {
        const broadPhase = this.m_world.m_contactManager.m_broadPhase;
        if (this.m_awakeFlag) {
            const xf1 = Body.SynchronizeFixtures_s_xf1;
            xf1.q.Set(this.m_sweep.a0);
            Rot.MultiplyVec2(xf1.q, this.m_sweep.localCenter, xf1.p);
            Vec2.Subtract(this.m_sweep.c0, xf1.p, xf1.p);

            for (let f: Fixture | null = this.m_fixtureList; f; f = f.m_next) {
                f.Synchronize(broadPhase, xf1, this.m_xf);
            }
        } else {
            for (let f: Fixture | null = this.m_fixtureList; f; f = f.m_next) {
                f.Synchronize(broadPhase, this.m_xf, this.m_xf);
            }
        }
    }

    /** @internal */
    public SynchronizeTransform(): void {
        this.m_xf.q.Set(this.m_sweep.a);
        Rot.MultiplyVec2(this.m_xf.q, this.m_sweep.localCenter, this.m_xf.p);
        Vec2.Subtract(this.m_sweep.c, this.m_xf.p, this.m_xf.p);
    }

    /**
     * This is used to prevent connected bodies from colliding.
     * It may lie, depending on the collideConnected flag.
     *
     * @internal
     */
    public ShouldCollide(other: Body): boolean {
        // At least one body should be dynamic.
        if (this.m_type !== BodyType.Dynamic && other.m_type !== BodyType.Dynamic) {
            return false;
        }
        return this.ShouldCollideConnected(other);
    }

    private ShouldCollideConnected(other: Body): boolean {
        // Does a joint prevent collision?
        for (let jn: JointEdge | null = this.m_jointList; jn; jn = jn.next) {
            if (jn.other === other) {
                if (!jn.joint.m_collideConnected) {
                    return false;
                }
            }
        }

        return true;
    }

    /** @internal */
    public Advance(alpha: number): void {
        // Advance to the new safe time. This doesn't sync the broad-phase.
        this.m_sweep.Advance(alpha);
        this.m_sweep.c.Copy(this.m_sweep.c0);
        this.m_sweep.a = this.m_sweep.a0;
        this.m_xf.q.Set(this.m_sweep.a);
        Rot.MultiplyVec2(this.m_xf.q, this.m_sweep.localCenter, this.m_xf.p);
        Vec2.Subtract(this.m_sweep.c, this.m_xf.p, this.m_xf.p);
    }
}
