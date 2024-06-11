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

import { b2Clamp, b2Vec2, b2Mat22, b2Rot, XY } from "../common/b2_math";
import { b2Joint, b2JointDef, b2JointType, b2IJointDef } from "./b2_joint";
import { b2SolverData } from "./b2_time_step";
import { b2Body } from "./b2_body";

// Point-to-point constraint
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)
// Angle constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

const temp = {
    qA: new b2Rot(),
    qB: new b2Rot(),
    lalcA: new b2Vec2(),
    lalcB: new b2Vec2(),
    Cdot: new b2Vec2(),
    impulse: new b2Vec2(),
    oldImpulse: new b2Vec2(),
};

export interface b2IFrictionJointDef extends b2IJointDef {
    localAnchorA: XY;

    localAnchorB: XY;

    maxForce?: number;

    maxTorque?: number;
}

/**
 * Friction joint definition.
 */
export class b2FrictionJointDef extends b2JointDef implements b2IFrictionJointDef {
    /** The local anchor point relative to bodyA's origin. */
    public readonly localAnchorA = new b2Vec2();

    /** The local anchor point relative to bodyB's origin. */
    public readonly localAnchorB = new b2Vec2();

    /** The maximum friction force in N. */
    public maxForce = 0;

    /** The maximum friction torque in N-m. */
    public maxTorque = 0;

    public constructor() {
        super(b2JointType.e_frictionJoint);
    }

    /**
     * Initialize the bodies, anchors, axis, and reference angle using the world
     * anchor and world axis.
     */
    public Initialize(bA: b2Body, bB: b2Body, anchor: b2Vec2): void {
        this.bodyA = bA;
        this.bodyB = bB;
        this.bodyA.GetLocalPoint(anchor, this.localAnchorA);
        this.bodyB.GetLocalPoint(anchor, this.localAnchorB);
    }
}

/**
 * Friction joint. This is used for top-down friction.
 * It provides 2D translational friction and angular friction.
 */
export class b2FrictionJoint extends b2Joint {
    protected readonly m_localAnchorA = new b2Vec2();

    protected readonly m_localAnchorB = new b2Vec2();

    // Solver shared
    protected readonly m_linearImpulse = new b2Vec2();

    protected m_angularImpulse = 0;

    protected m_maxForce = 0;

    protected m_maxTorque = 0;

    // Solver temp
    protected m_indexA = 0;

    protected m_indexB = 0;

    protected readonly m_rA = new b2Vec2();

    protected readonly m_rB = new b2Vec2();

    protected readonly m_localCenterA = new b2Vec2();

    protected readonly m_localCenterB = new b2Vec2();

    protected m_invMassA = 0;

    protected m_invMassB = 0;

    protected m_invIA = 0;

    protected m_invIB = 0;

    protected readonly m_linearMass = new b2Mat22();

    protected m_angularMass = 0;

    /** @internal protected */
    public constructor(def: b2IFrictionJointDef) {
        super(def);

        this.m_localAnchorA.Copy(def.localAnchorA);
        this.m_localAnchorB.Copy(def.localAnchorB);

        this.m_linearImpulse.SetZero();
        this.m_maxForce = def.maxForce ?? 0;
        this.m_maxTorque = def.maxTorque ?? 0;
    }

    public InitVelocityConstraints(data: b2SolverData): void {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterA.Copy(this.m_bodyA.m_sweep.localCenter);
        this.m_localCenterB.Copy(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassA = this.m_bodyA.m_invMass;
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIA = this.m_bodyA.m_invI;
        this.m_invIB = this.m_bodyB.m_invI;

        const aA = data.positions[this.m_indexA].a;
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;

        const aB = data.positions[this.m_indexB].a;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;

        const { qA, qB, lalcA, lalcB } = temp;
        qA.Set(aA);
        qB.Set(aB);

        // Compute the effective mass matrix.
        b2Rot.MultiplyVec2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), this.m_rA);
        b2Rot.MultiplyVec2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), this.m_rB);

        // J = [-I -r1_skew I r2_skew]
        //     [ 0       -1 0       1]
        // r_skew = [-ry; rx]

        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
        //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
        //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

        const mA = this.m_invMassA;
        const mB = this.m_invMassB;
        const iA = this.m_invIA;
        const iB = this.m_invIB;

        const K = this.m_linearMass;
        K.ex.x = mA + mB + iA * this.m_rA.y * this.m_rA.y + iB * this.m_rB.y * this.m_rB.y;
        K.ex.y = -iA * this.m_rA.x * this.m_rA.y - iB * this.m_rB.x * this.m_rB.y;
        K.ey.x = K.ex.y;
        K.ey.y = mA + mB + iA * this.m_rA.x * this.m_rA.x + iB * this.m_rB.x * this.m_rB.x;

        K.Inverse();

        this.m_angularMass = iA + iB;
        if (this.m_angularMass > 0) {
            this.m_angularMass = 1 / this.m_angularMass;
        }

        if (data.step.warmStarting) {
            // Scale impulses to support a variable time step.
            this.m_linearImpulse.Scale(data.step.dtRatio);
            this.m_angularImpulse *= data.step.dtRatio;

            const P = this.m_linearImpulse;
            vA.SubtractScaled(mA, P);
            wA -= iA * (b2Vec2.Cross(this.m_rA, P) + this.m_angularImpulse);
            vB.AddScaled(mB, P);
            wB += iB * (b2Vec2.Cross(this.m_rB, P) + this.m_angularImpulse);
        } else {
            this.m_linearImpulse.SetZero();
            this.m_angularImpulse = 0;
        }

        data.velocities[this.m_indexA].w = wA;
        data.velocities[this.m_indexB].w = wB;
    }

    public SolveVelocityConstraints(data: b2SolverData): void {
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;

        const mA = this.m_invMassA;
        const mB = this.m_invMassB;
        const iA = this.m_invIA;
        const iB = this.m_invIB;

        const h = data.step.dt;

        // Solve angular friction
        {
            const Cdot = wB - wA;
            let impulse = -this.m_angularMass * Cdot;

            const oldImpulse = this.m_angularImpulse;
            const maxImpulse = h * this.m_maxTorque;
            this.m_angularImpulse = b2Clamp(this.m_angularImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.m_angularImpulse - oldImpulse;

            wA -= iA * impulse;
            wB += iB * impulse;
        }

        // Solve linear friction
        {
            const { Cdot, impulse, oldImpulse } = temp;
            b2Vec2.Subtract(
                b2Vec2.AddCrossScalarVec2(vB, wB, this.m_rB, b2Vec2.s_t0),
                b2Vec2.AddCrossScalarVec2(vA, wA, this.m_rA, b2Vec2.s_t1),
                Cdot,
            );

            b2Mat22.MultiplyVec2(this.m_linearMass, Cdot, impulse).Negate();
            oldImpulse.Copy(this.m_linearImpulse);
            this.m_linearImpulse.Add(impulse);

            const maxImpulse = h * this.m_maxForce;

            if (this.m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse) {
                this.m_linearImpulse.Normalize();
                this.m_linearImpulse.Scale(maxImpulse);
            }

            b2Vec2.Subtract(this.m_linearImpulse, oldImpulse, impulse);

            vA.SubtractScaled(mA, impulse);
            wA -= iA * b2Vec2.Cross(this.m_rA, impulse);

            vB.AddScaled(mB, impulse);
            wB += iB * b2Vec2.Cross(this.m_rB, impulse);
        }

        data.velocities[this.m_indexA].w = wA;
        data.velocities[this.m_indexB].w = wB;
    }

    public SolvePositionConstraints(_data: b2SolverData): boolean {
        return true;
    }

    public GetAnchorA<T extends XY>(out: T): T {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    }

    public GetAnchorB<T extends XY>(out: T): T {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }

    public GetReactionForce<T extends XY>(inv_dt: number, out: T): T {
        out.x = inv_dt * this.m_linearImpulse.x;
        out.y = inv_dt * this.m_linearImpulse.y;
        return out;
    }

    public GetReactionTorque(inv_dt: number): number {
        return inv_dt * this.m_angularImpulse;
    }

    /** The local anchor point relative to bodyA's origin. */
    public GetLocalAnchorA(): Readonly<b2Vec2> {
        return this.m_localAnchorA;
    }

    /** The local anchor point relative to bodyB's origin. */
    public GetLocalAnchorB(): Readonly<b2Vec2> {
        return this.m_localAnchorB;
    }

    /** Set the maximum friction force in N. */
    public SetMaxForce(force: number): void {
        // DEBUG: b2Assert(Number.isFinite(force) && force >= 0);
        this.m_maxForce = force;
    }

    /** Get the maximum friction force in N. */
    public GetMaxForce(): number {
        return this.m_maxForce;
    }

    /** Set the maximum friction torque in N*m. */
    public SetMaxTorque(torque: number): void {
        // DEBUG: b2Assert(Number.isFinite(torque) && torque >= 0);
        this.m_maxTorque = torque;
    }

    /** Get the maximum friction torque in N*m. */
    public GetMaxTorque(): number {
        return this.m_maxTorque;
    }
}
