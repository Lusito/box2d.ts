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

import { Clamp, Vec2, Mat22, Rot, XY } from "../common/b2_math";
import { Joint, JointDef, JointType, IJointDef } from "./b2_joint";
import { SolverData } from "./b2_time_step";
import { Body } from "./b2_body";

const temp = {
    qA: new Rot(),
    qB: new Rot(),
    lalcA: new Vec2(),
    lalcB: new Vec2(),
    Cdot: new Vec2(),
    impulse: new Vec2(),
    oldImpulse: new Vec2(),
};

export interface IFrictionJointDef extends IJointDef {
    localAnchorA: XY;

    localAnchorB: XY;

    maxForce?: number;

    maxTorque?: number;
}

/**
 * Friction joint definition.
 */
export class FrictionJointDef extends JointDef implements IFrictionJointDef {
    /** The local anchor point relative to bodyA's origin. */
    public readonly localAnchorA = new Vec2();

    /** The local anchor point relative to bodyB's origin. */
    public readonly localAnchorB = new Vec2();

    /** The maximum friction force in N. */
    public maxForce = 0;

    /** The maximum friction torque in N-m. */
    public maxTorque = 0;

    public constructor() {
        super(JointType.Friction);
    }

    public Initialize(bA: Body, bB: Body, anchor: Vec2): void {
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
export class FrictionJoint extends Joint {
    protected readonly m_localAnchorA = new Vec2();

    protected readonly m_localAnchorB = new Vec2();

    // Solver shared
    protected readonly m_linearImpulse = new Vec2();

    protected m_angularImpulse = 0;

    protected m_maxForce = 0;

    protected m_maxTorque = 0;

    // Solver temp
    protected m_indexA = 0;

    protected m_indexB = 0;

    protected readonly m_rA = new Vec2();

    protected readonly m_rB = new Vec2();

    protected readonly m_localCenterA = new Vec2();

    protected readonly m_localCenterB = new Vec2();

    protected m_invMassA = 0;

    protected m_invMassB = 0;

    protected m_invIA = 0;

    protected m_invIB = 0;

    protected readonly m_linearMass = new Mat22();

    protected m_angularMass = 0;

    /** @internal protected */
    public constructor(def: IFrictionJointDef) {
        super(def);

        this.m_localAnchorA.Copy(def.localAnchorA);
        this.m_localAnchorB.Copy(def.localAnchorB);

        this.m_linearImpulse.SetZero();
        this.m_maxForce = def.maxForce ?? 0;
        this.m_maxTorque = def.maxTorque ?? 0;
    }

    public InitVelocityConstraints(data: SolverData): void {
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
        Rot.MultiplyVec2(qA, Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), this.m_rA);
        Rot.MultiplyVec2(qB, Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), this.m_rB);

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
            wA -= iA * (Vec2.Cross(this.m_rA, P) + this.m_angularImpulse);
            vB.AddScaled(mB, P);
            wB += iB * (Vec2.Cross(this.m_rB, P) + this.m_angularImpulse);
        } else {
            this.m_linearImpulse.SetZero();
            this.m_angularImpulse = 0;
        }

        data.velocities[this.m_indexA].w = wA;
        data.velocities[this.m_indexB].w = wB;
    }

    public SolveVelocityConstraints(data: SolverData): void {
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
            this.m_angularImpulse = Clamp(this.m_angularImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.m_angularImpulse - oldImpulse;

            wA -= iA * impulse;
            wB += iB * impulse;
        }

        // Solve linear friction
        {
            const { Cdot, impulse, oldImpulse } = temp;
            Vec2.Subtract(
                Vec2.AddCrossScalarVec2(vB, wB, this.m_rB, Vec2.s_t0),
                Vec2.AddCrossScalarVec2(vA, wA, this.m_rA, Vec2.s_t1),
                Cdot,
            );

            Mat22.MultiplyVec2(this.m_linearMass, Cdot, impulse).Negate();
            oldImpulse.Copy(this.m_linearImpulse);
            this.m_linearImpulse.Add(impulse);

            const maxImpulse = h * this.m_maxForce;

            if (this.m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse) {
                this.m_linearImpulse.Normalize();
                this.m_linearImpulse.Scale(maxImpulse);
            }

            Vec2.Subtract(this.m_linearImpulse, oldImpulse, impulse);

            vA.SubtractScaled(mA, impulse);
            wA -= iA * Vec2.Cross(this.m_rA, impulse);

            vB.AddScaled(mB, impulse);
            wB += iB * Vec2.Cross(this.m_rB, impulse);
        }

        data.velocities[this.m_indexA].w = wA;
        data.velocities[this.m_indexB].w = wB;
    }

    public SolvePositionConstraints(_data: SolverData): boolean {
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

    public GetLocalAnchorA(): Readonly<Vec2> {
        return this.m_localAnchorA;
    }

    public GetLocalAnchorB(): Readonly<Vec2> {
        return this.m_localAnchorB;
    }

    public SetMaxForce(force: number): void {
        // DEBUG: Assert(Number.isFinite(force) && force >= 0);
        this.m_maxForce = force;
    }

    public GetMaxForce(): number {
        return this.m_maxForce;
    }

    public SetMaxTorque(torque: number): void {
        // DEBUG: Assert(Number.isFinite(torque) && torque >= 0);
        this.m_maxTorque = torque;
    }

    public GetMaxTorque(): number {
        return this.m_maxTorque;
    }
}
