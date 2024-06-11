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

// DEBUG: import { b2Assert } from "../common/b2_common";
import { b2Clamp, b2Vec2, b2Mat22, b2Rot, XY } from "../common/b2_math";
import { b2Body } from "./b2_body";
import { b2Joint, b2JointDef, b2JointType, b2IJointDef } from "./b2_joint";
import { b2SolverData } from "./b2_time_step";

// Point-to-point constraint
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)
//
// r1 = offset - c1
// r2 = -c2

// Angle constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

const temp = {
    qA: new b2Rot(),
    qB: new b2Rot(),
    K: new b2Mat22(),
    Cdot: new b2Vec2(),
    impulse: new b2Vec2(),
    oldImpulse: new b2Vec2(),
};

export interface b2IMotorJointDef extends b2IJointDef {
    linearOffset?: XY;

    angularOffset?: number;

    maxForce?: number;

    maxTorque?: number;

    correctionFactor?: number;
}

/**
 * Motor joint definition.
 */
export class b2MotorJointDef extends b2JointDef implements b2IMotorJointDef {
    /** Position of bodyB minus the position of bodyA, in bodyA's frame, in meters. */
    public readonly linearOffset = new b2Vec2();

    /** The bodyB angle minus bodyA angle in radians. */
    public angularOffset = 0;

    /** The maximum motor force in N. */
    public maxForce = 1;

    /** The maximum motor torque in N-m. */
    public maxTorque = 1;

    /** Position correction factor in the range [0,1]. */
    public correctionFactor = 0.3;

    public constructor() {
        super(b2JointType.e_motorJoint);
    }

    /** Initialize the bodies and offsets using the current transforms. */
    public Initialize(bodyA: b2Body, bodyB: b2Body): void {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.bodyA.GetLocalPoint(bodyB.GetPosition(), this.linearOffset);

        const angleA = bodyA.GetAngle();
        const angleB = bodyB.GetAngle();
        this.angularOffset = angleB - angleA;
    }
}

/**
 * A motor joint is used to control the relative motion
 * between two bodies. A typical usage is to control the movement
 * of a dynamic body with respect to the ground.
 */
export class b2MotorJoint extends b2Joint {
    // Solver shared
    protected readonly m_linearOffset = new b2Vec2();

    protected m_angularOffset: number;

    protected readonly m_linearImpulse = new b2Vec2();

    protected m_angularImpulse = 0;

    protected m_maxForce: number;

    protected m_maxTorque: number;

    protected m_correctionFactor: number;

    // Solver temp
    protected m_indexA = 0;

    protected m_indexB = 0;

    protected readonly m_rA = new b2Vec2();

    protected readonly m_rB = new b2Vec2();

    protected readonly m_localCenterA = new b2Vec2();

    protected readonly m_localCenterB = new b2Vec2();

    protected readonly m_linearError = new b2Vec2();

    protected m_angularError = 0;

    protected m_invMassA = 0;

    protected m_invMassB = 0;

    protected m_invIA = 0;

    protected m_invIB = 0;

    protected readonly m_linearMass = new b2Mat22();

    protected m_angularMass = 0;

    /** @internal protected */
    public constructor(def: b2IMotorJointDef) {
        super(def);

        this.m_linearOffset.Copy(def.linearOffset ?? b2Vec2.ZERO);
        this.m_angularOffset = def.angularOffset ?? 0;
        this.m_linearImpulse.SetZero();
        this.m_maxForce = def.maxForce ?? 1;
        this.m_maxTorque = def.maxTorque ?? 1;
        this.m_correctionFactor = def.correctionFactor ?? 0.3;
    }

    public GetAnchorA<T extends XY>(out: T): T {
        const pos = this.m_bodyA.GetPosition();
        out.x = pos.x;
        out.y = pos.y;
        return out;
    }

    public GetAnchorB<T extends XY>(out: T): T {
        const pos = this.m_bodyB.GetPosition();
        out.x = pos.x;
        out.y = pos.y;
        return out;
    }

    public GetReactionForce<T extends XY>(inv_dt: number, out: T): T {
        return b2Vec2.Scale(inv_dt, this.m_linearImpulse, out);
    }

    public GetReactionTorque(inv_dt: number): number {
        return inv_dt * this.m_angularImpulse;
    }

    /** Set the target linear offset, in frame A, in meters. */
    public SetLinearOffset(linearOffset: b2Vec2): void {
        if (!b2Vec2.Equals(linearOffset, this.m_linearOffset)) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_linearOffset.Copy(linearOffset);
        }
    }

    /** Get the target linear offset, in frame A, in meters. */
    public GetLinearOffset() {
        return this.m_linearOffset;
    }

    /** Set the target angular offset, in radians. */
    public SetAngularOffset(angularOffset: number): void {
        if (angularOffset !== this.m_angularOffset) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_angularOffset = angularOffset;
        }
    }

    /** Get the target angular offset, in radians. */
    public GetAngularOffset() {
        return this.m_angularOffset;
    }

    /** Set the maximum friction force in N. */
    public SetMaxForce(force: number): void {
        // DEBUG: b2Assert(Number.isFinite(force) && force >= 0);
        this.m_maxForce = force;
    }

    /** Get the maximum friction force in N. */
    public GetMaxForce() {
        return this.m_maxForce;
    }

    /** Set the maximum friction torque in N*m. */
    public SetMaxTorque(torque: number): void {
        // DEBUG: b2Assert(Number.isFinite(torque) && torque >= 0);
        this.m_maxTorque = torque;
    }

    /** Get the maximum friction torque in N*m. */
    public GetMaxTorque() {
        return this.m_maxTorque;
    }

    /** Get the position correction factor in the range [0,1]. */
    public GetCorrectionFactor() {
        return this.m_correctionFactor;
    }

    /** Set the position correction factor in the range [0,1]. */
    public SetCorrectionFactor(factor: number) {
        // DEBUG: b2Assert(Number.isFinite(factor) && factor >= 0 && factor <= 1);
        this.m_correctionFactor = factor;
    }

    /** @internal protected */
    public InitVelocityConstraints(data: b2SolverData): void {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterA.Copy(this.m_bodyA.m_sweep.localCenter);
        this.m_localCenterB.Copy(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassA = this.m_bodyA.m_invMass;
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIA = this.m_bodyA.m_invI;
        this.m_invIB = this.m_bodyB.m_invI;

        const cA = data.positions[this.m_indexA].c;
        const aA = data.positions[this.m_indexA].a;
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;

        const cB = data.positions[this.m_indexB].c;
        const aB = data.positions[this.m_indexB].a;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;

        const { qA, qB } = temp;
        qA.Set(aA);
        qB.Set(aB);

        // Compute the effective mass matrix.
        const rA = b2Rot.MultiplyVec2(
            qA,
            b2Vec2.Subtract(this.m_linearOffset, this.m_localCenterA, b2Vec2.s_t0),
            this.m_rA,
        );
        const rB = b2Rot.MultiplyVec2(qB, b2Vec2.Negate(this.m_localCenterB, b2Vec2.s_t0), this.m_rB);

        // J = [-I -r1_skew I r2_skew]
        // r_skew = [-ry; rx]

        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
        //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
        //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

        const mA = this.m_invMassA;
        const mB = this.m_invMassB;
        const iA = this.m_invIA;
        const iB = this.m_invIB;

        // Upper 2 by 2 of K for point to point
        const K = this.m_linearMass;
        K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
        K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
        K.ey.x = K.ex.y;
        K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;

        K.Inverse();

        this.m_angularMass = iA + iB;
        if (this.m_angularMass > 0) {
            this.m_angularMass = 1 / this.m_angularMass;
        }

        b2Vec2.Subtract(b2Vec2.Add(cB, rB, b2Vec2.s_t0), b2Vec2.Add(cA, rA, b2Vec2.s_t1), this.m_linearError);
        this.m_angularError = aB - aA - this.m_angularOffset;

        if (data.step.warmStarting) {
            // Scale impulses to support a variable time step.
            this.m_linearImpulse.Scale(data.step.dtRatio);
            this.m_angularImpulse *= data.step.dtRatio;

            const P = this.m_linearImpulse;
            vA.SubtractScaled(mA, P);
            wA -= iA * (b2Vec2.Cross(rA, P) + this.m_angularImpulse);
            vB.AddScaled(mB, P);
            wB += iB * (b2Vec2.Cross(rB, P) + this.m_angularImpulse);
        } else {
            this.m_linearImpulse.SetZero();
            this.m_angularImpulse = 0;
        }

        data.velocities[this.m_indexA].w = wA;
        data.velocities[this.m_indexB].w = wB;
    }

    /** @internal protected */
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
        const inv_h = data.step.inv_dt;

        // Solve angular friction
        {
            const Cdot = wB - wA + inv_h * this.m_correctionFactor * this.m_angularError;
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
            const { impulse, oldImpulse, Cdot } = temp;

            b2Vec2.AddScaled(
                b2Vec2.Subtract(
                    b2Vec2.AddCrossScalarVec2(vB, wB, this.m_rB, b2Vec2.s_t0),
                    b2Vec2.AddCrossScalarVec2(vA, wA, this.m_rA, b2Vec2.s_t1),
                    b2Vec2.s_t2,
                ),
                inv_h * this.m_correctionFactor,
                this.m_linearError,
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

    /** @internal protected */
    public SolvePositionConstraints(_data: b2SolverData): boolean {
        return true;
    }
}
