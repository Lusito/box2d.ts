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
import { clamp, Vec2, Mat22, Rot, XY } from "../common/b2_math";
import { Body } from "./b2_body";
import { Joint, JointDef, JointType, IJointDef } from "./b2_joint";
import { SolverData } from "./b2_time_step";

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
    qA: new Rot(),
    qB: new Rot(),
    K: new Mat22(),
    Cdot: new Vec2(),
    impulse: new Vec2(),
    oldImpulse: new Vec2(),
};

export interface IMotorJointDef extends IJointDef {
    linearOffset?: XY;

    angularOffset?: number;

    maxForce?: number;

    maxTorque?: number;

    correctionFactor?: number;
}

/**
 * Motor joint definition.
 */
export class MotorJointDef extends JointDef implements IMotorJointDef {
    /** Position of bodyB minus the position of bodyA, in bodyA's frame, in meters. */
    public readonly linearOffset = new Vec2();

    /** The bodyB angle minus bodyA angle in radians. */
    public angularOffset = 0;

    /** The maximum motor force in N. */
    public maxForce = 1;

    /** The maximum motor torque in N-m. */
    public maxTorque = 1;

    /** Position correction factor in the range [0,1]. */
    public correctionFactor = 0.3;

    public constructor() {
        super(JointType.Motor);
    }

    public initialize(bodyA: Body, bodyB: Body): void {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.bodyA.getLocalPoint(bodyB.getPosition(), this.linearOffset);

        const angleA = bodyA.getAngle();
        const angleB = bodyB.getAngle();
        this.angularOffset = angleB - angleA;
    }
}

/**
 * A motor joint is used to control the relative motion
 * between two bodies. A typical usage is to control the movement
 * of a dynamic body with respect to the ground.
 */
export class MotorJoint extends Joint {
    // Solver shared
    protected readonly m_linearOffset = new Vec2();

    protected m_angularOffset: number;

    protected readonly m_linearImpulse = new Vec2();

    protected m_angularImpulse = 0;

    protected m_maxForce: number;

    protected m_maxTorque: number;

    protected m_correctionFactor: number;

    // Solver temp
    protected m_indexA = 0;

    protected m_indexB = 0;

    protected readonly m_rA = new Vec2();

    protected readonly m_rB = new Vec2();

    protected readonly m_localCenterA = new Vec2();

    protected readonly m_localCenterB = new Vec2();

    protected readonly m_linearError = new Vec2();

    protected m_angularError = 0;

    protected m_invMassA = 0;

    protected m_invMassB = 0;

    protected m_invIA = 0;

    protected m_invIB = 0;

    protected readonly m_linearMass = new Mat22();

    protected m_angularMass = 0;

    /** @internal protected */
    public constructor(def: IMotorJointDef) {
        super(def);

        this.m_linearOffset.copy(def.linearOffset ?? Vec2.ZERO);
        this.m_angularOffset = def.angularOffset ?? 0;
        this.m_linearImpulse.setZero();
        this.m_maxForce = def.maxForce ?? 1;
        this.m_maxTorque = def.maxTorque ?? 1;
        this.m_correctionFactor = def.correctionFactor ?? 0.3;
    }

    public getAnchorA<T extends XY>(out: T): T {
        const pos = this.m_bodyA.getPosition();
        out.x = pos.x;
        out.y = pos.y;
        return out;
    }

    public getAnchorB<T extends XY>(out: T): T {
        const pos = this.m_bodyB.getPosition();
        out.x = pos.x;
        out.y = pos.y;
        return out;
    }

    public getReactionForce<T extends XY>(inv_dt: number, out: T): T {
        return Vec2.scale(inv_dt, this.m_linearImpulse, out);
    }

    public getReactionTorque(inv_dt: number): number {
        return inv_dt * this.m_angularImpulse;
    }

    public setLinearOffset(linearOffset: Vec2): void {
        if (!Vec2.equals(linearOffset, this.m_linearOffset)) {
            this.m_bodyA.setAwake(true);
            this.m_bodyB.setAwake(true);
            this.m_linearOffset.copy(linearOffset);
        }
    }

    public getLinearOffset() {
        return this.m_linearOffset;
    }

    public setAngularOffset(angularOffset: number): void {
        if (angularOffset !== this.m_angularOffset) {
            this.m_bodyA.setAwake(true);
            this.m_bodyB.setAwake(true);
            this.m_angularOffset = angularOffset;
        }
    }

    public getAngularOffset() {
        return this.m_angularOffset;
    }

    public setMaxForce(force: number): void {
        // DEBUG: assert(Number.isFinite(force) && force >= 0);
        this.m_maxForce = force;
    }

    public getMaxForce() {
        return this.m_maxForce;
    }

    public setMaxTorque(torque: number): void {
        // DEBUG: assert(Number.isFinite(torque) && torque >= 0);
        this.m_maxTorque = torque;
    }

    public getMaxTorque() {
        return this.m_maxTorque;
    }

    public getCorrectionFactor() {
        return this.m_correctionFactor;
    }

    public setCorrectionFactor(factor: number) {
        // DEBUG: assert(Number.isFinite(factor) && factor >= 0 && factor <= 1);
        this.m_correctionFactor = factor;
    }

    /** @internal protected */
    public initVelocityConstraints(data: SolverData): void {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterA.copy(this.m_bodyA.m_sweep.localCenter);
        this.m_localCenterB.copy(this.m_bodyB.m_sweep.localCenter);
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
        qA.set(aA);
        qB.set(aB);

        // Compute the effective mass matrix.
        const rA = Rot.multiplyVec2(qA, Vec2.subtract(this.m_linearOffset, this.m_localCenterA, Vec2.s_t0), this.m_rA);
        const rB = Rot.multiplyVec2(qB, Vec2.negate(this.m_localCenterB, Vec2.s_t0), this.m_rB);

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

        K.inverse();

        this.m_angularMass = iA + iB;
        if (this.m_angularMass > 0) {
            this.m_angularMass = 1 / this.m_angularMass;
        }

        Vec2.subtract(Vec2.add(cB, rB, Vec2.s_t0), Vec2.add(cA, rA, Vec2.s_t1), this.m_linearError);
        this.m_angularError = aB - aA - this.m_angularOffset;

        if (data.step.warmStarting) {
            // Scale impulses to support a variable time step.
            this.m_linearImpulse.scale(data.step.dtRatio);
            this.m_angularImpulse *= data.step.dtRatio;

            const P = this.m_linearImpulse;
            vA.subtractScaled(mA, P);
            wA -= iA * (Vec2.cross(rA, P) + this.m_angularImpulse);
            vB.addScaled(mB, P);
            wB += iB * (Vec2.cross(rB, P) + this.m_angularImpulse);
        } else {
            this.m_linearImpulse.setZero();
            this.m_angularImpulse = 0;
        }

        data.velocities[this.m_indexA].w = wA;
        data.velocities[this.m_indexB].w = wB;
    }

    /** @internal protected */
    public solveVelocityConstraints(data: SolverData): void {
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
            this.m_angularImpulse = clamp(this.m_angularImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.m_angularImpulse - oldImpulse;

            wA -= iA * impulse;
            wB += iB * impulse;
        }

        // Solve linear friction
        {
            const { impulse, oldImpulse, Cdot } = temp;

            Vec2.addScaled(
                Vec2.subtract(
                    Vec2.addCrossScalarVec2(vB, wB, this.m_rB, Vec2.s_t0),
                    Vec2.addCrossScalarVec2(vA, wA, this.m_rA, Vec2.s_t1),
                    Vec2.s_t2,
                ),
                inv_h * this.m_correctionFactor,
                this.m_linearError,
                Cdot,
            );

            Mat22.multiplyVec2(this.m_linearMass, Cdot, impulse).negate();
            oldImpulse.copy(this.m_linearImpulse);
            this.m_linearImpulse.add(impulse);

            const maxImpulse = h * this.m_maxForce;

            if (this.m_linearImpulse.lengthSquared() > maxImpulse * maxImpulse) {
                this.m_linearImpulse.normalize();
                this.m_linearImpulse.scale(maxImpulse);
            }

            Vec2.subtract(this.m_linearImpulse, oldImpulse, impulse);

            vA.subtractScaled(mA, impulse);
            wA -= iA * Vec2.cross(this.m_rA, impulse);

            vB.addScaled(mB, impulse);
            wB += iB * Vec2.cross(this.m_rB, impulse);
        }

        data.velocities[this.m_indexA].w = wA;
        data.velocities[this.m_indexB].w = wB;
    }

    /** @internal protected */
    public solvePositionConstraints(_data: SolverData): boolean {
        return true;
    }
}
