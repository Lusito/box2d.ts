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
    protected readonly linearOffset = new Vec2();

    protected angularOffset: number;

    protected readonly linearImpulse = new Vec2();

    protected angularImpulse = 0;

    protected maxForce: number;

    protected maxTorque: number;

    protected correctionFactor: number;

    // Solver temp
    protected indexA = 0;

    protected indexB = 0;

    protected readonly rA = new Vec2();

    protected readonly rB = new Vec2();

    protected readonly localCenterA = new Vec2();

    protected readonly localCenterB = new Vec2();

    protected readonly linearError = new Vec2();

    protected angularError = 0;

    protected invMassA = 0;

    protected invMassB = 0;

    protected invIA = 0;

    protected invIB = 0;

    protected readonly linearMass = new Mat22();

    protected angularMass = 0;

    /** @internal protected */
    public constructor(def: IMotorJointDef) {
        super(def);

        this.linearOffset.copy(def.linearOffset ?? Vec2.ZERO);
        this.angularOffset = def.angularOffset ?? 0;
        this.linearImpulse.setZero();
        this.maxForce = def.maxForce ?? 1;
        this.maxTorque = def.maxTorque ?? 1;
        this.correctionFactor = def.correctionFactor ?? 0.3;
    }

    public getAnchorA<T extends XY>(out: T): T {
        const pos = this.bodyA.getPosition();
        out.x = pos.x;
        out.y = pos.y;
        return out;
    }

    public getAnchorB<T extends XY>(out: T): T {
        const pos = this.bodyB.getPosition();
        out.x = pos.x;
        out.y = pos.y;
        return out;
    }

    public getReactionForce<T extends XY>(inv_dt: number, out: T): T {
        return Vec2.scale(inv_dt, this.linearImpulse, out);
    }

    public getReactionTorque(inv_dt: number): number {
        return inv_dt * this.angularImpulse;
    }

    public setLinearOffset(linearOffset: Vec2): void {
        if (!Vec2.equals(linearOffset, this.linearOffset)) {
            this.bodyA.setAwake(true);
            this.bodyB.setAwake(true);
            this.linearOffset.copy(linearOffset);
        }
    }

    public getLinearOffset() {
        return this.linearOffset;
    }

    public setAngularOffset(angularOffset: number): void {
        if (angularOffset !== this.angularOffset) {
            this.bodyA.setAwake(true);
            this.bodyB.setAwake(true);
            this.angularOffset = angularOffset;
        }
    }

    public getAngularOffset() {
        return this.angularOffset;
    }

    public setMaxForce(force: number): void {
        // DEBUG: assert(Number.isFinite(force) && force >= 0);
        this.maxForce = force;
    }

    public getMaxForce() {
        return this.maxForce;
    }

    public setMaxTorque(torque: number): void {
        // DEBUG: assert(Number.isFinite(torque) && torque >= 0);
        this.maxTorque = torque;
    }

    public getMaxTorque() {
        return this.maxTorque;
    }

    public getCorrectionFactor() {
        return this.correctionFactor;
    }

    public setCorrectionFactor(factor: number) {
        // DEBUG: assert(Number.isFinite(factor) && factor >= 0 && factor <= 1);
        this.correctionFactor = factor;
    }

    /** @internal protected */
    public initVelocityConstraints(data: SolverData): void {
        this.indexA = this.bodyA.islandIndex;
        this.indexB = this.bodyB.islandIndex;
        this.localCenterA.copy(this.bodyA.sweep.localCenter);
        this.localCenterB.copy(this.bodyB.sweep.localCenter);
        this.invMassA = this.bodyA.invMass;
        this.invMassB = this.bodyB.invMass;
        this.invIA = this.bodyA.invI;
        this.invIB = this.bodyB.invI;

        const cA = data.positions[this.indexA].c;
        const aA = data.positions[this.indexA].a;
        const vA = data.velocities[this.indexA].v;
        let wA = data.velocities[this.indexA].w;

        const cB = data.positions[this.indexB].c;
        const aB = data.positions[this.indexB].a;
        const vB = data.velocities[this.indexB].v;
        let wB = data.velocities[this.indexB].w;

        const { qA, qB } = temp;
        qA.set(aA);
        qB.set(aB);

        // Compute the effective mass matrix.
        const rA = Rot.multiplyVec2(qA, Vec2.subtract(this.linearOffset, this.localCenterA, Vec2.s_t0), this.rA);
        const rB = Rot.multiplyVec2(qB, Vec2.negate(this.localCenterB, Vec2.s_t0), this.rB);

        // J = [-I -r1_skew I r2_skew]
        // r_skew = [-ry; rx]

        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
        //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
        //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

        const mA = this.invMassA;
        const mB = this.invMassB;
        const iA = this.invIA;
        const iB = this.invIB;

        // Upper 2 by 2 of K for point to point
        const K = this.linearMass;
        K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
        K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
        K.ey.x = K.ex.y;
        K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;

        K.inverse();

        this.angularMass = iA + iB;
        if (this.angularMass > 0) {
            this.angularMass = 1 / this.angularMass;
        }

        Vec2.subtract(Vec2.add(cB, rB, Vec2.s_t0), Vec2.add(cA, rA, Vec2.s_t1), this.linearError);
        this.angularError = aB - aA - this.angularOffset;

        if (data.step.warmStarting) {
            // Scale impulses to support a variable time step.
            this.linearImpulse.scale(data.step.dtRatio);
            this.angularImpulse *= data.step.dtRatio;

            const P = this.linearImpulse;
            vA.subtractScaled(mA, P);
            wA -= iA * (Vec2.cross(rA, P) + this.angularImpulse);
            vB.addScaled(mB, P);
            wB += iB * (Vec2.cross(rB, P) + this.angularImpulse);
        } else {
            this.linearImpulse.setZero();
            this.angularImpulse = 0;
        }

        data.velocities[this.indexA].w = wA;
        data.velocities[this.indexB].w = wB;
    }

    /** @internal protected */
    public solveVelocityConstraints(data: SolverData): void {
        const vA = data.velocities[this.indexA].v;
        let wA = data.velocities[this.indexA].w;
        const vB = data.velocities[this.indexB].v;
        let wB = data.velocities[this.indexB].w;

        const mA = this.invMassA;
        const mB = this.invMassB;
        const iA = this.invIA;
        const iB = this.invIB;

        const h = data.step.dt;
        const inv_h = data.step.inv_dt;

        // Solve angular friction
        {
            const Cdot = wB - wA + inv_h * this.correctionFactor * this.angularError;
            let impulse = -this.angularMass * Cdot;

            const oldImpulse = this.angularImpulse;
            const maxImpulse = h * this.maxTorque;
            this.angularImpulse = clamp(this.angularImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.angularImpulse - oldImpulse;

            wA -= iA * impulse;
            wB += iB * impulse;
        }

        // Solve linear friction
        {
            const { impulse, oldImpulse, Cdot } = temp;

            Vec2.addScaled(
                Vec2.subtract(
                    Vec2.addCrossScalarVec2(vB, wB, this.rB, Vec2.s_t0),
                    Vec2.addCrossScalarVec2(vA, wA, this.rA, Vec2.s_t1),
                    Vec2.s_t2,
                ),
                inv_h * this.correctionFactor,
                this.linearError,
                Cdot,
            );

            Mat22.multiplyVec2(this.linearMass, Cdot, impulse).negate();
            oldImpulse.copy(this.linearImpulse);
            this.linearImpulse.add(impulse);

            const maxImpulse = h * this.maxForce;

            if (this.linearImpulse.lengthSquared() > maxImpulse * maxImpulse) {
                this.linearImpulse.normalize();
                this.linearImpulse.scale(maxImpulse);
            }

            Vec2.subtract(this.linearImpulse, oldImpulse, impulse);

            vA.subtractScaled(mA, impulse);
            wA -= iA * Vec2.cross(this.rA, impulse);

            vB.addScaled(mB, impulse);
            wB += iB * Vec2.cross(this.rB, impulse);
        }

        data.velocities[this.indexA].w = wA;
        data.velocities[this.indexB].w = wB;
    }

    /** @internal protected */
    public solvePositionConstraints(_data: SolverData): boolean {
        return true;
    }
}
