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

import { LINEAR_SLOP, ANGULAR_SLOP } from "../common/b2_common";
import { Vec2, Vec3, Mat33, Rot, XY } from "../common/b2_math";
import { Body } from "./b2_body";
import { Joint, JointDef, JointType, IJointDef } from "./b2_joint";
import { SolverData } from "./b2_time_step";

const temp = {
    qA: new Rot(),
    qB: new Rot(),
    rA: new Vec2(),
    rB: new Vec2(),
    lalcA: new Vec2(),
    lalcB: new Vec2(),
    K: new Mat33(),
    P: new Vec2(),
    Cdot1: new Vec3(),
    impulse1: new Vec2(),
    impulse: new Vec3(),
    C1: new Vec2(),
    C: new Vec3(),
};

export interface IWeldJointDef extends IJointDef {
    localAnchorA?: XY;

    localAnchorB?: XY;

    referenceAngle?: number;

    stiffness?: number;

    damping?: number;
}

/**
 * Weld joint definition. You need to specify local anchor points
 * where they are attached and the relative body angle. The position
 * of the anchor points is important for computing the reaction torque.
 */
export class WeldJointDef extends JointDef implements IWeldJointDef {
    /** The local anchor point relative to bodyA's origin. */
    public readonly localAnchorA = new Vec2();

    /** The local anchor point relative to bodyB's origin. */
    public readonly localAnchorB = new Vec2();

    /** The bodyB angle minus bodyA angle in the reference state (radians). */
    public referenceAngle = 0;

    /**
     * The rotational stiffness in N*m
     * Disable softness with a value of 0
     */
    public stiffness = 0;

    /** The rotational damping in N*m*s */
    public damping = 0;

    public constructor() {
        super(JointType.Weld);
    }

    public initialize(bA: Body, bB: Body, anchor: Vec2): void {
        this.bodyA = bA;
        this.bodyB = bB;
        this.bodyA.getLocalPoint(anchor, this.localAnchorA);
        this.bodyB.getLocalPoint(anchor, this.localAnchorB);
        this.referenceAngle = this.bodyB.getAngle() - this.bodyA.getAngle();
    }
}

/**
 * A weld joint essentially glues two bodies together. A weld joint may
 * distort somewhat because the island constraint solver is approximate.
 */
export class WeldJoint extends Joint {
    protected m_stiffness = 0;

    protected m_damping = 0;

    protected m_bias = 0;

    // Solver shared
    protected readonly m_localAnchorA = new Vec2();

    protected readonly m_localAnchorB = new Vec2();

    protected m_referenceAngle = 0;

    protected m_gamma = 0;

    protected readonly m_impulse = new Vec3();

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

    protected readonly m_mass = new Mat33();

    /** @internal protected */
    public constructor(def: IWeldJointDef) {
        super(def);

        this.m_localAnchorA.copy(def.localAnchorA ?? Vec2.ZERO);
        this.m_localAnchorB.copy(def.localAnchorB ?? Vec2.ZERO);
        this.m_referenceAngle = def.referenceAngle ?? 0;
        this.m_stiffness = def.stiffness ?? 0;
        this.m_damping = def.damping ?? 0;
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

        const aA = data.positions[this.m_indexA].a;
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;

        const aB = data.positions[this.m_indexB].a;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;

        const { qA, qB, lalcA, lalcB, K } = temp;
        qA.set(aA);
        qB.set(aB);

        Rot.multiplyVec2(qA, Vec2.subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), this.m_rA);
        Rot.multiplyVec2(qB, Vec2.subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), this.m_rB);

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

        K.ex.x = mA + mB + this.m_rA.y * this.m_rA.y * iA + this.m_rB.y * this.m_rB.y * iB;
        K.ey.x = -this.m_rA.y * this.m_rA.x * iA - this.m_rB.y * this.m_rB.x * iB;
        K.ez.x = -this.m_rA.y * iA - this.m_rB.y * iB;
        K.ex.y = K.ey.x;
        K.ey.y = mA + mB + this.m_rA.x * this.m_rA.x * iA + this.m_rB.x * this.m_rB.x * iB;
        K.ez.y = this.m_rA.x * iA + this.m_rB.x * iB;
        K.ex.z = K.ez.x;
        K.ey.z = K.ez.y;
        K.ez.z = iA + iB;

        if (this.m_stiffness > 0) {
            K.getInverse22(this.m_mass);

            let invM = iA + iB;

            const C = aB - aA - this.m_referenceAngle;

            // Damping coefficient
            const d = this.m_damping;

            // Spring stiffness
            const k = this.m_stiffness;

            // magic formulas
            const h = data.step.dt;
            this.m_gamma = h * (d + h * k);
            this.m_gamma = this.m_gamma !== 0 ? 1 / this.m_gamma : 0;
            this.m_bias = C * h * k * this.m_gamma;

            invM += this.m_gamma;
            this.m_mass.ez.z = invM !== 0 ? 1 / invM : 0;
        } else if (K.ez.z === 0) {
            K.getInverse22(this.m_mass);
            this.m_gamma = 0;
            this.m_bias = 0;
        } else {
            K.getSymInverse33(this.m_mass);
            this.m_gamma = 0;
            this.m_bias = 0;
        }

        if (data.step.warmStarting) {
            // Scale impulses to support a variable time step.
            this.m_impulse.scale(data.step.dtRatio);

            const { P } = temp;
            P.copy(this.m_impulse);

            vA.subtractScaled(mA, P);
            wA -= iA * (Vec2.cross(this.m_rA, P) + this.m_impulse.z);

            vB.addScaled(mB, P);
            wB += iB * (Vec2.cross(this.m_rB, P) + this.m_impulse.z);
        } else {
            this.m_impulse.setZero();
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

        if (this.m_stiffness > 0) {
            const Cdot2 = wB - wA;

            const impulse2 = -this.m_mass.ez.z * (Cdot2 + this.m_bias + this.m_gamma * this.m_impulse.z);
            this.m_impulse.z += impulse2;

            wA -= iA * impulse2;
            wB += iB * impulse2;

            const { Cdot1, impulse1 } = temp;
            Vec2.subtract(
                Vec2.addCrossScalarVec2(vB, wB, this.m_rB, Vec2.s_t0),
                Vec2.addCrossScalarVec2(vA, wA, this.m_rA, Vec2.s_t1),
                Cdot1,
            );

            Mat33.multiplyVec2(this.m_mass, Cdot1, impulse1).negate();
            this.m_impulse.x += impulse1.x;
            this.m_impulse.y += impulse1.y;

            const P = impulse1;

            vA.subtractScaled(mA, P);
            wA -= iA * Vec2.cross(this.m_rA, P);

            vB.addScaled(mB, P);
            wB += iB * Vec2.cross(this.m_rB, P);
        } else {
            const { Cdot1, impulse, P } = temp;
            Vec2.subtract(
                Vec2.addCrossScalarVec2(vB, wB, this.m_rB, Vec2.s_t0),
                Vec2.addCrossScalarVec2(vA, wA, this.m_rA, Vec2.s_t1),
                Cdot1,
            );
            Cdot1.z = wB - wA;

            Mat33.multiplyVec3(this.m_mass, Cdot1, impulse).negate();
            this.m_impulse.add(impulse);

            P.set(impulse.x, impulse.y);

            vA.subtractScaled(mA, P);
            wA -= iA * (Vec2.cross(this.m_rA, P) + impulse.z);

            vB.addScaled(mB, P);
            wB += iB * (Vec2.cross(this.m_rB, P) + impulse.z);
        }

        data.velocities[this.m_indexA].w = wA;
        data.velocities[this.m_indexB].w = wB;
    }

    /** @internal protected */
    public solvePositionConstraints(data: SolverData): boolean {
        const cA = data.positions[this.m_indexA].c;
        let aA = data.positions[this.m_indexA].a;
        const cB = data.positions[this.m_indexB].c;
        let aB = data.positions[this.m_indexB].a;

        const { qA, qB, lalcA, lalcB, K, C1, P, rA, rB } = temp;
        qA.set(aA);
        qB.set(aB);

        const mA = this.m_invMassA;
        const mB = this.m_invMassB;
        const iA = this.m_invIA;
        const iB = this.m_invIB;

        Rot.multiplyVec2(qA, Vec2.subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), rA);
        Rot.multiplyVec2(qB, Vec2.subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), rB);

        let positionError: number;
        let angularError: number;

        K.ex.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
        K.ey.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
        K.ez.x = -rA.y * iA - rB.y * iB;
        K.ex.y = K.ey.x;
        K.ey.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
        K.ez.y = rA.x * iA + rB.x * iB;
        K.ex.z = K.ez.x;
        K.ey.z = K.ez.y;
        K.ez.z = iA + iB;

        if (this.m_stiffness > 0) {
            Vec2.add(cB, rB, C1).subtract(cA).subtract(rA);
            positionError = C1.length();
            angularError = 0;

            K.solve22(C1.x, C1.y, P).negate();

            cA.subtractScaled(mA, P);
            aA -= iA * Vec2.cross(rA, P);

            cB.addScaled(mB, P);
            aB += iB * Vec2.cross(rB, P);
        } else {
            Vec2.add(cB, rB, C1).subtract(cA).subtract(rA);
            Vec2.subtract(Vec2.add(cB, rB, Vec2.s_t0), Vec2.add(cA, rA, Vec2.s_t1), C1);
            const C2 = aB - aA - this.m_referenceAngle;

            positionError = C1.length();
            angularError = Math.abs(C2);

            const { impulse, C } = temp;
            C.set(C1.x, C1.y, C2);

            if (K.ez.z > 0) {
                K.solve33(C.x, C.y, C.z, impulse).negate();
            } else {
                K.solve22(C1.x, C1.y, impulse).negate();
                impulse.z = 0;
            }

            P.copy(impulse);

            cA.subtractScaled(mA, P);
            aA -= iA * (Vec2.cross(rA, P) + impulse.z);

            cB.addScaled(mB, P);
            aB += iB * (Vec2.cross(rB, P) + impulse.z);
        }

        data.positions[this.m_indexA].a = aA;
        data.positions[this.m_indexB].a = aB;

        return positionError <= LINEAR_SLOP && angularError <= ANGULAR_SLOP;
    }

    public getAnchorA<T extends XY>(out: T): T {
        return this.m_bodyA.getWorldPoint(this.m_localAnchorA, out);
    }

    public getAnchorB<T extends XY>(out: T): T {
        return this.m_bodyB.getWorldPoint(this.m_localAnchorB, out);
    }

    public getReactionForce<T extends XY>(inv_dt: number, out: T): T {
        out.x = inv_dt * this.m_impulse.x;
        out.y = inv_dt * this.m_impulse.y;
        return out;
    }

    public getReactionTorque(inv_dt: number): number {
        return inv_dt * this.m_impulse.z;
    }

    public getLocalAnchorA(): Readonly<Vec2> {
        return this.m_localAnchorA;
    }

    public getLocalAnchorB(): Readonly<Vec2> {
        return this.m_localAnchorB;
    }

    public getReferenceAngle(): number {
        return this.m_referenceAngle;
    }

    public setStiffness(stiffness: number): void {
        this.m_stiffness = stiffness;
    }

    public getStiffness(): number {
        return this.m_stiffness;
    }

    public setDamping(damping: number) {
        this.m_damping = damping;
    }

    public getDamping() {
        return this.m_damping;
    }
}
