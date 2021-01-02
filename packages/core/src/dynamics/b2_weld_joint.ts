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
    protected stiffness = 0;

    protected damping = 0;

    protected bias = 0;

    // Solver shared
    protected readonly localAnchorA = new Vec2();

    protected readonly localAnchorB = new Vec2();

    protected referenceAngle = 0;

    protected gamma = 0;

    protected readonly impulse = new Vec3();

    // Solver temp
    protected indexA = 0;

    protected indexB = 0;

    protected readonly rA = new Vec2();

    protected readonly rB = new Vec2();

    protected readonly localCenterA = new Vec2();

    protected readonly localCenterB = new Vec2();

    protected invMassA = 0;

    protected invMassB = 0;

    protected invIA = 0;

    protected invIB = 0;

    protected readonly mass = new Mat33();

    /** @internal protected */
    public constructor(def: IWeldJointDef) {
        super(def);

        this.localAnchorA.copy(def.localAnchorA ?? Vec2.ZERO);
        this.localAnchorB.copy(def.localAnchorB ?? Vec2.ZERO);
        this.referenceAngle = def.referenceAngle ?? 0;
        this.stiffness = def.stiffness ?? 0;
        this.damping = def.damping ?? 0;
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

        const aA = data.positions[this.indexA].a;
        const vA = data.velocities[this.indexA].v;
        let wA = data.velocities[this.indexA].w;

        const aB = data.positions[this.indexB].a;
        const vB = data.velocities[this.indexB].v;
        let wB = data.velocities[this.indexB].w;

        const { qA, qB, lalcA, lalcB, K } = temp;
        qA.set(aA);
        qB.set(aB);

        Rot.multiplyVec2(qA, Vec2.subtract(this.localAnchorA, this.localCenterA, lalcA), this.rA);
        Rot.multiplyVec2(qB, Vec2.subtract(this.localAnchorB, this.localCenterB, lalcB), this.rB);

        // J = [-I -r1_skew I r2_skew]
        //     [ 0       -1 0       1]
        // r_skew = [-ry; rx]

        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
        //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
        //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

        const mA = this.invMassA;
        const mB = this.invMassB;
        const iA = this.invIA;
        const iB = this.invIB;

        K.ex.x = mA + mB + this.rA.y * this.rA.y * iA + this.rB.y * this.rB.y * iB;
        K.ey.x = -this.rA.y * this.rA.x * iA - this.rB.y * this.rB.x * iB;
        K.ez.x = -this.rA.y * iA - this.rB.y * iB;
        K.ex.y = K.ey.x;
        K.ey.y = mA + mB + this.rA.x * this.rA.x * iA + this.rB.x * this.rB.x * iB;
        K.ez.y = this.rA.x * iA + this.rB.x * iB;
        K.ex.z = K.ez.x;
        K.ey.z = K.ez.y;
        K.ez.z = iA + iB;

        if (this.stiffness > 0) {
            K.getInverse22(this.mass);

            let invM = iA + iB;

            const C = aB - aA - this.referenceAngle;

            // Damping coefficient
            const d = this.damping;

            // Spring stiffness
            const k = this.stiffness;

            // magic formulas
            const h = data.step.dt;
            this.gamma = h * (d + h * k);
            this.gamma = this.gamma !== 0 ? 1 / this.gamma : 0;
            this.bias = C * h * k * this.gamma;

            invM += this.gamma;
            this.mass.ez.z = invM !== 0 ? 1 / invM : 0;
        } else if (K.ez.z === 0) {
            K.getInverse22(this.mass);
            this.gamma = 0;
            this.bias = 0;
        } else {
            K.getSymInverse33(this.mass);
            this.gamma = 0;
            this.bias = 0;
        }

        if (data.step.warmStarting) {
            // Scale impulses to support a variable time step.
            this.impulse.scale(data.step.dtRatio);

            const { P } = temp;
            P.copy(this.impulse);

            vA.subtractScaled(mA, P);
            wA -= iA * (Vec2.cross(this.rA, P) + this.impulse.z);

            vB.addScaled(mB, P);
            wB += iB * (Vec2.cross(this.rB, P) + this.impulse.z);
        } else {
            this.impulse.setZero();
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

        if (this.stiffness > 0) {
            const Cdot2 = wB - wA;

            const impulse2 = -this.mass.ez.z * (Cdot2 + this.bias + this.gamma * this.impulse.z);
            this.impulse.z += impulse2;

            wA -= iA * impulse2;
            wB += iB * impulse2;

            const { Cdot1, impulse1 } = temp;
            Vec2.subtract(
                Vec2.addCrossScalarVec2(vB, wB, this.rB, Vec2.s_t0),
                Vec2.addCrossScalarVec2(vA, wA, this.rA, Vec2.s_t1),
                Cdot1,
            );

            Mat33.multiplyVec2(this.mass, Cdot1, impulse1).negate();
            this.impulse.x += impulse1.x;
            this.impulse.y += impulse1.y;

            const P = impulse1;

            vA.subtractScaled(mA, P);
            wA -= iA * Vec2.cross(this.rA, P);

            vB.addScaled(mB, P);
            wB += iB * Vec2.cross(this.rB, P);
        } else {
            const { Cdot1, impulse, P } = temp;
            Vec2.subtract(
                Vec2.addCrossScalarVec2(vB, wB, this.rB, Vec2.s_t0),
                Vec2.addCrossScalarVec2(vA, wA, this.rA, Vec2.s_t1),
                Cdot1,
            );
            Cdot1.z = wB - wA;

            Mat33.multiplyVec3(this.mass, Cdot1, impulse).negate();
            this.impulse.add(impulse);

            P.set(impulse.x, impulse.y);

            vA.subtractScaled(mA, P);
            wA -= iA * (Vec2.cross(this.rA, P) + impulse.z);

            vB.addScaled(mB, P);
            wB += iB * (Vec2.cross(this.rB, P) + impulse.z);
        }

        data.velocities[this.indexA].w = wA;
        data.velocities[this.indexB].w = wB;
    }

    /** @internal protected */
    public solvePositionConstraints(data: SolverData): boolean {
        const cA = data.positions[this.indexA].c;
        let aA = data.positions[this.indexA].a;
        const cB = data.positions[this.indexB].c;
        let aB = data.positions[this.indexB].a;

        const { qA, qB, lalcA, lalcB, K, C1, P, rA, rB } = temp;
        qA.set(aA);
        qB.set(aB);

        const mA = this.invMassA;
        const mB = this.invMassB;
        const iA = this.invIA;
        const iB = this.invIB;

        Rot.multiplyVec2(qA, Vec2.subtract(this.localAnchorA, this.localCenterA, lalcA), rA);
        Rot.multiplyVec2(qB, Vec2.subtract(this.localAnchorB, this.localCenterB, lalcB), rB);

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

        if (this.stiffness > 0) {
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
            const C2 = aB - aA - this.referenceAngle;

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

        data.positions[this.indexA].a = aA;
        data.positions[this.indexB].a = aB;

        return positionError <= LINEAR_SLOP && angularError <= ANGULAR_SLOP;
    }

    public getAnchorA<T extends XY>(out: T): T {
        return this.bodyA.getWorldPoint(this.localAnchorA, out);
    }

    public getAnchorB<T extends XY>(out: T): T {
        return this.bodyB.getWorldPoint(this.localAnchorB, out);
    }

    public getReactionForce<T extends XY>(inv_dt: number, out: T): T {
        out.x = inv_dt * this.impulse.x;
        out.y = inv_dt * this.impulse.y;
        return out;
    }

    public getReactionTorque(inv_dt: number): number {
        return inv_dt * this.impulse.z;
    }

    public getLocalAnchorA(): Readonly<Vec2> {
        return this.localAnchorA;
    }

    public getLocalAnchorB(): Readonly<Vec2> {
        return this.localAnchorB;
    }

    public getReferenceAngle(): number {
        return this.referenceAngle;
    }

    public setStiffness(stiffness: number): void {
        this.stiffness = stiffness;
    }

    public getStiffness(): number {
        return this.stiffness;
    }

    public setDamping(damping: number) {
        this.damping = damping;
    }

    public getDamping() {
        return this.damping;
    }
}
