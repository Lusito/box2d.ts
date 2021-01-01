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
import { LINEAR_SLOP } from "../common/b2_common";
import { clamp, Vec2, Rot, XY, Transform } from "../common/b2_math";
import { Joint, JointDef, JointType, IJointDef } from "./b2_joint";
import { SolverData } from "./b2_time_step";
import { Body } from "./b2_body";
import { Draw, debugColors } from "../common/b2_draw";

const temp = {
    qA: new Rot(),
    qB: new Rot(),
    lalcA: new Vec2(),
    lalcB: new Vec2(),
    rA: new Vec2(),
    rB: new Vec2(),
    d: new Vec2(),
    P: new Vec2(),
    ay: new Vec2(),
    pA: new Vec2(),
    pB: new Vec2(),
    axis: new Vec2(),
    Draw: {
        p1: new Vec2(),
        p2: new Vec2(),
        pA: new Vec2(),
        pB: new Vec2(),
        axis: new Vec2(),
        lower: new Vec2(),
        upper: new Vec2(),
        perp: new Vec2(),
    },
};

export interface IWheelJointDef extends IJointDef {
    /** The local anchor point relative to bodyA's origin. */
    localAnchorA?: XY;

    /** The local anchor point relative to bodyB's origin. */
    localAnchorB?: XY;

    /** The local translation axis in bodyA. */
    localAxisA?: XY;

    /** Enable/disable the joint limit. */
    enableLimit?: boolean;

    /** The lower translation limit, usually in meters. */
    lowerTranslation?: number;

    /** The upper translation limit, usually in meters. */
    upperTranslation?: number;

    /** Enable/disable the joint motor. */
    enableMotor?: boolean;

    /** The maximum motor torque, usually in N-m. */
    maxMotorTorque?: number;

    /** The desired motor speed in radians per second. */
    motorSpeed?: number;

    /** Suspension stiffness. Typically in units N/m. */
    stiffness?: number;

    /** Suspension damping. Typically in units of N*s/m. */
    damping?: number;
}

/**
 * Wheel joint definition. This requires defining a line of
 * motion using an axis and an anchor point. The definition uses local
 * anchor points and a local axis so that the initial configuration
 * can violate the constraint slightly. The joint translation is zero
 * when the local anchor points coincide in world space. Using local
 * anchors and a local axis helps when saving and loading a game.
 */
export class WheelJointDef extends JointDef implements IWheelJointDef {
    /** The local anchor point relative to bodyA's origin. */
    public readonly localAnchorA = new Vec2();

    /** The local anchor point relative to bodyB's origin. */
    public readonly localAnchorB = new Vec2();

    /** The local translation axis in bodyA. */
    public readonly localAxisA = new Vec2(1, 0);

    /** Enable/disable the joint limit. */
    public enableLimit = false;

    /** The lower translation limit, usually in meters. */
    public lowerTranslation = 0;

    /** The upper translation limit, usually in meters. */
    public upperTranslation = 0;

    /** Enable/disable the joint motor. */
    public enableMotor = false;

    /** The maximum motor torque, usually in N-m. */
    public maxMotorTorque = 0;

    /** The desired motor speed in radians per second. */
    public motorSpeed = 0;

    /** Suspension stiffness. Typically in units N/m. */
    public stiffness = 0;

    /** Suspension damping. Typically in units of N*s/m. */
    public damping = 0;

    public constructor() {
        super(JointType.Wheel);
    }

    public initialize(bA: Body, bB: Body, anchor: Vec2, axis: Vec2): void {
        this.bodyA = bA;
        this.bodyB = bB;
        this.bodyA.getLocalPoint(anchor, this.localAnchorA);
        this.bodyB.getLocalPoint(anchor, this.localAnchorB);
        this.bodyA.getLocalVector(axis, this.localAxisA);
    }
}

/**
 * A wheel joint. This joint provides two degrees of freedom: translation
 * along an axis fixed in bodyA and rotation in the plane. In other words, it is a point to
 * line constraint with a rotational motor and a linear spring/damper. The spring/damper is
 * initialized upon creation. This joint is designed for vehicle suspensions.
 */
export class WheelJoint extends Joint {
    protected readonly m_localAnchorA = new Vec2();

    protected readonly m_localAnchorB = new Vec2();

    protected readonly m_localXAxisA = new Vec2();

    protected readonly m_localYAxisA = new Vec2();

    protected m_impulse = 0;

    protected m_motorImpulse = 0;

    protected m_springImpulse = 0;

    protected m_lowerImpulse = 0;

    protected m_upperImpulse = 0;

    protected m_translation = 0;

    protected m_lowerTranslation = 0;

    protected m_upperTranslation = 0;

    protected m_maxMotorTorque = 0;

    protected m_motorSpeed = 0;

    protected m_enableLimit = false;

    protected m_enableMotor = false;

    protected m_stiffness = 0;

    protected m_damping = 0;

    // Solver temp
    protected m_indexA = 0;

    protected m_indexB = 0;

    protected readonly m_localCenterA = new Vec2();

    protected readonly m_localCenterB = new Vec2();

    protected m_invMassA = 0;

    protected m_invMassB = 0;

    protected m_invIA = 0;

    protected m_invIB = 0;

    protected readonly m_ax = new Vec2();

    protected readonly m_ay = new Vec2();

    protected m_sAx = 0;

    protected m_sBx = 0;

    protected m_sAy = 0;

    protected m_sBy = 0;

    protected m_mass = 0;

    protected m_motorMass = 0;

    protected m_axialMass = 0;

    protected m_springMass = 0;

    protected m_bias = 0;

    protected m_gamma = 0;

    /** @internal protected */
    public constructor(def: IWheelJointDef) {
        super(def);

        this.m_localAnchorA.copy(def.localAnchorA ?? Vec2.ZERO);
        this.m_localAnchorB.copy(def.localAnchorB ?? Vec2.ZERO);
        this.m_localXAxisA.copy(def.localAxisA ?? Vec2.UNITX);
        Vec2.crossOneVec2(this.m_localXAxisA, this.m_localYAxisA);

        this.m_lowerTranslation = def.lowerTranslation ?? 0;
        this.m_upperTranslation = def.upperTranslation ?? 0;
        this.m_enableLimit = def.enableLimit ?? false;

        this.m_maxMotorTorque = def.maxMotorTorque ?? 0;
        this.m_motorSpeed = def.motorSpeed ?? 0;
        this.m_enableMotor = def.enableMotor ?? false;

        this.m_ax.setZero();
        this.m_ay.setZero();

        this.m_stiffness = def.stiffness ?? 0;
        this.m_damping = def.damping ?? 0;
    }

    public getMotorSpeed(): number {
        return this.m_motorSpeed;
    }

    public getMaxMotorTorque(): number {
        return this.m_maxMotorTorque;
    }

    public setStiffness(stiffness: number): void {
        this.m_stiffness = stiffness;
    }

    public getStiffness(): number {
        return this.m_stiffness;
    }

    public setDamping(damping: number): void {
        this.m_damping = damping;
    }

    public getDamping(): number {
        return this.m_damping;
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

        const mA = this.m_invMassA;
        const mB = this.m_invMassB;
        const iA = this.m_invIA;
        const iB = this.m_invIB;

        const cA = data.positions[this.m_indexA].c;
        const aA = data.positions[this.m_indexA].a;
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;

        const cB = data.positions[this.m_indexB].c;
        const aB = data.positions[this.m_indexB].a;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;

        const { qA, qB, lalcA, lalcB, rA, rB, d } = temp;
        qA.set(aA);
        qB.set(aB);

        // Compute the effective masses.
        Rot.multiplyVec2(qA, Vec2.subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), rA);
        Rot.multiplyVec2(qB, Vec2.subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), rB);
        Vec2.add(cB, rB, d).subtract(cA).subtract(rA);

        // Point to line constraint
        Rot.multiplyVec2(qA, this.m_localYAxisA, this.m_ay);
        this.m_sAy = Vec2.cross(Vec2.add(d, rA, Vec2.s_t0), this.m_ay);
        this.m_sBy = Vec2.cross(rB, this.m_ay);

        this.m_mass = mA + mB + iA * this.m_sAy * this.m_sAy + iB * this.m_sBy * this.m_sBy;

        if (this.m_mass > 0) {
            this.m_mass = 1 / this.m_mass;
        }

        // Spring constraint
        Rot.multiplyVec2(qA, this.m_localXAxisA, this.m_ax);
        this.m_sAx = Vec2.cross(Vec2.add(d, rA, Vec2.s_t0), this.m_ax);
        this.m_sBx = Vec2.cross(rB, this.m_ax);

        const invMass = mA + mB + iA * this.m_sAx * this.m_sAx + iB * this.m_sBx * this.m_sBx;
        if (invMass > 0) {
            this.m_axialMass = 1 / invMass;
        } else {
            this.m_axialMass = 0;
        }

        this.m_springMass = 0;
        this.m_bias = 0;
        this.m_gamma = 0;

        if (this.m_stiffness > 0 && invMass > 0) {
            this.m_springMass = 1 / invMass;

            const C = Vec2.dot(d, this.m_ax);

            // magic formulas
            const h = data.step.dt;
            this.m_gamma = h * (this.m_damping + h * this.m_stiffness);
            if (this.m_gamma > 0) {
                this.m_gamma = 1 / this.m_gamma;
            }

            this.m_bias = C * h * this.m_stiffness * this.m_gamma;

            this.m_springMass = invMass + this.m_gamma;
            if (this.m_springMass > 0) {
                this.m_springMass = 1 / this.m_springMass;
            }
        } else {
            this.m_springImpulse = 0;
        }

        if (this.m_enableLimit) {
            this.m_translation = Vec2.dot(this.m_ax, d);
        } else {
            this.m_lowerImpulse = 0;
            this.m_upperImpulse = 0;
        }

        if (this.m_enableMotor) {
            this.m_motorMass = iA + iB;
            if (this.m_motorMass > 0) {
                this.m_motorMass = 1 / this.m_motorMass;
            }
        } else {
            this.m_motorMass = 0;
            this.m_motorImpulse = 0;
        }

        if (data.step.warmStarting) {
            // Account for variable time step.
            this.m_impulse *= data.step.dtRatio;
            this.m_springImpulse *= data.step.dtRatio;
            this.m_motorImpulse *= data.step.dtRatio;

            const axialImpulse = this.m_springImpulse + this.m_lowerImpulse - this.m_upperImpulse;
            const { P } = temp;
            Vec2.scale(this.m_impulse, this.m_ay, P).addScaled(axialImpulse, this.m_ax);
            const LA = this.m_impulse * this.m_sAy + axialImpulse * this.m_sAx + this.m_motorImpulse;
            const LB = this.m_impulse * this.m_sBy + axialImpulse * this.m_sBx + this.m_motorImpulse;

            vA.subtractScaled(this.m_invMassA, P);
            wA -= this.m_invIA * LA;

            vB.addScaled(this.m_invMassB, P);
            wB += this.m_invIB * LB;
        } else {
            this.m_impulse = 0;
            this.m_springImpulse = 0;
            this.m_motorImpulse = 0;
            this.m_lowerImpulse = 0;
            this.m_upperImpulse = 0;
        }

        data.velocities[this.m_indexA].w = wA;
        data.velocities[this.m_indexB].w = wB;
    }

    /** @internal protected */
    public solveVelocityConstraints(data: SolverData): void {
        const mA = this.m_invMassA;
        const mB = this.m_invMassB;
        const iA = this.m_invIA;
        const iB = this.m_invIB;

        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;

        const { P } = temp;
        // Solve spring constraint
        {
            const Cdot = Vec2.dot(this.m_ax, Vec2.subtract(vB, vA, Vec2.s_t0)) + this.m_sBx * wB - this.m_sAx * wA;
            const impulse = -this.m_springMass * (Cdot + this.m_bias + this.m_gamma * this.m_springImpulse);
            this.m_springImpulse += impulse;

            Vec2.scale(impulse, this.m_ax, P);
            const LA = impulse * this.m_sAx;
            const LB = impulse * this.m_sBx;

            vA.subtractScaled(mA, P);
            wA -= iA * LA;

            vB.addScaled(mB, P);
            wB += iB * LB;
        }

        // Solve rotational motor constraint
        {
            const Cdot = wB - wA - this.m_motorSpeed;
            let impulse = -this.m_motorMass * Cdot;

            const oldImpulse = this.m_motorImpulse;
            const maxImpulse = data.step.dt * this.m_maxMotorTorque;
            this.m_motorImpulse = clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.m_motorImpulse - oldImpulse;

            wA -= iA * impulse;
            wB += iB * impulse;
        }

        if (this.m_enableLimit) {
            // Lower limit
            {
                const C = this.m_translation - this.m_lowerTranslation;
                const Cdot = Vec2.dot(this.m_ax, Vec2.subtract(vB, vA, Vec2.s_t0)) + this.m_sBx * wB - this.m_sAx * wA;
                let impulse = -this.m_axialMass * (Cdot + Math.max(C, 0) * data.step.inv_dt);
                const oldImpulse = this.m_lowerImpulse;
                this.m_lowerImpulse = Math.max(this.m_lowerImpulse + impulse, 0);
                impulse = this.m_lowerImpulse - oldImpulse;

                Vec2.scale(impulse, this.m_ax, P);
                const LA = impulse * this.m_sAx;
                const LB = impulse * this.m_sBx;

                vA.subtractScaled(mA, P);
                wA -= iA * LA;
                vB.addScaled(mB, P);
                wB += iB * LB;
            }

            // Upper limit
            // Note: signs are flipped to keep C positive when the constraint is satisfied.
            // This also keeps the impulse positive when the limit is active.
            {
                const C = this.m_upperTranslation - this.m_translation;
                const Cdot = Vec2.dot(this.m_ax, Vec2.subtract(vA, vB, Vec2.s_t0)) + this.m_sAx * wA - this.m_sBx * wB;
                let impulse = -this.m_axialMass * (Cdot + Math.max(C, 0) * data.step.inv_dt);
                const oldImpulse = this.m_upperImpulse;
                this.m_upperImpulse = Math.max(this.m_upperImpulse + impulse, 0);
                impulse = this.m_upperImpulse - oldImpulse;

                Vec2.scale(impulse, this.m_ax, P);
                const LA = impulse * this.m_sAx;
                const LB = impulse * this.m_sBx;

                vA.addScaled(mA, P);
                wA += iA * LA;
                vB.subtractScaled(mB, P);
                wB -= iB * LB;
            }
        }

        // Solve point to line constraint
        {
            const Cdot = Vec2.dot(this.m_ay, Vec2.subtract(vB, vA, Vec2.s_t0)) + this.m_sBy * wB - this.m_sAy * wA;
            const impulse = -this.m_mass * Cdot;
            this.m_impulse += impulse;

            Vec2.scale(impulse, this.m_ay, P);
            const LA = impulse * this.m_sAy;
            const LB = impulse * this.m_sBy;

            vA.subtractScaled(mA, P);
            wA -= iA * LA;

            vB.addScaled(mB, P);
            wB += iB * LB;
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

        let linearError = 0;

        const { qA, qB, lalcA, lalcB, rA, rB, d, P, ay } = temp;

        if (this.m_enableLimit) {
            qA.set(aA);
            qB.set(aB);

            Rot.multiplyVec2(qA, Vec2.subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), rA);
            Rot.multiplyVec2(qB, Vec2.subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), rB);
            Vec2.subtract(cB, cA, d).add(rB).subtract(rA);

            const ax = Rot.multiplyVec2(qA, this.m_localXAxisA, this.m_ax);
            const sAx = Vec2.cross(Vec2.add(d, rA, Vec2.s_t0), this.m_ax);
            const sBx = Vec2.cross(rB, this.m_ax);

            let C = 0;
            const translation = Vec2.dot(ax, d);
            if (Math.abs(this.m_upperTranslation - this.m_lowerTranslation) < 2 * LINEAR_SLOP) {
                C = translation;
            } else if (translation <= this.m_lowerTranslation) {
                C = Math.min(translation - this.m_lowerTranslation, 0);
            } else if (translation >= this.m_upperTranslation) {
                C = Math.max(translation - this.m_upperTranslation, 0);
            }

            if (C !== 0) {
                const invMass = this.m_invMassA + this.m_invMassB + this.m_invIA * sAx * sAx + this.m_invIB * sBx * sBx;
                let impulse = 0;
                if (invMass !== 0) {
                    impulse = -C / invMass;
                }

                Vec2.scale(impulse, ax, P);
                const LA = impulse * sAx;
                const LB = impulse * sBx;

                cA.subtractScaled(this.m_invMassA, P);
                aA -= this.m_invIA * LA;
                cB.addScaled(this.m_invMassB, P);
                aB += this.m_invIB * LB;

                linearError = Math.abs(C);
            }
        }

        // Solve perpendicular constraint
        {
            qA.set(aA);
            qB.set(aB);

            Rot.multiplyVec2(qA, Vec2.subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), rA);
            Rot.multiplyVec2(qB, Vec2.subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), rB);
            Vec2.subtract(cB, cA, d).add(rB).subtract(rA);

            Rot.multiplyVec2(qA, this.m_localYAxisA, ay);

            const sAy = Vec2.cross(Vec2.add(d, rA, Vec2.s_t0), ay);
            const sBy = Vec2.cross(rB, ay);

            const C = Vec2.dot(d, ay);

            const invMass =
                this.m_invMassA +
                this.m_invMassB +
                this.m_invIA * this.m_sAy * this.m_sAy +
                this.m_invIB * this.m_sBy * this.m_sBy;

            let impulse = 0;
            if (invMass !== 0) {
                impulse = -C / invMass;
            }

            Vec2.scale(impulse, ay, P);
            const LA = impulse * sAy;
            const LB = impulse * sBy;

            cA.subtractScaled(this.m_invMassA, P);
            aA -= this.m_invIA * LA;
            cB.addScaled(this.m_invMassB, P);
            aB += this.m_invIB * LB;

            linearError = Math.max(linearError, Math.abs(C));
        }

        data.positions[this.m_indexA].a = aA;
        data.positions[this.m_indexB].a = aB;

        return linearError <= LINEAR_SLOP;
    }

    public getAnchorA<T extends XY>(out: T): T {
        return this.m_bodyA.getWorldPoint(this.m_localAnchorA, out);
    }

    public getAnchorB<T extends XY>(out: T): T {
        return this.m_bodyB.getWorldPoint(this.m_localAnchorB, out);
    }

    public getReactionForce<T extends XY>(inv_dt: number, out: T): T {
        const f = this.m_springImpulse + this.m_lowerImpulse - this.m_upperImpulse;
        out.x = inv_dt * (this.m_impulse * this.m_ay.x + f * this.m_ax.x);
        out.y = inv_dt * (this.m_impulse * this.m_ay.y + f * this.m_ax.y);
        return out;
    }

    public getReactionTorque(inv_dt: number): number {
        return inv_dt * this.m_motorImpulse;
    }

    public getLocalAnchorA(): Readonly<Vec2> {
        return this.m_localAnchorA;
    }

    public getLocalAnchorB(): Readonly<Vec2> {
        return this.m_localAnchorB;
    }

    public getLocalAxisA(): Readonly<Vec2> {
        return this.m_localXAxisA;
    }

    public getJointTranslation(): number {
        const bA = this.m_bodyA;
        const bB = this.m_bodyB;

        const { pA, pB, d, axis } = temp;
        bA.getWorldPoint(this.m_localAnchorA, pA);
        bB.getWorldPoint(this.m_localAnchorB, pB);
        Vec2.subtract(pB, pA, d);
        bA.getWorldVector(this.m_localXAxisA, axis);

        const translation = Vec2.dot(d, axis);
        return translation;
    }

    public getJointLinearSpeed(): number {
        const bA = this.m_bodyA;
        const bB = this.m_bodyB;

        const { rA, rB, lalcA, lalcB, axis } = temp;
        Rot.multiplyVec2(bA.m_xf.q, Vec2.subtract(this.m_localAnchorA, bA.m_sweep.localCenter, lalcA), rA);
        Rot.multiplyVec2(bB.m_xf.q, Vec2.subtract(this.m_localAnchorB, bB.m_sweep.localCenter, lalcB), rB);
        const p1 = Vec2.add(bA.m_sweep.c, rA, Vec2.s_t0);
        const p2 = Vec2.add(bB.m_sweep.c, rB, Vec2.s_t1);
        const d = Vec2.subtract(p2, p1, Vec2.s_t2);
        Rot.multiplyVec2(bA.m_xf.q, this.m_localXAxisA, axis);

        const vA = bA.m_linearVelocity;
        const vB = bB.m_linearVelocity;
        const wA = bA.m_angularVelocity;
        const wB = bB.m_angularVelocity;

        const speed =
            Vec2.dot(d, Vec2.crossScalarVec2(wA, axis, Vec2.s_t0)) +
            Vec2.dot(
                axis,
                Vec2.addCrossScalarVec2(vB, wB, rB, Vec2.s_t0)
                    .subtract(vA)
                    .subtract(Vec2.crossScalarVec2(wA, rA, Vec2.s_t1)),
            );
        return speed;
    }

    public getJointAngle(): number {
        return this.m_bodyB.m_sweep.a - this.m_bodyA.m_sweep.a;
    }

    public getJointAngularSpeed(): number {
        const wA = this.m_bodyA.m_angularVelocity;
        const wB = this.m_bodyB.m_angularVelocity;
        return wB - wA;
    }

    public isMotorEnabled(): boolean {
        return this.m_enableMotor;
    }

    public enableMotor(flag: boolean): boolean {
        if (flag !== this.m_enableMotor) {
            this.m_bodyA.setAwake(true);
            this.m_bodyB.setAwake(true);
            this.m_enableMotor = flag;
        }
        return flag;
    }

    public setMotorSpeed(speed: number): number {
        if (speed !== this.m_motorSpeed) {
            this.m_bodyA.setAwake(true);
            this.m_bodyB.setAwake(true);
            this.m_motorSpeed = speed;
        }
        return speed;
    }

    public setMaxMotorTorque(torque: number): void {
        if (torque !== this.m_maxMotorTorque) {
            this.m_bodyA.setAwake(true);
            this.m_bodyB.setAwake(true);
            this.m_maxMotorTorque = torque;
        }
    }

    public getMotorTorque(inv_dt: number): number {
        return inv_dt * this.m_motorImpulse;
    }

    /**
     * Is the joint limit enabled?
     */
    public isLimitEnabled(): boolean {
        return this.m_enableLimit;
    }

    /**
     * Enable/disable the joint translation limit.
     */
    public enableLimit(flag: boolean): boolean {
        if (flag !== this.m_enableLimit) {
            this.m_bodyA.setAwake(true);
            this.m_bodyB.setAwake(true);
            this.m_enableLimit = flag;
            this.m_lowerImpulse = 0;
            this.m_upperImpulse = 0;
        }
        return flag;
    }

    /**
     * Get the lower joint translation limit, usually in meters.
     */
    public getLowerLimit(): number {
        return this.m_lowerTranslation;
    }

    /**
     * Get the upper joint translation limit, usually in meters.
     */
    public getUpperLimit(): number {
        return this.m_upperTranslation;
    }

    /**
     * Set the joint translation limits, usually in meters.
     */
    public setLimits(lower: number, upper: number): void {
        // assert(lower <= upper);
        if (lower !== this.m_lowerTranslation || upper !== this.m_upperTranslation) {
            this.m_bodyA.setAwake(true);
            this.m_bodyB.setAwake(true);
            this.m_lowerTranslation = lower;
            this.m_upperTranslation = upper;
            this.m_lowerImpulse = 0;
            this.m_upperImpulse = 0;
        }
    }

    public draw(draw: Draw): void {
        const { p1, p2, pA, pB, axis } = temp.Draw;
        const xfA = this.m_bodyA.getTransform();
        const xfB = this.m_bodyB.getTransform();
        Transform.multiplyVec2(xfA, this.m_localAnchorA, pA);
        Transform.multiplyVec2(xfB, this.m_localAnchorB, pB);

        Rot.multiplyVec2(xfA.q, this.m_localXAxisA, axis);

        draw.drawSegment(pA, pB, debugColors.joint5);

        if (this.m_enableLimit) {
            const { lower, upper, perp } = temp.Draw;
            Vec2.addScaled(pA, this.m_lowerTranslation, axis, lower);
            Vec2.addScaled(pA, this.m_upperTranslation, axis, upper);
            Rot.multiplyVec2(xfA.q, this.m_localYAxisA, perp);
            draw.drawSegment(lower, upper, debugColors.joint1);
            draw.drawSegment(
                Vec2.subtractScaled(lower, 0.5, perp, p1),
                Vec2.addScaled(lower, 0.5, perp, p2),
                debugColors.joint2,
            );
            draw.drawSegment(
                Vec2.subtractScaled(upper, 0.5, perp, p1),
                Vec2.addScaled(upper, 0.5, perp, p2),
                debugColors.joint3,
            );
        } else {
            draw.drawSegment(Vec2.subtract(pA, axis, p1), Vec2.add(pA, axis, p2), debugColors.joint1);
        }

        draw.drawPoint(pA, 5, debugColors.joint1);
        draw.drawPoint(pB, 5, debugColors.joint4);
    }
}
