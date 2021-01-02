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
    protected readonly localAnchorA = new Vec2();

    protected readonly localAnchorB = new Vec2();

    protected readonly localXAxisA = new Vec2();

    protected readonly localYAxisA = new Vec2();

    protected impulse = 0;

    protected motorImpulse = 0;

    protected springImpulse = 0;

    protected lowerImpulse = 0;

    protected upperImpulse = 0;

    protected translation = 0;

    protected lowerTranslation = 0;

    protected upperTranslation = 0;

    protected maxMotorTorque = 0;

    protected motorSpeed = 0;

    protected m_enableLimit = false;

    protected m_enableMotor = false;

    protected stiffness = 0;

    protected damping = 0;

    // Solver temp
    protected indexA = 0;

    protected indexB = 0;

    protected readonly localCenterA = new Vec2();

    protected readonly localCenterB = new Vec2();

    protected invMassA = 0;

    protected invMassB = 0;

    protected invIA = 0;

    protected invIB = 0;

    protected readonly ax = new Vec2();

    protected readonly ay = new Vec2();

    protected sAx = 0;

    protected sBx = 0;

    protected sAy = 0;

    protected sBy = 0;

    protected mass = 0;

    protected motorMass = 0;

    protected axialMass = 0;

    protected springMass = 0;

    protected bias = 0;

    protected gamma = 0;

    /** @internal protected */
    public constructor(def: IWheelJointDef) {
        super(def);

        this.localAnchorA.copy(def.localAnchorA ?? Vec2.ZERO);
        this.localAnchorB.copy(def.localAnchorB ?? Vec2.ZERO);
        this.localXAxisA.copy(def.localAxisA ?? Vec2.UNITX);
        Vec2.crossOneVec2(this.localXAxisA, this.localYAxisA);

        this.lowerTranslation = def.lowerTranslation ?? 0;
        this.upperTranslation = def.upperTranslation ?? 0;
        this.m_enableLimit = def.enableLimit ?? false;

        this.maxMotorTorque = def.maxMotorTorque ?? 0;
        this.motorSpeed = def.motorSpeed ?? 0;
        this.m_enableMotor = def.enableMotor ?? false;

        this.ax.setZero();
        this.ay.setZero();

        this.stiffness = def.stiffness ?? 0;
        this.damping = def.damping ?? 0;
    }

    public getMotorSpeed(): number {
        return this.motorSpeed;
    }

    public getMaxMotorTorque(): number {
        return this.maxMotorTorque;
    }

    public setStiffness(stiffness: number): void {
        this.stiffness = stiffness;
    }

    public getStiffness(): number {
        return this.stiffness;
    }

    public setDamping(damping: number): void {
        this.damping = damping;
    }

    public getDamping(): number {
        return this.damping;
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

        const mA = this.invMassA;
        const mB = this.invMassB;
        const iA = this.invIA;
        const iB = this.invIB;

        const cA = data.positions[this.indexA].c;
        const aA = data.positions[this.indexA].a;
        const vA = data.velocities[this.indexA].v;
        let wA = data.velocities[this.indexA].w;

        const cB = data.positions[this.indexB].c;
        const aB = data.positions[this.indexB].a;
        const vB = data.velocities[this.indexB].v;
        let wB = data.velocities[this.indexB].w;

        const { qA, qB, lalcA, lalcB, rA, rB, d } = temp;
        qA.set(aA);
        qB.set(aB);

        // Compute the effective masses.
        Rot.multiplyVec2(qA, Vec2.subtract(this.localAnchorA, this.localCenterA, lalcA), rA);
        Rot.multiplyVec2(qB, Vec2.subtract(this.localAnchorB, this.localCenterB, lalcB), rB);
        Vec2.add(cB, rB, d).subtract(cA).subtract(rA);

        // Point to line constraint
        Rot.multiplyVec2(qA, this.localYAxisA, this.ay);
        this.sAy = Vec2.cross(Vec2.add(d, rA, Vec2.s_t0), this.ay);
        this.sBy = Vec2.cross(rB, this.ay);

        this.mass = mA + mB + iA * this.sAy * this.sAy + iB * this.sBy * this.sBy;

        if (this.mass > 0) {
            this.mass = 1 / this.mass;
        }

        // Spring constraint
        Rot.multiplyVec2(qA, this.localXAxisA, this.ax);
        this.sAx = Vec2.cross(Vec2.add(d, rA, Vec2.s_t0), this.ax);
        this.sBx = Vec2.cross(rB, this.ax);

        const invMass = mA + mB + iA * this.sAx * this.sAx + iB * this.sBx * this.sBx;
        if (invMass > 0) {
            this.axialMass = 1 / invMass;
        } else {
            this.axialMass = 0;
        }

        this.springMass = 0;
        this.bias = 0;
        this.gamma = 0;

        if (this.stiffness > 0 && invMass > 0) {
            this.springMass = 1 / invMass;

            const C = Vec2.dot(d, this.ax);

            // magic formulas
            const h = data.step.dt;
            this.gamma = h * (this.damping + h * this.stiffness);
            if (this.gamma > 0) {
                this.gamma = 1 / this.gamma;
            }

            this.bias = C * h * this.stiffness * this.gamma;

            this.springMass = invMass + this.gamma;
            if (this.springMass > 0) {
                this.springMass = 1 / this.springMass;
            }
        } else {
            this.springImpulse = 0;
        }

        if (this.m_enableLimit) {
            this.translation = Vec2.dot(this.ax, d);
        } else {
            this.lowerImpulse = 0;
            this.upperImpulse = 0;
        }

        if (this.m_enableMotor) {
            this.motorMass = iA + iB;
            if (this.motorMass > 0) {
                this.motorMass = 1 / this.motorMass;
            }
        } else {
            this.motorMass = 0;
            this.motorImpulse = 0;
        }

        if (data.step.warmStarting) {
            // Account for variable time step.
            this.impulse *= data.step.dtRatio;
            this.springImpulse *= data.step.dtRatio;
            this.motorImpulse *= data.step.dtRatio;

            const axialImpulse = this.springImpulse + this.lowerImpulse - this.upperImpulse;
            const { P } = temp;
            Vec2.scale(this.impulse, this.ay, P).addScaled(axialImpulse, this.ax);
            const LA = this.impulse * this.sAy + axialImpulse * this.sAx + this.motorImpulse;
            const LB = this.impulse * this.sBy + axialImpulse * this.sBx + this.motorImpulse;

            vA.subtractScaled(this.invMassA, P);
            wA -= this.invIA * LA;

            vB.addScaled(this.invMassB, P);
            wB += this.invIB * LB;
        } else {
            this.impulse = 0;
            this.springImpulse = 0;
            this.motorImpulse = 0;
            this.lowerImpulse = 0;
            this.upperImpulse = 0;
        }

        data.velocities[this.indexA].w = wA;
        data.velocities[this.indexB].w = wB;
    }

    /** @internal protected */
    public solveVelocityConstraints(data: SolverData): void {
        const mA = this.invMassA;
        const mB = this.invMassB;
        const iA = this.invIA;
        const iB = this.invIB;

        const vA = data.velocities[this.indexA].v;
        let wA = data.velocities[this.indexA].w;
        const vB = data.velocities[this.indexB].v;
        let wB = data.velocities[this.indexB].w;

        const { P } = temp;
        // Solve spring constraint
        {
            const Cdot = Vec2.dot(this.ax, Vec2.subtract(vB, vA, Vec2.s_t0)) + this.sBx * wB - this.sAx * wA;
            const impulse = -this.springMass * (Cdot + this.bias + this.gamma * this.springImpulse);
            this.springImpulse += impulse;

            Vec2.scale(impulse, this.ax, P);
            const LA = impulse * this.sAx;
            const LB = impulse * this.sBx;

            vA.subtractScaled(mA, P);
            wA -= iA * LA;

            vB.addScaled(mB, P);
            wB += iB * LB;
        }

        // Solve rotational motor constraint
        {
            const Cdot = wB - wA - this.motorSpeed;
            let impulse = -this.motorMass * Cdot;

            const oldImpulse = this.motorImpulse;
            const maxImpulse = data.step.dt * this.maxMotorTorque;
            this.motorImpulse = clamp(this.motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.motorImpulse - oldImpulse;

            wA -= iA * impulse;
            wB += iB * impulse;
        }

        if (this.m_enableLimit) {
            // Lower limit
            {
                const C = this.translation - this.lowerTranslation;
                const Cdot = Vec2.dot(this.ax, Vec2.subtract(vB, vA, Vec2.s_t0)) + this.sBx * wB - this.sAx * wA;
                let impulse = -this.axialMass * (Cdot + Math.max(C, 0) * data.step.inv_dt);
                const oldImpulse = this.lowerImpulse;
                this.lowerImpulse = Math.max(this.lowerImpulse + impulse, 0);
                impulse = this.lowerImpulse - oldImpulse;

                Vec2.scale(impulse, this.ax, P);
                const LA = impulse * this.sAx;
                const LB = impulse * this.sBx;

                vA.subtractScaled(mA, P);
                wA -= iA * LA;
                vB.addScaled(mB, P);
                wB += iB * LB;
            }

            // Upper limit
            // Note: signs are flipped to keep C positive when the constraint is satisfied.
            // This also keeps the impulse positive when the limit is active.
            {
                const C = this.upperTranslation - this.translation;
                const Cdot = Vec2.dot(this.ax, Vec2.subtract(vA, vB, Vec2.s_t0)) + this.sAx * wA - this.sBx * wB;
                let impulse = -this.axialMass * (Cdot + Math.max(C, 0) * data.step.inv_dt);
                const oldImpulse = this.upperImpulse;
                this.upperImpulse = Math.max(this.upperImpulse + impulse, 0);
                impulse = this.upperImpulse - oldImpulse;

                Vec2.scale(impulse, this.ax, P);
                const LA = impulse * this.sAx;
                const LB = impulse * this.sBx;

                vA.addScaled(mA, P);
                wA += iA * LA;
                vB.subtractScaled(mB, P);
                wB -= iB * LB;
            }
        }

        // Solve point to line constraint
        {
            const Cdot = Vec2.dot(this.ay, Vec2.subtract(vB, vA, Vec2.s_t0)) + this.sBy * wB - this.sAy * wA;
            const impulse = -this.mass * Cdot;
            this.impulse += impulse;

            Vec2.scale(impulse, this.ay, P);
            const LA = impulse * this.sAy;
            const LB = impulse * this.sBy;

            vA.subtractScaled(mA, P);
            wA -= iA * LA;

            vB.addScaled(mB, P);
            wB += iB * LB;
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

        let linearError = 0;

        const { qA, qB, lalcA, lalcB, rA, rB, d, P, ay } = temp;

        if (this.m_enableLimit) {
            qA.set(aA);
            qB.set(aB);

            Rot.multiplyVec2(qA, Vec2.subtract(this.localAnchorA, this.localCenterA, lalcA), rA);
            Rot.multiplyVec2(qB, Vec2.subtract(this.localAnchorB, this.localCenterB, lalcB), rB);
            Vec2.subtract(cB, cA, d).add(rB).subtract(rA);

            const ax = Rot.multiplyVec2(qA, this.localXAxisA, this.ax);
            const sAx = Vec2.cross(Vec2.add(d, rA, Vec2.s_t0), this.ax);
            const sBx = Vec2.cross(rB, this.ax);

            let C = 0;
            const translation = Vec2.dot(ax, d);
            if (Math.abs(this.upperTranslation - this.lowerTranslation) < 2 * LINEAR_SLOP) {
                C = translation;
            } else if (translation <= this.lowerTranslation) {
                C = Math.min(translation - this.lowerTranslation, 0);
            } else if (translation >= this.upperTranslation) {
                C = Math.max(translation - this.upperTranslation, 0);
            }

            if (C !== 0) {
                const invMass = this.invMassA + this.invMassB + this.invIA * sAx * sAx + this.invIB * sBx * sBx;
                let impulse = 0;
                if (invMass !== 0) {
                    impulse = -C / invMass;
                }

                Vec2.scale(impulse, ax, P);
                const LA = impulse * sAx;
                const LB = impulse * sBx;

                cA.subtractScaled(this.invMassA, P);
                aA -= this.invIA * LA;
                cB.addScaled(this.invMassB, P);
                aB += this.invIB * LB;

                linearError = Math.abs(C);
            }
        }

        // Solve perpendicular constraint
        {
            qA.set(aA);
            qB.set(aB);

            Rot.multiplyVec2(qA, Vec2.subtract(this.localAnchorA, this.localCenterA, lalcA), rA);
            Rot.multiplyVec2(qB, Vec2.subtract(this.localAnchorB, this.localCenterB, lalcB), rB);
            Vec2.subtract(cB, cA, d).add(rB).subtract(rA);

            Rot.multiplyVec2(qA, this.localYAxisA, ay);

            const sAy = Vec2.cross(Vec2.add(d, rA, Vec2.s_t0), ay);
            const sBy = Vec2.cross(rB, ay);

            const C = Vec2.dot(d, ay);

            const invMass =
                this.invMassA + this.invMassB + this.invIA * this.sAy * this.sAy + this.invIB * this.sBy * this.sBy;

            let impulse = 0;
            if (invMass !== 0) {
                impulse = -C / invMass;
            }

            Vec2.scale(impulse, ay, P);
            const LA = impulse * sAy;
            const LB = impulse * sBy;

            cA.subtractScaled(this.invMassA, P);
            aA -= this.invIA * LA;
            cB.addScaled(this.invMassB, P);
            aB += this.invIB * LB;

            linearError = Math.max(linearError, Math.abs(C));
        }

        data.positions[this.indexA].a = aA;
        data.positions[this.indexB].a = aB;

        return linearError <= LINEAR_SLOP;
    }

    public getAnchorA<T extends XY>(out: T): T {
        return this.bodyA.getWorldPoint(this.localAnchorA, out);
    }

    public getAnchorB<T extends XY>(out: T): T {
        return this.bodyB.getWorldPoint(this.localAnchorB, out);
    }

    public getReactionForce<T extends XY>(inv_dt: number, out: T): T {
        const f = this.springImpulse + this.lowerImpulse - this.upperImpulse;
        out.x = inv_dt * (this.impulse * this.ay.x + f * this.ax.x);
        out.y = inv_dt * (this.impulse * this.ay.y + f * this.ax.y);
        return out;
    }

    public getReactionTorque(inv_dt: number): number {
        return inv_dt * this.motorImpulse;
    }

    public getLocalAnchorA(): Readonly<Vec2> {
        return this.localAnchorA;
    }

    public getLocalAnchorB(): Readonly<Vec2> {
        return this.localAnchorB;
    }

    public getLocalAxisA(): Readonly<Vec2> {
        return this.localXAxisA;
    }

    public getJointTranslation(): number {
        const bA = this.bodyA;
        const bB = this.bodyB;

        const { pA, pB, d, axis } = temp;
        bA.getWorldPoint(this.localAnchorA, pA);
        bB.getWorldPoint(this.localAnchorB, pB);
        Vec2.subtract(pB, pA, d);
        bA.getWorldVector(this.localXAxisA, axis);

        const translation = Vec2.dot(d, axis);
        return translation;
    }

    public getJointLinearSpeed(): number {
        const bA = this.bodyA;
        const bB = this.bodyB;

        const { rA, rB, lalcA, lalcB, axis } = temp;
        Rot.multiplyVec2(bA.xf.q, Vec2.subtract(this.localAnchorA, bA.sweep.localCenter, lalcA), rA);
        Rot.multiplyVec2(bB.xf.q, Vec2.subtract(this.localAnchorB, bB.sweep.localCenter, lalcB), rB);
        const p1 = Vec2.add(bA.sweep.c, rA, Vec2.s_t0);
        const p2 = Vec2.add(bB.sweep.c, rB, Vec2.s_t1);
        const d = Vec2.subtract(p2, p1, Vec2.s_t2);
        Rot.multiplyVec2(bA.xf.q, this.localXAxisA, axis);

        const vA = bA.linearVelocity;
        const vB = bB.linearVelocity;
        const wA = bA.angularVelocity;
        const wB = bB.angularVelocity;

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
        return this.bodyB.sweep.a - this.bodyA.sweep.a;
    }

    public getJointAngularSpeed(): number {
        const wA = this.bodyA.angularVelocity;
        const wB = this.bodyB.angularVelocity;
        return wB - wA;
    }

    public isMotorEnabled(): boolean {
        return this.m_enableMotor;
    }

    public enableMotor(flag: boolean): boolean {
        if (flag !== this.m_enableMotor) {
            this.bodyA.setAwake(true);
            this.bodyB.setAwake(true);
            this.m_enableMotor = flag;
        }
        return flag;
    }

    public setMotorSpeed(speed: number): number {
        if (speed !== this.motorSpeed) {
            this.bodyA.setAwake(true);
            this.bodyB.setAwake(true);
            this.motorSpeed = speed;
        }
        return speed;
    }

    public setMaxMotorTorque(torque: number): void {
        if (torque !== this.maxMotorTorque) {
            this.bodyA.setAwake(true);
            this.bodyB.setAwake(true);
            this.maxMotorTorque = torque;
        }
    }

    public getMotorTorque(inv_dt: number): number {
        return inv_dt * this.motorImpulse;
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
            this.bodyA.setAwake(true);
            this.bodyB.setAwake(true);
            this.m_enableLimit = flag;
            this.lowerImpulse = 0;
            this.upperImpulse = 0;
        }
        return flag;
    }

    /**
     * Get the lower joint translation limit, usually in meters.
     */
    public getLowerLimit(): number {
        return this.lowerTranslation;
    }

    /**
     * Get the upper joint translation limit, usually in meters.
     */
    public getUpperLimit(): number {
        return this.upperTranslation;
    }

    /**
     * Set the joint translation limits, usually in meters.
     */
    public setLimits(lower: number, upper: number): void {
        // assert(lower <= upper);
        if (lower !== this.lowerTranslation || upper !== this.upperTranslation) {
            this.bodyA.setAwake(true);
            this.bodyB.setAwake(true);
            this.lowerTranslation = lower;
            this.upperTranslation = upper;
            this.lowerImpulse = 0;
            this.upperImpulse = 0;
        }
    }

    public draw(draw: Draw): void {
        const { p1, p2, pA, pB, axis } = temp.Draw;
        const xfA = this.bodyA.getTransform();
        const xfB = this.bodyB.getTransform();
        Transform.multiplyVec2(xfA, this.localAnchorA, pA);
        Transform.multiplyVec2(xfB, this.localAnchorB, pB);

        Rot.multiplyVec2(xfA.q, this.localXAxisA, axis);

        draw.drawSegment(pA, pB, debugColors.joint5);

        if (this.m_enableLimit) {
            const { lower, upper, perp } = temp.Draw;
            Vec2.addScaled(pA, this.lowerTranslation, axis, lower);
            Vec2.addScaled(pA, this.upperTranslation, axis, upper);
            Rot.multiplyVec2(xfA.q, this.localYAxisA, perp);
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
