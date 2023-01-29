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
import { b2_linearSlop } from "../common/b2_common";
import { b2Clamp, b2Vec2, b2Rot, XY, b2Transform } from "../common/b2_math";
import { b2Joint, b2JointDef, b2JointType, b2IJointDef } from "./b2_joint";
import { b2SolverData } from "./b2_time_step";
import { b2Body } from "./b2_body";
import { b2Draw, debugColors } from "../common/b2_draw";

const temp = {
    qA: new b2Rot(),
    qB: new b2Rot(),
    lalcA: new b2Vec2(),
    lalcB: new b2Vec2(),
    rA: new b2Vec2(),
    rB: new b2Vec2(),
    d: new b2Vec2(),
    P: new b2Vec2(),
    ay: new b2Vec2(),
    pA: new b2Vec2(),
    pB: new b2Vec2(),
    axis: new b2Vec2(),
    Draw: {
        p1: new b2Vec2(),
        p2: new b2Vec2(),
        pA: new b2Vec2(),
        pB: new b2Vec2(),
        axis: new b2Vec2(),
        lower: new b2Vec2(),
        upper: new b2Vec2(),
        perp: new b2Vec2(),
    },
};

export interface b2IWheelJointDef extends b2IJointDef {
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
export class b2WheelJointDef extends b2JointDef implements b2IWheelJointDef {
    /** The local anchor point relative to bodyA's origin. */
    public readonly localAnchorA = new b2Vec2();

    /** The local anchor point relative to bodyB's origin. */
    public readonly localAnchorB = new b2Vec2();

    /** The local translation axis in bodyA. */
    public readonly localAxisA = new b2Vec2(1, 0);

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
        super(b2JointType.e_wheelJoint);
    }

    public Initialize(bA: b2Body, bB: b2Body, anchor: Readonly<XY>, axis: Readonly<XY>): void {
        this.bodyA = bA;
        this.bodyB = bB;
        this.bodyA.GetLocalPoint(anchor, this.localAnchorA);
        this.bodyB.GetLocalPoint(anchor, this.localAnchorB);
        this.bodyA.GetLocalVector(axis, this.localAxisA);
    }
}

/**
 * A wheel joint. This joint provides two degrees of freedom: translation
 * along an axis fixed in bodyA and rotation in the plane. In other words, it is a point to
 * line constraint with a rotational motor and a linear spring/damper. The spring/damper is
 * initialized upon creation. This joint is designed for vehicle suspensions.
 */
export class b2WheelJoint extends b2Joint {
    protected readonly m_localAnchorA = new b2Vec2();

    protected readonly m_localAnchorB = new b2Vec2();

    protected readonly m_localXAxisA = new b2Vec2();

    protected readonly m_localYAxisA = new b2Vec2();

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

    protected readonly m_localCenterA = new b2Vec2();

    protected readonly m_localCenterB = new b2Vec2();

    protected m_invMassA = 0;

    protected m_invMassB = 0;

    protected m_invIA = 0;

    protected m_invIB = 0;

    protected readonly m_ax = new b2Vec2();

    protected readonly m_ay = new b2Vec2();

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
    public constructor(def: b2IWheelJointDef) {
        super(def);

        this.m_localAnchorA.Copy(def.localAnchorA ?? b2Vec2.ZERO);
        this.m_localAnchorB.Copy(def.localAnchorB ?? b2Vec2.ZERO);
        this.m_localXAxisA.Copy(def.localAxisA ?? b2Vec2.UNITX);
        b2Vec2.CrossOneVec2(this.m_localXAxisA, this.m_localYAxisA);

        this.m_lowerTranslation = def.lowerTranslation ?? 0;
        this.m_upperTranslation = def.upperTranslation ?? 0;
        this.m_enableLimit = def.enableLimit ?? false;

        this.m_maxMotorTorque = def.maxMotorTorque ?? 0;
        this.m_motorSpeed = def.motorSpeed ?? 0;
        this.m_enableMotor = def.enableMotor ?? false;

        this.m_ax.SetZero();
        this.m_ay.SetZero();

        this.m_stiffness = def.stiffness ?? 0;
        this.m_damping = def.damping ?? 0;
    }

    public GetMotorSpeed(): number {
        return this.m_motorSpeed;
    }

    public GetMaxMotorTorque(): number {
        return this.m_maxMotorTorque;
    }

    public SetStiffness(stiffness: number): void {
        this.m_stiffness = stiffness;
    }

    public GetStiffness(): number {
        return this.m_stiffness;
    }

    public SetDamping(damping: number): void {
        this.m_damping = damping;
    }

    public GetDamping(): number {
        return this.m_damping;
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
        qA.Set(aA);
        qB.Set(aB);

        // Compute the effective masses.
        b2Rot.MultiplyVec2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), rA);
        b2Rot.MultiplyVec2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), rB);
        b2Vec2.Add(cB, rB, d).Subtract(cA).Subtract(rA);

        // Point to line constraint
        b2Rot.MultiplyVec2(qA, this.m_localYAxisA, this.m_ay);
        this.m_sAy = b2Vec2.Cross(b2Vec2.Add(d, rA, b2Vec2.s_t0), this.m_ay);
        this.m_sBy = b2Vec2.Cross(rB, this.m_ay);

        this.m_mass = mA + mB + iA * this.m_sAy * this.m_sAy + iB * this.m_sBy * this.m_sBy;

        if (this.m_mass > 0) {
            this.m_mass = 1 / this.m_mass;
        }

        // Spring constraint
        b2Rot.MultiplyVec2(qA, this.m_localXAxisA, this.m_ax);
        this.m_sAx = b2Vec2.Cross(b2Vec2.Add(d, rA, b2Vec2.s_t0), this.m_ax);
        this.m_sBx = b2Vec2.Cross(rB, this.m_ax);

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

            const C = b2Vec2.Dot(d, this.m_ax);

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
            this.m_translation = b2Vec2.Dot(this.m_ax, d);
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
            b2Vec2.Scale(this.m_impulse, this.m_ay, P).AddScaled(axialImpulse, this.m_ax);
            const LA = this.m_impulse * this.m_sAy + axialImpulse * this.m_sAx + this.m_motorImpulse;
            const LB = this.m_impulse * this.m_sBy + axialImpulse * this.m_sBx + this.m_motorImpulse;

            vA.SubtractScaled(this.m_invMassA, P);
            wA -= this.m_invIA * LA;

            vB.AddScaled(this.m_invMassB, P);
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
    public SolveVelocityConstraints(data: b2SolverData): void {
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
            const Cdot =
                b2Vec2.Dot(this.m_ax, b2Vec2.Subtract(vB, vA, b2Vec2.s_t0)) + this.m_sBx * wB - this.m_sAx * wA;
            const impulse = -this.m_springMass * (Cdot + this.m_bias + this.m_gamma * this.m_springImpulse);
            this.m_springImpulse += impulse;

            b2Vec2.Scale(impulse, this.m_ax, P);
            const LA = impulse * this.m_sAx;
            const LB = impulse * this.m_sBx;

            vA.SubtractScaled(mA, P);
            wA -= iA * LA;

            vB.AddScaled(mB, P);
            wB += iB * LB;
        }

        // Solve rotational motor constraint
        {
            const Cdot = wB - wA - this.m_motorSpeed;
            let impulse = -this.m_motorMass * Cdot;

            const oldImpulse = this.m_motorImpulse;
            const maxImpulse = data.step.dt * this.m_maxMotorTorque;
            this.m_motorImpulse = b2Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.m_motorImpulse - oldImpulse;

            wA -= iA * impulse;
            wB += iB * impulse;
        }

        if (this.m_enableLimit) {
            // Lower limit
            {
                const C = this.m_translation - this.m_lowerTranslation;
                const Cdot =
                    b2Vec2.Dot(this.m_ax, b2Vec2.Subtract(vB, vA, b2Vec2.s_t0)) + this.m_sBx * wB - this.m_sAx * wA;
                let impulse = -this.m_axialMass * (Cdot + Math.max(C, 0) * data.step.inv_dt);
                const oldImpulse = this.m_lowerImpulse;
                this.m_lowerImpulse = Math.max(this.m_lowerImpulse + impulse, 0);
                impulse = this.m_lowerImpulse - oldImpulse;

                b2Vec2.Scale(impulse, this.m_ax, P);
                const LA = impulse * this.m_sAx;
                const LB = impulse * this.m_sBx;

                vA.SubtractScaled(mA, P);
                wA -= iA * LA;
                vB.AddScaled(mB, P);
                wB += iB * LB;
            }

            // Upper limit
            // Note: signs are flipped to keep C positive when the constraint is satisfied.
            // This also keeps the impulse positive when the limit is active.
            {
                const C = this.m_upperTranslation - this.m_translation;
                const Cdot =
                    b2Vec2.Dot(this.m_ax, b2Vec2.Subtract(vA, vB, b2Vec2.s_t0)) + this.m_sAx * wA - this.m_sBx * wB;
                let impulse = -this.m_axialMass * (Cdot + Math.max(C, 0) * data.step.inv_dt);
                const oldImpulse = this.m_upperImpulse;
                this.m_upperImpulse = Math.max(this.m_upperImpulse + impulse, 0);
                impulse = this.m_upperImpulse - oldImpulse;

                b2Vec2.Scale(impulse, this.m_ax, P);
                const LA = impulse * this.m_sAx;
                const LB = impulse * this.m_sBx;

                vA.AddScaled(mA, P);
                wA += iA * LA;
                vB.SubtractScaled(mB, P);
                wB -= iB * LB;
            }
        }

        // Solve point to line constraint
        {
            const Cdot =
                b2Vec2.Dot(this.m_ay, b2Vec2.Subtract(vB, vA, b2Vec2.s_t0)) + this.m_sBy * wB - this.m_sAy * wA;
            const impulse = -this.m_mass * Cdot;
            this.m_impulse += impulse;

            b2Vec2.Scale(impulse, this.m_ay, P);
            const LA = impulse * this.m_sAy;
            const LB = impulse * this.m_sBy;

            vA.SubtractScaled(mA, P);
            wA -= iA * LA;

            vB.AddScaled(mB, P);
            wB += iB * LB;
        }

        data.velocities[this.m_indexA].w = wA;
        data.velocities[this.m_indexB].w = wB;
    }

    /** @internal protected */
    public SolvePositionConstraints(data: b2SolverData): boolean {
        const cA = data.positions[this.m_indexA].c;
        let aA = data.positions[this.m_indexA].a;
        const cB = data.positions[this.m_indexB].c;
        let aB = data.positions[this.m_indexB].a;

        let linearError = 0;

        const { qA, qB, lalcA, lalcB, rA, rB, d, P, ay } = temp;

        if (this.m_enableLimit) {
            qA.Set(aA);
            qB.Set(aB);

            b2Rot.MultiplyVec2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), rA);
            b2Rot.MultiplyVec2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), rB);
            b2Vec2.Subtract(cB, cA, d).Add(rB).Subtract(rA);

            const ax = b2Rot.MultiplyVec2(qA, this.m_localXAxisA, this.m_ax);
            const sAx = b2Vec2.Cross(b2Vec2.Add(d, rA, b2Vec2.s_t0), this.m_ax);
            const sBx = b2Vec2.Cross(rB, this.m_ax);

            let C = 0;
            const translation = b2Vec2.Dot(ax, d);
            if (Math.abs(this.m_upperTranslation - this.m_lowerTranslation) < 2 * b2_linearSlop) {
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

                b2Vec2.Scale(impulse, ax, P);
                const LA = impulse * sAx;
                const LB = impulse * sBx;

                cA.SubtractScaled(this.m_invMassA, P);
                aA -= this.m_invIA * LA;
                cB.AddScaled(this.m_invMassB, P);
                aB += this.m_invIB * LB;

                linearError = Math.abs(C);
            }
        }

        // Solve perpendicular constraint
        {
            qA.Set(aA);
            qB.Set(aB);

            b2Rot.MultiplyVec2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), rA);
            b2Rot.MultiplyVec2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), rB);
            b2Vec2.Subtract(cB, cA, d).Add(rB).Subtract(rA);

            b2Rot.MultiplyVec2(qA, this.m_localYAxisA, ay);

            const sAy = b2Vec2.Cross(b2Vec2.Add(d, rA, b2Vec2.s_t0), ay);
            const sBy = b2Vec2.Cross(rB, ay);

            const C = b2Vec2.Dot(d, ay);

            const invMass =
                this.m_invMassA +
                this.m_invMassB +
                this.m_invIA * this.m_sAy * this.m_sAy +
                this.m_invIB * this.m_sBy * this.m_sBy;

            let impulse = 0;
            if (invMass !== 0) {
                impulse = -C / invMass;
            }

            b2Vec2.Scale(impulse, ay, P);
            const LA = impulse * sAy;
            const LB = impulse * sBy;

            cA.SubtractScaled(this.m_invMassA, P);
            aA -= this.m_invIA * LA;
            cB.AddScaled(this.m_invMassB, P);
            aB += this.m_invIB * LB;

            linearError = Math.max(linearError, Math.abs(C));
        }

        data.positions[this.m_indexA].a = aA;
        data.positions[this.m_indexB].a = aB;

        return linearError <= b2_linearSlop;
    }

    public GetAnchorA<T extends XY>(out: T): T {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    }

    public GetAnchorB<T extends XY>(out: T): T {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }

    public GetReactionForce<T extends XY>(inv_dt: number, out: T): T {
        const f = this.m_springImpulse + this.m_lowerImpulse - this.m_upperImpulse;
        out.x = inv_dt * (this.m_impulse * this.m_ay.x + f * this.m_ax.x);
        out.y = inv_dt * (this.m_impulse * this.m_ay.y + f * this.m_ax.y);
        return out;
    }

    public GetReactionTorque(inv_dt: number): number {
        return inv_dt * this.m_motorImpulse;
    }

    public GetLocalAnchorA(): Readonly<b2Vec2> {
        return this.m_localAnchorA;
    }

    public GetLocalAnchorB(): Readonly<b2Vec2> {
        return this.m_localAnchorB;
    }

    public GetLocalAxisA(): Readonly<b2Vec2> {
        return this.m_localXAxisA;
    }

    public GetJointTranslation(): number {
        const bA = this.m_bodyA;
        const bB = this.m_bodyB;

        const { pA, pB, d, axis } = temp;
        bA.GetWorldPoint(this.m_localAnchorA, pA);
        bB.GetWorldPoint(this.m_localAnchorB, pB);
        b2Vec2.Subtract(pB, pA, d);
        bA.GetWorldVector(this.m_localXAxisA, axis);

        const translation = b2Vec2.Dot(d, axis);
        return translation;
    }

    public GetJointLinearSpeed(): number {
        const bA = this.m_bodyA;
        const bB = this.m_bodyB;

        const { rA, rB, lalcA, lalcB, axis } = temp;
        b2Rot.MultiplyVec2(bA.m_xf.q, b2Vec2.Subtract(this.m_localAnchorA, bA.m_sweep.localCenter, lalcA), rA);
        b2Rot.MultiplyVec2(bB.m_xf.q, b2Vec2.Subtract(this.m_localAnchorB, bB.m_sweep.localCenter, lalcB), rB);
        const p1 = b2Vec2.Add(bA.m_sweep.c, rA, b2Vec2.s_t0);
        const p2 = b2Vec2.Add(bB.m_sweep.c, rB, b2Vec2.s_t1);
        const d = b2Vec2.Subtract(p2, p1, b2Vec2.s_t2);
        b2Rot.MultiplyVec2(bA.m_xf.q, this.m_localXAxisA, axis);

        const vA = bA.m_linearVelocity;
        const vB = bB.m_linearVelocity;
        const wA = bA.m_angularVelocity;
        const wB = bB.m_angularVelocity;

        const speed =
            b2Vec2.Dot(d, b2Vec2.CrossScalarVec2(wA, axis, b2Vec2.s_t0)) +
            b2Vec2.Dot(
                axis,
                b2Vec2
                    .AddCrossScalarVec2(vB, wB, rB, b2Vec2.s_t0)
                    .Subtract(vA)
                    .Subtract(b2Vec2.CrossScalarVec2(wA, rA, b2Vec2.s_t1)),
            );
        return speed;
    }

    public GetJointAngle(): number {
        return this.m_bodyB.m_sweep.a - this.m_bodyA.m_sweep.a;
    }

    public GetJointAngularSpeed(): number {
        const wA = this.m_bodyA.m_angularVelocity;
        const wB = this.m_bodyB.m_angularVelocity;
        return wB - wA;
    }

    public IsMotorEnabled(): boolean {
        return this.m_enableMotor;
    }

    public EnableMotor(flag: boolean): boolean {
        if (flag !== this.m_enableMotor) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_enableMotor = flag;
        }
        return flag;
    }

    public SetMotorSpeed(speed: number): number {
        if (speed !== this.m_motorSpeed) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_motorSpeed = speed;
        }
        return speed;
    }

    public SetMaxMotorTorque(torque: number): void {
        if (torque !== this.m_maxMotorTorque) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_maxMotorTorque = torque;
        }
    }

    public GetMotorTorque(inv_dt: number): number {
        return inv_dt * this.m_motorImpulse;
    }

    /**
     * Is the joint limit enabled?
     */
    public IsLimitEnabled(): boolean {
        return this.m_enableLimit;
    }

    /**
     * Enable/disable the joint translation limit.
     */
    public EnableLimit(flag: boolean): boolean {
        if (flag !== this.m_enableLimit) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_enableLimit = flag;
            this.m_lowerImpulse = 0;
            this.m_upperImpulse = 0;
        }
        return flag;
    }

    /**
     * Get the lower joint translation limit, usually in meters.
     */
    public GetLowerLimit(): number {
        return this.m_lowerTranslation;
    }

    /**
     * Get the upper joint translation limit, usually in meters.
     */
    public GetUpperLimit(): number {
        return this.m_upperTranslation;
    }

    /**
     * Set the joint translation limits, usually in meters.
     */
    public SetLimits(lower: number, upper: number): void {
        // b2Assert(lower <= upper);
        if (lower !== this.m_lowerTranslation || upper !== this.m_upperTranslation) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_lowerTranslation = lower;
            this.m_upperTranslation = upper;
            this.m_lowerImpulse = 0;
            this.m_upperImpulse = 0;
        }
    }

    public Draw(draw: b2Draw): void {
        const { p1, p2, pA, pB, axis } = temp.Draw;
        const xfA = this.m_bodyA.GetTransform();
        const xfB = this.m_bodyB.GetTransform();
        b2Transform.MultiplyVec2(xfA, this.m_localAnchorA, pA);
        b2Transform.MultiplyVec2(xfB, this.m_localAnchorB, pB);

        b2Rot.MultiplyVec2(xfA.q, this.m_localXAxisA, axis);

        draw.DrawSegment(pA, pB, debugColors.joint5);

        if (this.m_enableLimit) {
            const { lower, upper, perp } = temp.Draw;
            b2Vec2.AddScaled(pA, this.m_lowerTranslation, axis, lower);
            b2Vec2.AddScaled(pA, this.m_upperTranslation, axis, upper);
            b2Rot.MultiplyVec2(xfA.q, this.m_localYAxisA, perp);
            draw.DrawSegment(lower, upper, debugColors.joint1);
            draw.DrawSegment(
                b2Vec2.SubtractScaled(lower, 0.5, perp, p1),
                b2Vec2.AddScaled(lower, 0.5, perp, p2),
                debugColors.joint2,
            );
            draw.DrawSegment(
                b2Vec2.SubtractScaled(upper, 0.5, perp, p1),
                b2Vec2.AddScaled(upper, 0.5, perp, p2),
                debugColors.joint3,
            );
        } else {
            draw.DrawSegment(b2Vec2.Subtract(pA, axis, p1), b2Vec2.Add(pA, axis, p2), debugColors.joint1);
        }

        draw.DrawPoint(pA, 5, debugColors.joint1);
        draw.DrawPoint(pB, 5, debugColors.joint4);
    }
}
