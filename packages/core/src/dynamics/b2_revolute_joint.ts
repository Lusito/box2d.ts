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

import { LINEAR_SLOP, ANGULAR_SLOP, MAX_ANGULAR_CORRECTION } from "../common/b2_common";
import { Draw, debugColors } from "../common/b2_draw";
import { Clamp, Vec2, Mat22, Rot, XY, Transform } from "../common/b2_math";
import { Body } from "./b2_body";
import { Joint, JointDef, JointType, IJointDef } from "./b2_joint";
import { SolverData } from "./b2_time_step";

const temp = {
    qA: new Rot(),
    qB: new Rot(),
    lalcA: new Vec2(),
    lalcB: new Vec2(),
    P: new Vec2(),
    Cdot: new Vec2(),
    C: new Vec2(),
    impulse: new Vec2(),
    p2: new Vec2(),
    r: new Vec2(),
    pA: new Vec2(),
    pB: new Vec2(),
    rlo: new Vec2(),
    rhi: new Vec2(),
};

export interface IRevoluteJointDef extends IJointDef {
    localAnchorA?: XY;

    localAnchorB?: XY;

    referenceAngle?: number;

    enableLimit?: boolean;

    lowerAngle?: number;

    upperAngle?: number;

    enableMotor?: boolean;

    motorSpeed?: number;

    maxMotorTorque?: number;
}

/**
 * Revolute joint definition. This requires defining an anchor point where the
 * bodies are joined. The definition uses local anchor points so that the
 * initial configuration can violate the constraint slightly. You also need to
 * specify the initial relative angle for joint limits. This helps when saving
 * and loading a game.
 * The local anchor points are measured from the body's origin
 * rather than the center of mass because:
 * 1. you might not know where the center of mass will be.
 * 2. if you add/remove shapes from a body and recompute the mass,
 * the joints will be broken.
 */
export class RevoluteJointDef extends JointDef implements IRevoluteJointDef {
    /** The local anchor point relative to bodyA's origin. */
    public readonly localAnchorA = new Vec2();

    /** The local anchor point relative to bodyB's origin. */
    public readonly localAnchorB = new Vec2();

    /** The bodyB angle minus bodyA angle in the reference state (radians). */
    public referenceAngle = 0;

    /** A flag to enable joint limits. */
    public enableLimit = false;

    /** The lower angle for the joint limit (radians). */
    public lowerAngle = 0;

    /** The upper angle for the joint limit (radians). */
    public upperAngle = 0;

    /** A flag to enable the joint motor. */
    public enableMotor = false;

    /** The desired motor speed. Usually in radians per second. */
    public motorSpeed = 0;

    /**
     * The maximum motor torque used to achieve the desired motor speed.
     * Usually in N-m.
     */
    public maxMotorTorque = 0;

    public constructor() {
        super(JointType.Revolute);
    }

    public Initialize(bA: Body, bB: Body, anchor: XY): void {
        this.bodyA = bA;
        this.bodyB = bB;
        this.bodyA.GetLocalPoint(anchor, this.localAnchorA);
        this.bodyB.GetLocalPoint(anchor, this.localAnchorB);
        this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
    }
}

/**
 * A revolute joint constrains two bodies to share a common point while they
 * are free to rotate about the point. The relative rotation about the shared
 * point is the joint angle. You can limit the relative rotation with
 * a joint limit that specifies a lower and upper angle. You can use a motor
 * to drive the relative rotation about the shared point. A maximum motor torque
 * is provided so that infinite forces are not generated.
 */
export class RevoluteJoint extends Joint {
    // Solver shared
    /** @internal protected */
    public readonly m_localAnchorA = new Vec2();

    /** @internal protected */
    public readonly m_localAnchorB = new Vec2();

    protected readonly m_impulse = new Vec2();

    protected m_motorImpulse = 0;

    protected m_lowerImpulse = 0;

    protected m_upperImpulse = 0;

    protected m_enableMotor = false;

    protected m_maxMotorTorque = 0;

    protected m_motorSpeed = 0;

    protected m_enableLimit = false;

    /** @internal protected */
    public m_referenceAngle = 0;

    protected m_lowerAngle = 0;

    protected m_upperAngle = 0;

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

    protected readonly m_K = new Mat22();

    protected m_angle = 0;

    protected m_axialMass = 0;

    /** @internal protected */
    public constructor(def: IRevoluteJointDef) {
        super(def);

        this.m_localAnchorA.Copy(def.localAnchorA ?? Vec2.ZERO);
        this.m_localAnchorB.Copy(def.localAnchorB ?? Vec2.ZERO);
        this.m_referenceAngle = def.referenceAngle ?? 0;

        this.m_impulse.SetZero();

        this.m_lowerAngle = def.lowerAngle ?? 0;
        this.m_upperAngle = def.upperAngle ?? 0;
        this.m_maxMotorTorque = def.maxMotorTorque ?? 0;
        this.m_motorSpeed = def.motorSpeed ?? 0;
        this.m_enableLimit = def.enableLimit ?? false;
        this.m_enableMotor = def.enableMotor ?? false;
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

        Rot.MultiplyVec2(qA, Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), this.m_rA);
        Rot.MultiplyVec2(qB, Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), this.m_rB);

        // J = [-I -r1_skew I r2_skew]
        // r_skew = [-ry; rx]

        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x]
        //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB]

        const mA = this.m_invMassA;
        const mB = this.m_invMassB;
        const iA = this.m_invIA;
        const iB = this.m_invIB;

        this.m_K.ex.x = mA + mB + this.m_rA.y * this.m_rA.y * iA + this.m_rB.y * this.m_rB.y * iB;
        this.m_K.ey.x = -this.m_rA.y * this.m_rA.x * iA - this.m_rB.y * this.m_rB.x * iB;
        this.m_K.ex.y = this.m_K.ey.x;
        this.m_K.ey.y = mA + mB + this.m_rA.x * this.m_rA.x * iA + this.m_rB.x * this.m_rB.x * iB;

        this.m_axialMass = iA + iB;
        let fixedRotation: boolean;
        if (this.m_axialMass > 0) {
            this.m_axialMass = 1 / this.m_axialMass;
            fixedRotation = false;
        } else {
            fixedRotation = true;
        }

        this.m_angle = aB - aA - this.m_referenceAngle;
        if (this.m_enableLimit === false || fixedRotation) {
            this.m_lowerImpulse = 0;
            this.m_upperImpulse = 0;
        }

        if (this.m_enableMotor === false || fixedRotation) {
            this.m_motorImpulse = 0;
        }

        if (data.step.warmStarting) {
            // Scale impulses to support a variable time step.
            this.m_impulse.Scale(data.step.dtRatio);
            this.m_motorImpulse *= data.step.dtRatio;
            this.m_lowerImpulse *= data.step.dtRatio;
            this.m_upperImpulse *= data.step.dtRatio;

            const axialImpulse = this.m_motorImpulse + this.m_lowerImpulse - this.m_upperImpulse;
            const P = temp.P.Set(this.m_impulse.x, this.m_impulse.y);

            vA.SubtractScaled(mA, P);
            wA -= iA * (Vec2.Cross(this.m_rA, P) + axialImpulse);

            vB.AddScaled(mB, P);
            wB += iB * (Vec2.Cross(this.m_rB, P) + axialImpulse);
        } else {
            this.m_impulse.SetZero();
            this.m_motorImpulse = 0;
            this.m_lowerImpulse = 0;
            this.m_upperImpulse = 0;
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

        const fixedRotation = iA + iB === 0;

        // Solve motor constraint.
        if (this.m_enableMotor && !fixedRotation) {
            const Cdot = wB - wA - this.m_motorSpeed;
            let impulse = -this.m_axialMass * Cdot;
            const oldImpulse = this.m_motorImpulse;
            const maxImpulse = data.step.dt * this.m_maxMotorTorque;
            this.m_motorImpulse = Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.m_motorImpulse - oldImpulse;

            wA -= iA * impulse;
            wB += iB * impulse;
        }

        // Solve limit constraint.
        if (this.m_enableLimit && !fixedRotation) {
            // Lower limit
            {
                const C = this.m_angle - this.m_lowerAngle;
                const Cdot = wB - wA;
                let impulse = -this.m_axialMass * (Cdot + Math.max(C, 0) * data.step.inv_dt);
                const oldImpulse = this.m_lowerImpulse;
                this.m_lowerImpulse = Math.max(this.m_lowerImpulse + impulse, 0);
                impulse = this.m_lowerImpulse - oldImpulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            // Upper limit
            // Note: signs are flipped to keep C positive when the constraint is satisfied.
            // This also keeps the impulse positive when the limit is active.
            {
                const C = this.m_upperAngle - this.m_angle;
                const Cdot = wA - wB;
                let impulse = -this.m_axialMass * (Cdot + Math.max(C, 0) * data.step.inv_dt);
                const oldImpulse = this.m_upperImpulse;
                this.m_upperImpulse = Math.max(this.m_upperImpulse + impulse, 0);
                impulse = this.m_upperImpulse - oldImpulse;

                wA += iA * impulse;
                wB -= iB * impulse;
            }
        }

        // Solve point-to-point constraint
        {
            const { Cdot, impulse } = temp;
            Vec2.Subtract(
                Vec2.AddCrossScalarVec2(vB, wB, this.m_rB, Vec2.s_t0),
                Vec2.AddCrossScalarVec2(vA, wA, this.m_rA, Vec2.s_t1),
                Cdot,
            );
            this.m_K.Solve(-Cdot.x, -Cdot.y, impulse);

            this.m_impulse.x += impulse.x;
            this.m_impulse.y += impulse.y;

            vA.SubtractScaled(mA, impulse);
            wA -= iA * Vec2.Cross(this.m_rA, impulse);

            vB.AddScaled(mB, impulse);
            wB += iB * Vec2.Cross(this.m_rB, impulse);
        }

        data.velocities[this.m_indexA].w = wA;
        data.velocities[this.m_indexB].w = wB;
    }

    public SolvePositionConstraints(data: SolverData): boolean {
        const cA = data.positions[this.m_indexA].c;
        let aA = data.positions[this.m_indexA].a;
        const cB = data.positions[this.m_indexB].c;
        let aB = data.positions[this.m_indexB].a;

        const { qA, qB, lalcA, lalcB, impulse } = temp;
        qA.Set(aA);
        qB.Set(aB);

        let angularError = 0;
        let positionError = 0;

        const fixedRotation = this.m_invIA + this.m_invIB === 0;

        // Solve angular limit constraint
        if (this.m_enableLimit && !fixedRotation) {
            const angle = aB - aA - this.m_referenceAngle;
            let C = 0;

            if (Math.abs(this.m_upperAngle - this.m_lowerAngle) < 2 * ANGULAR_SLOP) {
                // Prevent large angular corrections
                C = Clamp(angle - this.m_lowerAngle, -MAX_ANGULAR_CORRECTION, MAX_ANGULAR_CORRECTION);
            } else if (angle <= this.m_lowerAngle) {
                // Prevent large angular corrections and allow some slop.
                C = Clamp(angle - this.m_lowerAngle + ANGULAR_SLOP, -MAX_ANGULAR_CORRECTION, 0);
            } else if (angle >= this.m_upperAngle) {
                // Prevent large angular corrections and allow some slop.
                C = Clamp(angle - this.m_upperAngle - ANGULAR_SLOP, 0, MAX_ANGULAR_CORRECTION);
            }

            const limitImpulse = -this.m_axialMass * C;
            aA -= this.m_invIA * limitImpulse;
            aB += this.m_invIB * limitImpulse;
            angularError = Math.abs(C);
        }

        // Solve point-to-point constraint.
        {
            qA.Set(aA);
            qB.Set(aB);
            const rA = Rot.MultiplyVec2(qA, Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), this.m_rA);
            const rB = Rot.MultiplyVec2(qB, Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), this.m_rB);

            const C = Vec2.Add(cB, rB, temp.C).Subtract(cA).Subtract(rA);
            positionError = C.Length();

            const mA = this.m_invMassA;
            const mB = this.m_invMassB;
            const iA = this.m_invIA;
            const iB = this.m_invIB;

            const K = this.m_K;
            K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
            K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
            K.ey.x = K.ex.y;
            K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;

            K.Solve(C.x, C.y, impulse).Negate();

            cA.SubtractScaled(mA, impulse);
            aA -= iA * Vec2.Cross(rA, impulse);

            cB.AddScaled(mB, impulse);
            aB += iB * Vec2.Cross(rB, impulse);
        }

        data.positions[this.m_indexA].a = aA;
        data.positions[this.m_indexB].a = aB;

        return positionError <= LINEAR_SLOP && angularError <= ANGULAR_SLOP;
    }

    public GetAnchorA<T extends XY>(out: T): T {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    }

    public GetAnchorB<T extends XY>(out: T): T {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }

    public GetReactionForce<T extends XY>(inv_dt: number, out: T): T {
        out.x = inv_dt * this.m_impulse.x;
        out.y = inv_dt * this.m_impulse.y;
        return out;
    }

    public GetReactionTorque(inv_dt: number): number {
        return inv_dt * (this.m_motorImpulse + this.m_lowerImpulse - this.m_upperImpulse);
    }

    public GetLocalAnchorA(): Readonly<Vec2> {
        return this.m_localAnchorA;
    }

    public GetLocalAnchorB(): Readonly<Vec2> {
        return this.m_localAnchorB;
    }

    public GetReferenceAngle(): number {
        return this.m_referenceAngle;
    }

    public GetJointAngle(): number {
        return this.m_bodyB.m_sweep.a - this.m_bodyA.m_sweep.a - this.m_referenceAngle;
    }

    public GetJointSpeed(): number {
        return this.m_bodyB.m_angularVelocity - this.m_bodyA.m_angularVelocity;
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

    public GetMotorTorque(inv_dt: number): number {
        return inv_dt * this.m_motorImpulse;
    }

    public GetMotorSpeed(): number {
        return this.m_motorSpeed;
    }

    public SetMaxMotorTorque(torque: number): void {
        if (torque !== this.m_maxMotorTorque) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_maxMotorTorque = torque;
        }
    }

    public GetMaxMotorTorque(): number {
        return this.m_maxMotorTorque;
    }

    public IsLimitEnabled(): boolean {
        return this.m_enableLimit;
    }

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

    public GetLowerLimit(): number {
        return this.m_lowerAngle;
    }

    public GetUpperLimit(): number {
        return this.m_upperAngle;
    }

    public SetLimits(lower: number, upper: number): void {
        if (lower !== this.m_lowerAngle || upper !== this.m_upperAngle) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_lowerImpulse = 0;
            this.m_upperImpulse = 0;
            this.m_lowerAngle = lower;
            this.m_upperAngle = upper;
        }
    }

    public SetMotorSpeed(speed: number): number {
        if (speed !== this.m_motorSpeed) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_motorSpeed = speed;
        }
        return speed;
    }

    public Draw(draw: Draw): void {
        const { p2, r, pA, pB } = temp;
        const xfA = this.m_bodyA.GetTransform();
        const xfB = this.m_bodyB.GetTransform();
        Transform.MultiplyVec2(xfA, this.m_localAnchorA, pA);
        Transform.MultiplyVec2(xfB, this.m_localAnchorB, pB);

        draw.DrawPoint(pA, 5, debugColors.joint4);
        draw.DrawPoint(pB, 5, debugColors.joint5);

        const aA = this.m_bodyA.GetAngle();
        const aB = this.m_bodyB.GetAngle();
        const angle = aB - aA - this.m_referenceAngle;

        const L = 0.5;

        r.Set(Math.cos(angle), Math.sin(angle)).Scale(L);
        draw.DrawSegment(pB, Vec2.Add(pB, r, p2), debugColors.joint1);
        draw.DrawCircle(pB, L, debugColors.joint1);

        if (this.m_enableLimit) {
            const { rlo, rhi } = temp;
            rlo.Set(Math.cos(this.m_lowerAngle), Math.sin(this.m_lowerAngle)).Scale(L);
            rhi.Set(Math.cos(this.m_upperAngle), Math.sin(this.m_upperAngle)).Scale(L);
            draw.DrawSegment(pB, Vec2.Add(pB, rlo, p2), debugColors.joint2);
            draw.DrawSegment(pB, Vec2.Add(pB, rhi, p2), debugColors.joint3);
        }

        draw.DrawSegment(xfA.p, pA, debugColors.joint6);
        draw.DrawSegment(pA, pB, debugColors.joint6);
        draw.DrawSegment(xfB.p, pB, debugColors.joint6);
    }
}
