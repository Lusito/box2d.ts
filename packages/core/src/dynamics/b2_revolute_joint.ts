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

import { b2_linearSlop, b2_angularSlop, b2_maxAngularCorrection } from "../common/b2_common";
import { b2Draw, debugColors } from "../common/b2_draw";
import { b2Clamp, b2Vec2, b2Mat22, b2Rot, XY, b2Transform } from "../common/b2_math";
import { b2Readonly } from "../common/b2_readonly";
import { b2Body } from "./b2_body";
import { b2Joint, b2JointDef, b2JointType, b2IJointDef } from "./b2_joint";
import { b2SolverData } from "./b2_time_step";

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)
// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

const temp = {
    qA: new b2Rot(),
    qB: new b2Rot(),
    lalcA: new b2Vec2(),
    lalcB: new b2Vec2(),
    P: new b2Vec2(),
    Cdot: new b2Vec2(),
    C: new b2Vec2(),
    impulse: new b2Vec2(),
    p2: new b2Vec2(),
    r: new b2Vec2(),
    pA: new b2Vec2(),
    pB: new b2Vec2(),
    rlo: new b2Vec2(),
    rhi: new b2Vec2(),
};

export interface b2IRevoluteJointDef extends b2IJointDef {
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
export class b2RevoluteJointDef extends b2JointDef implements b2IRevoluteJointDef {
    /** The local anchor point relative to bodyA's origin. */
    public readonly localAnchorA = new b2Vec2();

    /** The local anchor point relative to bodyB's origin. */
    public readonly localAnchorB = new b2Vec2();

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
        super(b2JointType.e_revoluteJoint);
    }

    /** Initialize the bodies, anchors, and reference angle using a world anchor point. */
    public Initialize(bA: b2Body, bB: b2Body, anchor: XY): void {
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
export class b2RevoluteJoint extends b2Joint {
    // Solver shared
    /** @internal protected */
    public readonly m_localAnchorA = new b2Vec2();

    /** @internal protected */
    public readonly m_localAnchorB = new b2Vec2();

    protected readonly m_impulse = new b2Vec2();

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

    protected readonly m_rA = new b2Vec2();

    protected readonly m_rB = new b2Vec2();

    protected readonly m_localCenterA = new b2Vec2();

    protected readonly m_localCenterB = new b2Vec2();

    protected m_invMassA = 0;

    protected m_invMassB = 0;

    protected m_invIA = 0;

    protected m_invIB = 0;

    protected readonly m_K = new b2Mat22();

    protected m_angle = 0;

    protected m_axialMass = 0;

    /** @internal protected */
    public constructor(def: b2IRevoluteJointDef) {
        super(def);

        this.m_localAnchorA.Copy(def.localAnchorA ?? b2Vec2.ZERO);
        this.m_localAnchorB.Copy(def.localAnchorB ?? b2Vec2.ZERO);
        this.m_referenceAngle = def.referenceAngle ?? 0;

        this.m_impulse.SetZero();

        this.m_lowerAngle = def.lowerAngle ?? 0;
        this.m_upperAngle = def.upperAngle ?? 0;
        this.m_maxMotorTorque = def.maxMotorTorque ?? 0;
        this.m_motorSpeed = def.motorSpeed ?? 0;
        this.m_enableLimit = def.enableLimit ?? false;
        this.m_enableMotor = def.enableMotor ?? false;
    }

    public InitVelocityConstraints(data: b2SolverData): void {
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

        b2Rot.MultiplyVec2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), this.m_rA);
        b2Rot.MultiplyVec2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), this.m_rB);

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
            wA -= iA * (b2Vec2.Cross(this.m_rA, P) + axialImpulse);

            vB.AddScaled(mB, P);
            wB += iB * (b2Vec2.Cross(this.m_rB, P) + axialImpulse);
        } else {
            this.m_impulse.SetZero();
            this.m_motorImpulse = 0;
            this.m_lowerImpulse = 0;
            this.m_upperImpulse = 0;
        }

        data.velocities[this.m_indexA].w = wA;
        data.velocities[this.m_indexB].w = wB;
    }

    public SolveVelocityConstraints(data: b2SolverData): void {
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
            this.m_motorImpulse = b2Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
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
            b2Vec2.Subtract(
                b2Vec2.AddCrossScalarVec2(vB, wB, this.m_rB, b2Vec2.s_t0),
                b2Vec2.AddCrossScalarVec2(vA, wA, this.m_rA, b2Vec2.s_t1),
                Cdot,
            );
            this.m_K.Solve(-Cdot.x, -Cdot.y, impulse);

            this.m_impulse.x += impulse.x;
            this.m_impulse.y += impulse.y;

            vA.SubtractScaled(mA, impulse);
            wA -= iA * b2Vec2.Cross(this.m_rA, impulse);

            vB.AddScaled(mB, impulse);
            wB += iB * b2Vec2.Cross(this.m_rB, impulse);
        }

        data.velocities[this.m_indexA].w = wA;
        data.velocities[this.m_indexB].w = wB;
    }

    public SolvePositionConstraints(data: b2SolverData): boolean {
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

            if (Math.abs(this.m_upperAngle - this.m_lowerAngle) < 2 * b2_angularSlop) {
                // Prevent large angular corrections
                C = b2Clamp(angle - this.m_lowerAngle, -b2_maxAngularCorrection, b2_maxAngularCorrection);
            } else if (angle <= this.m_lowerAngle) {
                // Prevent large angular corrections and allow some slop.
                C = b2Clamp(angle - this.m_lowerAngle + b2_angularSlop, -b2_maxAngularCorrection, 0);
            } else if (angle >= this.m_upperAngle) {
                // Prevent large angular corrections and allow some slop.
                C = b2Clamp(angle - this.m_upperAngle - b2_angularSlop, 0, b2_maxAngularCorrection);
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
            const rA = b2Rot.MultiplyVec2(
                qA,
                b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA, lalcA),
                this.m_rA,
            );
            const rB = b2Rot.MultiplyVec2(
                qB,
                b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB, lalcB),
                this.m_rB,
            );

            const C = b2Vec2.Add(cB, rB, temp.C).Subtract(cA).Subtract(rA);
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
            aA -= iA * b2Vec2.Cross(rA, impulse);

            cB.AddScaled(mB, impulse);
            aB += iB * b2Vec2.Cross(rB, impulse);
        }

        data.positions[this.m_indexA].a = aA;
        data.positions[this.m_indexB].a = aB;

        return positionError <= b2_linearSlop && angularError <= b2_angularSlop;
    }

    public GetAnchorA<T extends XY>(out: T): T {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    }

    public GetAnchorB<T extends XY>(out: T): T {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }

    /**
     * Get the reaction force given the inverse time step.
     * Unit is N.
     */
    public GetReactionForce<T extends XY>(inv_dt: number, out: T): T {
        out.x = inv_dt * this.m_impulse.x;
        out.y = inv_dt * this.m_impulse.y;
        return out;
    }

    /**
     * Get the reaction torque due to the joint limit given the inverse time step.
     * Unit is N*m.
     */
    public GetReactionTorque(inv_dt: number): number {
        return inv_dt * (this.m_motorImpulse + this.m_lowerImpulse - this.m_upperImpulse);
    }

    /** The local anchor point relative to bodyA's origin. */
    public GetLocalAnchorA(): b2Readonly<b2Vec2> {
        return this.m_localAnchorA;
    }

    /** The local anchor point relative to bodyB's origin. */
    public GetLocalAnchorB(): b2Readonly<b2Vec2> {
        return this.m_localAnchorB;
    }

    /** Get the reference angle. */
    public GetReferenceAngle(): number {
        return this.m_referenceAngle;
    }

    /** Get the current joint angle in radians. */
    public GetJointAngle(): number {
        return this.m_bodyB.m_sweep.a - this.m_bodyA.m_sweep.a - this.m_referenceAngle;
    }

    /** Get the current joint angle speed in radians per second. */
    public GetJointSpeed(): number {
        return this.m_bodyB.m_angularVelocity - this.m_bodyA.m_angularVelocity;
    }

    /** Is the joint motor enabled? */
    public IsMotorEnabled(): boolean {
        return this.m_enableMotor;
    }

    /** Enable/disable the joint motor. */
    public EnableMotor(flag: boolean): boolean {
        if (flag !== this.m_enableMotor) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_enableMotor = flag;
        }
        return flag;
    }

    /**
     * Get the current motor torque given the inverse time step.
     * Unit is N*m.
     */
    public GetMotorTorque(inv_dt: number): number {
        return inv_dt * this.m_motorImpulse;
    }

    /** Get the motor speed in radians per second. */
    public GetMotorSpeed(): number {
        return this.m_motorSpeed;
    }

    /** Set the maximum motor torque, usually in N-m. */
    public SetMaxMotorTorque(torque: number): void {
        if (torque !== this.m_maxMotorTorque) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_maxMotorTorque = torque;
        }
    }

    /** Get the maximum motor torque, usually in N-m. */
    public GetMaxMotorTorque(): number {
        return this.m_maxMotorTorque;
    }

    /** Is the joint limit enabled?  */
    public IsLimitEnabled(): boolean {
        return this.m_enableLimit;
    }

    /** Enable/disable the joint limit. */
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

    /** Get the lower joint limit in radians. */
    public GetLowerLimit(): number {
        return this.m_lowerAngle;
    }

    /** Get the upper joint limit in radians. */
    public GetUpperLimit(): number {
        return this.m_upperAngle;
    }

    /** Set the joint limits in radians. */
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

    /** Set the motor speed in radians per second. */
    public SetMotorSpeed(speed: number): number {
        if (speed !== this.m_motorSpeed) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_motorSpeed = speed;
        }
        return speed;
    }

    public override Draw(draw: b2Draw): void {
        const { p2, r, pA, pB } = temp;
        const xfA = this.m_bodyA.GetTransform();
        const xfB = this.m_bodyB.GetTransform();
        b2Transform.MultiplyVec2(xfA, this.m_localAnchorA, pA);
        b2Transform.MultiplyVec2(xfB, this.m_localAnchorB, pB);

        draw.DrawPoint(pA, 5, debugColors.joint4);
        draw.DrawPoint(pB, 5, debugColors.joint5);

        const aA = this.m_bodyA.GetAngle();
        const aB = this.m_bodyB.GetAngle();
        const angle = aB - aA - this.m_referenceAngle;

        const L = 0.5;

        r.Set(Math.cos(angle), Math.sin(angle)).Scale(L);
        draw.DrawSegment(pB, b2Vec2.Add(pB, r, p2), debugColors.joint1);
        draw.DrawCircle(pB, L, debugColors.joint1);

        if (this.m_enableLimit) {
            const { rlo, rhi } = temp;
            rlo.Set(Math.cos(this.m_lowerAngle), Math.sin(this.m_lowerAngle)).Scale(L);
            rhi.Set(Math.cos(this.m_upperAngle), Math.sin(this.m_upperAngle)).Scale(L);
            draw.DrawSegment(pB, b2Vec2.Add(pB, rlo, p2), debugColors.joint2);
            draw.DrawSegment(pB, b2Vec2.Add(pB, rhi, p2), debugColors.joint3);
        }

        draw.DrawSegment(xfA.p, pA, debugColors.joint6);
        draw.DrawSegment(pA, pB, debugColors.joint6);
        draw.DrawSegment(xfB.p, pB, debugColors.joint6);
    }
}
