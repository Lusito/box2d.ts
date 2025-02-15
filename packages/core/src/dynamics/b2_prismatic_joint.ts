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

import { b2_linearSlop, b2_angularSlop } from "../common/b2_common";
import { b2Draw, debugColors } from "../common/b2_draw";
import { b2Clamp, b2Vec2, b2Mat22, b2Vec3, b2Mat33, b2Rot, XY, b2Transform } from "../common/b2_math";
import { b2Readonly } from "../common/b2_readonly";
import { b2Body } from "./b2_body";
import { b2Joint, b2JointDef, b2JointType, b2IJointDef } from "./b2_joint";
import { b2SolverData } from "./b2_time_step";

// Linear constraint (point-to-line)
// d = p2 - p1 = x2 + r2 - x1 - r1
// C = dot(perp, d)
// Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
// J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
//
// Angular constraint
// C = a2 - a1 + a_initial
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
//
// K = J * invM * JT
//
// J = [-a -s1 a s2]
//     [0  -1  0  1]
// a = perp
// s1 = cross(d + r1, a) = cross(p2 - x1, a)
// s2 = cross(r2, a) = cross(p2 - x2, a)

// Motor/Limit linear constraint
// C = dot(ax1, d)
// Cdot = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

// Predictive limit is applied even when the limit is not active.
// Prevents a constraint speed that can lead to a constraint error in one time step.
// Want C2 = C1 + h * Cdot >= 0
// Or:
// Cdot + C1/h >= 0
// I do not apply a negative constraint error because that is handled in position correction.
// So:
// Cdot + max(C1, 0)/h >= 0

// Block Solver
// We develop a block solver that includes the angular and linear constraints. This makes the limit stiffer.
//
// The Jacobian has 2 rows:
// J = [-uT -s1 uT s2] // linear
//     [0   -1   0  1] // angular
//
// u = perp
// s1 = cross(d + r1, u), s2 = cross(r2, u)
// a1 = cross(d + r1, v), a2 = cross(r2, v)

const temp = {
    K3: new b2Mat33(),
    K2: new b2Mat22(),
    qA: new b2Rot(),
    qB: new b2Rot(),
    lalcA: new b2Vec2(),
    lalcB: new b2Vec2(),
    rA: new b2Vec2(),
    rB: new b2Vec2(),
    GetJointTranslation: {
        pA: new b2Vec2(),
        pB: new b2Vec2(),
        d: new b2Vec2(),
        axis: new b2Vec2(),
    },
    InitVelocityConstraints: {
        d: new b2Vec2(),
        P: new b2Vec2(),
    },
    SolveVelocityConstraints: {
        P: new b2Vec2(),
        df: new b2Vec2(),
    },
    SolvePositionConstraints: {
        d: new b2Vec2(),
        impulse: new b2Vec3(),
        impulse1: new b2Vec2(),
        P: new b2Vec2(),
    },
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

export interface b2IPrismaticJointDef extends b2IJointDef {
    localAnchorA?: XY;

    localAnchorB?: XY;

    localAxisA?: XY;

    referenceAngle?: number;

    enableLimit?: boolean;

    lowerTranslation?: number;

    upperTranslation?: number;

    enableMotor?: boolean;

    maxMotorForce?: number;

    motorSpeed?: number;
}

/**
 * Prismatic joint definition. This requires defining a line of
 * motion using an axis and an anchor point. The definition uses local
 * anchor points and a local axis so that the initial configuration
 * can violate the constraint slightly. The joint translation is zero
 * when the local anchor points coincide in world space. Using local
 * anchors and a local axis helps when saving and loading a game.
 */
export class b2PrismaticJointDef extends b2JointDef implements b2IPrismaticJointDef {
    /** The local anchor point relative to bodyA's origin. */
    public readonly localAnchorA = new b2Vec2();

    /** The local anchor point relative to bodyB's origin. */
    public readonly localAnchorB = new b2Vec2();

    /** The local translation unit axis in bodyA. */
    public readonly localAxisA = new b2Vec2(1, 0);

    /** The constrained angle between the bodies: bodyB_angle - bodyA_angle. */
    public referenceAngle = 0;

    /** Enable/disable the joint limit. */
    public enableLimit = false;

    /** The lower translation limit, usually in meters. */
    public lowerTranslation = 0;

    /** The upper translation limit, usually in meters. */
    public upperTranslation = 0;

    /** Enable/disable the joint motor. */
    public enableMotor = false;

    /** The maximum motor torque, usually in N-m. */
    public maxMotorForce = 0;

    /** The desired motor speed in radians per second. */
    public motorSpeed = 0;

    public constructor() {
        super(b2JointType.e_prismaticJoint);
    }

    /**
     * Initialize the bodies, anchors, axis, and reference angle using the world
     * anchor and unit world axis.
     */
    public Initialize(bA: b2Body, bB: b2Body, anchor: XY, axis: XY): void {
        this.bodyA = bA;
        this.bodyB = bB;
        this.bodyA.GetLocalPoint(anchor, this.localAnchorA);
        this.bodyB.GetLocalPoint(anchor, this.localAnchorB);
        this.bodyA.GetLocalVector(axis, this.localAxisA);
        this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
    }
}

/**
 * A prismatic joint. This joint provides one degree of freedom: translation
 * along an axis fixed in bodyA. Relative rotation is prevented. You can
 * use a joint limit to restrict the range of motion and a joint motor to
 * drive the motion or to model joint friction.
 */
export class b2PrismaticJoint extends b2Joint {
    /** @internal protected */
    public readonly m_localAnchorA = new b2Vec2();

    /** @internal protected */
    public readonly m_localAnchorB = new b2Vec2();

    /** @internal protected */
    public readonly m_localXAxisA = new b2Vec2();

    protected readonly m_localYAxisA = new b2Vec2();

    /** @internal protected */
    public m_referenceAngle = 0;

    protected readonly m_impulse = new b2Vec2();

    protected m_motorImpulse = 0;

    protected m_lowerImpulse = 0;

    protected m_upperImpulse = 0;

    protected m_lowerTranslation = 0;

    protected m_upperTranslation = 0;

    protected m_maxMotorForce = 0;

    protected m_motorSpeed = 0;

    protected m_enableLimit = false;

    protected m_enableMotor = false;

    // Solver temp
    protected m_indexA = 0;

    protected m_indexB = 0;

    protected readonly m_localCenterA = new b2Vec2();

    protected readonly m_localCenterB = new b2Vec2();

    protected m_invMassA = 0;

    protected m_invMassB = 0;

    protected m_invIA = 0;

    protected m_invIB = 0;

    protected readonly m_axis = new b2Vec2();

    protected readonly m_perp = new b2Vec2();

    protected m_s1 = 0;

    protected m_s2 = 0;

    protected m_a1 = 0;

    protected m_a2 = 0;

    protected readonly m_K = new b2Mat22();

    protected m_translation = 0;

    protected m_axialMass = 0;

    /** @internal protected */
    public constructor(def: b2IPrismaticJointDef) {
        super(def);

        this.m_localAnchorA.Copy(def.localAnchorA ?? b2Vec2.ZERO);
        this.m_localAnchorB.Copy(def.localAnchorB ?? b2Vec2.ZERO);
        b2Vec2.Normalize(def.localAxisA ?? b2Vec2.UNITX, this.m_localXAxisA);
        b2Vec2.CrossOneVec2(this.m_localXAxisA, this.m_localYAxisA);
        this.m_referenceAngle = def.referenceAngle ?? 0;
        this.m_lowerTranslation = def.lowerTranslation ?? 0;
        this.m_upperTranslation = def.upperTranslation ?? 0;
        // b2Assert(this.m_lowerTranslation <= this.m_upperTranslation);
        this.m_maxMotorForce = def.maxMotorForce ?? 0;
        this.m_motorSpeed = def.motorSpeed ?? 0;
        this.m_enableLimit = def.enableLimit ?? false;
        this.m_enableMotor = def.enableMotor ?? false;
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

        const { qA, qB, lalcA, lalcB, rA, rB } = temp;
        qA.Set(aA);
        qB.Set(aB);

        // Compute the effective masses.
        b2Rot.MultiplyVec2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), rA);
        b2Rot.MultiplyVec2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), rB);
        const d = b2Vec2.Subtract(cB, cA, temp.InitVelocityConstraints.d).Add(rB).Subtract(rA);

        const mA = this.m_invMassA;
        const mB = this.m_invMassB;
        const iA = this.m_invIA;
        const iB = this.m_invIB;

        // Compute motor Jacobian and effective mass.
        b2Rot.MultiplyVec2(qA, this.m_localXAxisA, this.m_axis);
        this.m_a1 = b2Vec2.Cross(b2Vec2.Add(d, rA, b2Vec2.s_t0), this.m_axis);
        this.m_a2 = b2Vec2.Cross(rB, this.m_axis);

        this.m_axialMass = mA + mB + iA * this.m_a1 * this.m_a1 + iB * this.m_a2 * this.m_a2;
        if (this.m_axialMass > 0) {
            this.m_axialMass = 1 / this.m_axialMass;
        }

        // Prismatic constraint.
        b2Rot.MultiplyVec2(qA, this.m_localYAxisA, this.m_perp);

        this.m_s1 = b2Vec2.Cross(b2Vec2.Add(d, rA, b2Vec2.s_t0), this.m_perp);
        this.m_s2 = b2Vec2.Cross(rB, this.m_perp);

        const k11 = mA + mB + iA * this.m_s1 * this.m_s1 + iB * this.m_s2 * this.m_s2;
        const k12 = iA * this.m_s1 + iB * this.m_s2;
        let k22 = iA + iB;
        if (k22 === 0) {
            // For bodies with fixed rotation.
            k22 = 1;
        }

        this.m_K.ex.Set(k11, k12);
        this.m_K.ey.Set(k12, k22);

        if (this.m_enableLimit) {
            this.m_translation = b2Vec2.Dot(this.m_axis, d);
        } else {
            this.m_lowerImpulse = 0;
            this.m_upperImpulse = 0;
        }

        if (!this.m_enableMotor) {
            this.m_motorImpulse = 0;
        }

        if (data.step.warmStarting) {
            // Account for variable time step.
            this.m_impulse.Scale(data.step.dtRatio);
            this.m_motorImpulse *= data.step.dtRatio;
            this.m_lowerImpulse *= data.step.dtRatio;
            this.m_upperImpulse *= data.step.dtRatio;

            const axialImpulse = this.m_motorImpulse + this.m_lowerImpulse - this.m_upperImpulse;
            const { P } = temp.InitVelocityConstraints;
            b2Vec2.Scale(this.m_impulse.x, this.m_perp, P).AddScaled(axialImpulse, this.m_axis);
            const LA = this.m_impulse.x * this.m_s1 + this.m_impulse.y + axialImpulse * this.m_a1;
            const LB = this.m_impulse.x * this.m_s2 + this.m_impulse.y + axialImpulse * this.m_a2;

            vA.SubtractScaled(mA, P);
            wA -= iA * LA;

            vB.AddScaled(mB, P);
            wB += iB * LB;
        } else {
            this.m_impulse.SetZero();
            this.m_motorImpulse = 0;
            this.m_lowerImpulse = 0;
            this.m_upperImpulse = 0;
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

        const { P, df } = temp.SolveVelocityConstraints;

        // Solve linear motor constraint.
        if (this.m_enableMotor) {
            const Cdot =
                b2Vec2.Dot(this.m_axis, b2Vec2.Subtract(vB, vA, b2Vec2.s_t0)) + this.m_a2 * wB - this.m_a1 * wA;
            let impulse = this.m_axialMass * (this.m_motorSpeed - Cdot);
            const oldImpulse = this.m_motorImpulse;
            const maxImpulse = data.step.dt * this.m_maxMotorForce;
            this.m_motorImpulse = b2Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.m_motorImpulse - oldImpulse;

            b2Vec2.Scale(impulse, this.m_axis, P);
            const LA = impulse * this.m_a1;
            const LB = impulse * this.m_a2;

            vA.SubtractScaled(mA, P);
            wA -= iA * LA;
            vB.AddScaled(mB, P);
            wB += iB * LB;
        }

        if (this.m_enableLimit) {
            // Lower limit
            {
                const C = this.m_translation - this.m_lowerTranslation;
                const Cdot =
                    b2Vec2.Dot(this.m_axis, b2Vec2.Subtract(vB, vA, b2Vec2.s_t0)) + this.m_a2 * wB - this.m_a1 * wA;
                let impulse = -this.m_axialMass * (Cdot + Math.max(C, 0) * data.step.inv_dt);
                const oldImpulse = this.m_lowerImpulse;
                this.m_lowerImpulse = Math.max(this.m_lowerImpulse + impulse, 0);
                impulse = this.m_lowerImpulse - oldImpulse;

                b2Vec2.Scale(impulse, this.m_axis, P);
                const LA = impulse * this.m_a1;
                const LB = impulse * this.m_a2;

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
                    b2Vec2.Dot(this.m_axis, b2Vec2.Subtract(vA, vB, b2Vec2.s_t0)) + this.m_a1 * wA - this.m_a2 * wB;
                let impulse = -this.m_axialMass * (Cdot + Math.max(C, 0) * data.step.inv_dt);
                const oldImpulse = this.m_upperImpulse;
                this.m_upperImpulse = Math.max(this.m_upperImpulse + impulse, 0);
                impulse = this.m_upperImpulse - oldImpulse;

                b2Vec2.Scale(impulse, this.m_axis, P);
                const LA = impulse * this.m_a1;
                const LB = impulse * this.m_a2;

                vA.AddScaled(mA, P);
                wA += iA * LA;
                vB.SubtractScaled(mB, P);
                wB -= iB * LB;
            }
        }

        // Solve the prismatic constraint in block form.
        {
            const Cdot_x =
                b2Vec2.Dot(this.m_perp, b2Vec2.Subtract(vB, vA, b2Vec2.s_t0)) + this.m_s2 * wB - this.m_s1 * wA;
            const Cdot_y = wB - wA;

            this.m_K.Solve(-Cdot_x, -Cdot_y, df);
            this.m_impulse.Add(df);

            b2Vec2.Scale(df.x, this.m_perp, P);
            const LA = df.x * this.m_s1 + df.y;
            const LB = df.x * this.m_s2 + df.y;

            vA.SubtractScaled(mA, P);
            wA -= iA * LA;

            vB.AddScaled(mB, P);
            wB += iB * LB;
        }

        data.velocities[this.m_indexA].w = wA;
        data.velocities[this.m_indexB].w = wB;
    }

    /**
     * A velocity based solver computes reaction forces(impulses) using the velocity constraint solver.Under this context,
     * the position solver is not there to resolve forces.It is only there to cope with integration error.
     *
     * Therefore, the pseudo impulses in the position solver do not have any physical meaning.Thus it is okay if they suck.
     *
     * We could take the active state from the velocity solver.However, the joint might push past the limit when the velocity
     * solver indicates the limit is inactive.
     *
     * @internal protected
     */
    public SolvePositionConstraints(data: b2SolverData): boolean {
        const cA = data.positions[this.m_indexA].c;
        let aA = data.positions[this.m_indexA].a;
        const cB = data.positions[this.m_indexB].c;
        let aB = data.positions[this.m_indexB].a;

        const { qA, qB, lalcA, lalcB, rA, rB } = temp;
        qA.Set(aA);
        qB.Set(aB);

        const mA = this.m_invMassA;
        const mB = this.m_invMassB;
        const iA = this.m_invIA;
        const iB = this.m_invIB;

        // Compute fresh Jacobians
        const { d, impulse, P } = temp.SolvePositionConstraints;
        b2Rot.MultiplyVec2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), rA);
        b2Rot.MultiplyVec2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), rB);
        b2Vec2.Add(cB, rB, d).Subtract(cA).Subtract(rA);

        const axis = b2Rot.MultiplyVec2(qA, this.m_localXAxisA, this.m_axis);
        const a1 = b2Vec2.Cross(b2Vec2.Add(d, rA, b2Vec2.s_t0), axis);
        const a2 = b2Vec2.Cross(rB, axis);
        const perp = b2Rot.MultiplyVec2(qA, this.m_localYAxisA, this.m_perp);

        const s1 = b2Vec2.Cross(b2Vec2.Add(d, rA, b2Vec2.s_t0), perp);
        const s2 = b2Vec2.Cross(rB, perp);

        const C1_x = b2Vec2.Dot(perp, d);
        const C1_y = aB - aA - this.m_referenceAngle;

        let linearError = Math.abs(C1_x);
        const angularError = Math.abs(C1_y);

        let active = false;
        let C2 = 0;
        if (this.m_enableLimit) {
            const translation = b2Vec2.Dot(axis, d);
            if (Math.abs(this.m_upperTranslation - this.m_lowerTranslation) < 2 * b2_linearSlop) {
                C2 = translation;
                linearError = Math.max(linearError, Math.abs(translation));
                active = true;
            } else if (translation <= this.m_lowerTranslation) {
                C2 = Math.min(translation - this.m_lowerTranslation, 0);
                linearError = Math.max(linearError, this.m_lowerTranslation - translation);
                active = true;
            } else if (translation >= this.m_upperTranslation) {
                C2 = Math.max(translation - this.m_upperTranslation, 0);
                linearError = Math.max(linearError, translation - this.m_upperTranslation);
                active = true;
            }
        }

        if (active) {
            const k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            const k12 = iA * s1 + iB * s2;
            const k13 = iA * s1 * a1 + iB * s2 * a2;
            let k22 = iA + iB;
            if (k22 === 0) {
                // For fixed rotation
                k22 = 1;
            }
            const k23 = iA * a1 + iB * a2;
            const k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

            const K = temp.K3;
            K.ex.Set(k11, k12, k13);
            K.ey.Set(k12, k22, k23);
            K.ez.Set(k13, k23, k33);

            K.Solve33(-C1_x, -C1_y, -C2, impulse);
        } else {
            const k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            const k12 = iA * s1 + iB * s2;
            let k22 = iA + iB;
            if (k22 === 0) {
                k22 = 1;
            }

            const K = temp.K2;
            K.ex.Set(k11, k12);
            K.ey.Set(k12, k22);

            const impulse1 = K.Solve(-C1_x, -C1_y, temp.SolvePositionConstraints.impulse1);
            impulse.x = impulse1.x;
            impulse.y = impulse1.y;
            impulse.z = 0;
        }

        b2Vec2.Scale(impulse.x, perp, P).AddScaled(impulse.z, axis);
        const LA = impulse.x * s1 + impulse.y + impulse.z * a1;
        const LB = impulse.x * s2 + impulse.y + impulse.z * a2;

        cA.SubtractScaled(mA, P);
        aA -= iA * LA;
        cB.AddScaled(mB, P);
        aB += iB * LB;

        data.positions[this.m_indexA].a = aA;
        data.positions[this.m_indexB].a = aB;

        return linearError <= b2_linearSlop && angularError <= b2_angularSlop;
    }

    public GetAnchorA<T extends XY>(out: T): T {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    }

    public GetAnchorB<T extends XY>(out: T): T {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }

    public GetReactionForce<T extends XY>(inv_dt: number, out: T): T {
        const f = this.m_motorImpulse + this.m_lowerImpulse - this.m_upperImpulse;
        out.x = inv_dt * (this.m_impulse.x * this.m_perp.x + f * this.m_axis.x);
        out.y = inv_dt * (this.m_impulse.x * this.m_perp.y + f * this.m_axis.y);
        return out;
    }

    public GetReactionTorque(inv_dt: number): number {
        return inv_dt * this.m_impulse.y;
    }

    /** The local anchor point relative to bodyA's origin. */
    public GetLocalAnchorA(): b2Readonly<b2Vec2> {
        return this.m_localAnchorA;
    }

    /** The local anchor point relative to bodyB's origin. */
    public GetLocalAnchorB(): b2Readonly<b2Vec2> {
        return this.m_localAnchorB;
    }

    /** The local joint axis relative to bodyA. */
    public GetLocalAxisA(): b2Readonly<b2Vec2> {
        return this.m_localXAxisA;
    }

    /** Get the reference angle. */
    public GetReferenceAngle() {
        return this.m_referenceAngle;
    }

    /** Get the current joint translation, usually in meters. */
    public GetJointTranslation(): number {
        const { pA, pB, axis, d } = temp.GetJointTranslation;
        this.m_bodyA.GetWorldPoint(this.m_localAnchorA, pA);
        this.m_bodyB.GetWorldPoint(this.m_localAnchorB, pB);
        b2Vec2.Subtract(pB, pA, d);
        this.m_bodyA.GetWorldVector(this.m_localXAxisA, axis);

        const translation = b2Vec2.Dot(d, axis);
        return translation;
    }

    /** Get the current joint translation speed, usually in meters per second. */
    public GetJointSpeed(): number {
        const bA = this.m_bodyA;
        const bB = this.m_bodyB;

        const { lalcA, lalcB, rA, rB } = temp;
        b2Rot.MultiplyVec2(bA.m_xf.q, b2Vec2.Subtract(this.m_localAnchorA, bA.m_sweep.localCenter, lalcA), rA);
        b2Rot.MultiplyVec2(bB.m_xf.q, b2Vec2.Subtract(this.m_localAnchorB, bB.m_sweep.localCenter, lalcB), rB);
        const p1 = b2Vec2.Add(bA.m_sweep.c, rA, b2Vec2.s_t0);
        const p2 = b2Vec2.Add(bB.m_sweep.c, rB, b2Vec2.s_t1);
        const d = b2Vec2.Subtract(p2, p1, b2Vec2.s_t2);
        const axis = b2Rot.MultiplyVec2(bA.m_xf.q, this.m_localXAxisA, this.m_axis);

        const vA = bA.m_linearVelocity;
        const vB = bB.m_linearVelocity;
        const wA = bA.m_angularVelocity;
        const wB = bB.m_angularVelocity;

        const speed =
            b2Vec2.Dot(d, b2Vec2.CrossScalarVec2(wA, axis, b2Vec2.s_t0)) +
            b2Vec2.Dot(
                axis,
                b2Vec2.Subtract(
                    b2Vec2.AddCrossScalarVec2(vB, wB, rB, b2Vec2.s_t0),
                    b2Vec2.AddCrossScalarVec2(vA, wA, rA, b2Vec2.s_t1),
                    b2Vec2.s_t0,
                ),
            );
        return speed;
    }

    /** Is the joint limit enabled? */
    public IsLimitEnabled() {
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

    /** Get the lower joint limit, usually in meters. */
    public GetLowerLimit() {
        return this.m_lowerTranslation;
    }

    /** Get the upper joint limit, usually in meters. */
    public GetUpperLimit() {
        return this.m_upperTranslation;
    }

    /** Set the joint limits, usually in meters. */
    public SetLimits(lower: number, upper: number): void {
        // DEBUG: b2Assert(lower <= upper);
        if (lower !== this.m_lowerTranslation || upper !== this.m_upperTranslation) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_lowerTranslation = lower;
            this.m_upperTranslation = upper;
            this.m_lowerImpulse = 0;
            this.m_upperImpulse = 0;
        }
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

    /** Set the motor speed, usually in meters per second. */
    public SetMotorSpeed(speed: number): number {
        if (speed !== this.m_motorSpeed) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_motorSpeed = speed;
        }
        return speed;
    }

    /** Get the motor speed, usually in meters per second. */
    public GetMotorSpeed() {
        return this.m_motorSpeed;
    }

    /** Set the maximum motor force, usually in N. */
    public SetMaxMotorForce(force: number): void {
        if (force !== this.m_maxMotorForce) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_maxMotorForce = force;
        }
    }

    /** Get the maximum motor force, usually in N. */
    public GetMaxMotorForce(): number {
        return this.m_maxMotorForce;
    }

    /** Get the current motor force given the inverse time step, usually in N. */
    public GetMotorForce(inv_dt: number): number {
        return inv_dt * this.m_motorImpulse;
    }

    public override Draw(draw: b2Draw): void {
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
