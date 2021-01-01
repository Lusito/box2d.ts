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
import { Draw, debugColors } from "../common/b2_draw";
import { clamp, Vec2, Mat22, Vec3, Mat33, Rot, XY, Transform } from "../common/b2_math";
import { Body } from "./b2_body";
import { Joint, JointDef, JointType, IJointDef } from "./b2_joint";
import { SolverData } from "./b2_time_step";

const temp = {
    K3: new Mat33(),
    K2: new Mat22(),
    qA: new Rot(),
    qB: new Rot(),
    lalcA: new Vec2(),
    lalcB: new Vec2(),
    rA: new Vec2(),
    rB: new Vec2(),
    GetJointTranslation: {
        pA: new Vec2(),
        pB: new Vec2(),
        d: new Vec2(),
        axis: new Vec2(),
    },
    InitVelocityConstraints: {
        d: new Vec2(),
        P: new Vec2(),
    },
    SolveVelocityConstraints: {
        P: new Vec2(),
        df: new Vec2(),
    },
    SolvePositionConstraints: {
        d: new Vec2(),
        impulse: new Vec3(),
        impulse1: new Vec2(),
        P: new Vec2(),
    },
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

export interface IPrismaticJointDef extends IJointDef {
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
export class PrismaticJointDef extends JointDef implements IPrismaticJointDef {
    /** The local anchor point relative to bodyA's origin. */
    public readonly localAnchorA = new Vec2();

    /** The local anchor point relative to bodyB's origin. */
    public readonly localAnchorB = new Vec2();

    /** The local translation unit axis in bodyA. */
    public readonly localAxisA = new Vec2(1, 0);

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
        super(JointType.Prismatic);
    }

    public initialize(bA: Body, bB: Body, anchor: XY, axis: XY): void {
        this.bodyA = bA;
        this.bodyB = bB;
        this.bodyA.getLocalPoint(anchor, this.localAnchorA);
        this.bodyB.getLocalPoint(anchor, this.localAnchorB);
        this.bodyA.getLocalVector(axis, this.localAxisA);
        this.referenceAngle = this.bodyB.getAngle() - this.bodyA.getAngle();
    }
}

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

/**
 * A prismatic joint. This joint provides one degree of freedom: translation
 * along an axis fixed in bodyA. Relative rotation is prevented. You can
 * use a joint limit to restrict the range of motion and a joint motor to
 * drive the motion or to model joint friction.
 */
export class PrismaticJoint extends Joint {
    /** @internal protected */
    public readonly m_localAnchorA = new Vec2();

    /** @internal protected */
    public readonly m_localAnchorB = new Vec2();

    /** @internal protected */
    public readonly m_localXAxisA = new Vec2();

    protected readonly m_localYAxisA = new Vec2();

    /** @internal protected */
    public m_referenceAngle = 0;

    protected readonly m_impulse = new Vec2();

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

    protected readonly m_localCenterA = new Vec2();

    protected readonly m_localCenterB = new Vec2();

    protected m_invMassA = 0;

    protected m_invMassB = 0;

    protected m_invIA = 0;

    protected m_invIB = 0;

    protected readonly m_axis = new Vec2();

    protected readonly m_perp = new Vec2();

    protected m_s1 = 0;

    protected m_s2 = 0;

    protected m_a1 = 0;

    protected m_a2 = 0;

    protected readonly m_K = new Mat22();

    protected m_translation = 0;

    protected m_axialMass = 0;

    /** @internal protected */
    public constructor(def: IPrismaticJointDef) {
        super(def);

        this.m_localAnchorA.copy(def.localAnchorA ?? Vec2.ZERO);
        this.m_localAnchorB.copy(def.localAnchorB ?? Vec2.ZERO);
        Vec2.normalize(def.localAxisA ?? Vec2.UNITX, this.m_localXAxisA);
        Vec2.crossOneVec2(this.m_localXAxisA, this.m_localYAxisA);
        this.m_referenceAngle = def.referenceAngle ?? 0;
        this.m_lowerTranslation = def.lowerTranslation ?? 0;
        this.m_upperTranslation = def.upperTranslation ?? 0;
        // assert(this.m_lowerTranslation <= this.m_upperTranslation);
        this.m_maxMotorForce = def.maxMotorForce ?? 0;
        this.m_motorSpeed = def.motorSpeed ?? 0;
        this.m_enableLimit = def.enableLimit ?? false;
        this.m_enableMotor = def.enableMotor ?? false;
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

        const { qA, qB, lalcA, lalcB, rA, rB } = temp;
        qA.set(aA);
        qB.set(aB);

        // Compute the effective masses.
        Rot.multiplyVec2(qA, Vec2.subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), rA);
        Rot.multiplyVec2(qB, Vec2.subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), rB);
        const d = Vec2.subtract(cB, cA, temp.InitVelocityConstraints.d).add(rB).subtract(rA);

        const mA = this.m_invMassA;
        const mB = this.m_invMassB;
        const iA = this.m_invIA;
        const iB = this.m_invIB;

        // Compute motor Jacobian and effective mass.
        Rot.multiplyVec2(qA, this.m_localXAxisA, this.m_axis);
        this.m_a1 = Vec2.cross(Vec2.add(d, rA, Vec2.s_t0), this.m_axis);
        this.m_a2 = Vec2.cross(rB, this.m_axis);

        this.m_axialMass = mA + mB + iA * this.m_a1 * this.m_a1 + iB * this.m_a2 * this.m_a2;
        if (this.m_axialMass > 0) {
            this.m_axialMass = 1 / this.m_axialMass;
        }

        // Prismatic constraint.
        Rot.multiplyVec2(qA, this.m_localYAxisA, this.m_perp);

        this.m_s1 = Vec2.cross(Vec2.add(d, rA, Vec2.s_t0), this.m_perp);
        this.m_s2 = Vec2.cross(rB, this.m_perp);

        const k11 = mA + mB + iA * this.m_s1 * this.m_s1 + iB * this.m_s2 * this.m_s2;
        const k12 = iA * this.m_s1 + iB * this.m_s2;
        let k22 = iA + iB;
        if (k22 === 0) {
            // For bodies with fixed rotation.
            k22 = 1;
        }

        this.m_K.ex.set(k11, k12);
        this.m_K.ey.set(k12, k22);

        if (this.m_enableLimit) {
            this.m_translation = Vec2.dot(this.m_axis, d);
        } else {
            this.m_lowerImpulse = 0;
            this.m_upperImpulse = 0;
        }

        if (!this.m_enableMotor) {
            this.m_motorImpulse = 0;
        }

        if (data.step.warmStarting) {
            // Account for variable time step.
            this.m_impulse.scale(data.step.dtRatio);
            this.m_motorImpulse *= data.step.dtRatio;
            this.m_lowerImpulse *= data.step.dtRatio;
            this.m_upperImpulse *= data.step.dtRatio;

            const axialImpulse = this.m_motorImpulse + this.m_lowerImpulse - this.m_upperImpulse;
            const { P } = temp.InitVelocityConstraints;
            Vec2.scale(this.m_impulse.x, this.m_perp, P).addScaled(axialImpulse, this.m_axis);
            const LA = this.m_impulse.x * this.m_s1 + this.m_impulse.y + axialImpulse * this.m_a1;
            const LB = this.m_impulse.x * this.m_s2 + this.m_impulse.y + axialImpulse * this.m_a2;

            vA.subtractScaled(mA, P);
            wA -= iA * LA;

            vB.addScaled(mB, P);
            wB += iB * LB;
        } else {
            this.m_impulse.setZero();
            this.m_motorImpulse = 0;
            this.m_lowerImpulse = 0;
            this.m_upperImpulse = 0;
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

        const { P, df } = temp.SolveVelocityConstraints;

        // Solve linear motor constraint.
        if (this.m_enableMotor) {
            const Cdot = Vec2.dot(this.m_axis, Vec2.subtract(vB, vA, Vec2.s_t0)) + this.m_a2 * wB - this.m_a1 * wA;
            let impulse = this.m_axialMass * (this.m_motorSpeed - Cdot);
            const oldImpulse = this.m_motorImpulse;
            const maxImpulse = data.step.dt * this.m_maxMotorForce;
            this.m_motorImpulse = clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.m_motorImpulse - oldImpulse;

            Vec2.scale(impulse, this.m_axis, P);
            const LA = impulse * this.m_a1;
            const LB = impulse * this.m_a2;

            vA.subtractScaled(mA, P);
            wA -= iA * LA;
            vB.addScaled(mB, P);
            wB += iB * LB;
        }

        if (this.m_enableLimit) {
            // Lower limit
            {
                const C = this.m_translation - this.m_lowerTranslation;
                const Cdot = Vec2.dot(this.m_axis, Vec2.subtract(vB, vA, Vec2.s_t0)) + this.m_a2 * wB - this.m_a1 * wA;
                let impulse = -this.m_axialMass * (Cdot + Math.max(C, 0) * data.step.inv_dt);
                const oldImpulse = this.m_lowerImpulse;
                this.m_lowerImpulse = Math.max(this.m_lowerImpulse + impulse, 0);
                impulse = this.m_lowerImpulse - oldImpulse;

                Vec2.scale(impulse, this.m_axis, P);
                const LA = impulse * this.m_a1;
                const LB = impulse * this.m_a2;

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
                const Cdot = Vec2.dot(this.m_axis, Vec2.subtract(vA, vB, Vec2.s_t0)) + this.m_a1 * wA - this.m_a2 * wB;
                let impulse = -this.m_axialMass * (Cdot + Math.max(C, 0) * data.step.inv_dt);
                const oldImpulse = this.m_upperImpulse;
                this.m_upperImpulse = Math.max(this.m_upperImpulse + impulse, 0);
                impulse = this.m_upperImpulse - oldImpulse;

                Vec2.scale(impulse, this.m_axis, P);
                const LA = impulse * this.m_a1;
                const LB = impulse * this.m_a2;

                vA.addScaled(mA, P);
                wA += iA * LA;
                vB.subtractScaled(mB, P);
                wB -= iB * LB;
            }
        }

        // Solve the prismatic constraint in block form.
        {
            const Cdot_x = Vec2.dot(this.m_perp, Vec2.subtract(vB, vA, Vec2.s_t0)) + this.m_s2 * wB - this.m_s1 * wA;
            const Cdot_y = wB - wA;

            this.m_K.solve(-Cdot_x, -Cdot_y, df);
            this.m_impulse.add(df);

            Vec2.scale(df.x, this.m_perp, P);
            const LA = df.x * this.m_s1 + df.y;
            const LB = df.x * this.m_s2 + df.y;

            vA.subtractScaled(mA, P);
            wA -= iA * LA;

            vB.addScaled(mB, P);
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
    public solvePositionConstraints(data: SolverData): boolean {
        const cA = data.positions[this.m_indexA].c;
        let aA = data.positions[this.m_indexA].a;
        const cB = data.positions[this.m_indexB].c;
        let aB = data.positions[this.m_indexB].a;

        const { qA, qB, lalcA, lalcB, rA, rB } = temp;
        qA.set(aA);
        qB.set(aB);

        const mA = this.m_invMassA;
        const mB = this.m_invMassB;
        const iA = this.m_invIA;
        const iB = this.m_invIB;

        // Compute fresh Jacobians
        const { d, impulse, P } = temp.SolvePositionConstraints;
        Rot.multiplyVec2(qA, Vec2.subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), rA);
        Rot.multiplyVec2(qB, Vec2.subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), rB);
        Vec2.add(cB, rB, d).subtract(cA).subtract(rA);

        const axis = Rot.multiplyVec2(qA, this.m_localXAxisA, this.m_axis);
        const a1 = Vec2.cross(Vec2.add(d, rA, Vec2.s_t0), axis);
        const a2 = Vec2.cross(rB, axis);
        const perp = Rot.multiplyVec2(qA, this.m_localYAxisA, this.m_perp);

        const s1 = Vec2.cross(Vec2.add(d, rA, Vec2.s_t0), perp);
        const s2 = Vec2.cross(rB, perp);

        const C1_x = Vec2.dot(perp, d);
        const C1_y = aB - aA - this.m_referenceAngle;

        let linearError = Math.abs(C1_x);
        const angularError = Math.abs(C1_y);

        let active = false;
        let C2 = 0;
        if (this.m_enableLimit) {
            const translation = Vec2.dot(axis, d);
            if (Math.abs(this.m_upperTranslation - this.m_lowerTranslation) < 2 * LINEAR_SLOP) {
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
            K.ex.set(k11, k12, k13);
            K.ey.set(k12, k22, k23);
            K.ez.set(k13, k23, k33);

            K.solve33(-C1_x, -C1_y, -C2, impulse);
        } else {
            const k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            const k12 = iA * s1 + iB * s2;
            let k22 = iA + iB;
            if (k22 === 0) {
                k22 = 1;
            }

            const K = temp.K2;
            K.ex.set(k11, k12);
            K.ey.set(k12, k22);

            const impulse1 = K.solve(-C1_x, -C1_y, temp.SolvePositionConstraints.impulse1);
            impulse.x = impulse1.x;
            impulse.y = impulse1.y;
            impulse.z = 0;
        }

        Vec2.scale(impulse.x, perp, P).addScaled(impulse.z, axis);
        const LA = impulse.x * s1 + impulse.y + impulse.z * a1;
        const LB = impulse.x * s2 + impulse.y + impulse.z * a2;

        cA.subtractScaled(mA, P);
        aA -= iA * LA;
        cB.addScaled(mB, P);
        aB += iB * LB;

        data.positions[this.m_indexA].a = aA;
        data.positions[this.m_indexB].a = aB;

        return linearError <= LINEAR_SLOP && angularError <= ANGULAR_SLOP;
    }

    public getAnchorA<T extends XY>(out: T): T {
        return this.m_bodyA.getWorldPoint(this.m_localAnchorA, out);
    }

    public getAnchorB<T extends XY>(out: T): T {
        return this.m_bodyB.getWorldPoint(this.m_localAnchorB, out);
    }

    public getReactionForce<T extends XY>(inv_dt: number, out: T): T {
        const f = this.m_motorImpulse + this.m_lowerImpulse - this.m_upperImpulse;
        out.x = inv_dt * (this.m_impulse.x * this.m_perp.x + f * this.m_axis.x);
        out.y = inv_dt * (this.m_impulse.x * this.m_perp.y + f * this.m_axis.y);
        return out;
    }

    public getReactionTorque(inv_dt: number): number {
        return inv_dt * this.m_impulse.y;
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

    public getReferenceAngle() {
        return this.m_referenceAngle;
    }

    public getJointTranslation(): number {
        const { pA, pB, axis, d } = temp.GetJointTranslation;
        this.m_bodyA.getWorldPoint(this.m_localAnchorA, pA);
        this.m_bodyB.getWorldPoint(this.m_localAnchorB, pB);
        Vec2.subtract(pB, pA, d);
        this.m_bodyA.getWorldVector(this.m_localXAxisA, axis);

        const translation = Vec2.dot(d, axis);
        return translation;
    }

    public getJointSpeed(): number {
        const bA = this.m_bodyA;
        const bB = this.m_bodyB;

        const { lalcA, lalcB, rA, rB } = temp;
        Rot.multiplyVec2(bA.m_xf.q, Vec2.subtract(this.m_localAnchorA, bA.m_sweep.localCenter, lalcA), rA);
        Rot.multiplyVec2(bB.m_xf.q, Vec2.subtract(this.m_localAnchorB, bB.m_sweep.localCenter, lalcB), rB);
        const p1 = Vec2.add(bA.m_sweep.c, rA, Vec2.s_t0);
        const p2 = Vec2.add(bB.m_sweep.c, rB, Vec2.s_t1);
        const d = Vec2.subtract(p2, p1, Vec2.s_t2);
        const axis = Rot.multiplyVec2(bA.m_xf.q, this.m_localXAxisA, this.m_axis);

        const vA = bA.m_linearVelocity;
        const vB = bB.m_linearVelocity;
        const wA = bA.m_angularVelocity;
        const wB = bB.m_angularVelocity;

        const speed =
            Vec2.dot(d, Vec2.crossScalarVec2(wA, axis, Vec2.s_t0)) +
            Vec2.dot(
                axis,
                Vec2.subtract(
                    Vec2.addCrossScalarVec2(vB, wB, rB, Vec2.s_t0),
                    Vec2.addCrossScalarVec2(vA, wA, rA, Vec2.s_t1),
                    Vec2.s_t0,
                ),
            );
        return speed;
    }

    public isLimitEnabled() {
        return this.m_enableLimit;
    }

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

    public getLowerLimit() {
        return this.m_lowerTranslation;
    }

    public getUpperLimit() {
        return this.m_upperTranslation;
    }

    public setLimits(lower: number, upper: number): void {
        // DEBUG: assert(lower <= upper);
        if (lower !== this.m_lowerTranslation || upper !== this.m_upperTranslation) {
            this.m_bodyA.setAwake(true);
            this.m_bodyB.setAwake(true);
            this.m_lowerTranslation = lower;
            this.m_upperTranslation = upper;
            this.m_lowerImpulse = 0;
            this.m_upperImpulse = 0;
        }
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

    public getMotorSpeed() {
        return this.m_motorSpeed;
    }

    public setMaxMotorForce(force: number): void {
        if (force !== this.m_maxMotorForce) {
            this.m_bodyA.setAwake(true);
            this.m_bodyB.setAwake(true);
            this.m_maxMotorForce = force;
        }
    }

    public getMaxMotorForce(): number {
        return this.m_maxMotorForce;
    }

    public getMotorForce(inv_dt: number): number {
        return inv_dt * this.m_motorImpulse;
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
