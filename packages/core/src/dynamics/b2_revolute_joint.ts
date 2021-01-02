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
import { clamp, Vec2, Mat22, Rot, XY, Transform } from "../common/b2_math";
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

    public initialize(bA: Body, bB: Body, anchor: XY): void {
        this.bodyA = bA;
        this.bodyB = bB;
        this.bodyA.getLocalPoint(anchor, this.localAnchorA);
        this.bodyB.getLocalPoint(anchor, this.localAnchorB);
        this.referenceAngle = this.bodyB.getAngle() - this.bodyA.getAngle();
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
    public readonly localAnchorA = new Vec2();

    /** @internal protected */
    public readonly localAnchorB = new Vec2();

    protected readonly impulse = new Vec2();

    protected motorImpulse = 0;

    protected lowerImpulse = 0;

    protected upperImpulse = 0;

    protected m_enableMotor = false;

    protected maxMotorTorque = 0;

    protected motorSpeed = 0;

    protected m_enableLimit = false;

    /** @internal protected */
    public referenceAngle = 0;

    protected lowerAngle = 0;

    protected upperAngle = 0;

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

    protected readonly K = new Mat22();

    protected angle = 0;

    protected axialMass = 0;

    /** @internal protected */
    public constructor(def: IRevoluteJointDef) {
        super(def);

        this.localAnchorA.copy(def.localAnchorA ?? Vec2.ZERO);
        this.localAnchorB.copy(def.localAnchorB ?? Vec2.ZERO);
        this.referenceAngle = def.referenceAngle ?? 0;

        this.impulse.setZero();

        this.lowerAngle = def.lowerAngle ?? 0;
        this.upperAngle = def.upperAngle ?? 0;
        this.maxMotorTorque = def.maxMotorTorque ?? 0;
        this.motorSpeed = def.motorSpeed ?? 0;
        this.m_enableLimit = def.enableLimit ?? false;
        this.m_enableMotor = def.enableMotor ?? false;
    }

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

        const { qA, qB, lalcA, lalcB } = temp;
        qA.set(aA);
        qB.set(aB);

        Rot.multiplyVec2(qA, Vec2.subtract(this.localAnchorA, this.localCenterA, lalcA), this.rA);
        Rot.multiplyVec2(qB, Vec2.subtract(this.localAnchorB, this.localCenterB, lalcB), this.rB);

        // J = [-I -r1_skew I r2_skew]
        // r_skew = [-ry; rx]

        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x]
        //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB]

        const mA = this.invMassA;
        const mB = this.invMassB;
        const iA = this.invIA;
        const iB = this.invIB;

        this.K.ex.x = mA + mB + this.rA.y * this.rA.y * iA + this.rB.y * this.rB.y * iB;
        this.K.ey.x = -this.rA.y * this.rA.x * iA - this.rB.y * this.rB.x * iB;
        this.K.ex.y = this.K.ey.x;
        this.K.ey.y = mA + mB + this.rA.x * this.rA.x * iA + this.rB.x * this.rB.x * iB;

        this.axialMass = iA + iB;
        let fixedRotation: boolean;
        if (this.axialMass > 0) {
            this.axialMass = 1 / this.axialMass;
            fixedRotation = false;
        } else {
            fixedRotation = true;
        }

        this.angle = aB - aA - this.referenceAngle;
        if (this.m_enableLimit === false || fixedRotation) {
            this.lowerImpulse = 0;
            this.upperImpulse = 0;
        }

        if (this.m_enableMotor === false || fixedRotation) {
            this.motorImpulse = 0;
        }

        if (data.step.warmStarting) {
            // Scale impulses to support a variable time step.
            this.impulse.scale(data.step.dtRatio);
            this.motorImpulse *= data.step.dtRatio;
            this.lowerImpulse *= data.step.dtRatio;
            this.upperImpulse *= data.step.dtRatio;

            const axialImpulse = this.motorImpulse + this.lowerImpulse - this.upperImpulse;
            const P = temp.P.set(this.impulse.x, this.impulse.y);

            vA.subtractScaled(mA, P);
            wA -= iA * (Vec2.cross(this.rA, P) + axialImpulse);

            vB.addScaled(mB, P);
            wB += iB * (Vec2.cross(this.rB, P) + axialImpulse);
        } else {
            this.impulse.setZero();
            this.motorImpulse = 0;
            this.lowerImpulse = 0;
            this.upperImpulse = 0;
        }

        data.velocities[this.indexA].w = wA;
        data.velocities[this.indexB].w = wB;
    }

    public solveVelocityConstraints(data: SolverData): void {
        const vA = data.velocities[this.indexA].v;
        let wA = data.velocities[this.indexA].w;
        const vB = data.velocities[this.indexB].v;
        let wB = data.velocities[this.indexB].w;

        const mA = this.invMassA;
        const mB = this.invMassB;
        const iA = this.invIA;
        const iB = this.invIB;

        const fixedRotation = iA + iB === 0;

        // Solve motor constraint.
        if (this.m_enableMotor && !fixedRotation) {
            const Cdot = wB - wA - this.motorSpeed;
            let impulse = -this.axialMass * Cdot;
            const oldImpulse = this.motorImpulse;
            const maxImpulse = data.step.dt * this.maxMotorTorque;
            this.motorImpulse = clamp(this.motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.motorImpulse - oldImpulse;

            wA -= iA * impulse;
            wB += iB * impulse;
        }

        // Solve limit constraint.
        if (this.m_enableLimit && !fixedRotation) {
            // Lower limit
            {
                const C = this.angle - this.lowerAngle;
                const Cdot = wB - wA;
                let impulse = -this.axialMass * (Cdot + Math.max(C, 0) * data.step.inv_dt);
                const oldImpulse = this.lowerImpulse;
                this.lowerImpulse = Math.max(this.lowerImpulse + impulse, 0);
                impulse = this.lowerImpulse - oldImpulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            // Upper limit
            // Note: signs are flipped to keep C positive when the constraint is satisfied.
            // This also keeps the impulse positive when the limit is active.
            {
                const C = this.upperAngle - this.angle;
                const Cdot = wA - wB;
                let impulse = -this.axialMass * (Cdot + Math.max(C, 0) * data.step.inv_dt);
                const oldImpulse = this.upperImpulse;
                this.upperImpulse = Math.max(this.upperImpulse + impulse, 0);
                impulse = this.upperImpulse - oldImpulse;

                wA += iA * impulse;
                wB -= iB * impulse;
            }
        }

        // Solve point-to-point constraint
        {
            const { Cdot, impulse } = temp;
            Vec2.subtract(
                Vec2.addCrossScalarVec2(vB, wB, this.rB, Vec2.s_t0),
                Vec2.addCrossScalarVec2(vA, wA, this.rA, Vec2.s_t1),
                Cdot,
            );
            this.K.solve(-Cdot.x, -Cdot.y, impulse);

            this.impulse.x += impulse.x;
            this.impulse.y += impulse.y;

            vA.subtractScaled(mA, impulse);
            wA -= iA * Vec2.cross(this.rA, impulse);

            vB.addScaled(mB, impulse);
            wB += iB * Vec2.cross(this.rB, impulse);
        }

        data.velocities[this.indexA].w = wA;
        data.velocities[this.indexB].w = wB;
    }

    public solvePositionConstraints(data: SolverData): boolean {
        const cA = data.positions[this.indexA].c;
        let aA = data.positions[this.indexA].a;
        const cB = data.positions[this.indexB].c;
        let aB = data.positions[this.indexB].a;

        const { qA, qB, lalcA, lalcB, impulse } = temp;
        qA.set(aA);
        qB.set(aB);

        let angularError = 0;
        let positionError = 0;

        const fixedRotation = this.invIA + this.invIB === 0;

        // Solve angular limit constraint
        if (this.m_enableLimit && !fixedRotation) {
            const angle = aB - aA - this.referenceAngle;
            let C = 0;

            if (Math.abs(this.upperAngle - this.lowerAngle) < 2 * ANGULAR_SLOP) {
                // Prevent large angular corrections
                C = clamp(angle - this.lowerAngle, -MAX_ANGULAR_CORRECTION, MAX_ANGULAR_CORRECTION);
            } else if (angle <= this.lowerAngle) {
                // Prevent large angular corrections and allow some slop.
                C = clamp(angle - this.lowerAngle + ANGULAR_SLOP, -MAX_ANGULAR_CORRECTION, 0);
            } else if (angle >= this.upperAngle) {
                // Prevent large angular corrections and allow some slop.
                C = clamp(angle - this.upperAngle - ANGULAR_SLOP, 0, MAX_ANGULAR_CORRECTION);
            }

            const limitImpulse = -this.axialMass * C;
            aA -= this.invIA * limitImpulse;
            aB += this.invIB * limitImpulse;
            angularError = Math.abs(C);
        }

        // Solve point-to-point constraint.
        {
            qA.set(aA);
            qB.set(aB);
            const rA = Rot.multiplyVec2(qA, Vec2.subtract(this.localAnchorA, this.localCenterA, lalcA), this.rA);
            const rB = Rot.multiplyVec2(qB, Vec2.subtract(this.localAnchorB, this.localCenterB, lalcB), this.rB);

            const C = Vec2.add(cB, rB, temp.C).subtract(cA).subtract(rA);
            positionError = C.length();

            const mA = this.invMassA;
            const mB = this.invMassB;
            const iA = this.invIA;
            const iB = this.invIB;

            const { K } = this;
            K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
            K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
            K.ey.x = K.ex.y;
            K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;

            K.solve(C.x, C.y, impulse).negate();

            cA.subtractScaled(mA, impulse);
            aA -= iA * Vec2.cross(rA, impulse);

            cB.addScaled(mB, impulse);
            aB += iB * Vec2.cross(rB, impulse);
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
        return inv_dt * (this.motorImpulse + this.lowerImpulse - this.upperImpulse);
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

    public getJointAngle(): number {
        return this.bodyB.sweep.a - this.bodyA.sweep.a - this.referenceAngle;
    }

    public getJointSpeed(): number {
        return this.bodyB.angularVelocity - this.bodyA.angularVelocity;
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

    public getMotorTorque(inv_dt: number): number {
        return inv_dt * this.motorImpulse;
    }

    public getMotorSpeed(): number {
        return this.motorSpeed;
    }

    public setMaxMotorTorque(torque: number): void {
        if (torque !== this.maxMotorTorque) {
            this.bodyA.setAwake(true);
            this.bodyB.setAwake(true);
            this.maxMotorTorque = torque;
        }
    }

    public getMaxMotorTorque(): number {
        return this.maxMotorTorque;
    }

    public isLimitEnabled(): boolean {
        return this.m_enableLimit;
    }

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

    public getLowerLimit(): number {
        return this.lowerAngle;
    }

    public getUpperLimit(): number {
        return this.upperAngle;
    }

    public setLimits(lower: number, upper: number): void {
        if (lower !== this.lowerAngle || upper !== this.upperAngle) {
            this.bodyA.setAwake(true);
            this.bodyB.setAwake(true);
            this.lowerImpulse = 0;
            this.upperImpulse = 0;
            this.lowerAngle = lower;
            this.upperAngle = upper;
        }
    }

    public setMotorSpeed(speed: number): number {
        if (speed !== this.motorSpeed) {
            this.bodyA.setAwake(true);
            this.bodyB.setAwake(true);
            this.motorSpeed = speed;
        }
        return speed;
    }

    public draw(draw: Draw): void {
        const { p2, r, pA, pB } = temp;
        const xfA = this.bodyA.getTransform();
        const xfB = this.bodyB.getTransform();
        Transform.multiplyVec2(xfA, this.localAnchorA, pA);
        Transform.multiplyVec2(xfB, this.localAnchorB, pB);

        draw.drawPoint(pA, 5, debugColors.joint4);
        draw.drawPoint(pB, 5, debugColors.joint5);

        const aA = this.bodyA.getAngle();
        const aB = this.bodyB.getAngle();
        const angle = aB - aA - this.referenceAngle;

        const L = 0.5;

        r.set(Math.cos(angle), Math.sin(angle)).scale(L);
        draw.drawSegment(pB, Vec2.add(pB, r, p2), debugColors.joint1);
        draw.drawCircle(pB, L, debugColors.joint1);

        if (this.m_enableLimit) {
            const { rlo, rhi } = temp;
            rlo.set(Math.cos(this.lowerAngle), Math.sin(this.lowerAngle)).scale(L);
            rhi.set(Math.cos(this.upperAngle), Math.sin(this.upperAngle)).scale(L);
            draw.drawSegment(pB, Vec2.add(pB, rlo, p2), debugColors.joint2);
            draw.drawSegment(pB, Vec2.add(pB, rhi, p2), debugColors.joint3);
        }

        draw.drawSegment(xfA.p, pA, debugColors.joint6);
        draw.drawSegment(pA, pB, debugColors.joint6);
        draw.drawSegment(xfB.p, pB, debugColors.joint6);
    }
}
