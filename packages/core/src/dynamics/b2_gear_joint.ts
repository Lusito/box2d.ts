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
import { Vec2, Rot, XY } from "../common/b2_math";
import { Joint, JointDef, JointType, IJointDef } from "./b2_joint";
import { PrismaticJoint } from "./b2_prismatic_joint";
import { RevoluteJoint } from "./b2_revolute_joint";
import { SolverData } from "./b2_time_step";
import { Body } from "./b2_body";

const temp = {
    qA: new Rot(),
    qB: new Rot(),
    qC: new Rot(),
    qD: new Rot(),
    lalcA: new Vec2(),
    lalcB: new Vec2(),
    lalcC: new Vec2(),
    lalcD: new Vec2(),
    u: new Vec2(),
    rA: new Vec2(),
    rB: new Vec2(),
    rC: new Vec2(),
    rD: new Vec2(),
    JvAC: new Vec2(),
    JvBD: new Vec2(),
};

export interface IGearJointDef extends IJointDef {
    joint1: RevoluteJoint | PrismaticJoint;

    joint2: RevoluteJoint | PrismaticJoint;

    ratio?: number;
}

/**
 * Gear joint definition. This definition requires two existing
 * revolute or prismatic joints (any combination will work).
 *
 * @warning bodyB on the input joints must both be dynamic
 */
export class GearJointDef extends JointDef implements IGearJointDef {
    /** The first revolute/prismatic joint attached to the gear joint. */
    public joint1!: RevoluteJoint | PrismaticJoint;

    /** The second revolute/prismatic joint attached to the gear joint. */
    public joint2!: RevoluteJoint | PrismaticJoint;

    /**
     * The gear ratio.
     *
     * @see GearJoint for explanation.
     */
    public ratio = 1;

    public constructor() {
        super(JointType.Gear);
    }
}

/**
 * A gear joint is used to connect two joints together. Either joint
 * can be a revolute or prismatic joint. You specify a gear ratio
 * to bind the motions together:
 * coordinate1 + ratio * coordinate2 = constant
 * The ratio can be negative or positive. If one joint is a revolute joint
 * and the other joint is a prismatic joint, then the ratio will have units
 * of length or units of 1/length.
 *
 * @warning You have to manually destroy the gear joint if joint1 or joint2
 * is destroyed.
 */
export class GearJoint extends Joint {
    protected m_joint1: RevoluteJoint | PrismaticJoint;

    protected m_joint2: RevoluteJoint | PrismaticJoint;

    protected m_typeA = JointType.Unknown;

    protected m_typeB = JointType.Unknown;

    /** Body A is connected to body C */
    protected m_bodyC: Body;

    /** Body B is connected to body D */
    protected m_bodyD: Body;

    // Solver shared
    protected readonly m_localAnchorA = new Vec2();

    protected readonly m_localAnchorB = new Vec2();

    protected readonly m_localAnchorC = new Vec2();

    protected readonly m_localAnchorD = new Vec2();

    protected readonly m_localAxisC = new Vec2();

    protected readonly m_localAxisD = new Vec2();

    protected m_referenceAngleA = 0;

    protected m_referenceAngleB = 0;

    protected m_constant = 0;

    protected m_ratio = 0;

    protected m_impulse = 0;

    // Solver temp
    protected m_indexA = 0;

    protected m_indexB = 0;

    protected m_indexC = 0;

    protected m_indexD = 0;

    protected readonly m_lcA = new Vec2();

    protected readonly m_lcB = new Vec2();

    protected readonly m_lcC = new Vec2();

    protected readonly m_lcD = new Vec2();

    protected m_mA = 0;

    protected m_mB = 0;

    protected m_mC = 0;

    protected m_mD = 0;

    protected m_iA = 0;

    protected m_iB = 0;

    protected m_iC = 0;

    protected m_iD = 0;

    protected readonly m_JvAC = new Vec2();

    protected readonly m_JvBD = new Vec2();

    protected m_JwA = 0;

    protected m_JwB = 0;

    protected m_JwC = 0;

    protected m_JwD = 0;

    protected m_mass = 0;

    /** @internal protected */
    public constructor(def: IGearJointDef) {
        super(def);

        this.m_joint1 = def.joint1;
        this.m_joint2 = def.joint2;

        this.m_typeA = this.m_joint1.getType();
        this.m_typeB = this.m_joint2.getType();

        // DEBUG: assert(this.m_typeA === JointType.Revolute || this.m_typeA === JointType.Prismatic);
        // DEBUG: assert(this.m_typeB === JointType.Revolute || this.m_typeB === JointType.Prismatic);

        let coordinateA: number;
        let coordinateB: number;

        // TODO_ERIN there might be some problem with the joint edges in Joint.

        this.m_bodyC = this.m_joint1.getBodyA();
        this.m_bodyA = this.m_joint1.getBodyB();

        // Body B on joint1 must be dynamic
        // DEBUG: assert(this.m_bodyA.m_type === BodyType.Dynamic);

        // Get geometry of joint1
        const xfA = this.m_bodyA.m_xf;
        const aA = this.m_bodyA.m_sweep.a;
        const xfC = this.m_bodyC.m_xf;
        const aC = this.m_bodyC.m_sweep.a;

        if (this.m_typeA === JointType.Revolute) {
            const revolute = def.joint1 as RevoluteJoint;
            this.m_localAnchorC.copy(revolute.m_localAnchorA);
            this.m_localAnchorA.copy(revolute.m_localAnchorB);
            this.m_referenceAngleA = revolute.m_referenceAngle;
            this.m_localAxisC.setZero();

            coordinateA = aA - aC - this.m_referenceAngleA;
        } else {
            const prismatic = def.joint1 as PrismaticJoint;
            this.m_localAnchorC.copy(prismatic.m_localAnchorA);
            this.m_localAnchorA.copy(prismatic.m_localAnchorB);
            this.m_referenceAngleA = prismatic.m_referenceAngle;
            this.m_localAxisC.copy(prismatic.m_localXAxisA);

            const pC = this.m_localAnchorC;
            const pA = Rot.transposeMultiplyVec2(
                xfC.q,
                Rot.multiplyVec2(xfA.q, this.m_localAnchorA, Vec2.s_t0).add(xfA.p).subtract(xfC.p),
                Vec2.s_t0,
            );
            coordinateA = Vec2.dot(pA.subtract(pC), this.m_localAxisC);
        }

        this.m_bodyD = this.m_joint2.getBodyA();
        this.m_bodyB = this.m_joint2.getBodyB();

        // Body B on joint2 must be dynamic
        // DEBUG: assert(this.m_bodyB.m_type === BodyType.Dynamic);

        // Get geometry of joint2
        const xfB = this.m_bodyB.m_xf;
        const aB = this.m_bodyB.m_sweep.a;
        const xfD = this.m_bodyD.m_xf;
        const aD = this.m_bodyD.m_sweep.a;

        if (this.m_typeB === JointType.Revolute) {
            const revolute = def.joint2 as RevoluteJoint;
            this.m_localAnchorD.copy(revolute.m_localAnchorA);
            this.m_localAnchorB.copy(revolute.m_localAnchorB);
            this.m_referenceAngleB = revolute.m_referenceAngle;
            this.m_localAxisD.setZero();

            coordinateB = aB - aD - this.m_referenceAngleB;
        } else {
            const prismatic = def.joint2 as PrismaticJoint;
            this.m_localAnchorD.copy(prismatic.m_localAnchorA);
            this.m_localAnchorB.copy(prismatic.m_localAnchorB);
            this.m_referenceAngleB = prismatic.m_referenceAngle;
            this.m_localAxisD.copy(prismatic.m_localXAxisA);

            const pD = this.m_localAnchorD;
            const pB = Rot.transposeMultiplyVec2(
                xfD.q,
                Rot.multiplyVec2(xfB.q, this.m_localAnchorB, Vec2.s_t0).add(xfB.p).subtract(xfD.p),
                Vec2.s_t0,
            );
            coordinateB = Vec2.dot(pB.subtract(pD), this.m_localAxisD);
        }

        this.m_ratio = def.ratio ?? 1;

        this.m_constant = coordinateA + this.m_ratio * coordinateB;

        this.m_impulse = 0;
    }

    /** @internal protected */
    public initVelocityConstraints(data: SolverData): void {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_indexC = this.m_bodyC.m_islandIndex;
        this.m_indexD = this.m_bodyD.m_islandIndex;
        this.m_lcA.copy(this.m_bodyA.m_sweep.localCenter);
        this.m_lcB.copy(this.m_bodyB.m_sweep.localCenter);
        this.m_lcC.copy(this.m_bodyC.m_sweep.localCenter);
        this.m_lcD.copy(this.m_bodyD.m_sweep.localCenter);
        this.m_mA = this.m_bodyA.m_invMass;
        this.m_mB = this.m_bodyB.m_invMass;
        this.m_mC = this.m_bodyC.m_invMass;
        this.m_mD = this.m_bodyD.m_invMass;
        this.m_iA = this.m_bodyA.m_invI;
        this.m_iB = this.m_bodyB.m_invI;
        this.m_iC = this.m_bodyC.m_invI;
        this.m_iD = this.m_bodyD.m_invI;

        const aA = data.positions[this.m_indexA].a;
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;

        const aB = data.positions[this.m_indexB].a;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;

        const aC = data.positions[this.m_indexC].a;
        const vC = data.velocities[this.m_indexC].v;
        let wC = data.velocities[this.m_indexC].w;

        const aD = data.positions[this.m_indexD].a;
        const vD = data.velocities[this.m_indexD].v;
        let wD = data.velocities[this.m_indexD].w;

        const { qA, qB, qC, qD } = temp;
        qA.set(aA);
        qB.set(aB);
        qC.set(aC);
        qD.set(aD);

        this.m_mass = 0;

        if (this.m_typeA === JointType.Revolute) {
            this.m_JvAC.setZero();
            this.m_JwA = 1;
            this.m_JwC = 1;
            this.m_mass += this.m_iA + this.m_iC;
        } else {
            const { u, rC, rA, lalcA, lalcC } = temp;
            Rot.multiplyVec2(qC, this.m_localAxisC, u);
            Rot.multiplyVec2(qC, Vec2.subtract(this.m_localAnchorC, this.m_lcC, lalcC), rC);
            Rot.multiplyVec2(qA, Vec2.subtract(this.m_localAnchorA, this.m_lcA, lalcA), rA);
            this.m_JvAC.copy(u);
            this.m_JwC = Vec2.cross(rC, u);
            this.m_JwA = Vec2.cross(rA, u);
            this.m_mass +=
                this.m_mC + this.m_mA + this.m_iC * this.m_JwC * this.m_JwC + this.m_iA * this.m_JwA * this.m_JwA;
        }

        if (this.m_typeB === JointType.Revolute) {
            this.m_JvBD.setZero();
            this.m_JwB = this.m_ratio;
            this.m_JwD = this.m_ratio;
            this.m_mass += this.m_ratio * this.m_ratio * (this.m_iB + this.m_iD);
        } else {
            const { u, rB, rD, lalcB, lalcD } = temp;
            Rot.multiplyVec2(qD, this.m_localAxisD, u);
            Rot.multiplyVec2(qD, Vec2.subtract(this.m_localAnchorD, this.m_lcD, lalcD), rD);
            Rot.multiplyVec2(qB, Vec2.subtract(this.m_localAnchorB, this.m_lcB, lalcB), rB);
            Vec2.scale(this.m_ratio, u, this.m_JvBD);
            this.m_JwD = this.m_ratio * Vec2.cross(rD, u);
            this.m_JwB = this.m_ratio * Vec2.cross(rB, u);
            this.m_mass +=
                this.m_ratio * this.m_ratio * (this.m_mD + this.m_mB) +
                this.m_iD * this.m_JwD * this.m_JwD +
                this.m_iB * this.m_JwB * this.m_JwB;
        }

        // Compute effective mass.
        this.m_mass = this.m_mass > 0 ? 1 / this.m_mass : 0;

        if (data.step.warmStarting) {
            vA.addScaled(this.m_mA * this.m_impulse, this.m_JvAC);
            wA += this.m_iA * this.m_impulse * this.m_JwA;
            vB.addScaled(this.m_mB * this.m_impulse, this.m_JvBD);
            wB += this.m_iB * this.m_impulse * this.m_JwB;
            vC.subtractScaled(this.m_mC * this.m_impulse, this.m_JvAC);
            wC -= this.m_iC * this.m_impulse * this.m_JwC;
            vD.subtractScaled(this.m_mD * this.m_impulse, this.m_JvBD);
            wD -= this.m_iD * this.m_impulse * this.m_JwD;
        } else {
            this.m_impulse = 0;
        }

        data.velocities[this.m_indexA].w = wA;
        data.velocities[this.m_indexB].w = wB;
        data.velocities[this.m_indexC].w = wC;
        data.velocities[this.m_indexD].w = wD;
    }

    /** @internal protected */
    public solveVelocityConstraints(data: SolverData): void {
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;
        const vC = data.velocities[this.m_indexC].v;
        let wC = data.velocities[this.m_indexC].w;
        const vD = data.velocities[this.m_indexD].v;
        let wD = data.velocities[this.m_indexD].w;

        let Cdot =
            Vec2.dot(this.m_JvAC, Vec2.subtract(vA, vC, Vec2.s_t0)) +
            Vec2.dot(this.m_JvBD, Vec2.subtract(vB, vD, Vec2.s_t0));
        Cdot += this.m_JwA * wA - this.m_JwC * wC + (this.m_JwB * wB - this.m_JwD * wD);

        const impulse = -this.m_mass * Cdot;
        this.m_impulse += impulse;

        vA.addScaled(this.m_mA * impulse, this.m_JvAC);
        wA += this.m_iA * impulse * this.m_JwA;
        vB.addScaled(this.m_mB * impulse, this.m_JvBD);
        wB += this.m_iB * impulse * this.m_JwB;
        vC.subtractScaled(this.m_mC * impulse, this.m_JvAC);
        wC -= this.m_iC * impulse * this.m_JwC;
        vD.subtractScaled(this.m_mD * impulse, this.m_JvBD);
        wD -= this.m_iD * impulse * this.m_JwD;

        data.velocities[this.m_indexA].w = wA;
        data.velocities[this.m_indexB].w = wB;
        data.velocities[this.m_indexC].w = wC;
        data.velocities[this.m_indexD].w = wD;
    }

    /** @internal protected */
    public solvePositionConstraints(data: SolverData): boolean {
        const cA = data.positions[this.m_indexA].c;
        let aA = data.positions[this.m_indexA].a;
        const cB = data.positions[this.m_indexB].c;
        let aB = data.positions[this.m_indexB].a;
        const cC = data.positions[this.m_indexC].c;
        let aC = data.positions[this.m_indexC].a;
        const cD = data.positions[this.m_indexD].c;
        let aD = data.positions[this.m_indexD].a;

        const { qA, qB, qC, qD, JvAC, JvBD } = temp;
        qA.set(aA);
        qB.set(aB);
        qC.set(aC);
        qD.set(aD);

        const linearError = 0;

        let coordinateA: number;
        let coordinateB: number;

        let JwA: number;
        let JwB: number;
        let JwC: number;
        let JwD: number;
        let mass = 0;

        if (this.m_typeA === JointType.Revolute) {
            JvAC.setZero();
            JwA = 1;
            JwC = 1;
            mass += this.m_iA + this.m_iC;

            coordinateA = aA - aC - this.m_referenceAngleA;
        } else {
            const { u, rC, rA, lalcC, lalcA } = temp;
            Rot.multiplyVec2(qC, this.m_localAxisC, u);
            Rot.multiplyVec2(qC, Vec2.subtract(this.m_localAnchorC, this.m_lcC, lalcC), rC);
            Rot.multiplyVec2(qA, Vec2.subtract(this.m_localAnchorA, this.m_lcA, lalcA), rA);
            JvAC.copy(u);
            JwC = Vec2.cross(rC, u);
            JwA = Vec2.cross(rA, u);
            mass += this.m_mC + this.m_mA + this.m_iC * JwC * JwC + this.m_iA * JwA * JwA;

            const pC = lalcC;
            const pA = Rot.transposeMultiplyVec2(qC, Vec2.add(rA, cA, Vec2.s_t0).subtract(cC), Vec2.s_t0);
            coordinateA = Vec2.dot(Vec2.subtract(pA, pC, Vec2.s_t0), this.m_localAxisC);
        }

        if (this.m_typeB === JointType.Revolute) {
            JvBD.setZero();
            JwB = this.m_ratio;
            JwD = this.m_ratio;
            mass += this.m_ratio * this.m_ratio * (this.m_iB + this.m_iD);

            coordinateB = aB - aD - this.m_referenceAngleB;
        } else {
            const { u, rD, rB, lalcD, lalcB } = temp;
            Rot.multiplyVec2(qD, this.m_localAxisD, u);
            Rot.multiplyVec2(qD, Vec2.subtract(this.m_localAnchorD, this.m_lcD, lalcD), rD);
            Rot.multiplyVec2(qB, Vec2.subtract(this.m_localAnchorB, this.m_lcB, lalcB), rB);
            Vec2.scale(this.m_ratio, u, JvBD);
            JwD = this.m_ratio * Vec2.cross(rD, u);
            JwB = this.m_ratio * Vec2.cross(rB, u);
            mass +=
                this.m_ratio * this.m_ratio * (this.m_mD + this.m_mB) + this.m_iD * JwD * JwD + this.m_iB * JwB * JwB;

            const pD = lalcD;
            const pB = Rot.transposeMultiplyVec2(qD, Vec2.add(rB, cB, Vec2.s_t0).subtract(cD), Vec2.s_t0);
            coordinateB = Vec2.dot(pB.subtract(pD), this.m_localAxisD);
        }

        const C = coordinateA + this.m_ratio * coordinateB - this.m_constant;

        let impulse = 0;
        if (mass > 0) {
            impulse = -C / mass;
        }

        cA.addScaled(this.m_mA * impulse, JvAC);
        aA += this.m_iA * impulse * JwA;
        cB.addScaled(this.m_mB * impulse, JvBD);
        aB += this.m_iB * impulse * JwB;
        cC.subtractScaled(this.m_mC * impulse, JvAC);
        aC -= this.m_iC * impulse * JwC;
        cD.subtractScaled(this.m_mD * impulse, JvBD);
        aD -= this.m_iD * impulse * JwD;

        data.positions[this.m_indexA].a = aA;
        data.positions[this.m_indexB].a = aB;
        data.positions[this.m_indexC].a = aC;
        data.positions[this.m_indexD].a = aD;

        // TODO_ERIN not implemented
        return linearError < LINEAR_SLOP;
    }

    public getAnchorA<T extends XY>(out: T): T {
        return this.m_bodyA.getWorldPoint(this.m_localAnchorA, out);
    }

    public getAnchorB<T extends XY>(out: T): T {
        return this.m_bodyB.getWorldPoint(this.m_localAnchorB, out);
    }

    public getReactionForce<T extends XY>(inv_dt: number, out: T): T {
        return Vec2.scale(inv_dt * this.m_impulse, this.m_JvAC, out);
    }

    public getReactionTorque(inv_dt: number): number {
        return inv_dt * this.m_impulse * this.m_JwA;
    }

    public getJoint1() {
        return this.m_joint1;
    }

    public getJoint2() {
        return this.m_joint2;
    }

    public getRatio() {
        return this.m_ratio;
    }

    public setRatio(ratio: number): void {
        // DEBUG: assert(Number.isFinite(ratio));
        this.m_ratio = ratio;
    }
}
