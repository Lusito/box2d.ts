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
    protected joint1: RevoluteJoint | PrismaticJoint;

    protected joint2: RevoluteJoint | PrismaticJoint;

    protected typeA = JointType.Unknown;

    protected typeB = JointType.Unknown;

    /** Body A is connected to body C */
    protected bodyC: Body;

    /** Body B is connected to body D */
    protected bodyD: Body;

    // Solver shared
    protected readonly localAnchorA = new Vec2();

    protected readonly localAnchorB = new Vec2();

    protected readonly localAnchorC = new Vec2();

    protected readonly localAnchorD = new Vec2();

    protected readonly localAxisC = new Vec2();

    protected readonly localAxisD = new Vec2();

    protected referenceAngleA = 0;

    protected referenceAngleB = 0;

    protected constant = 0;

    protected ratio = 0;

    protected impulse = 0;

    // Solver temp
    protected indexA = 0;

    protected indexB = 0;

    protected indexC = 0;

    protected indexD = 0;

    protected readonly lcA = new Vec2();

    protected readonly lcB = new Vec2();

    protected readonly lcC = new Vec2();

    protected readonly lcD = new Vec2();

    protected mA = 0;

    protected mB = 0;

    protected mC = 0;

    protected mD = 0;

    protected iA = 0;

    protected iB = 0;

    protected iC = 0;

    protected iD = 0;

    protected readonly JvAC = new Vec2();

    protected readonly JvBD = new Vec2();

    protected JwA = 0;

    protected JwB = 0;

    protected JwC = 0;

    protected JwD = 0;

    protected mass = 0;

    /** @internal protected */
    public constructor(def: IGearJointDef) {
        super(def);

        this.joint1 = def.joint1;
        this.joint2 = def.joint2;

        this.typeA = this.joint1.getType();
        this.typeB = this.joint2.getType();

        // DEBUG: assert(this.typeA === JointType.Revolute || this.typeA === JointType.Prismatic);
        // DEBUG: assert(this.typeB === JointType.Revolute || this.typeB === JointType.Prismatic);

        let coordinateA: number;
        let coordinateB: number;

        // TODO_ERIN there might be some problem with the joint edges in Joint.

        this.bodyC = this.joint1.getBodyA();
        this.bodyA = this.joint1.getBodyB();

        // Body B on joint1 must be dynamic
        // DEBUG: assert(this.bodyA.type === BodyType.Dynamic);

        // Get geometry of joint1
        const xfA = this.bodyA.xf;
        const aA = this.bodyA.sweep.a;
        const xfC = this.bodyC.xf;
        const aC = this.bodyC.sweep.a;

        if (this.typeA === JointType.Revolute) {
            const revolute = def.joint1 as RevoluteJoint;
            this.localAnchorC.copy(revolute.localAnchorA);
            this.localAnchorA.copy(revolute.localAnchorB);
            this.referenceAngleA = revolute.referenceAngle;
            this.localAxisC.setZero();

            coordinateA = aA - aC - this.referenceAngleA;
        } else {
            const prismatic = def.joint1 as PrismaticJoint;
            this.localAnchorC.copy(prismatic.localAnchorA);
            this.localAnchorA.copy(prismatic.localAnchorB);
            this.referenceAngleA = prismatic.referenceAngle;
            this.localAxisC.copy(prismatic.localXAxisA);

            const pC = this.localAnchorC;
            const pA = Rot.transposeMultiplyVec2(
                xfC.q,
                Rot.multiplyVec2(xfA.q, this.localAnchorA, Vec2.s_t0).add(xfA.p).subtract(xfC.p),
                Vec2.s_t0,
            );
            coordinateA = Vec2.dot(pA.subtract(pC), this.localAxisC);
        }

        this.bodyD = this.joint2.getBodyA();
        this.bodyB = this.joint2.getBodyB();

        // Body B on joint2 must be dynamic
        // DEBUG: assert(this.bodyB.type === BodyType.Dynamic);

        // Get geometry of joint2
        const xfB = this.bodyB.xf;
        const aB = this.bodyB.sweep.a;
        const xfD = this.bodyD.xf;
        const aD = this.bodyD.sweep.a;

        if (this.typeB === JointType.Revolute) {
            const revolute = def.joint2 as RevoluteJoint;
            this.localAnchorD.copy(revolute.localAnchorA);
            this.localAnchorB.copy(revolute.localAnchorB);
            this.referenceAngleB = revolute.referenceAngle;
            this.localAxisD.setZero();

            coordinateB = aB - aD - this.referenceAngleB;
        } else {
            const prismatic = def.joint2 as PrismaticJoint;
            this.localAnchorD.copy(prismatic.localAnchorA);
            this.localAnchorB.copy(prismatic.localAnchorB);
            this.referenceAngleB = prismatic.referenceAngle;
            this.localAxisD.copy(prismatic.localXAxisA);

            const pD = this.localAnchorD;
            const pB = Rot.transposeMultiplyVec2(
                xfD.q,
                Rot.multiplyVec2(xfB.q, this.localAnchorB, Vec2.s_t0).add(xfB.p).subtract(xfD.p),
                Vec2.s_t0,
            );
            coordinateB = Vec2.dot(pB.subtract(pD), this.localAxisD);
        }

        this.ratio = def.ratio ?? 1;

        this.constant = coordinateA + this.ratio * coordinateB;

        this.impulse = 0;
    }

    /** @internal protected */
    public initVelocityConstraints(data: SolverData): void {
        this.indexA = this.bodyA.islandIndex;
        this.indexB = this.bodyB.islandIndex;
        this.indexC = this.bodyC.islandIndex;
        this.indexD = this.bodyD.islandIndex;
        this.lcA.copy(this.bodyA.sweep.localCenter);
        this.lcB.copy(this.bodyB.sweep.localCenter);
        this.lcC.copy(this.bodyC.sweep.localCenter);
        this.lcD.copy(this.bodyD.sweep.localCenter);
        this.mA = this.bodyA.invMass;
        this.mB = this.bodyB.invMass;
        this.mC = this.bodyC.invMass;
        this.mD = this.bodyD.invMass;
        this.iA = this.bodyA.invI;
        this.iB = this.bodyB.invI;
        this.iC = this.bodyC.invI;
        this.iD = this.bodyD.invI;

        const aA = data.positions[this.indexA].a;
        const vA = data.velocities[this.indexA].v;
        let wA = data.velocities[this.indexA].w;

        const aB = data.positions[this.indexB].a;
        const vB = data.velocities[this.indexB].v;
        let wB = data.velocities[this.indexB].w;

        const aC = data.positions[this.indexC].a;
        const vC = data.velocities[this.indexC].v;
        let wC = data.velocities[this.indexC].w;

        const aD = data.positions[this.indexD].a;
        const vD = data.velocities[this.indexD].v;
        let wD = data.velocities[this.indexD].w;

        const { qA, qB, qC, qD } = temp;
        qA.set(aA);
        qB.set(aB);
        qC.set(aC);
        qD.set(aD);

        this.mass = 0;

        if (this.typeA === JointType.Revolute) {
            this.JvAC.setZero();
            this.JwA = 1;
            this.JwC = 1;
            this.mass += this.iA + this.iC;
        } else {
            const { u, rC, rA, lalcA, lalcC } = temp;
            Rot.multiplyVec2(qC, this.localAxisC, u);
            Rot.multiplyVec2(qC, Vec2.subtract(this.localAnchorC, this.lcC, lalcC), rC);
            Rot.multiplyVec2(qA, Vec2.subtract(this.localAnchorA, this.lcA, lalcA), rA);
            this.JvAC.copy(u);
            this.JwC = Vec2.cross(rC, u);
            this.JwA = Vec2.cross(rA, u);
            this.mass += this.mC + this.mA + this.iC * this.JwC * this.JwC + this.iA * this.JwA * this.JwA;
        }

        if (this.typeB === JointType.Revolute) {
            this.JvBD.setZero();
            this.JwB = this.ratio;
            this.JwD = this.ratio;
            this.mass += this.ratio * this.ratio * (this.iB + this.iD);
        } else {
            const { u, rB, rD, lalcB, lalcD } = temp;
            Rot.multiplyVec2(qD, this.localAxisD, u);
            Rot.multiplyVec2(qD, Vec2.subtract(this.localAnchorD, this.lcD, lalcD), rD);
            Rot.multiplyVec2(qB, Vec2.subtract(this.localAnchorB, this.lcB, lalcB), rB);
            Vec2.scale(this.ratio, u, this.JvBD);
            this.JwD = this.ratio * Vec2.cross(rD, u);
            this.JwB = this.ratio * Vec2.cross(rB, u);
            this.mass +=
                this.ratio * this.ratio * (this.mD + this.mB) +
                this.iD * this.JwD * this.JwD +
                this.iB * this.JwB * this.JwB;
        }

        // Compute effective mass.
        this.mass = this.mass > 0 ? 1 / this.mass : 0;

        if (data.step.warmStarting) {
            vA.addScaled(this.mA * this.impulse, this.JvAC);
            wA += this.iA * this.impulse * this.JwA;
            vB.addScaled(this.mB * this.impulse, this.JvBD);
            wB += this.iB * this.impulse * this.JwB;
            vC.subtractScaled(this.mC * this.impulse, this.JvAC);
            wC -= this.iC * this.impulse * this.JwC;
            vD.subtractScaled(this.mD * this.impulse, this.JvBD);
            wD -= this.iD * this.impulse * this.JwD;
        } else {
            this.impulse = 0;
        }

        data.velocities[this.indexA].w = wA;
        data.velocities[this.indexB].w = wB;
        data.velocities[this.indexC].w = wC;
        data.velocities[this.indexD].w = wD;
    }

    /** @internal protected */
    public solveVelocityConstraints(data: SolverData): void {
        const vA = data.velocities[this.indexA].v;
        let wA = data.velocities[this.indexA].w;
        const vB = data.velocities[this.indexB].v;
        let wB = data.velocities[this.indexB].w;
        const vC = data.velocities[this.indexC].v;
        let wC = data.velocities[this.indexC].w;
        const vD = data.velocities[this.indexD].v;
        let wD = data.velocities[this.indexD].w;

        let Cdot =
            Vec2.dot(this.JvAC, Vec2.subtract(vA, vC, Vec2.s_t0)) +
            Vec2.dot(this.JvBD, Vec2.subtract(vB, vD, Vec2.s_t0));
        Cdot += this.JwA * wA - this.JwC * wC + (this.JwB * wB - this.JwD * wD);

        const impulse = -this.mass * Cdot;
        this.impulse += impulse;

        vA.addScaled(this.mA * impulse, this.JvAC);
        wA += this.iA * impulse * this.JwA;
        vB.addScaled(this.mB * impulse, this.JvBD);
        wB += this.iB * impulse * this.JwB;
        vC.subtractScaled(this.mC * impulse, this.JvAC);
        wC -= this.iC * impulse * this.JwC;
        vD.subtractScaled(this.mD * impulse, this.JvBD);
        wD -= this.iD * impulse * this.JwD;

        data.velocities[this.indexA].w = wA;
        data.velocities[this.indexB].w = wB;
        data.velocities[this.indexC].w = wC;
        data.velocities[this.indexD].w = wD;
    }

    /** @internal protected */
    public solvePositionConstraints(data: SolverData): boolean {
        const cA = data.positions[this.indexA].c;
        let aA = data.positions[this.indexA].a;
        const cB = data.positions[this.indexB].c;
        let aB = data.positions[this.indexB].a;
        const cC = data.positions[this.indexC].c;
        let aC = data.positions[this.indexC].a;
        const cD = data.positions[this.indexD].c;
        let aD = data.positions[this.indexD].a;

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

        if (this.typeA === JointType.Revolute) {
            JvAC.setZero();
            JwA = 1;
            JwC = 1;
            mass += this.iA + this.iC;

            coordinateA = aA - aC - this.referenceAngleA;
        } else {
            const { u, rC, rA, lalcC, lalcA } = temp;
            Rot.multiplyVec2(qC, this.localAxisC, u);
            Rot.multiplyVec2(qC, Vec2.subtract(this.localAnchorC, this.lcC, lalcC), rC);
            Rot.multiplyVec2(qA, Vec2.subtract(this.localAnchorA, this.lcA, lalcA), rA);
            JvAC.copy(u);
            JwC = Vec2.cross(rC, u);
            JwA = Vec2.cross(rA, u);
            mass += this.mC + this.mA + this.iC * JwC * JwC + this.iA * JwA * JwA;

            const pC = lalcC;
            const pA = Rot.transposeMultiplyVec2(qC, Vec2.add(rA, cA, Vec2.s_t0).subtract(cC), Vec2.s_t0);
            coordinateA = Vec2.dot(Vec2.subtract(pA, pC, Vec2.s_t0), this.localAxisC);
        }

        if (this.typeB === JointType.Revolute) {
            JvBD.setZero();
            JwB = this.ratio;
            JwD = this.ratio;
            mass += this.ratio * this.ratio * (this.iB + this.iD);

            coordinateB = aB - aD - this.referenceAngleB;
        } else {
            const { u, rD, rB, lalcD, lalcB } = temp;
            Rot.multiplyVec2(qD, this.localAxisD, u);
            Rot.multiplyVec2(qD, Vec2.subtract(this.localAnchorD, this.lcD, lalcD), rD);
            Rot.multiplyVec2(qB, Vec2.subtract(this.localAnchorB, this.lcB, lalcB), rB);
            Vec2.scale(this.ratio, u, JvBD);
            JwD = this.ratio * Vec2.cross(rD, u);
            JwB = this.ratio * Vec2.cross(rB, u);
            mass += this.ratio * this.ratio * (this.mD + this.mB) + this.iD * JwD * JwD + this.iB * JwB * JwB;

            const pD = lalcD;
            const pB = Rot.transposeMultiplyVec2(qD, Vec2.add(rB, cB, Vec2.s_t0).subtract(cD), Vec2.s_t0);
            coordinateB = Vec2.dot(pB.subtract(pD), this.localAxisD);
        }

        const C = coordinateA + this.ratio * coordinateB - this.constant;

        let impulse = 0;
        if (mass > 0) {
            impulse = -C / mass;
        }

        cA.addScaled(this.mA * impulse, JvAC);
        aA += this.iA * impulse * JwA;
        cB.addScaled(this.mB * impulse, JvBD);
        aB += this.iB * impulse * JwB;
        cC.subtractScaled(this.mC * impulse, JvAC);
        aC -= this.iC * impulse * JwC;
        cD.subtractScaled(this.mD * impulse, JvBD);
        aD -= this.iD * impulse * JwD;

        data.positions[this.indexA].a = aA;
        data.positions[this.indexB].a = aB;
        data.positions[this.indexC].a = aC;
        data.positions[this.indexD].a = aD;

        // TODO_ERIN not implemented
        return linearError < LINEAR_SLOP;
    }

    public getAnchorA<T extends XY>(out: T): T {
        return this.bodyA.getWorldPoint(this.localAnchorA, out);
    }

    public getAnchorB<T extends XY>(out: T): T {
        return this.bodyB.getWorldPoint(this.localAnchorB, out);
    }

    public getReactionForce<T extends XY>(inv_dt: number, out: T): T {
        return Vec2.scale(inv_dt * this.impulse, this.JvAC, out);
    }

    public getReactionTorque(inv_dt: number): number {
        return inv_dt * this.impulse * this.JwA;
    }

    public getJoint1() {
        return this.joint1;
    }

    public getJoint2() {
        return this.joint2;
    }

    public getRatio() {
        return this.ratio;
    }

    public setRatio(ratio: number): void {
        // DEBUG: assert(Number.isFinite(ratio));
        this.ratio = ratio;
    }
}
