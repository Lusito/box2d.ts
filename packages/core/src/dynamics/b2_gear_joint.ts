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
import { b2_angularSlop, b2_linearSlop } from "../common/b2_common";
import { b2Vec2, b2Rot, XY } from "../common/b2_math";
import { b2Joint, b2JointDef, b2JointType, b2IJointDef } from "./b2_joint";
import { b2PrismaticJoint } from "./b2_prismatic_joint";
import { b2RevoluteJoint } from "./b2_revolute_joint";
import { b2SolverData } from "./b2_time_step";
import { b2Body } from "./b2_body";

const temp = {
    qA: new b2Rot(),
    qB: new b2Rot(),
    qC: new b2Rot(),
    qD: new b2Rot(),
    lalcA: new b2Vec2(),
    lalcB: new b2Vec2(),
    lalcC: new b2Vec2(),
    lalcD: new b2Vec2(),
    u: new b2Vec2(),
    rA: new b2Vec2(),
    rB: new b2Vec2(),
    rC: new b2Vec2(),
    rD: new b2Vec2(),
    JvAC: new b2Vec2(),
    JvBD: new b2Vec2(),
};

export interface b2IGearJointDef extends b2IJointDef {
    joint1: b2RevoluteJoint | b2PrismaticJoint;

    joint2: b2RevoluteJoint | b2PrismaticJoint;

    ratio?: number;
}

/**
 * Gear joint definition. This definition requires two existing
 * revolute or prismatic joints (any combination will work).
 *
 * @warning bodyB on the input joints must both be dynamic
 */
export class b2GearJointDef extends b2JointDef implements b2IGearJointDef {
    /** The first revolute/prismatic joint attached to the gear joint. */
    public joint1!: b2RevoluteJoint | b2PrismaticJoint;

    /** The second revolute/prismatic joint attached to the gear joint. */
    public joint2!: b2RevoluteJoint | b2PrismaticJoint;

    /**
     * The gear ratio.
     *
     * @see b2GearJoint for explanation.
     */
    public ratio = 1;

    public constructor() {
        super(b2JointType.e_gearJoint);
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
export class b2GearJoint extends b2Joint {
    protected m_joint1: b2RevoluteJoint | b2PrismaticJoint;

    protected m_joint2: b2RevoluteJoint | b2PrismaticJoint;

    protected m_typeA = b2JointType.e_unknownJoint;

    protected m_typeB = b2JointType.e_unknownJoint;

    /** Body A is connected to body C */
    protected m_bodyC: b2Body;

    /** Body B is connected to body D */
    protected m_bodyD: b2Body;

    // Solver shared
    protected readonly m_localAnchorA = new b2Vec2();

    protected readonly m_localAnchorB = new b2Vec2();

    protected readonly m_localAnchorC = new b2Vec2();

    protected readonly m_localAnchorD = new b2Vec2();

    protected readonly m_localAxisC = new b2Vec2();

    protected readonly m_localAxisD = new b2Vec2();

    protected m_referenceAngleA = 0;

    protected m_referenceAngleB = 0;

    protected m_constant = 0;

    protected m_ratio = 0;

    protected m_tolerance = 0;

    protected m_impulse = 0;

    // Solver temp
    protected m_indexA = 0;

    protected m_indexB = 0;

    protected m_indexC = 0;

    protected m_indexD = 0;

    protected readonly m_lcA = new b2Vec2();

    protected readonly m_lcB = new b2Vec2();

    protected readonly m_lcC = new b2Vec2();

    protected readonly m_lcD = new b2Vec2();

    protected m_mA = 0;

    protected m_mB = 0;

    protected m_mC = 0;

    protected m_mD = 0;

    protected m_iA = 0;

    protected m_iB = 0;

    protected m_iC = 0;

    protected m_iD = 0;

    protected readonly m_JvAC = new b2Vec2();

    protected readonly m_JvBD = new b2Vec2();

    protected m_JwA = 0;

    protected m_JwB = 0;

    protected m_JwC = 0;

    protected m_JwD = 0;

    protected m_mass = 0;

    /** @internal protected */
    public constructor(def: b2IGearJointDef) {
        super(def);

        this.m_joint1 = def.joint1;
        this.m_joint2 = def.joint2;

        this.m_typeA = this.m_joint1.GetType();
        this.m_typeB = this.m_joint2.GetType();

        // DEBUG: b2Assert(this.m_typeA === b2JointType.e_revoluteJoint || this.m_typeA === b2JointType.e_prismaticJoint);
        // DEBUG: b2Assert(this.m_typeB === b2JointType.e_revoluteJoint || this.m_typeB === b2JointType.e_prismaticJoint);

        let coordinateA: number;
        let coordinateB: number;

        // TODO_ERIN there might be some problem with the joint edges in b2Joint.

        this.m_bodyC = this.m_joint1.GetBodyA();
        this.m_bodyA = this.m_joint1.GetBodyB();

        // Body B on joint1 must be dynamic
        // DEBUG: b2Assert(this.m_bodyA.m_type === b2BodyType.b2_dynamicBody);

        // Get geometry of joint1
        const xfA = this.m_bodyA.m_xf;
        const aA = this.m_bodyA.m_sweep.a;
        const xfC = this.m_bodyC.m_xf;
        const aC = this.m_bodyC.m_sweep.a;

        if (this.m_typeA === b2JointType.e_revoluteJoint) {
            const revolute = def.joint1 as b2RevoluteJoint;
            this.m_localAnchorC.Copy(revolute.m_localAnchorA);
            this.m_localAnchorA.Copy(revolute.m_localAnchorB);
            this.m_referenceAngleA = revolute.m_referenceAngle;
            this.m_localAxisC.SetZero();

            coordinateA = aA - aC - this.m_referenceAngleA;

            // position error is measured in radians
            this.m_tolerance = b2_angularSlop;
        } else {
            const prismatic = def.joint1 as b2PrismaticJoint;
            this.m_localAnchorC.Copy(prismatic.m_localAnchorA);
            this.m_localAnchorA.Copy(prismatic.m_localAnchorB);
            this.m_referenceAngleA = prismatic.m_referenceAngle;
            this.m_localAxisC.Copy(prismatic.m_localXAxisA);

            const pC = this.m_localAnchorC;
            const pA = b2Rot.TransposeMultiplyVec2(
                xfC.q,
                b2Rot.MultiplyVec2(xfA.q, this.m_localAnchorA, b2Vec2.s_t0).Add(xfA.p).Subtract(xfC.p),
                b2Vec2.s_t0,
            );
            coordinateA = b2Vec2.Dot(pA.Subtract(pC), this.m_localAxisC);

            // position error is measured in meters
            this.m_tolerance = b2_linearSlop;
        }

        this.m_bodyD = this.m_joint2.GetBodyA();
        this.m_bodyB = this.m_joint2.GetBodyB();

        // Body B on joint2 must be dynamic
        // DEBUG: b2Assert(this.m_bodyB.m_type === b2BodyType.b2_dynamicBody);

        // Get geometry of joint2
        const xfB = this.m_bodyB.m_xf;
        const aB = this.m_bodyB.m_sweep.a;
        const xfD = this.m_bodyD.m_xf;
        const aD = this.m_bodyD.m_sweep.a;

        if (this.m_typeB === b2JointType.e_revoluteJoint) {
            const revolute = def.joint2 as b2RevoluteJoint;
            this.m_localAnchorD.Copy(revolute.m_localAnchorA);
            this.m_localAnchorB.Copy(revolute.m_localAnchorB);
            this.m_referenceAngleB = revolute.m_referenceAngle;
            this.m_localAxisD.SetZero();

            coordinateB = aB - aD - this.m_referenceAngleB;
        } else {
            const prismatic = def.joint2 as b2PrismaticJoint;
            this.m_localAnchorD.Copy(prismatic.m_localAnchorA);
            this.m_localAnchorB.Copy(prismatic.m_localAnchorB);
            this.m_referenceAngleB = prismatic.m_referenceAngle;
            this.m_localAxisD.Copy(prismatic.m_localXAxisA);

            const pD = this.m_localAnchorD;
            const pB = b2Rot.TransposeMultiplyVec2(
                xfD.q,
                b2Rot.MultiplyVec2(xfB.q, this.m_localAnchorB, b2Vec2.s_t0).Add(xfB.p).Subtract(xfD.p),
                b2Vec2.s_t0,
            );
            coordinateB = b2Vec2.Dot(pB.Subtract(pD), this.m_localAxisD);
        }

        this.m_ratio = def.ratio ?? 1;

        this.m_constant = coordinateA + this.m_ratio * coordinateB;

        this.m_impulse = 0;
    }

    /** @internal protected */
    public InitVelocityConstraints(data: b2SolverData): void {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_indexC = this.m_bodyC.m_islandIndex;
        this.m_indexD = this.m_bodyD.m_islandIndex;
        this.m_lcA.Copy(this.m_bodyA.m_sweep.localCenter);
        this.m_lcB.Copy(this.m_bodyB.m_sweep.localCenter);
        this.m_lcC.Copy(this.m_bodyC.m_sweep.localCenter);
        this.m_lcD.Copy(this.m_bodyD.m_sweep.localCenter);
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
        qA.Set(aA);
        qB.Set(aB);
        qC.Set(aC);
        qD.Set(aD);

        this.m_mass = 0;

        if (this.m_typeA === b2JointType.e_revoluteJoint) {
            this.m_JvAC.SetZero();
            this.m_JwA = 1;
            this.m_JwC = 1;
            this.m_mass += this.m_iA + this.m_iC;
        } else {
            const { u, rC, rA, lalcA, lalcC } = temp;
            b2Rot.MultiplyVec2(qC, this.m_localAxisC, u);
            b2Rot.MultiplyVec2(qC, b2Vec2.Subtract(this.m_localAnchorC, this.m_lcC, lalcC), rC);
            b2Rot.MultiplyVec2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_lcA, lalcA), rA);
            this.m_JvAC.Copy(u);
            this.m_JwC = b2Vec2.Cross(rC, u);
            this.m_JwA = b2Vec2.Cross(rA, u);
            this.m_mass +=
                this.m_mC + this.m_mA + this.m_iC * this.m_JwC * this.m_JwC + this.m_iA * this.m_JwA * this.m_JwA;
        }

        if (this.m_typeB === b2JointType.e_revoluteJoint) {
            this.m_JvBD.SetZero();
            this.m_JwB = this.m_ratio;
            this.m_JwD = this.m_ratio;
            this.m_mass += this.m_ratio * this.m_ratio * (this.m_iB + this.m_iD);
        } else {
            const { u, rB, rD, lalcB, lalcD } = temp;
            b2Rot.MultiplyVec2(qD, this.m_localAxisD, u);
            b2Rot.MultiplyVec2(qD, b2Vec2.Subtract(this.m_localAnchorD, this.m_lcD, lalcD), rD);
            b2Rot.MultiplyVec2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_lcB, lalcB), rB);
            b2Vec2.Scale(this.m_ratio, u, this.m_JvBD);
            this.m_JwD = this.m_ratio * b2Vec2.Cross(rD, u);
            this.m_JwB = this.m_ratio * b2Vec2.Cross(rB, u);
            this.m_mass +=
                this.m_ratio * this.m_ratio * (this.m_mD + this.m_mB) +
                this.m_iD * this.m_JwD * this.m_JwD +
                this.m_iB * this.m_JwB * this.m_JwB;
        }

        // Compute effective mass.
        this.m_mass = this.m_mass > 0 ? 1 / this.m_mass : 0;

        if (data.step.warmStarting) {
            vA.AddScaled(this.m_mA * this.m_impulse, this.m_JvAC);
            wA += this.m_iA * this.m_impulse * this.m_JwA;
            vB.AddScaled(this.m_mB * this.m_impulse, this.m_JvBD);
            wB += this.m_iB * this.m_impulse * this.m_JwB;
            vC.SubtractScaled(this.m_mC * this.m_impulse, this.m_JvAC);
            wC -= this.m_iC * this.m_impulse * this.m_JwC;
            vD.SubtractScaled(this.m_mD * this.m_impulse, this.m_JvBD);
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
    public SolveVelocityConstraints(data: b2SolverData): void {
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;
        const vC = data.velocities[this.m_indexC].v;
        let wC = data.velocities[this.m_indexC].w;
        const vD = data.velocities[this.m_indexD].v;
        let wD = data.velocities[this.m_indexD].w;

        let Cdot =
            b2Vec2.Dot(this.m_JvAC, b2Vec2.Subtract(vA, vC, b2Vec2.s_t0)) +
            b2Vec2.Dot(this.m_JvBD, b2Vec2.Subtract(vB, vD, b2Vec2.s_t0));
        Cdot += this.m_JwA * wA - this.m_JwC * wC + (this.m_JwB * wB - this.m_JwD * wD);

        const impulse = -this.m_mass * Cdot;
        this.m_impulse += impulse;

        vA.AddScaled(this.m_mA * impulse, this.m_JvAC);
        wA += this.m_iA * impulse * this.m_JwA;
        vB.AddScaled(this.m_mB * impulse, this.m_JvBD);
        wB += this.m_iB * impulse * this.m_JwB;
        vC.SubtractScaled(this.m_mC * impulse, this.m_JvAC);
        wC -= this.m_iC * impulse * this.m_JwC;
        vD.SubtractScaled(this.m_mD * impulse, this.m_JvBD);
        wD -= this.m_iD * impulse * this.m_JwD;

        data.velocities[this.m_indexA].w = wA;
        data.velocities[this.m_indexB].w = wB;
        data.velocities[this.m_indexC].w = wC;
        data.velocities[this.m_indexD].w = wD;
    }

    /** @internal protected */
    public SolvePositionConstraints(data: b2SolverData): boolean {
        const cA = data.positions[this.m_indexA].c;
        let aA = data.positions[this.m_indexA].a;
        const cB = data.positions[this.m_indexB].c;
        let aB = data.positions[this.m_indexB].a;
        const cC = data.positions[this.m_indexC].c;
        let aC = data.positions[this.m_indexC].a;
        const cD = data.positions[this.m_indexD].c;
        let aD = data.positions[this.m_indexD].a;

        const { qA, qB, qC, qD, JvAC, JvBD } = temp;
        qA.Set(aA);
        qB.Set(aB);
        qC.Set(aC);
        qD.Set(aD);

        let coordinateA: number;
        let coordinateB: number;

        let JwA: number;
        let JwB: number;
        let JwC: number;
        let JwD: number;
        let mass = 0;

        if (this.m_typeA === b2JointType.e_revoluteJoint) {
            JvAC.SetZero();
            JwA = 1;
            JwC = 1;
            mass += this.m_iA + this.m_iC;

            coordinateA = aA - aC - this.m_referenceAngleA;
        } else {
            const { u, rC, rA, lalcC, lalcA } = temp;
            b2Rot.MultiplyVec2(qC, this.m_localAxisC, u);
            b2Rot.MultiplyVec2(qC, b2Vec2.Subtract(this.m_localAnchorC, this.m_lcC, lalcC), rC);
            b2Rot.MultiplyVec2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_lcA, lalcA), rA);
            JvAC.Copy(u);
            JwC = b2Vec2.Cross(rC, u);
            JwA = b2Vec2.Cross(rA, u);
            mass += this.m_mC + this.m_mA + this.m_iC * JwC * JwC + this.m_iA * JwA * JwA;

            const pC = lalcC;
            const pA = b2Rot.TransposeMultiplyVec2(qC, b2Vec2.Add(rA, cA, b2Vec2.s_t0).Subtract(cC), b2Vec2.s_t0);
            coordinateA = b2Vec2.Dot(b2Vec2.Subtract(pA, pC, b2Vec2.s_t0), this.m_localAxisC);
        }

        if (this.m_typeB === b2JointType.e_revoluteJoint) {
            JvBD.SetZero();
            JwB = this.m_ratio;
            JwD = this.m_ratio;
            mass += this.m_ratio * this.m_ratio * (this.m_iB + this.m_iD);

            coordinateB = aB - aD - this.m_referenceAngleB;
        } else {
            const { u, rD, rB, lalcD, lalcB } = temp;
            b2Rot.MultiplyVec2(qD, this.m_localAxisD, u);
            b2Rot.MultiplyVec2(qD, b2Vec2.Subtract(this.m_localAnchorD, this.m_lcD, lalcD), rD);
            b2Rot.MultiplyVec2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_lcB, lalcB), rB);
            b2Vec2.Scale(this.m_ratio, u, JvBD);
            JwD = this.m_ratio * b2Vec2.Cross(rD, u);
            JwB = this.m_ratio * b2Vec2.Cross(rB, u);
            mass +=
                this.m_ratio * this.m_ratio * (this.m_mD + this.m_mB) + this.m_iD * JwD * JwD + this.m_iB * JwB * JwB;

            const pD = lalcD;
            const pB = b2Rot.TransposeMultiplyVec2(qD, b2Vec2.Add(rB, cB, b2Vec2.s_t0).Subtract(cD), b2Vec2.s_t0);
            coordinateB = b2Vec2.Dot(pB.Subtract(pD), this.m_localAxisD);
        }

        const C = coordinateA + this.m_ratio * coordinateB - this.m_constant;

        let impulse = 0;
        if (mass > 0) {
            impulse = -C / mass;
        }

        cA.AddScaled(this.m_mA * impulse, JvAC);
        aA += this.m_iA * impulse * JwA;
        cB.AddScaled(this.m_mB * impulse, JvBD);
        aB += this.m_iB * impulse * JwB;
        cC.SubtractScaled(this.m_mC * impulse, JvAC);
        aC -= this.m_iC * impulse * JwC;
        cD.SubtractScaled(this.m_mD * impulse, JvBD);
        aD -= this.m_iD * impulse * JwD;

        data.positions[this.m_indexA].a = aA;
        data.positions[this.m_indexB].a = aB;
        data.positions[this.m_indexC].a = aC;
        data.positions[this.m_indexD].a = aD;

        if (Math.abs(C) < this.m_tolerance) {
            return true;
        }

        return false;
    }

    public GetAnchorA<T extends XY>(out: T): T {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    }

    public GetAnchorB<T extends XY>(out: T): T {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }

    public GetReactionForce<T extends XY>(inv_dt: number, out: T): T {
        return b2Vec2.Scale(inv_dt * this.m_impulse, this.m_JvAC, out);
    }

    public GetReactionTorque(inv_dt: number): number {
        return inv_dt * this.m_impulse * this.m_JwA;
    }

    public GetJoint1() {
        return this.m_joint1;
    }

    public GetJoint2() {
        return this.m_joint2;
    }

    public GetRatio() {
        return this.m_ratio;
    }

    public SetRatio(ratio: number): void {
        // DEBUG: b2Assert(Number.isFinite(ratio));
        this.m_ratio = ratio;
    }
}
