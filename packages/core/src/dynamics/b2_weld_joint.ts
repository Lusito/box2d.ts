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
import { b2Vec2, b2Vec3, b2Mat33, b2Rot, XY } from "../common/b2_math";
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
// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

const temp = {
    qA: new b2Rot(),
    qB: new b2Rot(),
    rA: new b2Vec2(),
    rB: new b2Vec2(),
    lalcA: new b2Vec2(),
    lalcB: new b2Vec2(),
    K: new b2Mat33(),
    P: new b2Vec2(),
    Cdot1: new b2Vec3(),
    impulse1: new b2Vec2(),
    impulse: new b2Vec3(),
    C1: new b2Vec2(),
    C: new b2Vec3(),
};

export interface b2IWeldJointDef extends b2IJointDef {
    localAnchorA?: XY;

    localAnchorB?: XY;

    referenceAngle?: number;

    stiffness?: number;

    damping?: number;
}

/**
 * Weld joint definition. You need to specify local anchor points
 * where they are attached and the relative body angle. The position
 * of the anchor points is important for computing the reaction torque.
 */
export class b2WeldJointDef extends b2JointDef implements b2IWeldJointDef {
    /** The local anchor point relative to bodyA's origin. */
    public readonly localAnchorA = new b2Vec2();

    /** The local anchor point relative to bodyB's origin. */
    public readonly localAnchorB = new b2Vec2();

    /** The bodyB angle minus bodyA angle in the reference state (radians). */
    public referenceAngle = 0;

    /**
     * The rotational stiffness in N*m
     * Disable softness with a value of 0
     */
    public stiffness = 0;

    /** The rotational damping in N*m*s */
    public damping = 0;

    public constructor() {
        super(b2JointType.e_weldJoint);
    }

    /**
     * Initialize the bodies, anchors, reference angle, stiffness, and damping.
     * @param bodyA the first body connected by this joint
     * @param bodyB the second body connected by this joint
     * @param anchor the point of connection in world coordinates
     */
    public Initialize(bA: b2Body, bB: b2Body, anchor: Readonly<XY>): void {
        this.bodyA = bA;
        this.bodyB = bB;
        this.bodyA.GetLocalPoint(anchor, this.localAnchorA);
        this.bodyB.GetLocalPoint(anchor, this.localAnchorB);
        this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
    }
}

/**
 * A weld joint essentially glues two bodies together. A weld joint may
 * distort somewhat because the island constraint solver is approximate.
 */
export class b2WeldJoint extends b2Joint {
    protected m_stiffness = 0;

    protected m_damping = 0;

    protected m_bias = 0;

    // Solver shared
    protected readonly m_localAnchorA = new b2Vec2();

    protected readonly m_localAnchorB = new b2Vec2();

    protected m_referenceAngle = 0;

    protected m_gamma = 0;

    protected readonly m_impulse = new b2Vec3();

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

    protected readonly m_mass = new b2Mat33();

    /** @internal protected */
    public constructor(def: b2IWeldJointDef) {
        super(def);

        this.m_localAnchorA.Copy(def.localAnchorA ?? b2Vec2.ZERO);
        this.m_localAnchorB.Copy(def.localAnchorB ?? b2Vec2.ZERO);
        this.m_referenceAngle = def.referenceAngle ?? 0;
        this.m_stiffness = def.stiffness ?? 0;
        this.m_damping = def.damping ?? 0;
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

        const aA = data.positions[this.m_indexA].a;
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;

        const aB = data.positions[this.m_indexB].a;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;

        const { qA, qB, lalcA, lalcB, K } = temp;
        qA.Set(aA);
        qB.Set(aB);

        b2Rot.MultiplyVec2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), this.m_rA);
        b2Rot.MultiplyVec2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), this.m_rB);

        // J = [-I -r1_skew I r2_skew]
        //     [ 0       -1 0       1]
        // r_skew = [-ry; rx]

        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
        //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
        //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

        const mA = this.m_invMassA;
        const mB = this.m_invMassB;
        const iA = this.m_invIA;
        const iB = this.m_invIB;

        K.ex.x = mA + mB + this.m_rA.y * this.m_rA.y * iA + this.m_rB.y * this.m_rB.y * iB;
        K.ey.x = -this.m_rA.y * this.m_rA.x * iA - this.m_rB.y * this.m_rB.x * iB;
        K.ez.x = -this.m_rA.y * iA - this.m_rB.y * iB;
        K.ex.y = K.ey.x;
        K.ey.y = mA + mB + this.m_rA.x * this.m_rA.x * iA + this.m_rB.x * this.m_rB.x * iB;
        K.ez.y = this.m_rA.x * iA + this.m_rB.x * iB;
        K.ex.z = K.ez.x;
        K.ey.z = K.ez.y;
        K.ez.z = iA + iB;

        if (this.m_stiffness > 0) {
            K.GetInverse22(this.m_mass);

            let invM = iA + iB;

            const C = aB - aA - this.m_referenceAngle;

            // Damping coefficient
            const d = this.m_damping;

            // Spring stiffness
            const k = this.m_stiffness;

            // magic formulas
            const h = data.step.dt;
            this.m_gamma = h * (d + h * k);
            this.m_gamma = this.m_gamma !== 0 ? 1 / this.m_gamma : 0;
            this.m_bias = C * h * k * this.m_gamma;

            invM += this.m_gamma;
            this.m_mass.ez.z = invM !== 0 ? 1 / invM : 0;
        } else if (K.ez.z === 0) {
            K.GetInverse22(this.m_mass);
            this.m_gamma = 0;
            this.m_bias = 0;
        } else {
            K.GetSymInverse33(this.m_mass);
            this.m_gamma = 0;
            this.m_bias = 0;
        }

        if (data.step.warmStarting) {
            // Scale impulses to support a variable time step.
            this.m_impulse.Scale(data.step.dtRatio);

            const { P } = temp;
            P.Copy(this.m_impulse);

            vA.SubtractScaled(mA, P);
            wA -= iA * (b2Vec2.Cross(this.m_rA, P) + this.m_impulse.z);

            vB.AddScaled(mB, P);
            wB += iB * (b2Vec2.Cross(this.m_rB, P) + this.m_impulse.z);
        } else {
            this.m_impulse.SetZero();
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

        if (this.m_stiffness > 0) {
            const Cdot2 = wB - wA;

            const impulse2 = -this.m_mass.ez.z * (Cdot2 + this.m_bias + this.m_gamma * this.m_impulse.z);
            this.m_impulse.z += impulse2;

            wA -= iA * impulse2;
            wB += iB * impulse2;

            const { Cdot1, impulse1 } = temp;
            b2Vec2.Subtract(
                b2Vec2.AddCrossScalarVec2(vB, wB, this.m_rB, b2Vec2.s_t0),
                b2Vec2.AddCrossScalarVec2(vA, wA, this.m_rA, b2Vec2.s_t1),
                Cdot1,
            );

            b2Mat33.MultiplyVec2(this.m_mass, Cdot1, impulse1).Negate();
            this.m_impulse.x += impulse1.x;
            this.m_impulse.y += impulse1.y;

            const P = impulse1;

            vA.SubtractScaled(mA, P);
            wA -= iA * b2Vec2.Cross(this.m_rA, P);

            vB.AddScaled(mB, P);
            wB += iB * b2Vec2.Cross(this.m_rB, P);
        } else {
            const { Cdot1, impulse, P } = temp;
            b2Vec2.Subtract(
                b2Vec2.AddCrossScalarVec2(vB, wB, this.m_rB, b2Vec2.s_t0),
                b2Vec2.AddCrossScalarVec2(vA, wA, this.m_rA, b2Vec2.s_t1),
                Cdot1,
            );
            Cdot1.z = wB - wA;

            b2Mat33.MultiplyVec3(this.m_mass, Cdot1, impulse).Negate();
            this.m_impulse.Add(impulse);

            P.Set(impulse.x, impulse.y);

            vA.SubtractScaled(mA, P);
            wA -= iA * (b2Vec2.Cross(this.m_rA, P) + impulse.z);

            vB.AddScaled(mB, P);
            wB += iB * (b2Vec2.Cross(this.m_rB, P) + impulse.z);
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

        const { qA, qB, lalcA, lalcB, K, C1, P, rA, rB } = temp;
        qA.Set(aA);
        qB.Set(aB);

        const mA = this.m_invMassA;
        const mB = this.m_invMassB;
        const iA = this.m_invIA;
        const iB = this.m_invIB;

        b2Rot.MultiplyVec2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), rA);
        b2Rot.MultiplyVec2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), rB);

        let positionError: number;
        let angularError: number;

        K.ex.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
        K.ey.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
        K.ez.x = -rA.y * iA - rB.y * iB;
        K.ex.y = K.ey.x;
        K.ey.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
        K.ez.y = rA.x * iA + rB.x * iB;
        K.ex.z = K.ez.x;
        K.ey.z = K.ez.y;
        K.ez.z = iA + iB;

        if (this.m_stiffness > 0) {
            b2Vec2.Add(cB, rB, C1).Subtract(cA).Subtract(rA);
            positionError = C1.Length();
            angularError = 0;

            K.Solve22(C1.x, C1.y, P).Negate();

            cA.SubtractScaled(mA, P);
            aA -= iA * b2Vec2.Cross(rA, P);

            cB.AddScaled(mB, P);
            aB += iB * b2Vec2.Cross(rB, P);
        } else {
            b2Vec2.Add(cB, rB, C1).Subtract(cA).Subtract(rA);
            b2Vec2.Subtract(b2Vec2.Add(cB, rB, b2Vec2.s_t0), b2Vec2.Add(cA, rA, b2Vec2.s_t1), C1);
            const C2 = aB - aA - this.m_referenceAngle;

            positionError = C1.Length();
            angularError = Math.abs(C2);

            const { impulse, C } = temp;
            C.Set(C1.x, C1.y, C2);

            if (K.ez.z > 0) {
                K.Solve33(C.x, C.y, C.z, impulse).Negate();
            } else {
                K.Solve22(C1.x, C1.y, impulse).Negate();
                impulse.z = 0;
            }

            P.Copy(impulse);

            cA.SubtractScaled(mA, P);
            aA -= iA * (b2Vec2.Cross(rA, P) + impulse.z);

            cB.AddScaled(mB, P);
            aB += iB * (b2Vec2.Cross(rB, P) + impulse.z);
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

    public GetReactionForce<T extends XY>(inv_dt: number, out: T): T {
        out.x = inv_dt * this.m_impulse.x;
        out.y = inv_dt * this.m_impulse.y;
        return out;
    }

    public GetReactionTorque(inv_dt: number): number {
        return inv_dt * this.m_impulse.z;
    }

    /** The local anchor point relative to bodyA's origin. */
    public GetLocalAnchorA(): Readonly<b2Vec2> {
        return this.m_localAnchorA;
    }

    /** The local anchor point relative to bodyB's origin. */
    public GetLocalAnchorB(): Readonly<b2Vec2> {
        return this.m_localAnchorB;
    }

    /** Get the reference angle. */
    public GetReferenceAngle(): number {
        return this.m_referenceAngle;
    }

    /** Set stiffness in N*m */
    public SetStiffness(stiffness: number): void {
        this.m_stiffness = stiffness;
    }

    /** Get stiffness in N*m */
    public GetStiffness(): number {
        return this.m_stiffness;
    }

    /** Set damping in N*m*s */
    public SetDamping(damping: number) {
        this.m_damping = damping;
    }

    /** Get damping in N*m*s */
    public GetDamping() {
        return this.m_damping;
    }
}
