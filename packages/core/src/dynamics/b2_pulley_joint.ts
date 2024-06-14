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

// DEBUG: import { b2Assert, b2_epsilon } from "../common/b2_common";
import { b2_linearSlop } from "../common/b2_common";
import { b2Draw, debugColors } from "../common/b2_draw";
import { b2Vec2, b2Rot, XY } from "../common/b2_math";
import { b2Readonly } from "../common/b2_readonly";
import { b2Body } from "./b2_body";
import { b2Joint, b2JointDef, b2JointType, b2IJointDef } from "./b2_joint";
import { b2SolverData } from "./b2_time_step";

// Pulley:
// length1 = norm(p1 - s1)
// length2 = norm(p2 - s2)
// C0 = (length1 + ratio * length2)_initial
// C = C0 - (length1 + ratio * length2)
// u1 = (p1 - s1) / norm(p1 - s1)
// u2 = (p2 - s2) / norm(p2 - s2)
// Cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
// J = -[u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)

export const b2_minPulleyLength = 2;

const temp = {
    qA: new b2Rot(),
    qB: new b2Rot(),
    lalcA: new b2Vec2(),
    lalcB: new b2Vec2(),
    p: new b2Vec2(),
    PA: new b2Vec2(),
    PB: new b2Vec2(),
    vpA: new b2Vec2(),
    vpB: new b2Vec2(),
    pA: new b2Vec2(),
    pB: new b2Vec2(),
};

export interface b2IPulleyJointDef extends b2IJointDef {
    groundAnchorA?: XY;

    groundAnchorB?: XY;

    localAnchorA?: XY;

    localAnchorB?: XY;

    lengthA?: number;

    lengthB?: number;

    ratio?: number;
}

/**
 * Pulley joint definition. This requires two ground anchors,
 * two dynamic body anchor points, and a pulley ratio.
 */
export class b2PulleyJointDef extends b2JointDef implements b2IPulleyJointDef {
    /** The first ground anchor in world coordinates. This point never moves. */
    public readonly groundAnchorA = new b2Vec2(-1, 1);

    /** The second ground anchor in world coordinates. This point never moves. */
    public readonly groundAnchorB = new b2Vec2(1, 1);

    /** The local anchor point relative to bodyA's origin. */
    public readonly localAnchorA = new b2Vec2(-1, 0);

    /** The local anchor point relative to bodyB's origin. */
    public readonly localAnchorB = new b2Vec2(1, 0);

    /** The a reference length for the segment attached to bodyA. */
    public lengthA = 0;

    /** The a reference length for the segment attached to bodyB. */
    public lengthB = 0;

    /** The pulley ratio, used to simulate a block-and-tackle. */
    public ratio = 1;

    public constructor() {
        super(b2JointType.e_pulleyJoint);
        this.collideConnected = true;
    }

    /** Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors. */
    public Initialize(
        bA: b2Body,
        bB: b2Body,
        groundA: Readonly<XY>,
        groundB: Readonly<XY>,
        anchorA: Readonly<XY>,
        anchorB: Readonly<XY>,
        r: number,
    ): void {
        this.bodyA = bA;
        this.bodyB = bB;
        this.groundAnchorA.Copy(groundA);
        this.groundAnchorB.Copy(groundB);
        this.bodyA.GetLocalPoint(anchorA, this.localAnchorA);
        this.bodyB.GetLocalPoint(anchorB, this.localAnchorB);
        this.lengthA = b2Vec2.Distance(anchorA, groundA);
        this.lengthB = b2Vec2.Distance(anchorB, groundB);
        this.ratio = r;
        // DEBUG: b2Assert(this.ratio > b2_epsilon);
    }
}

const defaultGroundAnchorA = new b2Vec2(-1, 1);
const defaultGroundAnchorB = b2Vec2.UNITX;
const defaultLocalAnchorA = new b2Vec2(-1, 0);
const defaultLocalAnchorB = b2Vec2.UNITX;

/**
 * The pulley joint is connected to two bodies and two fixed ground points.
 * The pulley supports a ratio such that:
 * length1 + ratio * length2 <= constant
 * Yes, the force transmitted is scaled by the ratio.
 * Warning: the pulley joint can get a bit squirrelly by itself. They often
 * work better when combined with prismatic joints. You should also cover the
 * the anchor points with static shapes to prevent one side from going to
 * zero length.
 */
export class b2PulleyJoint extends b2Joint {
    protected readonly m_groundAnchorA = new b2Vec2();

    protected readonly m_groundAnchorB = new b2Vec2();

    protected m_lengthA = 0;

    protected m_lengthB = 0;

    // Solver shared
    protected readonly m_localAnchorA = new b2Vec2();

    protected readonly m_localAnchorB = new b2Vec2();

    protected m_constant = 0;

    protected m_ratio = 0;

    protected m_impulse = 0;

    // Solver temp
    protected m_indexA = 0;

    protected m_indexB = 0;

    protected readonly m_uA = new b2Vec2();

    protected readonly m_uB = new b2Vec2();

    protected readonly m_rA = new b2Vec2();

    protected readonly m_rB = new b2Vec2();

    protected readonly m_localCenterA = new b2Vec2();

    protected readonly m_localCenterB = new b2Vec2();

    protected m_invMassA = 0;

    protected m_invMassB = 0;

    protected m_invIA = 0;

    protected m_invIB = 0;

    protected m_mass = 0;

    /** @internal protected */
    public constructor(def: b2IPulleyJointDef) {
        super(def);

        this.m_groundAnchorA.Copy(def.groundAnchorA ?? defaultGroundAnchorA);
        this.m_groundAnchorB.Copy(def.groundAnchorB ?? defaultGroundAnchorB);
        this.m_localAnchorA.Copy(def.localAnchorA ?? defaultLocalAnchorA);
        this.m_localAnchorB.Copy(def.localAnchorB ?? defaultLocalAnchorB);

        this.m_lengthA = def.lengthA ?? 0;
        this.m_lengthB = def.lengthB ?? 0;

        // DEBUG: b2Assert((def.ratio ?? 1) !== 0);
        this.m_ratio = def.ratio ?? 1;

        this.m_constant = this.m_lengthA + this.m_ratio * this.m_lengthB;

        this.m_impulse = 0;
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

        const { qA, qB, lalcA, lalcB } = temp;
        qA.Set(aA);
        qB.Set(aB);

        b2Rot.MultiplyVec2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), this.m_rA);
        b2Rot.MultiplyVec2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), this.m_rB);

        // Get the pulley axes.
        b2Vec2.Add(cA, this.m_rA, this.m_uA).Subtract(this.m_groundAnchorA);
        b2Vec2.Add(cB, this.m_rB, this.m_uB).Subtract(this.m_groundAnchorB);

        const lengthA = this.m_uA.Length();
        const lengthB = this.m_uB.Length();

        if (lengthA > 10 * b2_linearSlop) {
            this.m_uA.Scale(1 / lengthA);
        } else {
            this.m_uA.SetZero();
        }

        if (lengthB > 10 * b2_linearSlop) {
            this.m_uB.Scale(1 / lengthB);
        } else {
            this.m_uB.SetZero();
        }

        // Compute effective mass.
        const ruA = b2Vec2.Cross(this.m_rA, this.m_uA);
        const ruB = b2Vec2.Cross(this.m_rB, this.m_uB);

        const mA = this.m_invMassA + this.m_invIA * ruA * ruA;
        const mB = this.m_invMassB + this.m_invIB * ruB * ruB;

        this.m_mass = mA + this.m_ratio * this.m_ratio * mB;

        if (this.m_mass > 0) {
            this.m_mass = 1 / this.m_mass;
        }

        if (data.step.warmStarting) {
            // Scale impulses to support variable time steps.
            this.m_impulse *= data.step.dtRatio;

            // Warm starting.
            const { PA, PB } = temp;
            b2Vec2.Scale(-this.m_impulse, this.m_uA, PA);
            b2Vec2.Scale(-this.m_ratio * this.m_impulse, this.m_uB, PB);

            vA.AddScaled(this.m_invMassA, PA);
            wA += this.m_invIA * b2Vec2.Cross(this.m_rA, PA);
            vB.AddScaled(this.m_invMassB, PB);
            wB += this.m_invIB * b2Vec2.Cross(this.m_rB, PB);
        } else {
            this.m_impulse = 0;
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

        const { PA, PB, vpA, vpB } = temp;
        b2Vec2.AddCrossScalarVec2(vA, wA, this.m_rA, vpA);
        b2Vec2.AddCrossScalarVec2(vB, wB, this.m_rB, vpB);

        const Cdot = -b2Vec2.Dot(this.m_uA, vpA) - this.m_ratio * b2Vec2.Dot(this.m_uB, vpB);
        const impulse = -this.m_mass * Cdot;
        this.m_impulse += impulse;

        b2Vec2.Scale(-impulse, this.m_uA, PA);
        b2Vec2.Scale(-this.m_ratio * impulse, this.m_uB, PB);
        vA.AddScaled(this.m_invMassA, PA);
        wA += this.m_invIA * b2Vec2.Cross(this.m_rA, PA);
        vB.AddScaled(this.m_invMassB, PB);
        wB += this.m_invIB * b2Vec2.Cross(this.m_rB, PB);

        data.velocities[this.m_indexA].w = wA;
        data.velocities[this.m_indexB].w = wB;
    }

    /** @internal protected */
    public SolvePositionConstraints(data: b2SolverData): boolean {
        const cA = data.positions[this.m_indexA].c;
        let aA = data.positions[this.m_indexA].a;
        const cB = data.positions[this.m_indexB].c;
        let aB = data.positions[this.m_indexB].a;

        const { qA, qB, lalcA, lalcB, PA, PB } = temp;
        qA.Set(aA);
        qB.Set(aB);

        const rA = b2Rot.MultiplyVec2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), this.m_rA);
        const rB = b2Rot.MultiplyVec2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), this.m_rB);

        // Get the pulley axes.
        const uA = b2Vec2.Add(cA, rA, this.m_uA).Subtract(this.m_groundAnchorA);
        const uB = b2Vec2.Add(cB, rB, this.m_uB).Subtract(this.m_groundAnchorB);

        const lengthA = uA.Length();
        const lengthB = uB.Length();

        if (lengthA > 10 * b2_linearSlop) {
            uA.Scale(1 / lengthA);
        } else {
            uA.SetZero();
        }

        if (lengthB > 10 * b2_linearSlop) {
            uB.Scale(1 / lengthB);
        } else {
            uB.SetZero();
        }

        // Compute effective mass.
        const ruA = b2Vec2.Cross(rA, uA);
        const ruB = b2Vec2.Cross(rB, uB);

        const mA = this.m_invMassA + this.m_invIA * ruA * ruA;
        const mB = this.m_invMassB + this.m_invIB * ruB * ruB;

        let mass = mA + this.m_ratio * this.m_ratio * mB;

        if (mass > 0) {
            mass = 1 / mass;
        }

        const C = this.m_constant - lengthA - this.m_ratio * lengthB;
        const linearError = Math.abs(C);

        const impulse = -mass * C;

        b2Vec2.Scale(-impulse, uA, PA);
        b2Vec2.Scale(-this.m_ratio * impulse, uB, PB);

        cA.AddScaled(this.m_invMassA, PA);
        aA += this.m_invIA * b2Vec2.Cross(rA, PA);
        cB.AddScaled(this.m_invMassB, PB);
        aB += this.m_invIB * b2Vec2.Cross(rB, PB);

        data.positions[this.m_indexA].a = aA;
        data.positions[this.m_indexB].a = aB;

        return linearError < b2_linearSlop;
    }

    public GetAnchorA<T extends XY>(out: T): T {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    }

    public GetAnchorB<T extends XY>(out: T): T {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }

    public GetReactionForce<T extends XY>(inv_dt: number, out: T): T {
        out.x = inv_dt * this.m_impulse * this.m_uB.x;
        out.y = inv_dt * this.m_impulse * this.m_uB.y;
        return out;
    }

    public GetReactionTorque(_inv_dt: number): number {
        return 0;
    }

    /** Get the first ground anchor. */
    public GetGroundAnchorA() {
        return this.m_groundAnchorA;
    }

    /** Get the second ground anchor. */
    public GetGroundAnchorB() {
        return this.m_groundAnchorB;
    }

    /** Get the current length of the segment attached to bodyA. */
    public GetLengthA() {
        return this.m_lengthA;
    }

    /** Get the current length of the segment attached to bodyB. */
    public GetLengthB() {
        return this.m_lengthB;
    }

    /** Get the pulley ratio. */
    public GetRatio() {
        return this.m_ratio;
    }

    /** Get the current length of the segment attached to bodyA. */
    public GetCurrentLengthA() {
        const p = this.m_bodyA.GetWorldPoint(this.m_localAnchorA, temp.p);
        const s = this.m_groundAnchorA;
        return b2Vec2.Distance(p, s);
    }

    /** Get the current length of the segment attached to bodyB. */
    public GetCurrentLengthB() {
        const p = this.m_bodyB.GetWorldPoint(this.m_localAnchorB, temp.p);
        const s = this.m_groundAnchorB;
        return b2Vec2.Distance(p, s);
    }

    public ShiftOrigin(newOrigin: b2Readonly<b2Vec2>) {
        this.m_groundAnchorA.Subtract(newOrigin);
        this.m_groundAnchorB.Subtract(newOrigin);
    }

    public Draw(draw: b2Draw): void {
        const p1 = this.GetAnchorA(temp.pA);
        const p2 = this.GetAnchorB(temp.pB);
        const s1 = this.GetGroundAnchorA();
        const s2 = this.GetGroundAnchorB();
        draw.DrawSegment(s1, p1, debugColors.joint6);
        draw.DrawSegment(s2, p2, debugColors.joint6);
        draw.DrawSegment(s1, s2, debugColors.joint6);
    }
}
