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

// DEBUG: import { assert, EPSILON } from "../common/b2_common";
import { LINEAR_SLOP } from "../common/b2_common";
import { Draw, debugColors } from "../common/b2_draw";
import { Vec2, Rot, XY } from "../common/b2_math";
import { Body } from "./b2_body";
import { Joint, JointDef, JointType, IJointDef } from "./b2_joint";
import { SolverData } from "./b2_time_step";

export const MIN_PULLEY_LENGTH = 2;

const temp = {
    qA: new Rot(),
    qB: new Rot(),
    lalcA: new Vec2(),
    lalcB: new Vec2(),
    p: new Vec2(),
    PA: new Vec2(),
    PB: new Vec2(),
    vpA: new Vec2(),
    vpB: new Vec2(),
    pA: new Vec2(),
    pB: new Vec2(),
};

export interface IPulleyJointDef extends IJointDef {
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
export class PulleyJointDef extends JointDef implements IPulleyJointDef {
    /** The first ground anchor in world coordinates. This point never moves. */
    public readonly groundAnchorA = new Vec2(-1, 1);

    /** The second ground anchor in world coordinates. This point never moves. */
    public readonly groundAnchorB = new Vec2(1, 1);

    /** The local anchor point relative to bodyA's origin. */
    public readonly localAnchorA = new Vec2(-1, 0);

    /** The local anchor point relative to bodyB's origin. */
    public readonly localAnchorB = new Vec2(1, 0);

    /** The a reference length for the segment attached to bodyA. */
    public lengthA = 0;

    /** The a reference length for the segment attached to bodyB. */
    public lengthB = 0;

    /** The pulley ratio, used to simulate a block-and-tackle. */
    public ratio = 1;

    public constructor() {
        super(JointType.Pulley);
        this.collideConnected = true;
    }

    public initialize(bA: Body, bB: Body, groundA: Vec2, groundB: Vec2, anchorA: Vec2, anchorB: Vec2, r: number): void {
        this.bodyA = bA;
        this.bodyB = bB;
        this.groundAnchorA.copy(groundA);
        this.groundAnchorB.copy(groundB);
        this.bodyA.getLocalPoint(anchorA, this.localAnchorA);
        this.bodyB.getLocalPoint(anchorB, this.localAnchorB);
        this.lengthA = Vec2.distance(anchorA, groundA);
        this.lengthB = Vec2.distance(anchorB, groundB);
        this.ratio = r;
        // DEBUG: assert(this.ratio > EPSILON);
    }
}

const defaultGroundAnchorA = new Vec2(-1, 1);
const defaultGroundAnchorB = Vec2.UNITX;
const defaultLocalAnchorA = new Vec2(-1, 0);
const defaultLocalAnchorB = Vec2.UNITX;

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
export class PulleyJoint extends Joint {
    protected readonly m_groundAnchorA = new Vec2();

    protected readonly m_groundAnchorB = new Vec2();

    protected m_lengthA = 0;

    protected m_lengthB = 0;

    // Solver shared
    protected readonly m_localAnchorA = new Vec2();

    protected readonly m_localAnchorB = new Vec2();

    protected m_constant = 0;

    protected m_ratio = 0;

    protected m_impulse = 0;

    // Solver temp
    protected m_indexA = 0;

    protected m_indexB = 0;

    protected readonly m_uA = new Vec2();

    protected readonly m_uB = new Vec2();

    protected readonly m_rA = new Vec2();

    protected readonly m_rB = new Vec2();

    protected readonly m_localCenterA = new Vec2();

    protected readonly m_localCenterB = new Vec2();

    protected m_invMassA = 0;

    protected m_invMassB = 0;

    protected m_invIA = 0;

    protected m_invIB = 0;

    protected m_mass = 0;

    /** @internal protected */
    public constructor(def: IPulleyJointDef) {
        super(def);

        this.m_groundAnchorA.copy(def.groundAnchorA ?? defaultGroundAnchorA);
        this.m_groundAnchorB.copy(def.groundAnchorB ?? defaultGroundAnchorB);
        this.m_localAnchorA.copy(def.localAnchorA ?? defaultLocalAnchorA);
        this.m_localAnchorB.copy(def.localAnchorB ?? defaultLocalAnchorB);

        this.m_lengthA = def.lengthA ?? 0;
        this.m_lengthB = def.lengthB ?? 0;

        // DEBUG: assert((def.ratio ?? 1) !== 0);
        this.m_ratio = def.ratio ?? 1;

        this.m_constant = this.m_lengthA + this.m_ratio * this.m_lengthB;

        this.m_impulse = 0;
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

        const { qA, qB, lalcA, lalcB } = temp;
        qA.set(aA);
        qB.set(aB);

        Rot.multiplyVec2(qA, Vec2.subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), this.m_rA);
        Rot.multiplyVec2(qB, Vec2.subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), this.m_rB);

        // Get the pulley axes.
        Vec2.add(cA, this.m_rA, this.m_uA).subtract(this.m_groundAnchorA);
        Vec2.add(cB, this.m_rB, this.m_uB).subtract(this.m_groundAnchorB);

        const lengthA = this.m_uA.length();
        const lengthB = this.m_uB.length();

        if (lengthA > 10 * LINEAR_SLOP) {
            this.m_uA.scale(1 / lengthA);
        } else {
            this.m_uA.setZero();
        }

        if (lengthB > 10 * LINEAR_SLOP) {
            this.m_uB.scale(1 / lengthB);
        } else {
            this.m_uB.setZero();
        }

        // Compute effective mass.
        const ruA = Vec2.cross(this.m_rA, this.m_uA);
        const ruB = Vec2.cross(this.m_rB, this.m_uB);

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
            Vec2.scale(-this.m_impulse, this.m_uA, PA);
            Vec2.scale(-this.m_ratio * this.m_impulse, this.m_uB, PB);

            vA.addScaled(this.m_invMassA, PA);
            wA += this.m_invIA * Vec2.cross(this.m_rA, PA);
            vB.addScaled(this.m_invMassB, PB);
            wB += this.m_invIB * Vec2.cross(this.m_rB, PB);
        } else {
            this.m_impulse = 0;
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

        const { PA, PB, vpA, vpB } = temp;
        Vec2.addCrossScalarVec2(vA, wA, this.m_rA, vpA);
        Vec2.addCrossScalarVec2(vB, wB, this.m_rB, vpB);

        const Cdot = -Vec2.dot(this.m_uA, vpA) - this.m_ratio * Vec2.dot(this.m_uB, vpB);
        const impulse = -this.m_mass * Cdot;
        this.m_impulse += impulse;

        Vec2.scale(-impulse, this.m_uA, PA);
        Vec2.scale(-this.m_ratio * impulse, this.m_uB, PB);
        vA.addScaled(this.m_invMassA, PA);
        wA += this.m_invIA * Vec2.cross(this.m_rA, PA);
        vB.addScaled(this.m_invMassB, PB);
        wB += this.m_invIB * Vec2.cross(this.m_rB, PB);

        data.velocities[this.m_indexA].w = wA;
        data.velocities[this.m_indexB].w = wB;
    }

    /** @internal protected */
    public solvePositionConstraints(data: SolverData): boolean {
        const cA = data.positions[this.m_indexA].c;
        let aA = data.positions[this.m_indexA].a;
        const cB = data.positions[this.m_indexB].c;
        let aB = data.positions[this.m_indexB].a;

        const { qA, qB, lalcA, lalcB, PA, PB } = temp;
        qA.set(aA);
        qB.set(aB);

        const rA = Rot.multiplyVec2(qA, Vec2.subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), this.m_rA);
        const rB = Rot.multiplyVec2(qB, Vec2.subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), this.m_rB);

        // Get the pulley axes.
        const uA = Vec2.add(cA, rA, this.m_uA).subtract(this.m_groundAnchorA);
        const uB = Vec2.add(cB, rB, this.m_uB).subtract(this.m_groundAnchorB);

        const lengthA = uA.length();
        const lengthB = uB.length();

        if (lengthA > 10 * LINEAR_SLOP) {
            uA.scale(1 / lengthA);
        } else {
            uA.setZero();
        }

        if (lengthB > 10 * LINEAR_SLOP) {
            uB.scale(1 / lengthB);
        } else {
            uB.setZero();
        }

        // Compute effective mass.
        const ruA = Vec2.cross(rA, uA);
        const ruB = Vec2.cross(rB, uB);

        const mA = this.m_invMassA + this.m_invIA * ruA * ruA;
        const mB = this.m_invMassB + this.m_invIB * ruB * ruB;

        let mass = mA + this.m_ratio * this.m_ratio * mB;

        if (mass > 0) {
            mass = 1 / mass;
        }

        const C = this.m_constant - lengthA - this.m_ratio * lengthB;
        const linearError = Math.abs(C);

        const impulse = -mass * C;

        Vec2.scale(-impulse, uA, PA);
        Vec2.scale(-this.m_ratio * impulse, uB, PB);

        cA.addScaled(this.m_invMassA, PA);
        aA += this.m_invIA * Vec2.cross(rA, PA);
        cB.addScaled(this.m_invMassB, PB);
        aB += this.m_invIB * Vec2.cross(rB, PB);

        data.positions[this.m_indexA].a = aA;
        data.positions[this.m_indexB].a = aB;

        return linearError < LINEAR_SLOP;
    }

    public getAnchorA<T extends XY>(out: T): T {
        return this.m_bodyA.getWorldPoint(this.m_localAnchorA, out);
    }

    public getAnchorB<T extends XY>(out: T): T {
        return this.m_bodyB.getWorldPoint(this.m_localAnchorB, out);
    }

    public getReactionForce<T extends XY>(inv_dt: number, out: T): T {
        out.x = inv_dt * this.m_impulse * this.m_uB.x;
        out.y = inv_dt * this.m_impulse * this.m_uB.y;
        return out;
    }

    public getReactionTorque(_inv_dt: number): number {
        return 0;
    }

    public getGroundAnchorA() {
        return this.m_groundAnchorA;
    }

    public getGroundAnchorB() {
        return this.m_groundAnchorB;
    }

    public getLengthA() {
        return this.m_lengthA;
    }

    public getLengthB() {
        return this.m_lengthB;
    }

    public getRatio() {
        return this.m_ratio;
    }

    public getCurrentLengthA() {
        const p = this.m_bodyA.getWorldPoint(this.m_localAnchorA, temp.p);
        const s = this.m_groundAnchorA;
        return Vec2.distance(p, s);
    }

    public getCurrentLengthB() {
        const p = this.m_bodyB.getWorldPoint(this.m_localAnchorB, temp.p);
        const s = this.m_groundAnchorB;
        return Vec2.distance(p, s);
    }

    public shiftOrigin(newOrigin: Vec2) {
        this.m_groundAnchorA.subtract(newOrigin);
        this.m_groundAnchorB.subtract(newOrigin);
    }

    public draw(draw: Draw): void {
        const p1 = this.getAnchorA(temp.pA);
        const p2 = this.getAnchorB(temp.pB);
        const s1 = this.getGroundAnchorA();
        const s2 = this.getGroundAnchorB();
        draw.drawSegment(s1, p1, debugColors.joint6);
        draw.drawSegment(s2, p2, debugColors.joint6);
        draw.drawSegment(s1, s2, debugColors.joint6);
    }
}
