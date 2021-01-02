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
    protected readonly groundAnchorA = new Vec2();

    protected readonly groundAnchorB = new Vec2();

    protected lengthA = 0;

    protected lengthB = 0;

    // Solver shared
    protected readonly localAnchorA = new Vec2();

    protected readonly localAnchorB = new Vec2();

    protected constant = 0;

    protected ratio = 0;

    protected impulse = 0;

    // Solver temp
    protected indexA = 0;

    protected indexB = 0;

    protected readonly uA = new Vec2();

    protected readonly uB = new Vec2();

    protected readonly rA = new Vec2();

    protected readonly rB = new Vec2();

    protected readonly localCenterA = new Vec2();

    protected readonly localCenterB = new Vec2();

    protected invMassA = 0;

    protected invMassB = 0;

    protected invIA = 0;

    protected invIB = 0;

    protected mass = 0;

    /** @internal protected */
    public constructor(def: IPulleyJointDef) {
        super(def);

        this.groundAnchorA.copy(def.groundAnchorA ?? defaultGroundAnchorA);
        this.groundAnchorB.copy(def.groundAnchorB ?? defaultGroundAnchorB);
        this.localAnchorA.copy(def.localAnchorA ?? defaultLocalAnchorA);
        this.localAnchorB.copy(def.localAnchorB ?? defaultLocalAnchorB);

        this.lengthA = def.lengthA ?? 0;
        this.lengthB = def.lengthB ?? 0;

        // DEBUG: assert((def.ratio ?? 1) !== 0);
        this.ratio = def.ratio ?? 1;

        this.constant = this.lengthA + this.ratio * this.lengthB;

        this.impulse = 0;
    }

    /** @internal protected */
    public initVelocityConstraints(data: SolverData): void {
        this.indexA = this.bodyA.islandIndex;
        this.indexB = this.bodyB.islandIndex;
        this.localCenterA.copy(this.bodyA.sweep.localCenter);
        this.localCenterB.copy(this.bodyB.sweep.localCenter);
        this.invMassA = this.bodyA.invMass;
        this.invMassB = this.bodyB.invMass;
        this.invIA = this.bodyA.invI;
        this.invIB = this.bodyB.invI;

        const cA = data.positions[this.indexA].c;
        const aA = data.positions[this.indexA].a;
        const vA = data.velocities[this.indexA].v;
        let wA = data.velocities[this.indexA].w;

        const cB = data.positions[this.indexB].c;
        const aB = data.positions[this.indexB].a;
        const vB = data.velocities[this.indexB].v;
        let wB = data.velocities[this.indexB].w;

        const { qA, qB, lalcA, lalcB } = temp;
        qA.set(aA);
        qB.set(aB);

        Rot.multiplyVec2(qA, Vec2.subtract(this.localAnchorA, this.localCenterA, lalcA), this.rA);
        Rot.multiplyVec2(qB, Vec2.subtract(this.localAnchorB, this.localCenterB, lalcB), this.rB);

        // Get the pulley axes.
        Vec2.add(cA, this.rA, this.uA).subtract(this.groundAnchorA);
        Vec2.add(cB, this.rB, this.uB).subtract(this.groundAnchorB);

        const lengthA = this.uA.length();
        const lengthB = this.uB.length();

        if (lengthA > 10 * LINEAR_SLOP) {
            this.uA.scale(1 / lengthA);
        } else {
            this.uA.setZero();
        }

        if (lengthB > 10 * LINEAR_SLOP) {
            this.uB.scale(1 / lengthB);
        } else {
            this.uB.setZero();
        }

        // Compute effective mass.
        const ruA = Vec2.cross(this.rA, this.uA);
        const ruB = Vec2.cross(this.rB, this.uB);

        const mA = this.invMassA + this.invIA * ruA * ruA;
        const mB = this.invMassB + this.invIB * ruB * ruB;

        this.mass = mA + this.ratio * this.ratio * mB;

        if (this.mass > 0) {
            this.mass = 1 / this.mass;
        }

        if (data.step.warmStarting) {
            // Scale impulses to support variable time steps.
            this.impulse *= data.step.dtRatio;

            // Warm starting.
            const { PA, PB } = temp;
            Vec2.scale(-this.impulse, this.uA, PA);
            Vec2.scale(-this.ratio * this.impulse, this.uB, PB);

            vA.addScaled(this.invMassA, PA);
            wA += this.invIA * Vec2.cross(this.rA, PA);
            vB.addScaled(this.invMassB, PB);
            wB += this.invIB * Vec2.cross(this.rB, PB);
        } else {
            this.impulse = 0;
        }

        data.velocities[this.indexA].w = wA;
        data.velocities[this.indexB].w = wB;
    }

    /** @internal protected */
    public solveVelocityConstraints(data: SolverData): void {
        const vA = data.velocities[this.indexA].v;
        let wA = data.velocities[this.indexA].w;
        const vB = data.velocities[this.indexB].v;
        let wB = data.velocities[this.indexB].w;

        const { PA, PB, vpA, vpB } = temp;
        Vec2.addCrossScalarVec2(vA, wA, this.rA, vpA);
        Vec2.addCrossScalarVec2(vB, wB, this.rB, vpB);

        const Cdot = -Vec2.dot(this.uA, vpA) - this.ratio * Vec2.dot(this.uB, vpB);
        const impulse = -this.mass * Cdot;
        this.impulse += impulse;

        Vec2.scale(-impulse, this.uA, PA);
        Vec2.scale(-this.ratio * impulse, this.uB, PB);
        vA.addScaled(this.invMassA, PA);
        wA += this.invIA * Vec2.cross(this.rA, PA);
        vB.addScaled(this.invMassB, PB);
        wB += this.invIB * Vec2.cross(this.rB, PB);

        data.velocities[this.indexA].w = wA;
        data.velocities[this.indexB].w = wB;
    }

    /** @internal protected */
    public solvePositionConstraints(data: SolverData): boolean {
        const cA = data.positions[this.indexA].c;
        let aA = data.positions[this.indexA].a;
        const cB = data.positions[this.indexB].c;
        let aB = data.positions[this.indexB].a;

        const { qA, qB, lalcA, lalcB, PA, PB } = temp;
        qA.set(aA);
        qB.set(aB);

        const rA = Rot.multiplyVec2(qA, Vec2.subtract(this.localAnchorA, this.localCenterA, lalcA), this.rA);
        const rB = Rot.multiplyVec2(qB, Vec2.subtract(this.localAnchorB, this.localCenterB, lalcB), this.rB);

        // Get the pulley axes.
        const uA = Vec2.add(cA, rA, this.uA).subtract(this.groundAnchorA);
        const uB = Vec2.add(cB, rB, this.uB).subtract(this.groundAnchorB);

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

        const mA = this.invMassA + this.invIA * ruA * ruA;
        const mB = this.invMassB + this.invIB * ruB * ruB;

        let mass = mA + this.ratio * this.ratio * mB;

        if (mass > 0) {
            mass = 1 / mass;
        }

        const C = this.constant - lengthA - this.ratio * lengthB;
        const linearError = Math.abs(C);

        const impulse = -mass * C;

        Vec2.scale(-impulse, uA, PA);
        Vec2.scale(-this.ratio * impulse, uB, PB);

        cA.addScaled(this.invMassA, PA);
        aA += this.invIA * Vec2.cross(rA, PA);
        cB.addScaled(this.invMassB, PB);
        aB += this.invIB * Vec2.cross(rB, PB);

        data.positions[this.indexA].a = aA;
        data.positions[this.indexB].a = aB;

        return linearError < LINEAR_SLOP;
    }

    public getAnchorA<T extends XY>(out: T): T {
        return this.bodyA.getWorldPoint(this.localAnchorA, out);
    }

    public getAnchorB<T extends XY>(out: T): T {
        return this.bodyB.getWorldPoint(this.localAnchorB, out);
    }

    public getReactionForce<T extends XY>(inv_dt: number, out: T): T {
        out.x = inv_dt * this.impulse * this.uB.x;
        out.y = inv_dt * this.impulse * this.uB.y;
        return out;
    }

    public getReactionTorque(_inv_dt: number): number {
        return 0;
    }

    public getGroundAnchorA() {
        return this.groundAnchorA;
    }

    public getGroundAnchorB() {
        return this.groundAnchorB;
    }

    public getLengthA() {
        return this.lengthA;
    }

    public getLengthB() {
        return this.lengthB;
    }

    public getRatio() {
        return this.ratio;
    }

    public getCurrentLengthA() {
        const p = this.bodyA.getWorldPoint(this.localAnchorA, temp.p);
        const s = this.groundAnchorA;
        return Vec2.distance(p, s);
    }

    public getCurrentLengthB() {
        const p = this.bodyB.getWorldPoint(this.localAnchorB, temp.p);
        const s = this.groundAnchorB;
        return Vec2.distance(p, s);
    }

    public shiftOrigin(newOrigin: Vec2) {
        this.groundAnchorA.subtract(newOrigin);
        this.groundAnchorB.subtract(newOrigin);
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
