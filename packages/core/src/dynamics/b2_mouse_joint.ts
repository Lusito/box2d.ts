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
import { Draw, debugColors } from "../common/b2_draw";
import { Vec2, Mat22, Rot, Transform, XY } from "../common/b2_math";
import { Joint, JointDef, JointType, IJointDef } from "./b2_joint";
import { SolverData } from "./b2_time_step";

const temp = {
    qB: new Rot(),
    lalcB: new Vec2(),
    Cdot: new Vec2(),
    impulse: new Vec2(),
    oldImpulse: new Vec2(),
    pA: new Vec2(),
    pB: new Vec2(),
};

export interface IMouseJointDef extends IJointDef {
    target?: XY;

    maxForce?: number;

    stiffness?: number;

    damping?: number;
}

/**
 * Mouse joint definition. This requires a world target point,
 * tuning parameters, and the time step.
 */
export class MouseJointDef extends JointDef implements IMouseJointDef {
    /**
     * The initial world target point. This is assumed
     * to coincide with the body anchor initially.
     */
    public readonly target = new Vec2();

    /**
     * The maximum constraint force that can be exerted
     * to move the candidate body. Usually you will express
     * as some multiple of the weight (multiplier * mass * gravity).
     */
    public maxForce = 0;

    /** The linear stiffness in N/m */
    public stiffness = 0;

    /** The linear damping in N*s/m */
    public damping = 0;

    public constructor() {
        super(JointType.Mouse);
    }
}

/**
 * A mouse joint is used to make a point on a body track a
 * specified world point. This a soft constraint with a maximum
 * force. This allows the constraint to stretch and without
 * applying huge forces.
 * NOTE: this joint is not documented in the manual because it was
 * developed to be used in the testbed. If you want to learn how to
 * use the mouse joint, look at the testbed.
 */
export class MouseJoint extends Joint {
    protected readonly m_localAnchorB = new Vec2();

    protected readonly m_targetA = new Vec2();

    protected m_stiffness = 0;

    protected m_damping = 0;

    protected m_beta = 0;

    // Solver shared
    protected readonly m_impulse = new Vec2();

    protected m_maxForce = 0;

    protected m_gamma = 0;

    // Solver temp
    protected m_indexB = 0;

    protected readonly m_rB = new Vec2();

    protected readonly m_localCenterB = new Vec2();

    protected m_invMassB = 0;

    protected m_invIB = 0;

    protected readonly m_mass = new Mat22();

    protected readonly m_C = new Vec2();

    /** @internal protected */
    public constructor(def: IMouseJointDef) {
        super(def);

        this.m_targetA.copy(def.target ?? Vec2.ZERO);
        Transform.transposeMultiplyVec2(this.m_bodyB.getTransform(), this.m_targetA, this.m_localAnchorB);
        this.m_maxForce = def.maxForce ?? 0;
        this.m_stiffness = def.stiffness ?? 0;
        this.m_damping = def.damping ?? 0;

        this.m_beta = 0;
        this.m_gamma = 0;
    }

    public setTarget(target: XY): void {
        if (!Vec2.equals(target, this.m_targetA)) {
            this.m_bodyB.setAwake(true);
            this.m_targetA.copy(target);
        }
    }

    public getTarget() {
        return this.m_targetA;
    }

    public setMaxForce(force: number): void {
        this.m_maxForce = force;
    }

    public getMaxForce() {
        return this.m_maxForce;
    }

    public setStiffness(stiffness: number): void {
        this.m_stiffness = stiffness;
    }

    public getStiffness() {
        return this.m_stiffness;
    }

    public setDamping(damping: number) {
        this.m_damping = damping;
    }

    public getDamping() {
        return this.m_damping;
    }

    /** @internal protected */
    public initVelocityConstraints(data: SolverData): void {
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterB.copy(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIB = this.m_bodyB.m_invI;

        const cB = data.positions[this.m_indexB].c;
        const aB = data.positions[this.m_indexB].a;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;

        const { qB, lalcB } = temp;

        qB.set(aB);

        const d = this.m_damping;
        const k = this.m_stiffness;

        // magic formulas
        // gamma has units of inverse mass.
        // beta has units of inverse time.
        const h = data.step.dt;
        this.m_gamma = h * (d + h * k);
        if (this.m_gamma !== 0) {
            this.m_gamma = 1 / this.m_gamma;
        }
        this.m_beta = h * k * this.m_gamma;

        // Compute the effective mass matrix.
        Rot.multiplyVec2(qB, Vec2.subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), this.m_rB);

        // K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
        //      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
        //        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
        const K = this.m_mass;
        K.ex.x = this.m_invMassB + this.m_invIB * this.m_rB.y * this.m_rB.y + this.m_gamma;
        K.ex.y = -this.m_invIB * this.m_rB.x * this.m_rB.y;
        K.ey.x = K.ex.y;
        K.ey.y = this.m_invMassB + this.m_invIB * this.m_rB.x * this.m_rB.x + this.m_gamma;

        K.inverse();

        Vec2.add(cB, this.m_rB, this.m_C).subtract(this.m_targetA);
        this.m_C.scale(this.m_beta);

        // Cheat with some damping
        wB *= 0.98;

        if (data.step.warmStarting) {
            this.m_impulse.scale(data.step.dtRatio);
            vB.addScaled(this.m_invMassB, this.m_impulse);
            wB += this.m_invIB * Vec2.cross(this.m_rB, this.m_impulse);
        } else {
            this.m_impulse.setZero();
        }
        data.velocities[this.m_indexB].w = wB;
    }

    /** @internal protected */
    public solveVelocityConstraints(data: SolverData): void {
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;

        // Cdot = v + cross(w, r)
        const { Cdot, impulse, oldImpulse } = temp;
        Vec2.addCrossScalarVec2(vB, wB, this.m_rB, Cdot);
        Mat22.multiplyVec2(
            this.m_mass,
            Vec2.add(Cdot, this.m_C, impulse).addScaled(this.m_gamma, this.m_impulse).negate(),
            impulse,
        );

        oldImpulse.copy(this.m_impulse);
        this.m_impulse.add(impulse);
        const maxImpulse = data.step.dt * this.m_maxForce;
        if (this.m_impulse.lengthSquared() > maxImpulse * maxImpulse) {
            this.m_impulse.scale(maxImpulse / this.m_impulse.length());
        }
        Vec2.subtract(this.m_impulse, oldImpulse, impulse);

        vB.addScaled(this.m_invMassB, impulse);
        wB += this.m_invIB * Vec2.cross(this.m_rB, impulse);

        data.velocities[this.m_indexB].w = wB;
    }

    /** @internal protected */
    public solvePositionConstraints(_data: SolverData): boolean {
        return true;
    }

    public getAnchorA<T extends XY>(out: T): T {
        out.x = this.m_targetA.x;
        out.y = this.m_targetA.y;
        return out;
    }

    public getAnchorB<T extends XY>(out: T): T {
        return this.m_bodyB.getWorldPoint(this.m_localAnchorB, out);
    }

    public getReactionForce<T extends XY>(inv_dt: number, out: T): T {
        return Vec2.scale(inv_dt, this.m_impulse, out);
    }

    public getReactionTorque(_inv_dt: number): number {
        return 0;
    }

    public shiftOrigin(newOrigin: XY) {
        this.m_targetA.subtract(newOrigin);
    }

    public draw(draw: Draw): void {
        const p1 = this.getAnchorA(temp.pA);
        const p2 = this.getAnchorB(temp.pB);
        draw.drawPoint(p1, 4, debugColors.joint7);
        draw.drawPoint(p2, 4, debugColors.joint7);
        draw.drawSegment(p1, p2, debugColors.joint8);
    }
}
