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
    protected readonly localAnchorB = new Vec2();

    protected readonly targetA = new Vec2();

    protected stiffness = 0;

    protected damping = 0;

    protected beta = 0;

    // Solver shared
    protected readonly impulse = new Vec2();

    protected maxForce = 0;

    protected gamma = 0;

    // Solver temp
    protected indexB = 0;

    protected readonly rB = new Vec2();

    protected readonly localCenterB = new Vec2();

    protected invMassB = 0;

    protected invIB = 0;

    protected readonly mass = new Mat22();

    protected readonly C = new Vec2();

    /** @internal protected */
    public constructor(def: IMouseJointDef) {
        super(def);

        this.targetA.copy(def.target ?? Vec2.ZERO);
        Transform.transposeMultiplyVec2(this.bodyB.getTransform(), this.targetA, this.localAnchorB);
        this.maxForce = def.maxForce ?? 0;
        this.stiffness = def.stiffness ?? 0;
        this.damping = def.damping ?? 0;

        this.beta = 0;
        this.gamma = 0;
    }

    public setTarget(target: XY): void {
        if (!Vec2.equals(target, this.targetA)) {
            this.bodyB.setAwake(true);
            this.targetA.copy(target);
        }
    }

    public getTarget() {
        return this.targetA;
    }

    public setMaxForce(force: number): void {
        this.maxForce = force;
    }

    public getMaxForce() {
        return this.maxForce;
    }

    public setStiffness(stiffness: number): void {
        this.stiffness = stiffness;
    }

    public getStiffness() {
        return this.stiffness;
    }

    public setDamping(damping: number) {
        this.damping = damping;
    }

    public getDamping() {
        return this.damping;
    }

    /** @internal protected */
    public initVelocityConstraints(data: SolverData): void {
        this.indexB = this.bodyB.islandIndex;
        this.localCenterB.copy(this.bodyB.sweep.localCenter);
        this.invMassB = this.bodyB.invMass;
        this.invIB = this.bodyB.invI;

        const cB = data.positions[this.indexB].c;
        const aB = data.positions[this.indexB].a;
        const vB = data.velocities[this.indexB].v;
        let wB = data.velocities[this.indexB].w;

        const { qB, lalcB } = temp;

        qB.set(aB);

        const d = this.damping;
        const k = this.stiffness;

        // magic formulas
        // gamma has units of inverse mass.
        // beta has units of inverse time.
        const h = data.step.dt;
        this.gamma = h * (d + h * k);
        if (this.gamma !== 0) {
            this.gamma = 1 / this.gamma;
        }
        this.beta = h * k * this.gamma;

        // Compute the effective mass matrix.
        Rot.multiplyVec2(qB, Vec2.subtract(this.localAnchorB, this.localCenterB, lalcB), this.rB);

        // K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
        //      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
        //        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
        const K = this.mass;
        K.ex.x = this.invMassB + this.invIB * this.rB.y * this.rB.y + this.gamma;
        K.ex.y = -this.invIB * this.rB.x * this.rB.y;
        K.ey.x = K.ex.y;
        K.ey.y = this.invMassB + this.invIB * this.rB.x * this.rB.x + this.gamma;

        K.inverse();

        Vec2.add(cB, this.rB, this.C).subtract(this.targetA);
        this.C.scale(this.beta);

        // Cheat with some damping
        wB *= 0.98;

        if (data.step.warmStarting) {
            this.impulse.scale(data.step.dtRatio);
            vB.addScaled(this.invMassB, this.impulse);
            wB += this.invIB * Vec2.cross(this.rB, this.impulse);
        } else {
            this.impulse.setZero();
        }
        data.velocities[this.indexB].w = wB;
    }

    /** @internal protected */
    public solveVelocityConstraints(data: SolverData): void {
        const vB = data.velocities[this.indexB].v;
        let wB = data.velocities[this.indexB].w;

        // Cdot = v + cross(w, r)
        const { Cdot, impulse, oldImpulse } = temp;
        Vec2.addCrossScalarVec2(vB, wB, this.rB, Cdot);
        Mat22.multiplyVec2(
            this.mass,
            Vec2.add(Cdot, this.C, impulse).addScaled(this.gamma, this.impulse).negate(),
            impulse,
        );

        oldImpulse.copy(this.impulse);
        this.impulse.add(impulse);
        const maxImpulse = data.step.dt * this.maxForce;
        if (this.impulse.lengthSquared() > maxImpulse * maxImpulse) {
            this.impulse.scale(maxImpulse / this.impulse.length());
        }
        Vec2.subtract(this.impulse, oldImpulse, impulse);

        vB.addScaled(this.invMassB, impulse);
        wB += this.invIB * Vec2.cross(this.rB, impulse);

        data.velocities[this.indexB].w = wB;
    }

    /** @internal protected */
    public solvePositionConstraints(_data: SolverData): boolean {
        return true;
    }

    public getAnchorA<T extends XY>(out: T): T {
        out.x = this.targetA.x;
        out.y = this.targetA.y;
        return out;
    }

    public getAnchorB<T extends XY>(out: T): T {
        return this.bodyB.getWorldPoint(this.localAnchorB, out);
    }

    public getReactionForce<T extends XY>(inv_dt: number, out: T): T {
        return Vec2.scale(inv_dt, this.impulse, out);
    }

    public getReactionTorque(_inv_dt: number): number {
        return 0;
    }

    public shiftOrigin(newOrigin: XY) {
        this.targetA.subtract(newOrigin);
    }

    public draw(draw: Draw): void {
        const p1 = this.getAnchorA(temp.pA);
        const p2 = this.getAnchorB(temp.pB);
        draw.drawPoint(p1, 4, debugColors.joint7);
        draw.drawPoint(p2, 4, debugColors.joint7);
        draw.drawSegment(p1, p2, debugColors.joint8);
    }
}
