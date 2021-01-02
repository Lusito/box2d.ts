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

import { clamp, Vec2, Mat22, Rot, XY } from "../common/b2_math";
import { Joint, JointDef, JointType, IJointDef } from "./b2_joint";
import { SolverData } from "./b2_time_step";
import { Body } from "./b2_body";

const temp = {
    qA: new Rot(),
    qB: new Rot(),
    lalcA: new Vec2(),
    lalcB: new Vec2(),
    Cdot: new Vec2(),
    impulse: new Vec2(),
    oldImpulse: new Vec2(),
};

export interface IFrictionJointDef extends IJointDef {
    localAnchorA: XY;

    localAnchorB: XY;

    maxForce?: number;

    maxTorque?: number;
}

/**
 * Friction joint definition.
 */
export class FrictionJointDef extends JointDef implements IFrictionJointDef {
    /** The local anchor point relative to bodyA's origin. */
    public readonly localAnchorA = new Vec2();

    /** The local anchor point relative to bodyB's origin. */
    public readonly localAnchorB = new Vec2();

    /** The maximum friction force in N. */
    public maxForce = 0;

    /** The maximum friction torque in N-m. */
    public maxTorque = 0;

    public constructor() {
        super(JointType.Friction);
    }

    public initialize(bA: Body, bB: Body, anchor: Vec2): void {
        this.bodyA = bA;
        this.bodyB = bB;
        this.bodyA.getLocalPoint(anchor, this.localAnchorA);
        this.bodyB.getLocalPoint(anchor, this.localAnchorB);
    }
}

/**
 * Friction joint. This is used for top-down friction.
 * It provides 2D translational friction and angular friction.
 */
export class FrictionJoint extends Joint {
    protected readonly localAnchorA = new Vec2();

    protected readonly localAnchorB = new Vec2();

    // Solver shared
    protected readonly linearImpulse = new Vec2();

    protected angularImpulse = 0;

    protected maxForce = 0;

    protected maxTorque = 0;

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

    protected readonly linearMass = new Mat22();

    protected angularMass = 0;

    /** @internal protected */
    public constructor(def: IFrictionJointDef) {
        super(def);

        this.localAnchorA.copy(def.localAnchorA);
        this.localAnchorB.copy(def.localAnchorB);

        this.linearImpulse.setZero();
        this.maxForce = def.maxForce ?? 0;
        this.maxTorque = def.maxTorque ?? 0;
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

        // Compute the effective mass matrix.
        Rot.multiplyVec2(qA, Vec2.subtract(this.localAnchorA, this.localCenterA, lalcA), this.rA);
        Rot.multiplyVec2(qB, Vec2.subtract(this.localAnchorB, this.localCenterB, lalcB), this.rB);

        // J = [-I -r1_skew I r2_skew]
        //     [ 0       -1 0       1]
        // r_skew = [-ry; rx]

        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
        //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
        //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

        const mA = this.invMassA;
        const mB = this.invMassB;
        const iA = this.invIA;
        const iB = this.invIB;

        const K = this.linearMass;
        K.ex.x = mA + mB + iA * this.rA.y * this.rA.y + iB * this.rB.y * this.rB.y;
        K.ex.y = -iA * this.rA.x * this.rA.y - iB * this.rB.x * this.rB.y;
        K.ey.x = K.ex.y;
        K.ey.y = mA + mB + iA * this.rA.x * this.rA.x + iB * this.rB.x * this.rB.x;

        K.inverse();

        this.angularMass = iA + iB;
        if (this.angularMass > 0) {
            this.angularMass = 1 / this.angularMass;
        }

        if (data.step.warmStarting) {
            // Scale impulses to support a variable time step.
            this.linearImpulse.scale(data.step.dtRatio);
            this.angularImpulse *= data.step.dtRatio;

            const P = this.linearImpulse;
            vA.subtractScaled(mA, P);
            wA -= iA * (Vec2.cross(this.rA, P) + this.angularImpulse);
            vB.addScaled(mB, P);
            wB += iB * (Vec2.cross(this.rB, P) + this.angularImpulse);
        } else {
            this.linearImpulse.setZero();
            this.angularImpulse = 0;
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

        const h = data.step.dt;

        // Solve angular friction
        {
            const Cdot = wB - wA;
            let impulse = -this.angularMass * Cdot;

            const oldImpulse = this.angularImpulse;
            const maxImpulse = h * this.maxTorque;
            this.angularImpulse = clamp(this.angularImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.angularImpulse - oldImpulse;

            wA -= iA * impulse;
            wB += iB * impulse;
        }

        // Solve linear friction
        {
            const { Cdot, impulse, oldImpulse } = temp;
            Vec2.subtract(
                Vec2.addCrossScalarVec2(vB, wB, this.rB, Vec2.s_t0),
                Vec2.addCrossScalarVec2(vA, wA, this.rA, Vec2.s_t1),
                Cdot,
            );

            Mat22.multiplyVec2(this.linearMass, Cdot, impulse).negate();
            oldImpulse.copy(this.linearImpulse);
            this.linearImpulse.add(impulse);

            const maxImpulse = h * this.maxForce;

            if (this.linearImpulse.lengthSquared() > maxImpulse * maxImpulse) {
                this.linearImpulse.normalize();
                this.linearImpulse.scale(maxImpulse);
            }

            Vec2.subtract(this.linearImpulse, oldImpulse, impulse);

            vA.subtractScaled(mA, impulse);
            wA -= iA * Vec2.cross(this.rA, impulse);

            vB.addScaled(mB, impulse);
            wB += iB * Vec2.cross(this.rB, impulse);
        }

        data.velocities[this.indexA].w = wA;
        data.velocities[this.indexB].w = wB;
    }

    public solvePositionConstraints(_data: SolverData): boolean {
        return true;
    }

    public getAnchorA<T extends XY>(out: T): T {
        return this.bodyA.getWorldPoint(this.localAnchorA, out);
    }

    public getAnchorB<T extends XY>(out: T): T {
        return this.bodyB.getWorldPoint(this.localAnchorB, out);
    }

    public getReactionForce<T extends XY>(inv_dt: number, out: T): T {
        out.x = inv_dt * this.linearImpulse.x;
        out.y = inv_dt * this.linearImpulse.y;
        return out;
    }

    public getReactionTorque(inv_dt: number): number {
        return inv_dt * this.angularImpulse;
    }

    public getLocalAnchorA(): Readonly<Vec2> {
        return this.localAnchorA;
    }

    public getLocalAnchorB(): Readonly<Vec2> {
        return this.localAnchorB;
    }

    public setMaxForce(force: number): void {
        // DEBUG: assert(Number.isFinite(force) && force >= 0);
        this.maxForce = force;
    }

    public getMaxForce(): number {
        return this.maxForce;
    }

    public setMaxTorque(torque: number): void {
        // DEBUG: assert(Number.isFinite(torque) && torque >= 0);
        this.maxTorque = torque;
    }

    public getMaxTorque(): number {
        return this.maxTorque;
    }
}
