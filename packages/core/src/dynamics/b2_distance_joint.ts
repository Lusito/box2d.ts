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

import { LINEAR_SLOP, MAX_FLOAT } from "../common/b2_common";
import { clamp, Vec2, Rot, XY, Transform } from "../common/b2_math";
import { Joint, JointDef, JointType, IJointDef } from "./b2_joint";
import { SolverData } from "./b2_time_step";
import type { Body } from "./b2_body";
import { Draw, debugColors } from "../common/b2_draw";

const temp = {
    worldPointA: new Vec2(),
    worldPointB: new Vec2(),
    vpA: new Vec2(),
    vpB: new Vec2(),
    vpBA: new Vec2(),
    P: new Vec2(),
    qA: new Rot(),
    qB: new Rot(),
    lalcA: new Vec2(),
    lalcB: new Vec2(),
    Draw: {
        pA: new Vec2(),
        pB: new Vec2(),
        axis: new Vec2(),
        pRest: new Vec2(),
        p1: new Vec2(),
        p2: new Vec2(),
    },
};

export interface IDistanceJointDef extends IJointDef {
    localAnchorA: XY;
    localAnchorB: XY;
    length: number;
    minLength: number;
    maxLength: number;
    stiffness?: number;
    damping?: number;
}

/**
 * Distance joint definition. This requires defining an anchor point on both
 * bodies and the non-zero distance of the distance joint. The definition uses
 * local anchor points so that the initial configuration can violate the
 * constraint slightly. This helps when saving and loading a game.
 */
export class DistanceJointDef extends JointDef implements IDistanceJointDef {
    /** The local anchor point relative to bodyA's origin. */
    public readonly localAnchorA = new Vec2();

    /** The local anchor point relative to bodyB's origin. */
    public readonly localAnchorB = new Vec2();

    /** The rest length of this joint. Clamped to a stable minimum value. */
    public length = 1;

    /** Minimum length. Clamped to a stable minimum value. */
    public minLength = 0;

    /** Maximum length. Must be greater than or equal to the minimum length. */
    public maxLength = MAX_FLOAT;

    /** The linear stiffness in N/m. */
    public stiffness = 0;

    /** The linear damping in N*s/m. */
    public damping = 0;

    public constructor() {
        super(JointType.Distance);
    }

    public initialize(b1: Body, b2: Body, anchor1: XY, anchor2: XY): void {
        this.bodyA = b1;
        this.bodyB = b2;
        this.bodyA.getLocalPoint(anchor1, this.localAnchorA);
        this.bodyB.getLocalPoint(anchor2, this.localAnchorB);
        this.length = Math.max(Vec2.distance(anchor1, anchor2), LINEAR_SLOP);
        this.minLength = this.length;
        this.maxLength = this.length;
    }
}

/**
 * A distance joint constrains two points on two bodies to remain at a fixed
 * distance from each other. You can view this as a massless, rigid rod.
 */
export class DistanceJoint extends Joint {
    protected stiffness: number;

    protected damping: number;

    protected bias = 0;

    protected length: number;

    protected minLength: number;

    protected maxLength: number;

    // Solver shared
    protected readonly localAnchorA = new Vec2();

    protected readonly localAnchorB = new Vec2();

    protected gamma = 0;

    protected impulse = 0;

    protected lowerImpulse = 0;

    protected upperImpulse = 0;

    // Solver temp
    protected indexA = 0;

    protected indexB = 0;

    protected readonly u = new Vec2();

    protected readonly rA = new Vec2();

    protected readonly rB = new Vec2();

    protected readonly localCenterA = new Vec2();

    protected readonly localCenterB = new Vec2();

    protected currentLength = 0;

    protected invMassA = 0;

    protected invMassB = 0;

    protected invIA = 0;

    protected invIB = 0;

    protected softMass = 0;

    protected mass = 0;

    /** @internal protected */
    public constructor(def: IDistanceJointDef) {
        super(def);

        this.localAnchorA.copy(def.localAnchorA);
        this.localAnchorB.copy(def.localAnchorB);
        this.length = Math.max(def.length, LINEAR_SLOP);
        this.minLength = Math.max(def.minLength, LINEAR_SLOP);
        this.maxLength = Math.max(def.maxLength, this.minLength);
        this.stiffness = def.stiffness ?? 0;
        this.damping = def.damping ?? 0;
    }

    public getAnchorA<T extends XY>(out: T): T {
        return this.bodyA.getWorldPoint(this.localAnchorA, out);
    }

    public getAnchorB<T extends XY>(out: T): T {
        return this.bodyB.getWorldPoint(this.localAnchorB, out);
    }

    public getReactionForce<T extends XY>(inv_dt: number, out: T): T {
        const f = inv_dt * (this.impulse + this.lowerImpulse - this.upperImpulse);
        out.x = f * this.u.x;
        out.y = f * this.u.y;
        return out;
    }

    public getReactionTorque(_inv_dt: number): number {
        return 0;
    }

    public getLocalAnchorA(): Readonly<Vec2> {
        return this.localAnchorA;
    }

    public getLocalAnchorB(): Readonly<Vec2> {
        return this.localAnchorB;
    }

    public setLength(length: number) {
        this.impulse = 0;
        this.length = Math.max(LINEAR_SLOP, length);
        return this.length;
    }

    public getLength() {
        return this.length;
    }

    public setMinLength(minLength: number) {
        this.lowerImpulse = 0;
        this.minLength = clamp(minLength, LINEAR_SLOP, this.maxLength);
        return this.minLength;
    }

    public getMinLength() {
        return this.minLength;
    }

    public setMaxLength(maxLength: number) {
        this.upperImpulse = 0;
        this.maxLength = Math.max(maxLength, this.minLength);
        return this.maxLength;
    }

    public getMaxLength() {
        return this.maxLength;
    }

    public getCurrentLength() {
        const pA = this.bodyA.getWorldPoint(this.localAnchorA, temp.worldPointA);
        const pB = this.bodyB.getWorldPoint(this.localAnchorB, temp.worldPointB);
        return Vec2.distance(pB, pA);
    }

    public setStiffness(stiffness: number): void {
        this.stiffness = stiffness;
    }

    public getStiffness() {
        return this.stiffness;
    }

    public setDamping(damping: number): void {
        this.damping = damping;
    }

    public getDamping() {
        return this.damping;
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
        this.u.x = cB.x + this.rB.x - cA.x - this.rA.x;
        this.u.y = cB.y + this.rB.y - cA.y - this.rA.y;

        // Handle singularity.
        this.currentLength = this.u.length();
        if (this.currentLength > LINEAR_SLOP) {
            this.u.scale(1 / this.currentLength);
        } else {
            this.u.setZero();
            this.mass = 0;
            this.impulse = 0;
            this.lowerImpulse = 0;
            this.upperImpulse = 0;
        }

        const crAu = Vec2.cross(this.rA, this.u);
        const crBu = Vec2.cross(this.rB, this.u);
        let invMass = this.invMassA + this.invIA * crAu * crAu + this.invMassB + this.invIB * crBu * crBu;
        this.mass = invMass !== 0 ? 1 / invMass : 0;

        if (this.stiffness > 0 && this.minLength < this.maxLength) {
            // soft
            const C = this.currentLength - this.length;

            const d = this.damping;
            const k = this.stiffness;

            // magic formulas
            const h = data.step.dt;

            // gamma = 1 / (h * (d + h * k))
            // the extra factor of h in the denominator is since the lambda is an impulse, not a force
            this.gamma = h * (d + h * k);
            this.gamma = this.gamma !== 0 ? 1 / this.gamma : 0;
            this.bias = C * h * k * this.gamma;

            invMass += this.gamma;
            this.softMass = invMass !== 0 ? 1 / invMass : 0;
        } else {
            // rigid
            this.gamma = 0;
            this.bias = 0;
            this.softMass = this.mass;
        }

        if (data.step.warmStarting) {
            // Scale the impulse to support a variable time step.
            this.impulse *= data.step.dtRatio;
            this.lowerImpulse *= data.step.dtRatio;
            this.upperImpulse *= data.step.dtRatio;

            const { P } = temp;
            Vec2.scale(this.impulse + this.lowerImpulse - this.upperImpulse, this.u, P);

            vA.subtractScaled(this.invMassA, P);
            wA -= this.invIA * Vec2.cross(this.rA, P);
            vB.addScaled(this.invMassB, P);
            wB += this.invIB * Vec2.cross(this.rB, P);
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

        if (this.minLength < this.maxLength) {
            if (this.stiffness > 0) {
                // Cdot = dot(u, v + cross(w, r))

                const vpA = Vec2.addCrossScalarVec2(vA, wA, this.rA, temp.vpA);
                const vpB = Vec2.addCrossScalarVec2(vB, wB, this.rB, temp.vpB);

                const Cdot = Vec2.dot(this.u, Vec2.subtract(vpB, vpA, temp.vpBA));

                const impulse = -this.softMass * (Cdot + this.bias + this.gamma * this.impulse);
                this.impulse += impulse;

                const P = Vec2.scale(impulse, this.u, temp.P);
                vA.subtractScaled(this.invMassA, P);
                wA -= this.invIA * Vec2.cross(this.rA, P);
                vB.addScaled(this.invMassB, P);
                wB += this.invIB * Vec2.cross(this.rB, P);
            }

            // lower
            {
                const C = this.currentLength - this.minLength;
                const bias = Math.max(0, C) * data.step.inv_dt;

                const vpA = Vec2.addCrossScalarVec2(vA, wA, this.rA, temp.vpA);
                const vpB = Vec2.addCrossScalarVec2(vB, wB, this.rB, temp.vpB);
                const Cdot = Vec2.dot(this.u, Vec2.subtract(vpB, vpA, temp.vpBA));

                let impulse = -this.mass * (Cdot + bias);
                const oldImpulse = this.lowerImpulse;
                this.lowerImpulse = Math.max(0, this.lowerImpulse + impulse);
                impulse = this.lowerImpulse - oldImpulse;
                const P = Vec2.scale(impulse, this.u, temp.P);

                vA.subtractScaled(this.invMassA, P);
                wA -= this.invIA * Vec2.cross(this.rA, P);
                vB.addScaled(this.invMassB, P);
                wB += this.invIB * Vec2.cross(this.rB, P);
            }

            // upper
            {
                const C = this.maxLength - this.currentLength;
                const bias = Math.max(0, C) * data.step.inv_dt;

                const vpA = Vec2.addCrossScalarVec2(vA, wA, this.rA, temp.vpA);
                const vpB = Vec2.addCrossScalarVec2(vB, wB, this.rB, temp.vpB);
                const Cdot = Vec2.dot(this.u, Vec2.subtract(vpA, vpB, temp.vpBA));

                let impulse = -this.mass * (Cdot + bias);
                const oldImpulse = this.upperImpulse;
                this.upperImpulse = Math.max(0, this.upperImpulse + impulse);
                impulse = this.upperImpulse - oldImpulse;
                const P = Vec2.scale(-impulse, this.u, temp.P);

                vA.subtractScaled(this.invMassA, P);
                wA -= this.invIA * Vec2.cross(this.rA, P);
                vB.addScaled(this.invMassB, P);
                wB += this.invIB * Vec2.cross(this.rB, P);
            }
        } else {
            // Equal limits

            // Cdot = dot(u, v + cross(w, r))
            const vpA = Vec2.addCrossScalarVec2(vA, wA, this.rA, temp.vpA);
            const vpB = Vec2.addCrossScalarVec2(vB, wB, this.rB, temp.vpB);
            const Cdot = Vec2.dot(this.u, Vec2.subtract(vpB, vpA, temp.vpBA));

            const impulse = -this.mass * Cdot;
            this.impulse += impulse;

            const P = Vec2.scale(impulse, this.u, temp.P);
            vA.subtractScaled(this.invMassA, P);
            wA -= this.invIA * Vec2.cross(this.rA, P);
            vB.addScaled(this.invMassB, P);
            wB += this.invIB * Vec2.cross(this.rB, P);
        }

        data.velocities[this.indexA].w = wA;
        data.velocities[this.indexB].w = wB;
    }

    /** @internal protected */
    public solvePositionConstraints(data: SolverData): boolean {
        const cA = data.positions[this.indexA].c;
        let aA = data.positions[this.indexA].a;
        const cB = data.positions[this.indexB].c;
        let aB = data.positions[this.indexB].a;

        const { qA, qB, lalcA, lalcB, P } = temp;
        qA.set(aA);
        qB.set(aB);

        const rA = Rot.multiplyVec2(qA, Vec2.subtract(this.localAnchorA, this.localCenterA, lalcA), this.rA);
        const rB = Rot.multiplyVec2(qB, Vec2.subtract(this.localAnchorB, this.localCenterB, lalcB), this.rB);
        this.u.x = cB.x + rB.x - cA.x - rA.x;
        this.u.y = cB.y + rB.y - cA.y - rA.y;

        const length = this.u.normalize();
        let C: number;
        if (this.minLength === this.maxLength) {
            C = length - this.minLength;
        } else if (length < this.minLength) {
            C = length - this.minLength;
        } else if (this.maxLength < length) {
            C = length - this.maxLength;
        } else {
            return true;
        }

        const impulse = -this.mass * C;
        Vec2.scale(impulse, this.u, P);

        cA.subtractScaled(this.invMassA, P);
        aA -= this.invIA * Vec2.cross(rA, P);
        cB.addScaled(this.invMassB, P);
        aB += this.invIB * Vec2.cross(rB, P);

        data.positions[this.indexA].a = aA;
        data.positions[this.indexB].a = aB;

        return Math.abs(C) < LINEAR_SLOP;
    }

    public draw(draw: Draw): void {
        const { pA, pB, axis, pRest } = temp.Draw;
        const xfA = this.bodyA.getTransform();
        const xfB = this.bodyB.getTransform();
        Transform.multiplyVec2(xfA, this.localAnchorA, pA);
        Transform.multiplyVec2(xfB, this.localAnchorB, pB);
        Vec2.subtract(pB, pA, axis);
        axis.normalize();
        draw.drawSegment(pA, pB, debugColors.joint5);
        Vec2.addScaled(pA, this.length, axis, pRest);
        draw.drawPoint(pRest, 8, debugColors.joint1);
        if (this.minLength !== this.maxLength) {
            if (this.minLength > LINEAR_SLOP) {
                const pMin = Vec2.addScaled(pA, this.minLength, axis, temp.Draw.p1);
                draw.drawPoint(pMin, 4, debugColors.joint2);
            }
            if (this.maxLength < MAX_FLOAT) {
                const pMax = Vec2.addScaled(pA, this.maxLength, axis, temp.Draw.p1);
                draw.drawPoint(pMax, 4, debugColors.joint3);
            }
        }
    }
}
