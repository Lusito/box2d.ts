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
import { Clamp, Vec2, Rot, XY, Transform } from "../common/b2_math";
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

    public Initialize(b1: Body, b2: Body, anchor1: XY, anchor2: XY): void {
        this.bodyA = b1;
        this.bodyB = b2;
        this.bodyA.GetLocalPoint(anchor1, this.localAnchorA);
        this.bodyB.GetLocalPoint(anchor2, this.localAnchorB);
        this.length = Math.max(Vec2.Distance(anchor1, anchor2), LINEAR_SLOP);
        this.minLength = this.length;
        this.maxLength = this.length;
    }
}

/**
 * A distance joint constrains two points on two bodies to remain at a fixed
 * distance from each other. You can view this as a massless, rigid rod.
 */
export class DistanceJoint extends Joint {
    protected m_stiffness: number;

    protected m_damping: number;

    protected m_bias = 0;

    protected m_length: number;

    protected m_minLength: number;

    protected m_maxLength: number;

    // Solver shared
    protected readonly m_localAnchorA = new Vec2();

    protected readonly m_localAnchorB = new Vec2();

    protected m_gamma = 0;

    protected m_impulse = 0;

    protected m_lowerImpulse = 0;

    protected m_upperImpulse = 0;

    // Solver temp
    protected m_indexA = 0;

    protected m_indexB = 0;

    protected readonly m_u = new Vec2();

    protected readonly m_rA = new Vec2();

    protected readonly m_rB = new Vec2();

    protected readonly m_localCenterA = new Vec2();

    protected readonly m_localCenterB = new Vec2();

    protected m_currentLength = 0;

    protected m_invMassA = 0;

    protected m_invMassB = 0;

    protected m_invIA = 0;

    protected m_invIB = 0;

    protected m_softMass = 0;

    protected m_mass = 0;

    /** @internal protected */
    public constructor(def: IDistanceJointDef) {
        super(def);

        this.m_localAnchorA.Copy(def.localAnchorA);
        this.m_localAnchorB.Copy(def.localAnchorB);
        this.m_length = Math.max(def.length, LINEAR_SLOP);
        this.m_minLength = Math.max(def.minLength, LINEAR_SLOP);
        this.m_maxLength = Math.max(def.maxLength, this.m_minLength);
        this.m_stiffness = def.stiffness ?? 0;
        this.m_damping = def.damping ?? 0;
    }

    public GetAnchorA<T extends XY>(out: T): T {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    }

    public GetAnchorB<T extends XY>(out: T): T {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }

    public GetReactionForce<T extends XY>(inv_dt: number, out: T): T {
        const f = inv_dt * (this.m_impulse + this.m_lowerImpulse - this.m_upperImpulse);
        out.x = f * this.m_u.x;
        out.y = f * this.m_u.y;
        return out;
    }

    public GetReactionTorque(_inv_dt: number): number {
        return 0;
    }

    public GetLocalAnchorA(): Readonly<Vec2> {
        return this.m_localAnchorA;
    }

    public GetLocalAnchorB(): Readonly<Vec2> {
        return this.m_localAnchorB;
    }

    public SetLength(length: number) {
        this.m_impulse = 0;
        this.m_length = Math.max(LINEAR_SLOP, length);
        return this.m_length;
    }

    public GetLength() {
        return this.m_length;
    }

    public SetMinLength(minLength: number) {
        this.m_lowerImpulse = 0;
        this.m_minLength = Clamp(minLength, LINEAR_SLOP, this.m_maxLength);
        return this.m_minLength;
    }

    public GetMinLength() {
        return this.m_minLength;
    }

    public SetMaxLength(maxLength: number) {
        this.m_upperImpulse = 0;
        this.m_maxLength = Math.max(maxLength, this.m_minLength);
        return this.m_maxLength;
    }

    public GetMaxLength() {
        return this.m_maxLength;
    }

    public GetCurrentLength() {
        const pA = this.m_bodyA.GetWorldPoint(this.m_localAnchorA, temp.worldPointA);
        const pB = this.m_bodyB.GetWorldPoint(this.m_localAnchorB, temp.worldPointB);
        return Vec2.Distance(pB, pA);
    }

    public SetStiffness(stiffness: number): void {
        this.m_stiffness = stiffness;
    }

    public GetStiffness() {
        return this.m_stiffness;
    }

    public SetDamping(damping: number): void {
        this.m_damping = damping;
    }

    public GetDamping() {
        return this.m_damping;
    }

    /** @internal protected */
    public InitVelocityConstraints(data: SolverData): void {
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

        Rot.MultiplyVec2(qA, Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), this.m_rA);
        Rot.MultiplyVec2(qB, Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), this.m_rB);
        this.m_u.x = cB.x + this.m_rB.x - cA.x - this.m_rA.x;
        this.m_u.y = cB.y + this.m_rB.y - cA.y - this.m_rA.y;

        // Handle singularity.
        this.m_currentLength = this.m_u.Length();
        if (this.m_currentLength > LINEAR_SLOP) {
            this.m_u.Scale(1 / this.m_currentLength);
        } else {
            this.m_u.SetZero();
            this.m_mass = 0;
            this.m_impulse = 0;
            this.m_lowerImpulse = 0;
            this.m_upperImpulse = 0;
        }

        const crAu = Vec2.Cross(this.m_rA, this.m_u);
        const crBu = Vec2.Cross(this.m_rB, this.m_u);
        let invMass = this.m_invMassA + this.m_invIA * crAu * crAu + this.m_invMassB + this.m_invIB * crBu * crBu;
        this.m_mass = invMass !== 0 ? 1 / invMass : 0;

        if (this.m_stiffness > 0 && this.m_minLength < this.m_maxLength) {
            // soft
            const C = this.m_currentLength - this.m_length;

            const d = this.m_damping;
            const k = this.m_stiffness;

            // magic formulas
            const h = data.step.dt;

            // gamma = 1 / (h * (d + h * k))
            // the extra factor of h in the denominator is since the lambda is an impulse, not a force
            this.m_gamma = h * (d + h * k);
            this.m_gamma = this.m_gamma !== 0 ? 1 / this.m_gamma : 0;
            this.m_bias = C * h * k * this.m_gamma;

            invMass += this.m_gamma;
            this.m_softMass = invMass !== 0 ? 1 / invMass : 0;
        } else {
            // rigid
            this.m_gamma = 0;
            this.m_bias = 0;
            this.m_softMass = this.m_mass;
        }

        if (data.step.warmStarting) {
            // Scale the impulse to support a variable time step.
            this.m_impulse *= data.step.dtRatio;
            this.m_lowerImpulse *= data.step.dtRatio;
            this.m_upperImpulse *= data.step.dtRatio;

            const { P } = temp;
            Vec2.Scale(this.m_impulse + this.m_lowerImpulse - this.m_upperImpulse, this.m_u, P);

            vA.SubtractScaled(this.m_invMassA, P);
            wA -= this.m_invIA * Vec2.Cross(this.m_rA, P);
            vB.AddScaled(this.m_invMassB, P);
            wB += this.m_invIB * Vec2.Cross(this.m_rB, P);
        } else {
            this.m_impulse = 0;
        }

        data.velocities[this.m_indexA].w = wA;
        data.velocities[this.m_indexB].w = wB;
    }

    /** @internal protected */
    public SolveVelocityConstraints(data: SolverData): void {
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;

        if (this.m_minLength < this.m_maxLength) {
            if (this.m_stiffness > 0) {
                // Cdot = dot(u, v + cross(w, r))

                const vpA = Vec2.AddCrossScalarVec2(vA, wA, this.m_rA, temp.vpA);
                const vpB = Vec2.AddCrossScalarVec2(vB, wB, this.m_rB, temp.vpB);

                const Cdot = Vec2.Dot(this.m_u, Vec2.Subtract(vpB, vpA, temp.vpBA));

                const impulse = -this.m_softMass * (Cdot + this.m_bias + this.m_gamma * this.m_impulse);
                this.m_impulse += impulse;

                const P = Vec2.Scale(impulse, this.m_u, temp.P);
                vA.SubtractScaled(this.m_invMassA, P);
                wA -= this.m_invIA * Vec2.Cross(this.m_rA, P);
                vB.AddScaled(this.m_invMassB, P);
                wB += this.m_invIB * Vec2.Cross(this.m_rB, P);
            }

            // lower
            {
                const C = this.m_currentLength - this.m_minLength;
                const bias = Math.max(0, C) * data.step.inv_dt;

                const vpA = Vec2.AddCrossScalarVec2(vA, wA, this.m_rA, temp.vpA);
                const vpB = Vec2.AddCrossScalarVec2(vB, wB, this.m_rB, temp.vpB);
                const Cdot = Vec2.Dot(this.m_u, Vec2.Subtract(vpB, vpA, temp.vpBA));

                let impulse = -this.m_mass * (Cdot + bias);
                const oldImpulse = this.m_lowerImpulse;
                this.m_lowerImpulse = Math.max(0, this.m_lowerImpulse + impulse);
                impulse = this.m_lowerImpulse - oldImpulse;
                const P = Vec2.Scale(impulse, this.m_u, temp.P);

                vA.SubtractScaled(this.m_invMassA, P);
                wA -= this.m_invIA * Vec2.Cross(this.m_rA, P);
                vB.AddScaled(this.m_invMassB, P);
                wB += this.m_invIB * Vec2.Cross(this.m_rB, P);
            }

            // upper
            {
                const C = this.m_maxLength - this.m_currentLength;
                const bias = Math.max(0, C) * data.step.inv_dt;

                const vpA = Vec2.AddCrossScalarVec2(vA, wA, this.m_rA, temp.vpA);
                const vpB = Vec2.AddCrossScalarVec2(vB, wB, this.m_rB, temp.vpB);
                const Cdot = Vec2.Dot(this.m_u, Vec2.Subtract(vpA, vpB, temp.vpBA));

                let impulse = -this.m_mass * (Cdot + bias);
                const oldImpulse = this.m_upperImpulse;
                this.m_upperImpulse = Math.max(0, this.m_upperImpulse + impulse);
                impulse = this.m_upperImpulse - oldImpulse;
                const P = Vec2.Scale(-impulse, this.m_u, temp.P);

                vA.SubtractScaled(this.m_invMassA, P);
                wA -= this.m_invIA * Vec2.Cross(this.m_rA, P);
                vB.AddScaled(this.m_invMassB, P);
                wB += this.m_invIB * Vec2.Cross(this.m_rB, P);
            }
        } else {
            // Equal limits

            // Cdot = dot(u, v + cross(w, r))
            const vpA = Vec2.AddCrossScalarVec2(vA, wA, this.m_rA, temp.vpA);
            const vpB = Vec2.AddCrossScalarVec2(vB, wB, this.m_rB, temp.vpB);
            const Cdot = Vec2.Dot(this.m_u, Vec2.Subtract(vpB, vpA, temp.vpBA));

            const impulse = -this.m_mass * Cdot;
            this.m_impulse += impulse;

            const P = Vec2.Scale(impulse, this.m_u, temp.P);
            vA.SubtractScaled(this.m_invMassA, P);
            wA -= this.m_invIA * Vec2.Cross(this.m_rA, P);
            vB.AddScaled(this.m_invMassB, P);
            wB += this.m_invIB * Vec2.Cross(this.m_rB, P);
        }

        data.velocities[this.m_indexA].w = wA;
        data.velocities[this.m_indexB].w = wB;
    }

    /** @internal protected */
    public SolvePositionConstraints(data: SolverData): boolean {
        const cA = data.positions[this.m_indexA].c;
        let aA = data.positions[this.m_indexA].a;
        const cB = data.positions[this.m_indexB].c;
        let aB = data.positions[this.m_indexB].a;

        const { qA, qB, lalcA, lalcB, P } = temp;
        qA.Set(aA);
        qB.Set(aB);

        const rA = Rot.MultiplyVec2(qA, Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), this.m_rA);
        const rB = Rot.MultiplyVec2(qB, Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), this.m_rB);
        this.m_u.x = cB.x + rB.x - cA.x - rA.x;
        this.m_u.y = cB.y + rB.y - cA.y - rA.y;

        const length = this.m_u.Normalize();
        let C: number;
        if (this.m_minLength === this.m_maxLength) {
            C = length - this.m_minLength;
        } else if (length < this.m_minLength) {
            C = length - this.m_minLength;
        } else if (this.m_maxLength < length) {
            C = length - this.m_maxLength;
        } else {
            return true;
        }

        const impulse = -this.m_mass * C;
        Vec2.Scale(impulse, this.m_u, P);

        cA.SubtractScaled(this.m_invMassA, P);
        aA -= this.m_invIA * Vec2.Cross(rA, P);
        cB.AddScaled(this.m_invMassB, P);
        aB += this.m_invIB * Vec2.Cross(rB, P);

        data.positions[this.m_indexA].a = aA;
        data.positions[this.m_indexB].a = aB;

        return Math.abs(C) < LINEAR_SLOP;
    }

    public Draw(draw: Draw): void {
        const { pA, pB, axis, pRest } = temp.Draw;
        const xfA = this.m_bodyA.GetTransform();
        const xfB = this.m_bodyB.GetTransform();
        Transform.MultiplyVec2(xfA, this.m_localAnchorA, pA);
        Transform.MultiplyVec2(xfB, this.m_localAnchorB, pB);
        Vec2.Subtract(pB, pA, axis);
        axis.Normalize();
        draw.DrawSegment(pA, pB, debugColors.joint5);
        Vec2.AddScaled(pA, this.m_length, axis, pRest);
        draw.DrawPoint(pRest, 8, debugColors.joint1);
        if (this.m_minLength !== this.m_maxLength) {
            if (this.m_minLength > LINEAR_SLOP) {
                const pMin = Vec2.AddScaled(pA, this.m_minLength, axis, temp.Draw.p1);
                draw.DrawPoint(pMin, 4, debugColors.joint2);
            }
            if (this.m_maxLength < MAX_FLOAT) {
                const pMax = Vec2.AddScaled(pA, this.m_maxLength, axis, temp.Draw.p1);
                draw.DrawPoint(pMax, 4, debugColors.joint3);
            }
        }
    }
}
