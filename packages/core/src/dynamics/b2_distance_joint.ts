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

import { b2_linearSlop, b2_maxFloat } from "../common/b2_common";
import { b2Clamp, b2Vec2, b2Rot, XY, b2Transform } from "../common/b2_math";
import { b2Joint, b2JointDef, b2JointType, b2IJointDef } from "./b2_joint";
import { b2SolverData } from "./b2_time_step";
import type { b2Body } from "./b2_body";
import { b2Draw, debugColors } from "../common/b2_draw";
import { b2Readonly } from "../common/b2_readonly";

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2
// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k *
// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

const temp = {
    worldPointA: new b2Vec2(),
    worldPointB: new b2Vec2(),
    vpA: new b2Vec2(),
    vpB: new b2Vec2(),
    vpBA: new b2Vec2(),
    P: new b2Vec2(),
    qA: new b2Rot(),
    qB: new b2Rot(),
    lalcA: new b2Vec2(),
    lalcB: new b2Vec2(),
    Draw: {
        pA: new b2Vec2(),
        pB: new b2Vec2(),
        axis: new b2Vec2(),
        pRest: new b2Vec2(),
        p1: new b2Vec2(),
        p2: new b2Vec2(),
    },
};

export interface b2IDistanceJointDef extends b2IJointDef {
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
export class b2DistanceJointDef extends b2JointDef implements b2IDistanceJointDef {
    /** The local anchor point relative to bodyA's origin. */
    public readonly localAnchorA = new b2Vec2();

    /** The local anchor point relative to bodyB's origin. */
    public readonly localAnchorB = new b2Vec2();

    /** The rest length of this joint. Clamped to a stable minimum value. */
    public length = 1;

    /** Minimum length. Clamped to a stable minimum value. */
    public minLength = 0;

    /** Maximum length. Must be greater than or equal to the minimum length. */
    public maxLength = b2_maxFloat;

    /** The linear stiffness in N/m. */
    public stiffness = 0;

    /** The linear damping in N*s/m. */
    public damping = 0;

    public constructor() {
        super(b2JointType.e_distanceJoint);
    }

    /**
     * Initialize the bodies, anchors, and rest length using world space anchors.
     * The minimum and maximum lengths are set to the rest length.
     */
    public Initialize(b1: b2Body, b2: b2Body, anchor1: XY, anchor2: XY): void {
        this.bodyA = b1;
        this.bodyB = b2;
        this.bodyA.GetLocalPoint(anchor1, this.localAnchorA);
        this.bodyB.GetLocalPoint(anchor2, this.localAnchorB);
        this.length = Math.max(b2Vec2.Distance(anchor1, anchor2), b2_linearSlop);
        this.minLength = this.length;
        this.maxLength = this.length;
    }
}

/**
 * A distance joint constrains two points on two bodies to remain at a fixed
 * distance from each other. You can view this as a massless, rigid rod.
 */
export class b2DistanceJoint extends b2Joint {
    protected m_stiffness: number;

    protected m_damping: number;

    protected m_bias = 0;

    protected m_length: number;

    protected m_minLength: number;

    protected m_maxLength: number;

    // Solver shared
    protected readonly m_localAnchorA = new b2Vec2();

    protected readonly m_localAnchorB = new b2Vec2();

    protected m_gamma = 0;

    protected m_impulse = 0;

    protected m_lowerImpulse = 0;

    protected m_upperImpulse = 0;

    // Solver temp
    protected m_indexA = 0;

    protected m_indexB = 0;

    protected readonly m_u = new b2Vec2();

    protected readonly m_rA = new b2Vec2();

    protected readonly m_rB = new b2Vec2();

    protected readonly m_localCenterA = new b2Vec2();

    protected readonly m_localCenterB = new b2Vec2();

    protected m_currentLength = 0;

    protected m_invMassA = 0;

    protected m_invMassB = 0;

    protected m_invIA = 0;

    protected m_invIB = 0;

    protected m_softMass = 0;

    protected m_mass = 0;

    /** @internal protected */
    public constructor(def: b2IDistanceJointDef) {
        super(def);

        this.m_localAnchorA.Copy(def.localAnchorA);
        this.m_localAnchorB.Copy(def.localAnchorB);
        this.m_length = Math.max(def.length, b2_linearSlop);
        this.m_minLength = Math.max(def.minLength, b2_linearSlop);
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

    /**
     * Get the reaction force given the inverse time step.
     * Unit is N.
     */
    public GetReactionForce<T extends XY>(inv_dt: number, out: T): T {
        const f = inv_dt * (this.m_impulse + this.m_lowerImpulse - this.m_upperImpulse);
        out.x = f * this.m_u.x;
        out.y = f * this.m_u.y;
        return out;
    }

    /**
     * Get the reaction torque given the inverse time step.
     * Unit is N*m. This is always zero for a distance joint.
     */
    public GetReactionTorque(_inv_dt: number): number {
        return 0;
    }

    /** The local anchor point relative to bodyA's origin. */
    public GetLocalAnchorA(): b2Readonly<b2Vec2> {
        return this.m_localAnchorA;
    }

    /** The local anchor point relative to bodyB's origin. */
    public GetLocalAnchorB(): b2Readonly<b2Vec2> {
        return this.m_localAnchorB;
    }

    /**
     * Set the rest length
     * @returns clamped rest length
     */
    public SetLength(length: number) {
        this.m_impulse = 0;
        this.m_length = Math.max(b2_linearSlop, length);
        return this.m_length;
    }

    /** Get the rest length */
    public GetLength() {
        return this.m_length;
    }

    /**
     * Set the minimum length
     * @returns the clamped minimum length
     */
    public SetMinLength(minLength: number) {
        this.m_lowerImpulse = 0;
        this.m_minLength = b2Clamp(minLength, b2_linearSlop, this.m_maxLength);
        return this.m_minLength;
    }

    /** Get the minimum length */
    public GetMinLength() {
        return this.m_minLength;
    }

    /**
     * Set the maximum length
     * @returns the clamped maximum length
     */
    public SetMaxLength(maxLength: number) {
        this.m_upperImpulse = 0;
        this.m_maxLength = Math.max(maxLength, this.m_minLength);
        return this.m_maxLength;
    }

    /** Get the maximum length */
    public GetMaxLength() {
        return this.m_maxLength;
    }

    /** Get the current length */
    public GetCurrentLength() {
        const pA = this.m_bodyA.GetWorldPoint(this.m_localAnchorA, temp.worldPointA);
        const pB = this.m_bodyB.GetWorldPoint(this.m_localAnchorB, temp.worldPointB);
        return b2Vec2.Distance(pB, pA);
    }

    /** Set the linear stiffness in N/m */
    public SetStiffness(stiffness: number): void {
        this.m_stiffness = stiffness;
    }

    /** Get the linear stiffness in N/m */
    public GetStiffness() {
        return this.m_stiffness;
    }

    /** Set linear damping in N*s/m */
    public SetDamping(damping: number): void {
        this.m_damping = damping;
    }

    /** Get linear damping in N*s/m */
    public GetDamping() {
        return this.m_damping;
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
        this.m_u.x = cB.x + this.m_rB.x - cA.x - this.m_rA.x;
        this.m_u.y = cB.y + this.m_rB.y - cA.y - this.m_rA.y;

        // Handle singularity.
        this.m_currentLength = this.m_u.Length();
        if (this.m_currentLength > b2_linearSlop) {
            this.m_u.Scale(1 / this.m_currentLength);
        } else {
            this.m_u.SetZero();
            this.m_mass = 0;
            this.m_impulse = 0;
            this.m_lowerImpulse = 0;
            this.m_upperImpulse = 0;
        }

        const crAu = b2Vec2.Cross(this.m_rA, this.m_u);
        const crBu = b2Vec2.Cross(this.m_rB, this.m_u);
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
            b2Vec2.Scale(this.m_impulse + this.m_lowerImpulse - this.m_upperImpulse, this.m_u, P);

            vA.SubtractScaled(this.m_invMassA, P);
            wA -= this.m_invIA * b2Vec2.Cross(this.m_rA, P);
            vB.AddScaled(this.m_invMassB, P);
            wB += this.m_invIB * b2Vec2.Cross(this.m_rB, P);
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

        if (this.m_minLength < this.m_maxLength) {
            if (this.m_stiffness > 0) {
                // Cdot = dot(u, v + cross(w, r))

                const vpA = b2Vec2.AddCrossScalarVec2(vA, wA, this.m_rA, temp.vpA);
                const vpB = b2Vec2.AddCrossScalarVec2(vB, wB, this.m_rB, temp.vpB);

                const Cdot = b2Vec2.Dot(this.m_u, b2Vec2.Subtract(vpB, vpA, temp.vpBA));

                const impulse = -this.m_softMass * (Cdot + this.m_bias + this.m_gamma * this.m_impulse);
                this.m_impulse += impulse;

                const P = b2Vec2.Scale(impulse, this.m_u, temp.P);
                vA.SubtractScaled(this.m_invMassA, P);
                wA -= this.m_invIA * b2Vec2.Cross(this.m_rA, P);
                vB.AddScaled(this.m_invMassB, P);
                wB += this.m_invIB * b2Vec2.Cross(this.m_rB, P);
            }

            // lower
            {
                const C = this.m_currentLength - this.m_minLength;
                const bias = Math.max(0, C) * data.step.inv_dt;

                const vpA = b2Vec2.AddCrossScalarVec2(vA, wA, this.m_rA, temp.vpA);
                const vpB = b2Vec2.AddCrossScalarVec2(vB, wB, this.m_rB, temp.vpB);
                const Cdot = b2Vec2.Dot(this.m_u, b2Vec2.Subtract(vpB, vpA, temp.vpBA));

                let impulse = -this.m_mass * (Cdot + bias);
                const oldImpulse = this.m_lowerImpulse;
                this.m_lowerImpulse = Math.max(0, this.m_lowerImpulse + impulse);
                impulse = this.m_lowerImpulse - oldImpulse;
                const P = b2Vec2.Scale(impulse, this.m_u, temp.P);

                vA.SubtractScaled(this.m_invMassA, P);
                wA -= this.m_invIA * b2Vec2.Cross(this.m_rA, P);
                vB.AddScaled(this.m_invMassB, P);
                wB += this.m_invIB * b2Vec2.Cross(this.m_rB, P);
            }

            // upper
            {
                const C = this.m_maxLength - this.m_currentLength;
                const bias = Math.max(0, C) * data.step.inv_dt;

                const vpA = b2Vec2.AddCrossScalarVec2(vA, wA, this.m_rA, temp.vpA);
                const vpB = b2Vec2.AddCrossScalarVec2(vB, wB, this.m_rB, temp.vpB);
                const Cdot = b2Vec2.Dot(this.m_u, b2Vec2.Subtract(vpA, vpB, temp.vpBA));

                let impulse = -this.m_mass * (Cdot + bias);
                const oldImpulse = this.m_upperImpulse;
                this.m_upperImpulse = Math.max(0, this.m_upperImpulse + impulse);
                impulse = this.m_upperImpulse - oldImpulse;
                const P = b2Vec2.Scale(-impulse, this.m_u, temp.P);

                vA.SubtractScaled(this.m_invMassA, P);
                wA -= this.m_invIA * b2Vec2.Cross(this.m_rA, P);
                vB.AddScaled(this.m_invMassB, P);
                wB += this.m_invIB * b2Vec2.Cross(this.m_rB, P);
            }
        } else {
            // Equal limits

            // Cdot = dot(u, v + cross(w, r))
            const vpA = b2Vec2.AddCrossScalarVec2(vA, wA, this.m_rA, temp.vpA);
            const vpB = b2Vec2.AddCrossScalarVec2(vB, wB, this.m_rB, temp.vpB);
            const Cdot = b2Vec2.Dot(this.m_u, b2Vec2.Subtract(vpB, vpA, temp.vpBA));

            const impulse = -this.m_mass * Cdot;
            this.m_impulse += impulse;

            const P = b2Vec2.Scale(impulse, this.m_u, temp.P);
            vA.SubtractScaled(this.m_invMassA, P);
            wA -= this.m_invIA * b2Vec2.Cross(this.m_rA, P);
            vB.AddScaled(this.m_invMassB, P);
            wB += this.m_invIB * b2Vec2.Cross(this.m_rB, P);
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

        const { qA, qB, lalcA, lalcB, P } = temp;
        qA.Set(aA);
        qB.Set(aB);

        const rA = b2Rot.MultiplyVec2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA, lalcA), this.m_rA);
        const rB = b2Rot.MultiplyVec2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), this.m_rB);
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
        b2Vec2.Scale(impulse, this.m_u, P);

        cA.SubtractScaled(this.m_invMassA, P);
        aA -= this.m_invIA * b2Vec2.Cross(rA, P);
        cB.AddScaled(this.m_invMassB, P);
        aB += this.m_invIB * b2Vec2.Cross(rB, P);

        data.positions[this.m_indexA].a = aA;
        data.positions[this.m_indexB].a = aB;

        return Math.abs(C) < b2_linearSlop;
    }

    public override Draw(draw: b2Draw): void {
        const { pA, pB, axis, pRest } = temp.Draw;
        const xfA = this.m_bodyA.GetTransform();
        const xfB = this.m_bodyB.GetTransform();
        b2Transform.MultiplyVec2(xfA, this.m_localAnchorA, pA);
        b2Transform.MultiplyVec2(xfB, this.m_localAnchorB, pB);
        b2Vec2.Subtract(pB, pA, axis);
        axis.Normalize();
        draw.DrawSegment(pA, pB, debugColors.joint5);
        b2Vec2.AddScaled(pA, this.m_length, axis, pRest);
        draw.DrawPoint(pRest, 8, debugColors.joint1);
        if (this.m_minLength !== this.m_maxLength) {
            if (this.m_minLength > b2_linearSlop) {
                const pMin = b2Vec2.AddScaled(pA, this.m_minLength, axis, temp.Draw.p1);
                draw.DrawPoint(pMin, 4, debugColors.joint2);
            }
            if (this.m_maxLength < b2_maxFloat) {
                const pMax = b2Vec2.AddScaled(pA, this.m_maxLength, axis, temp.Draw.p1);
                draw.DrawPoint(pMax, 4, debugColors.joint3);
            }
        }
    }
}
