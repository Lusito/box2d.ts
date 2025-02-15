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
import { b2Draw, debugColors } from "../common/b2_draw";
import { b2Vec2, b2Mat22, b2Rot, b2Transform, XY } from "../common/b2_math";
import { b2Joint, b2JointDef, b2JointType, b2IJointDef } from "./b2_joint";
import { b2SolverData } from "./b2_time_step";

// p = attached point, m = mouse point
// C = p - m
// Cdot = v
//      = v + cross(w, r)
// J = [I r_skew]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

const temp = {
    qB: new b2Rot(),
    lalcB: new b2Vec2(),
    Cdot: new b2Vec2(),
    impulse: new b2Vec2(),
    oldImpulse: new b2Vec2(),
    pA: new b2Vec2(),
    pB: new b2Vec2(),
};

export interface b2IMouseJointDef extends b2IJointDef {
    target?: XY;

    maxForce?: number;

    stiffness?: number;

    damping?: number;
}

/**
 * Mouse joint definition. This requires a world target point,
 * tuning parameters, and the time step.
 */
export class b2MouseJointDef extends b2JointDef implements b2IMouseJointDef {
    /**
     * The initial world target point. This is assumed
     * to coincide with the body anchor initially.
     */
    public readonly target = new b2Vec2();

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
        super(b2JointType.e_mouseJoint);
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
export class b2MouseJoint extends b2Joint {
    protected readonly m_localAnchorB = new b2Vec2();

    protected readonly m_targetA = new b2Vec2();

    protected m_stiffness = 0;

    protected m_damping = 0;

    protected m_beta = 0;

    // Solver shared
    protected readonly m_impulse = new b2Vec2();

    protected m_maxForce = 0;

    protected m_gamma = 0;

    // Solver temp
    protected m_indexB = 0;

    protected readonly m_rB = new b2Vec2();

    protected readonly m_localCenterB = new b2Vec2();

    protected m_invMassB = 0;

    protected m_invIB = 0;

    protected readonly m_mass = new b2Mat22();

    protected readonly m_C = new b2Vec2();

    /** @internal protected */
    public constructor(def: b2IMouseJointDef) {
        super(def);

        this.m_targetA.Copy(def.target ?? b2Vec2.ZERO);
        b2Transform.TransposeMultiplyVec2(this.m_bodyB.GetTransform(), this.m_targetA, this.m_localAnchorB);
        this.m_maxForce = def.maxForce ?? 0;
        this.m_stiffness = def.stiffness ?? 0;
        this.m_damping = def.damping ?? 0;

        this.m_beta = 0;
        this.m_gamma = 0;
    }

    /** Use this to update the target point. */
    public SetTarget(target: XY): void {
        if (!b2Vec2.Equals(target, this.m_targetA)) {
            this.m_bodyB.SetAwake(true);
            this.m_targetA.Copy(target);
        }
    }

    public GetTarget() {
        return this.m_targetA;
    }

    /** Set the maximum force in Newtons. */
    public SetMaxForce(force: number): void {
        this.m_maxForce = force;
    }

    /** Get the maximum force in Newtons. */
    public GetMaxForce() {
        return this.m_maxForce;
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
    public SetDamping(damping: number) {
        this.m_damping = damping;
    }

    /** Get linear damping in N*s/m */
    public GetDamping() {
        return this.m_damping;
    }

    /** @internal protected */
    public InitVelocityConstraints(data: b2SolverData): void {
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterB.Copy(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIB = this.m_bodyB.m_invI;

        const cB = data.positions[this.m_indexB].c;
        const aB = data.positions[this.m_indexB].a;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;

        const { qB, lalcB } = temp;

        qB.Set(aB);

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
        b2Rot.MultiplyVec2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB, lalcB), this.m_rB);

        // K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
        //      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
        //        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
        const K = this.m_mass;
        K.ex.x = this.m_invMassB + this.m_invIB * this.m_rB.y * this.m_rB.y + this.m_gamma;
        K.ex.y = -this.m_invIB * this.m_rB.x * this.m_rB.y;
        K.ey.x = K.ex.y;
        K.ey.y = this.m_invMassB + this.m_invIB * this.m_rB.x * this.m_rB.x + this.m_gamma;

        K.Inverse();

        b2Vec2.Add(cB, this.m_rB, this.m_C).Subtract(this.m_targetA);
        this.m_C.Scale(this.m_beta);

        // Cheat with some damping
        wB *= Math.max(0, 1 - 0.02 * (60 * data.step.dt));

        if (data.step.warmStarting) {
            this.m_impulse.Scale(data.step.dtRatio);
            vB.AddScaled(this.m_invMassB, this.m_impulse);
            wB += this.m_invIB * b2Vec2.Cross(this.m_rB, this.m_impulse);
        } else {
            this.m_impulse.SetZero();
        }
        data.velocities[this.m_indexB].w = wB;
    }

    /** @internal protected */
    public SolveVelocityConstraints(data: b2SolverData): void {
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;

        // Cdot = v + cross(w, r)
        const { Cdot, impulse, oldImpulse } = temp;
        b2Vec2.AddCrossScalarVec2(vB, wB, this.m_rB, Cdot);
        b2Mat22.MultiplyVec2(
            this.m_mass,
            b2Vec2.Add(Cdot, this.m_C, impulse).AddScaled(this.m_gamma, this.m_impulse).Negate(),
            impulse,
        );

        oldImpulse.Copy(this.m_impulse);
        this.m_impulse.Add(impulse);
        const maxImpulse = data.step.dt * this.m_maxForce;
        if (this.m_impulse.LengthSquared() > maxImpulse * maxImpulse) {
            this.m_impulse.Scale(maxImpulse / this.m_impulse.Length());
        }
        b2Vec2.Subtract(this.m_impulse, oldImpulse, impulse);

        vB.AddScaled(this.m_invMassB, impulse);
        wB += this.m_invIB * b2Vec2.Cross(this.m_rB, impulse);

        data.velocities[this.m_indexB].w = wB;
    }

    /** @internal protected */
    public SolvePositionConstraints(_data: b2SolverData): boolean {
        return true;
    }

    public GetAnchorA<T extends XY>(out: T): T {
        out.x = this.m_targetA.x;
        out.y = this.m_targetA.y;
        return out;
    }

    public GetAnchorB<T extends XY>(out: T): T {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }

    public GetReactionForce<T extends XY>(inv_dt: number, out: T): T {
        return b2Vec2.Scale(inv_dt, this.m_impulse, out);
    }

    public GetReactionTorque(_inv_dt: number): number {
        return 0;
    }

    public override ShiftOrigin(newOrigin: XY) {
        this.m_targetA.Subtract(newOrigin);
    }

    public override Draw(draw: b2Draw): void {
        const p1 = this.GetAnchorA(temp.pA);
        const p2 = this.GetAnchorB(temp.pB);
        draw.DrawPoint(p1, 4, debugColors.joint7);
        draw.DrawPoint(p2, 4, debugColors.joint7);
        draw.DrawSegment(p1, p2, debugColors.joint8);
    }
}
