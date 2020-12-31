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

import { Assert, MakeArray, MakeNumberArray } from "../common/b2_common";
import { Color, Draw, debugColors } from "../common/b2_draw";
import { Vec2, XY } from "../common/b2_math";

const temp = {
    J1: new Vec2(),
    J2: new Vec2(),
    J3: new Vec2(),
    r: new Vec2(),
    e1: new Vec2(),
    e2: new Vec2(),
    Jd1: new Vec2(),
    Jd2: new Vec2(),
    d: new Vec2(),
    u: new Vec2(),
    dp1: new Vec2(),
    dp2: new Vec2(),
    dp3: new Vec2(),
    d1: new Vec2(),
    d2: new Vec2(),
    dHat: new Vec2(),
};

export enum StretchingModel {
    Pbd,
    Xpbd,
}

export enum BendingModel {
    SpringAngle = 0,
    PbdAngle,
    XpbdAngle,
    PbdDistance,
    PbdHeight,
    PbdTriangle,
}

export class RopeTuning {
    public stretchingModel = StretchingModel.Pbd;

    public bendingModel = BendingModel.PbdAngle;

    public damping = 0;

    public stretchStiffness = 1;

    public stretchHertz = 0;

    public stretchDamping = 0;

    public bendStiffness = 0.5;

    public bendHertz = 1;

    public bendDamping = 0;

    public isometric = false;

    public fixedEffectiveMass = false;

    public warmStart = false;

    public Copy(other: Readonly<RopeTuning>) {
        this.stretchingModel = other.stretchingModel;
        this.bendingModel = other.bendingModel;
        this.damping = other.damping;
        this.stretchStiffness = other.stretchStiffness;
        this.stretchHertz = other.stretchHertz;
        this.stretchDamping = other.stretchDamping;
        this.bendStiffness = other.bendStiffness;
        this.bendHertz = other.bendHertz;
        this.bendDamping = other.bendDamping;
        this.isometric = other.isometric;
        this.fixedEffectiveMass = other.fixedEffectiveMass;
        this.warmStart = other.warmStart;
        return this;
    }
}

export interface RopeDef {
    position: XY;

    vertices: XY[];

    masses: number[];

    gravity: XY;

    tuning: RopeTuning;
}

class RopeStretch {
    public i1 = 0;

    public i2 = 0;

    public invMass1 = 0;

    public invMass2 = 0;

    public L = 0;

    public lambda = 0;

    public spring = 0;

    public damper = 0;
}

class RopeBend {
    public i1 = 0;

    public i2 = 0;

    public i3 = 0;

    public invMass1 = 0;

    public invMass2 = 0;

    public invMass3 = 0;

    public invEffectiveMass = 0;

    public lambda = 0;

    public L1 = 0;

    public L2 = 0;

    public alpha1 = 0;

    public alpha2 = 0;

    public spring = 0;

    public damper = 0;
}

export class Rope {
    private readonly m_position = new Vec2();

    private m_count = 0;

    private m_stretchCount = 0;

    private m_bendCount = 0;

    private readonly m_stretchConstraints: RopeStretch[];

    private readonly m_bendConstraints: RopeBend[];

    private readonly m_bindPositions: Vec2[];

    private readonly m_ps: Vec2[];

    private readonly m_p0s: Vec2[];

    private readonly m_vs: Vec2[];

    private readonly m_invMasses: number[];

    private readonly m_gravity = new Vec2();

    private readonly m_tuning = new RopeTuning();

    public constructor(def: RopeDef) {
        Assert(def.vertices.length >= 3);
        this.m_position.Copy(def.position);
        this.m_count = def.vertices.length;
        this.m_bindPositions = MakeArray(this.m_count, Vec2);
        this.m_ps = MakeArray(this.m_count, Vec2);
        this.m_p0s = MakeArray(this.m_count, Vec2);
        this.m_vs = MakeArray(this.m_count, Vec2);
        this.m_invMasses = MakeNumberArray(this.m_count);

        for (let i = 0; i < this.m_count; ++i) {
            this.m_bindPositions[i].Copy(def.vertices[i]);
            Vec2.Add(def.vertices[i], this.m_position, this.m_ps[i]);
            Vec2.Add(def.vertices[i], this.m_position, this.m_p0s[i]);
            this.m_vs[i].SetZero();

            const m = def.masses[i];
            if (m > 0) {
                this.m_invMasses[i] = 1 / m;
            } else {
                this.m_invMasses[i] = 0;
            }
        }

        this.m_stretchCount = this.m_count - 1;
        this.m_bendCount = this.m_count - 2;

        this.m_stretchConstraints = new Array<RopeStretch>(this.m_stretchCount);
        for (let i = 0; i < this.m_stretchCount; i++) this.m_stretchConstraints[i] = new RopeStretch();
        this.m_bendConstraints = new Array<RopeBend>(this.m_bendCount);
        for (let i = 0; i < this.m_bendCount; i++) this.m_bendConstraints[i] = new RopeBend();

        for (let i = 0; i < this.m_stretchCount; ++i) {
            const c = this.m_stretchConstraints[i];

            const p1 = this.m_ps[i];
            const p2 = this.m_ps[i + 1];

            c.i1 = i;
            c.i2 = i + 1;
            c.L = Vec2.Distance(p1, p2);
            c.invMass1 = this.m_invMasses[i];
            c.invMass2 = this.m_invMasses[i + 1];
            c.lambda = 0;
            c.damper = 0;
            c.spring = 0;
        }

        const { J1, J2, r, e1, e2, Jd1, Jd2 } = temp;
        for (let i = 0; i < this.m_bendCount; ++i) {
            const c = this.m_bendConstraints[i];

            const p1 = this.m_ps[i];
            const p2 = this.m_ps[i + 1];
            const p3 = this.m_ps[i + 2];

            c.i1 = i;
            c.i2 = i + 1;
            c.i3 = i + 2;
            c.invMass1 = this.m_invMasses[i];
            c.invMass2 = this.m_invMasses[i + 1];
            c.invMass3 = this.m_invMasses[i + 2];
            c.invEffectiveMass = 0;
            c.L1 = Vec2.Distance(p1, p2);
            c.L2 = Vec2.Distance(p2, p3);
            c.lambda = 0;

            // Pre-compute effective mass (TODO use flattened config)
            Vec2.Subtract(p2, p1, e1);
            Vec2.Subtract(p3, p2, e2);
            const L1sqr = e1.LengthSquared();
            const L2sqr = e2.LengthSquared();

            if (L1sqr * L2sqr === 0) {
                continue;
            }

            Vec2.Skew(e1, Jd1).Scale(-1 / L1sqr);
            Vec2.Skew(e2, Jd2).Scale(1 / L2sqr);

            Vec2.Negate(Jd1, J1);
            Vec2.Subtract(Jd1, Jd2, J2);
            const J3 = Jd2;

            c.invEffectiveMass =
                c.invMass1 * Vec2.Dot(J1, J1) + c.invMass2 * Vec2.Dot(J2, J2) + c.invMass3 * Vec2.Dot(J3, J3);

            Vec2.Subtract(p3, p1, r);

            const rr = r.LengthSquared();
            if (rr === 0) {
                continue;
            }

            // a1 = h2 / (h1 + h2)
            // a2 = h1 / (h1 + h2)
            c.alpha1 = Vec2.Dot(e2, r) / rr;
            c.alpha2 = Vec2.Dot(e1, r) / rr;
        }

        this.m_gravity.Copy(def.gravity);

        this.SetTuning(def.tuning);
    }

    public SetTuning(tuning: RopeTuning): void {
        this.m_tuning.Copy(tuning);

        // Pre-compute spring and damper values based on tuning

        const bendOmega = 2 * Math.PI * this.m_tuning.bendHertz;

        for (let i = 0; i < this.m_bendCount; ++i) {
            const c = this.m_bendConstraints[i];

            const L1sqr = c.L1 * c.L1;
            const L2sqr = c.L2 * c.L2;

            if (L1sqr * L2sqr === 0) {
                c.spring = 0;
                c.damper = 0;
                continue;
            }

            // Flatten the triangle formed by the two edges
            const J2 = 1 / c.L1 + 1 / c.L2;
            const sum = c.invMass1 / L1sqr + c.invMass2 * J2 * J2 + c.invMass3 / L2sqr;
            if (sum === 0) {
                c.spring = 0;
                c.damper = 0;
                continue;
            }

            const mass = 1 / sum;

            c.spring = mass * bendOmega * bendOmega;
            c.damper = 2 * mass * this.m_tuning.bendDamping * bendOmega;
        }

        const stretchOmega = 2 * Math.PI * this.m_tuning.stretchHertz;

        for (let i = 0; i < this.m_stretchCount; ++i) {
            const c = this.m_stretchConstraints[i];

            const sum = c.invMass1 + c.invMass2;
            if (sum === 0) {
                continue;
            }

            const mass = 1 / sum;

            c.spring = mass * stretchOmega * stretchOmega;
            c.damper = 2 * mass * this.m_tuning.stretchDamping * stretchOmega;
        }
    }

    public Step(dt: number, iterations: number, position: Readonly<Vec2>): void {
        if (dt === 0) {
            return;
        }

        const inv_dt = 1 / dt;
        const d = Math.exp(-dt * this.m_tuning.damping);

        // Apply gravity and damping
        for (let i = 0; i < this.m_count; ++i) {
            if (this.m_invMasses[i] > 0) {
                this.m_vs[i].Scale(d);
                this.m_vs[i].AddScaled(dt, this.m_gravity);
            } else {
                this.m_vs[i].x = inv_dt * (this.m_bindPositions[i].x + position.x - this.m_p0s[i].x);
                this.m_vs[i].y = inv_dt * (this.m_bindPositions[i].y + position.y - this.m_p0s[i].y);
            }
        }

        // Apply bending spring
        if (this.m_tuning.bendingModel === BendingModel.SpringAngle) {
            this.ApplyBendForces(dt);
        }

        for (let i = 0; i < this.m_bendCount; ++i) {
            this.m_bendConstraints[i].lambda = 0;
        }

        for (let i = 0; i < this.m_stretchCount; ++i) {
            this.m_stretchConstraints[i].lambda = 0;
        }

        // Update position
        for (let i = 0; i < this.m_count; ++i) {
            this.m_ps[i].AddScaled(dt, this.m_vs[i]);
        }

        // Solve constraints
        for (let i = 0; i < iterations; ++i) {
            if (this.m_tuning.bendingModel === BendingModel.PbdAngle) {
                this.SolveBend_PBD_Angle();
            } else if (this.m_tuning.bendingModel === BendingModel.XpbdAngle) {
                this.SolveBend_XPBD_Angle(dt);
            } else if (this.m_tuning.bendingModel === BendingModel.PbdDistance) {
                this.SolveBend_PBD_Distance();
            } else if (this.m_tuning.bendingModel === BendingModel.PbdHeight) {
                this.SolveBend_PBD_Height();
            } else if (this.m_tuning.bendingModel === BendingModel.PbdTriangle) {
                this.SolveBend_PBD_Triangle();
            }

            if (this.m_tuning.stretchingModel === StretchingModel.Pbd) {
                this.SolveStretch_PBD();
            } else if (this.m_tuning.stretchingModel === StretchingModel.Xpbd) {
                this.SolveStretch_XPBD(dt);
            }
        }

        // Constrain velocity
        for (let i = 0; i < this.m_count; ++i) {
            this.m_vs[i].x = inv_dt * (this.m_ps[i].x - this.m_p0s[i].x);
            this.m_vs[i].y = inv_dt * (this.m_ps[i].y - this.m_p0s[i].y);
            this.m_p0s[i].Copy(this.m_ps[i]);
        }
    }

    public Reset(position: Readonly<Vec2>): void {
        this.m_position.Copy(position);

        for (let i = 0; i < this.m_count; ++i) {
            Vec2.Add(this.m_bindPositions[i], this.m_position, this.m_ps[i]);
            this.m_p0s[i].Copy(this.m_ps[i]);
            this.m_vs[i].SetZero();
        }

        for (let i = 0; i < this.m_bendCount; ++i) {
            this.m_bendConstraints[i].lambda = 0;
        }

        for (let i = 0; i < this.m_stretchCount; ++i) {
            this.m_stretchConstraints[i].lambda = 0;
        }
    }

    private SolveStretch_PBD(): void {
        const stiffness = this.m_tuning.stretchStiffness;

        const { d } = temp;
        for (let i = 0; i < this.m_stretchCount; ++i) {
            const c = this.m_stretchConstraints[i];

            const p1 = this.m_ps[c.i1];
            const p2 = this.m_ps[c.i2];

            Vec2.Subtract(p2, p1, d);
            const L = d.Normalize();

            const sum = c.invMass1 + c.invMass2;
            if (sum === 0) {
                continue;
            }

            const s1 = c.invMass1 / sum;
            const s2 = c.invMass2 / sum;

            p1.SubtractScaled(stiffness * s1 * (c.L - L), d);
            p2.AddScaled(stiffness * s2 * (c.L - L), d);
        }
    }

    private SolveStretch_XPBD(dt: number): void {
        // 	Assert(dt > 0);

        const { dp1, dp2, u, J1 } = temp;
        for (let i = 0; i < this.m_stretchCount; ++i) {
            const c = this.m_stretchConstraints[i];

            const p1 = this.m_ps[c.i1];
            const p2 = this.m_ps[c.i2];

            Vec2.Subtract(p1, this.m_p0s[c.i1], dp1);
            Vec2.Subtract(p2, this.m_p0s[c.i2], dp2);

            Vec2.Subtract(p2, p1, u);
            const L = u.Normalize();

            Vec2.Negate(u, J1);
            const J2 = u;

            const sum = c.invMass1 + c.invMass2;
            if (sum === 0) {
                continue;
            }

            const alpha = 1 / (c.spring * dt * dt); // 1 / kg
            const beta = dt * dt * c.damper; // kg * s
            const sigma = (alpha * beta) / dt; // non-dimensional
            const C = L - c.L;

            // This is using the initial velocities
            const Cdot = Vec2.Dot(J1, dp1) + Vec2.Dot(J2, dp2);

            const B = C + alpha * c.lambda + sigma * Cdot;
            const sum2 = (1 + sigma) * sum + alpha;

            const impulse = -B / sum2;

            p1.AddScaled(c.invMass1 * impulse, J1);
            p2.AddScaled(c.invMass2 * impulse, J2);

            c.lambda += impulse;
        }
    }

    private SolveBend_PBD_Angle(): void {
        const stiffness = this.m_tuning.bendStiffness;

        const { Jd1, Jd2, J1, J2, d1, d2 } = temp;
        for (let i = 0; i < this.m_bendCount; ++i) {
            const c = this.m_bendConstraints[i];

            const p1 = this.m_ps[c.i1];
            const p2 = this.m_ps[c.i2];
            const p3 = this.m_ps[c.i3];

            Vec2.Subtract(p2, p1, d1);
            Vec2.Subtract(p3, p2, d2);
            const a = Vec2.Cross(d1, d2);
            const b = Vec2.Dot(d1, d2);

            const angle = Math.atan2(a, b);

            let L1sqr: number;
            let L2sqr: number;

            if (this.m_tuning.isometric) {
                L1sqr = c.L1 * c.L1;
                L2sqr = c.L2 * c.L2;
            } else {
                L1sqr = d1.LengthSquared();
                L2sqr = d2.LengthSquared();
            }

            if (L1sqr * L2sqr === 0) {
                continue;
            }

            Vec2.Skew(d1, Jd1).Scale(-1 / L1sqr);
            Vec2.Skew(d2, Jd2).Scale(1 / L2sqr);

            Vec2.Negate(Jd1, J1);
            Vec2.Subtract(Jd1, Jd2, J2);
            const J3 = Jd2;

            let sum: number;
            if (this.m_tuning.fixedEffectiveMass) {
                sum = c.invEffectiveMass;
            } else {
                sum = c.invMass1 * Vec2.Dot(J1, J1) + c.invMass2 * Vec2.Dot(J2, J2) + c.invMass3 * Vec2.Dot(J3, J3);
            }

            if (sum === 0) {
                sum = c.invEffectiveMass;
            }

            const impulse = (-stiffness * angle) / sum;

            p1.AddScaled(c.invMass1 * impulse, J1);
            p2.AddScaled(c.invMass2 * impulse, J2);
            p3.AddScaled(c.invMass3 * impulse, J3);
        }
    }

    private SolveBend_XPBD_Angle(dt: number): void {
        // Assert(dt > 0);

        const { dp1, dp2, dp3, d1, d2, Jd1, Jd2, J1, J2 } = temp;
        for (let i = 0; i < this.m_bendCount; ++i) {
            const c = this.m_bendConstraints[i];

            const p1 = this.m_ps[c.i1];
            const p2 = this.m_ps[c.i2];
            const p3 = this.m_ps[c.i3];

            Vec2.Subtract(p1, this.m_p0s[c.i1], dp1);
            Vec2.Subtract(p2, this.m_p0s[c.i2], dp2);
            Vec2.Subtract(p3, this.m_p0s[c.i3], dp3);

            Vec2.Subtract(p2, p1, d1);
            Vec2.Subtract(p3, p2, d2);

            let L1sqr: number;
            let L2sqr: number;

            if (this.m_tuning.isometric) {
                L1sqr = c.L1 * c.L1;
                L2sqr = c.L2 * c.L2;
            } else {
                L1sqr = d1.LengthSquared();
                L2sqr = d2.LengthSquared();
            }

            if (L1sqr * L2sqr === 0) {
                continue;
            }

            const a = Vec2.Cross(d1, d2);
            const b = Vec2.Dot(d1, d2);

            const angle = Math.atan2(a, b);

            Vec2.Skew(d1, Jd1).Scale(-1 / L1sqr);
            Vec2.Skew(d2, Jd2).Scale(1 / L2sqr);

            Vec2.Negate(Jd1, J1);
            Vec2.Subtract(Jd1, Jd2, J2);
            const J3 = Jd2;

            let sum: number;
            if (this.m_tuning.fixedEffectiveMass) {
                sum = c.invEffectiveMass;
            } else {
                sum = c.invMass1 * Vec2.Dot(J1, J1) + c.invMass2 * Vec2.Dot(J2, J2) + c.invMass3 * Vec2.Dot(J3, J3);
            }

            if (sum === 0) {
                continue;
            }

            const alpha = 1 / (c.spring * dt * dt);
            const beta = dt * dt * c.damper;
            const sigma = (alpha * beta) / dt;
            const C = angle;

            // This is using the initial velocities
            const Cdot = Vec2.Dot(J1, dp1) + Vec2.Dot(J2, dp2) + Vec2.Dot(J3, dp3);

            const B = C + alpha * c.lambda + sigma * Cdot;
            const sum2 = (1 + sigma) * sum + alpha;

            const impulse = -B / sum2;

            p1.AddScaled(c.invMass1 * impulse, J1);
            p2.AddScaled(c.invMass2 * impulse, J2);
            p3.AddScaled(c.invMass3 * impulse, J3);

            c.lambda += impulse;
        }
    }

    private SolveBend_PBD_Distance(): void {
        const stiffness = this.m_tuning.bendStiffness;

        const { d } = temp;
        for (let i = 0; i < this.m_bendCount; ++i) {
            const c = this.m_bendConstraints[i];

            const { i1 } = c;
            const i2 = c.i3;

            const p1 = this.m_ps[i1];
            const p2 = this.m_ps[i2];

            Vec2.Subtract(p2, p1, d);
            const L = d.Normalize();

            const sum = c.invMass1 + c.invMass3;
            if (sum === 0) {
                continue;
            }

            const s1 = c.invMass1 / sum;
            const s2 = c.invMass3 / sum;

            p1.SubtractScaled(stiffness * s1 * (c.L1 + c.L2 - L), d);
            p2.AddScaled(stiffness * s2 * (c.L1 + c.L2 - L), d);
        }
    }

    private SolveBend_PBD_Height(): void {
        const stiffness = this.m_tuning.bendStiffness;

        const { dHat, J1, J2, J3, d } = temp;
        for (let i = 0; i < this.m_bendCount; ++i) {
            const c = this.m_bendConstraints[i];

            const p1 = this.m_ps[c.i1];
            const p2 = this.m_ps[c.i2];
            const p3 = this.m_ps[c.i3];

            // Barycentric coordinates are held constant
            d.x = c.alpha1 * p1.x + c.alpha2 * p3.x - p2.x;
            d.y = c.alpha1 * p1.y + c.alpha2 * p3.y - p2.y;
            const dLen = d.Length();

            if (dLen === 0) {
                continue;
            }

            Vec2.Scale(1 / dLen, d, dHat);

            Vec2.Scale(c.alpha1, dHat, J1);
            Vec2.Negate(dHat, J2);
            Vec2.Scale(c.alpha2, dHat, J3);

            const sum = c.invMass1 * c.alpha1 * c.alpha1 + c.invMass2 + c.invMass3 * c.alpha2 * c.alpha2;

            if (sum === 0) {
                continue;
            }

            const C = dLen;
            const mass = 1 / sum;
            const impulse = -stiffness * mass * C;

            p1.AddScaled(c.invMass1 * impulse, J1);
            p2.AddScaled(c.invMass2 * impulse, J2);
            p3.AddScaled(c.invMass3 * impulse, J3);
        }
    }

    private SolveBend_PBD_Triangle(): void {
        const stiffness = this.m_tuning.bendStiffness;

        const { d } = temp;
        for (let i = 0; i < this.m_bendCount; ++i) {
            const c = this.m_bendConstraints[i];

            const b0 = this.m_ps[c.i1];
            const v = this.m_ps[c.i2];
            const b1 = this.m_ps[c.i3];

            const wb0 = c.invMass1;
            const wv = c.invMass2;
            const wb1 = c.invMass3;

            const W = wb0 + wb1 + 2 * wv;
            const invW = stiffness / W;

            d.x = v.x - (1 / 3) * (b0.x + v.x + b1.x);
            d.y = v.y - (1 / 3) * (b0.y + v.y + b1.y);

            b0.AddScaled(2 * wb0 * invW, d);
            v.AddScaled(-4 * wv * invW, d);
            b1.AddScaled(2 * wb1 * invW, d);
        }
    }

    private ApplyBendForces(dt: number): void {
        // omega = 2 * pi * hz
        const omega = 2 * Math.PI * this.m_tuning.bendHertz;

        const { d1, d2, Jd1, Jd2, J1, J2 } = temp;
        for (let i = 0; i < this.m_bendCount; ++i) {
            const c = this.m_bendConstraints[i];

            const p1 = this.m_ps[c.i1];
            const p2 = this.m_ps[c.i2];
            const p3 = this.m_ps[c.i3];

            const v1 = this.m_vs[c.i1];
            const v2 = this.m_vs[c.i2];
            const v3 = this.m_vs[c.i3];

            Vec2.Subtract(p2, p1, d1);
            Vec2.Subtract(p3, p2, d2);

            let L1sqr: number;
            let L2sqr: number;

            if (this.m_tuning.isometric) {
                L1sqr = c.L1 * c.L1;
                L2sqr = c.L2 * c.L2;
            } else {
                L1sqr = d1.LengthSquared();
                L2sqr = d2.LengthSquared();
            }

            if (L1sqr * L2sqr === 0) {
                continue;
            }

            const a = Vec2.Cross(d1, d2);
            const b = Vec2.Dot(d1, d2);

            const angle = Math.atan2(a, b);

            Vec2.Skew(d1, Jd1).Scale(-1 / L1sqr);
            Vec2.Skew(d2, Jd2).Scale(1 / L2sqr);

            Vec2.Negate(Jd1, J1);
            Vec2.Subtract(Jd1, Jd2, J2);
            const J3 = Jd2;

            let sum: number;
            if (this.m_tuning.fixedEffectiveMass) {
                sum = c.invEffectiveMass;
            } else {
                sum = c.invMass1 * Vec2.Dot(J1, J1) + c.invMass2 * Vec2.Dot(J2, J2) + c.invMass3 * Vec2.Dot(J3, J3);
            }

            if (sum === 0) {
                continue;
            }

            const mass = 1 / sum;

            const spring = mass * omega * omega;
            const damper = 2 * mass * this.m_tuning.bendDamping * omega;

            const C = angle;
            const Cdot = Vec2.Dot(J1, v1) + Vec2.Dot(J2, v2) + Vec2.Dot(J3, v3);

            const impulse = -dt * (spring * C + damper * Cdot);

            this.m_vs[c.i1].AddScaled(c.invMass1 * impulse, J1);
            this.m_vs[c.i2].AddScaled(c.invMass2 * impulse, J2);
            this.m_vs[c.i3].AddScaled(c.invMass3 * impulse, J3);
        }
    }

    public Draw(draw: Draw): void {
        for (let i = 0; i < this.m_count - 1; ++i) {
            draw.DrawSegment(this.m_ps[i], this.m_ps[i + 1], debugColors.rope);

            const pc: Readonly<Color> = this.m_invMasses[i] > 0 ? debugColors.ropePointD : debugColors.ropePointG;
            draw.DrawPoint(this.m_ps[i], 5, pc);
        }

        const pc: Readonly<Color> =
            this.m_invMasses[this.m_count - 1] > 0 ? debugColors.ropePointD : debugColors.ropePointG;
        draw.DrawPoint(this.m_ps[this.m_count - 1], 5, pc);
    }
}
