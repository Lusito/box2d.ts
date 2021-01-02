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

import { assert, makeArray, makeNumberArray } from "../common/b2_common";
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

    public copy(other: Readonly<RopeTuning>) {
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
    private readonly position = new Vec2();

    private count = 0;

    private stretchCount = 0;

    private bendCount = 0;

    private readonly stretchConstraints: RopeStretch[];

    private readonly bendConstraints: RopeBend[];

    private readonly bindPositions: Vec2[];

    private readonly ps: Vec2[];

    private readonly p0s: Vec2[];

    private readonly vs: Vec2[];

    private readonly invMasses: number[];

    private readonly gravity = new Vec2();

    private readonly tuning = new RopeTuning();

    public constructor(def: RopeDef) {
        assert(def.vertices.length >= 3);
        this.position.copy(def.position);
        this.count = def.vertices.length;
        this.bindPositions = makeArray(this.count, Vec2);
        this.ps = makeArray(this.count, Vec2);
        this.p0s = makeArray(this.count, Vec2);
        this.vs = makeArray(this.count, Vec2);
        this.invMasses = makeNumberArray(this.count);

        for (let i = 0; i < this.count; ++i) {
            this.bindPositions[i].copy(def.vertices[i]);
            Vec2.add(def.vertices[i], this.position, this.ps[i]);
            Vec2.add(def.vertices[i], this.position, this.p0s[i]);
            this.vs[i].setZero();

            const m = def.masses[i];
            if (m > 0) {
                this.invMasses[i] = 1 / m;
            } else {
                this.invMasses[i] = 0;
            }
        }

        this.stretchCount = this.count - 1;
        this.bendCount = this.count - 2;

        this.stretchConstraints = new Array<RopeStretch>(this.stretchCount);
        for (let i = 0; i < this.stretchCount; i++) this.stretchConstraints[i] = new RopeStretch();
        this.bendConstraints = new Array<RopeBend>(this.bendCount);
        for (let i = 0; i < this.bendCount; i++) this.bendConstraints[i] = new RopeBend();

        for (let i = 0; i < this.stretchCount; ++i) {
            const c = this.stretchConstraints[i];

            const p1 = this.ps[i];
            const p2 = this.ps[i + 1];

            c.i1 = i;
            c.i2 = i + 1;
            c.L = Vec2.distance(p1, p2);
            c.invMass1 = this.invMasses[i];
            c.invMass2 = this.invMasses[i + 1];
            c.lambda = 0;
            c.damper = 0;
            c.spring = 0;
        }

        const { J1, J2, r, e1, e2, Jd1, Jd2 } = temp;
        for (let i = 0; i < this.bendCount; ++i) {
            const c = this.bendConstraints[i];

            const p1 = this.ps[i];
            const p2 = this.ps[i + 1];
            const p3 = this.ps[i + 2];

            c.i1 = i;
            c.i2 = i + 1;
            c.i3 = i + 2;
            c.invMass1 = this.invMasses[i];
            c.invMass2 = this.invMasses[i + 1];
            c.invMass3 = this.invMasses[i + 2];
            c.invEffectiveMass = 0;
            c.L1 = Vec2.distance(p1, p2);
            c.L2 = Vec2.distance(p2, p3);
            c.lambda = 0;

            // Pre-compute effective mass (TODO use flattened config)
            Vec2.subtract(p2, p1, e1);
            Vec2.subtract(p3, p2, e2);
            const L1sqr = e1.lengthSquared();
            const L2sqr = e2.lengthSquared();

            if (L1sqr * L2sqr === 0) {
                continue;
            }

            Vec2.skew(e1, Jd1).scale(-1 / L1sqr);
            Vec2.skew(e2, Jd2).scale(1 / L2sqr);

            Vec2.negate(Jd1, J1);
            Vec2.subtract(Jd1, Jd2, J2);
            const J3 = Jd2;

            c.invEffectiveMass =
                c.invMass1 * Vec2.dot(J1, J1) + c.invMass2 * Vec2.dot(J2, J2) + c.invMass3 * Vec2.dot(J3, J3);

            Vec2.subtract(p3, p1, r);

            const rr = r.lengthSquared();
            if (rr === 0) {
                continue;
            }

            // a1 = h2 / (h1 + h2)
            // a2 = h1 / (h1 + h2)
            c.alpha1 = Vec2.dot(e2, r) / rr;
            c.alpha2 = Vec2.dot(e1, r) / rr;
        }

        this.gravity.copy(def.gravity);

        this.setTuning(def.tuning);
    }

    public setTuning(tuning: RopeTuning): void {
        this.tuning.copy(tuning);

        // Pre-compute spring and damper values based on tuning

        const bendOmega = 2 * Math.PI * this.tuning.bendHertz;

        for (let i = 0; i < this.bendCount; ++i) {
            const c = this.bendConstraints[i];

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
            c.damper = 2 * mass * this.tuning.bendDamping * bendOmega;
        }

        const stretchOmega = 2 * Math.PI * this.tuning.stretchHertz;

        for (let i = 0; i < this.stretchCount; ++i) {
            const c = this.stretchConstraints[i];

            const sum = c.invMass1 + c.invMass2;
            if (sum === 0) {
                continue;
            }

            const mass = 1 / sum;

            c.spring = mass * stretchOmega * stretchOmega;
            c.damper = 2 * mass * this.tuning.stretchDamping * stretchOmega;
        }
    }

    public step(dt: number, iterations: number, position: Readonly<Vec2>): void {
        if (dt === 0) {
            return;
        }

        const inv_dt = 1 / dt;
        const d = Math.exp(-dt * this.tuning.damping);

        // Apply gravity and damping
        for (let i = 0; i < this.count; ++i) {
            if (this.invMasses[i] > 0) {
                this.vs[i].scale(d);
                this.vs[i].addScaled(dt, this.gravity);
            } else {
                this.vs[i].x = inv_dt * (this.bindPositions[i].x + position.x - this.p0s[i].x);
                this.vs[i].y = inv_dt * (this.bindPositions[i].y + position.y - this.p0s[i].y);
            }
        }

        // Apply bending spring
        if (this.tuning.bendingModel === BendingModel.SpringAngle) {
            this.applyBendForces(dt);
        }

        for (let i = 0; i < this.bendCount; ++i) {
            this.bendConstraints[i].lambda = 0;
        }

        for (let i = 0; i < this.stretchCount; ++i) {
            this.stretchConstraints[i].lambda = 0;
        }

        // Update position
        for (let i = 0; i < this.count; ++i) {
            this.ps[i].addScaled(dt, this.vs[i]);
        }

        // Solve constraints
        for (let i = 0; i < iterations; ++i) {
            if (this.tuning.bendingModel === BendingModel.PbdAngle) {
                this.solveBend_PBD_Angle();
            } else if (this.tuning.bendingModel === BendingModel.XpbdAngle) {
                this.solveBend_XPBD_Angle(dt);
            } else if (this.tuning.bendingModel === BendingModel.PbdDistance) {
                this.solveBend_PBD_Distance();
            } else if (this.tuning.bendingModel === BendingModel.PbdHeight) {
                this.solveBend_PBD_Height();
            } else if (this.tuning.bendingModel === BendingModel.PbdTriangle) {
                this.solveBend_PBD_Triangle();
            }

            if (this.tuning.stretchingModel === StretchingModel.Pbd) {
                this.solveStretch_PBD();
            } else if (this.tuning.stretchingModel === StretchingModel.Xpbd) {
                this.solveStretch_XPBD(dt);
            }
        }

        // Constrain velocity
        for (let i = 0; i < this.count; ++i) {
            this.vs[i].x = inv_dt * (this.ps[i].x - this.p0s[i].x);
            this.vs[i].y = inv_dt * (this.ps[i].y - this.p0s[i].y);
            this.p0s[i].copy(this.ps[i]);
        }
    }

    public reset(position: Readonly<Vec2>): void {
        this.position.copy(position);

        for (let i = 0; i < this.count; ++i) {
            Vec2.add(this.bindPositions[i], this.position, this.ps[i]);
            this.p0s[i].copy(this.ps[i]);
            this.vs[i].setZero();
        }

        for (let i = 0; i < this.bendCount; ++i) {
            this.bendConstraints[i].lambda = 0;
        }

        for (let i = 0; i < this.stretchCount; ++i) {
            this.stretchConstraints[i].lambda = 0;
        }
    }

    private solveStretch_PBD(): void {
        const stiffness = this.tuning.stretchStiffness;

        const { d } = temp;
        for (let i = 0; i < this.stretchCount; ++i) {
            const c = this.stretchConstraints[i];

            const p1 = this.ps[c.i1];
            const p2 = this.ps[c.i2];

            Vec2.subtract(p2, p1, d);
            const L = d.normalize();

            const sum = c.invMass1 + c.invMass2;
            if (sum === 0) {
                continue;
            }

            const s1 = c.invMass1 / sum;
            const s2 = c.invMass2 / sum;

            p1.subtractScaled(stiffness * s1 * (c.L - L), d);
            p2.addScaled(stiffness * s2 * (c.L - L), d);
        }
    }

    private solveStretch_XPBD(dt: number): void {
        // 	assert(dt > 0);

        const { dp1, dp2, u, J1 } = temp;
        for (let i = 0; i < this.stretchCount; ++i) {
            const c = this.stretchConstraints[i];

            const p1 = this.ps[c.i1];
            const p2 = this.ps[c.i2];

            Vec2.subtract(p1, this.p0s[c.i1], dp1);
            Vec2.subtract(p2, this.p0s[c.i2], dp2);

            Vec2.subtract(p2, p1, u);
            const L = u.normalize();

            Vec2.negate(u, J1);
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
            const Cdot = Vec2.dot(J1, dp1) + Vec2.dot(J2, dp2);

            const B = C + alpha * c.lambda + sigma * Cdot;
            const sum2 = (1 + sigma) * sum + alpha;

            const impulse = -B / sum2;

            p1.addScaled(c.invMass1 * impulse, J1);
            p2.addScaled(c.invMass2 * impulse, J2);

            c.lambda += impulse;
        }
    }

    private solveBend_PBD_Angle(): void {
        const stiffness = this.tuning.bendStiffness;

        const { Jd1, Jd2, J1, J2, d1, d2 } = temp;
        for (let i = 0; i < this.bendCount; ++i) {
            const c = this.bendConstraints[i];

            const p1 = this.ps[c.i1];
            const p2 = this.ps[c.i2];
            const p3 = this.ps[c.i3];

            Vec2.subtract(p2, p1, d1);
            Vec2.subtract(p3, p2, d2);
            const a = Vec2.cross(d1, d2);
            const b = Vec2.dot(d1, d2);

            const angle = Math.atan2(a, b);

            let L1sqr: number;
            let L2sqr: number;

            if (this.tuning.isometric) {
                L1sqr = c.L1 * c.L1;
                L2sqr = c.L2 * c.L2;
            } else {
                L1sqr = d1.lengthSquared();
                L2sqr = d2.lengthSquared();
            }

            if (L1sqr * L2sqr === 0) {
                continue;
            }

            Vec2.skew(d1, Jd1).scale(-1 / L1sqr);
            Vec2.skew(d2, Jd2).scale(1 / L2sqr);

            Vec2.negate(Jd1, J1);
            Vec2.subtract(Jd1, Jd2, J2);
            const J3 = Jd2;

            let sum: number;
            if (this.tuning.fixedEffectiveMass) {
                sum = c.invEffectiveMass;
            } else {
                sum = c.invMass1 * Vec2.dot(J1, J1) + c.invMass2 * Vec2.dot(J2, J2) + c.invMass3 * Vec2.dot(J3, J3);
            }

            if (sum === 0) {
                sum = c.invEffectiveMass;
            }

            const impulse = (-stiffness * angle) / sum;

            p1.addScaled(c.invMass1 * impulse, J1);
            p2.addScaled(c.invMass2 * impulse, J2);
            p3.addScaled(c.invMass3 * impulse, J3);
        }
    }

    private solveBend_XPBD_Angle(dt: number): void {
        // assert(dt > 0);

        const { dp1, dp2, dp3, d1, d2, Jd1, Jd2, J1, J2 } = temp;
        for (let i = 0; i < this.bendCount; ++i) {
            const c = this.bendConstraints[i];

            const p1 = this.ps[c.i1];
            const p2 = this.ps[c.i2];
            const p3 = this.ps[c.i3];

            Vec2.subtract(p1, this.p0s[c.i1], dp1);
            Vec2.subtract(p2, this.p0s[c.i2], dp2);
            Vec2.subtract(p3, this.p0s[c.i3], dp3);

            Vec2.subtract(p2, p1, d1);
            Vec2.subtract(p3, p2, d2);

            let L1sqr: number;
            let L2sqr: number;

            if (this.tuning.isometric) {
                L1sqr = c.L1 * c.L1;
                L2sqr = c.L2 * c.L2;
            } else {
                L1sqr = d1.lengthSquared();
                L2sqr = d2.lengthSquared();
            }

            if (L1sqr * L2sqr === 0) {
                continue;
            }

            const a = Vec2.cross(d1, d2);
            const b = Vec2.dot(d1, d2);

            const angle = Math.atan2(a, b);

            Vec2.skew(d1, Jd1).scale(-1 / L1sqr);
            Vec2.skew(d2, Jd2).scale(1 / L2sqr);

            Vec2.negate(Jd1, J1);
            Vec2.subtract(Jd1, Jd2, J2);
            const J3 = Jd2;

            let sum: number;
            if (this.tuning.fixedEffectiveMass) {
                sum = c.invEffectiveMass;
            } else {
                sum = c.invMass1 * Vec2.dot(J1, J1) + c.invMass2 * Vec2.dot(J2, J2) + c.invMass3 * Vec2.dot(J3, J3);
            }

            if (sum === 0) {
                continue;
            }

            const alpha = 1 / (c.spring * dt * dt);
            const beta = dt * dt * c.damper;
            const sigma = (alpha * beta) / dt;
            const C = angle;

            // This is using the initial velocities
            const Cdot = Vec2.dot(J1, dp1) + Vec2.dot(J2, dp2) + Vec2.dot(J3, dp3);

            const B = C + alpha * c.lambda + sigma * Cdot;
            const sum2 = (1 + sigma) * sum + alpha;

            const impulse = -B / sum2;

            p1.addScaled(c.invMass1 * impulse, J1);
            p2.addScaled(c.invMass2 * impulse, J2);
            p3.addScaled(c.invMass3 * impulse, J3);

            c.lambda += impulse;
        }
    }

    private solveBend_PBD_Distance(): void {
        const stiffness = this.tuning.bendStiffness;

        const { d } = temp;
        for (let i = 0; i < this.bendCount; ++i) {
            const c = this.bendConstraints[i];

            const { i1 } = c;
            const i2 = c.i3;

            const p1 = this.ps[i1];
            const p2 = this.ps[i2];

            Vec2.subtract(p2, p1, d);
            const L = d.normalize();

            const sum = c.invMass1 + c.invMass3;
            if (sum === 0) {
                continue;
            }

            const s1 = c.invMass1 / sum;
            const s2 = c.invMass3 / sum;

            p1.subtractScaled(stiffness * s1 * (c.L1 + c.L2 - L), d);
            p2.addScaled(stiffness * s2 * (c.L1 + c.L2 - L), d);
        }
    }

    private solveBend_PBD_Height(): void {
        const stiffness = this.tuning.bendStiffness;

        const { dHat, J1, J2, J3, d } = temp;
        for (let i = 0; i < this.bendCount; ++i) {
            const c = this.bendConstraints[i];

            const p1 = this.ps[c.i1];
            const p2 = this.ps[c.i2];
            const p3 = this.ps[c.i3];

            // Barycentric coordinates are held constant
            d.x = c.alpha1 * p1.x + c.alpha2 * p3.x - p2.x;
            d.y = c.alpha1 * p1.y + c.alpha2 * p3.y - p2.y;
            const dLen = d.length();

            if (dLen === 0) {
                continue;
            }

            Vec2.scale(1 / dLen, d, dHat);

            Vec2.scale(c.alpha1, dHat, J1);
            Vec2.negate(dHat, J2);
            Vec2.scale(c.alpha2, dHat, J3);

            const sum = c.invMass1 * c.alpha1 * c.alpha1 + c.invMass2 + c.invMass3 * c.alpha2 * c.alpha2;

            if (sum === 0) {
                continue;
            }

            const C = dLen;
            const mass = 1 / sum;
            const impulse = -stiffness * mass * C;

            p1.addScaled(c.invMass1 * impulse, J1);
            p2.addScaled(c.invMass2 * impulse, J2);
            p3.addScaled(c.invMass3 * impulse, J3);
        }
    }

    private solveBend_PBD_Triangle(): void {
        const stiffness = this.tuning.bendStiffness;

        const { d } = temp;
        for (let i = 0; i < this.bendCount; ++i) {
            const c = this.bendConstraints[i];

            const b0 = this.ps[c.i1];
            const v = this.ps[c.i2];
            const b1 = this.ps[c.i3];

            const wb0 = c.invMass1;
            const wv = c.invMass2;
            const wb1 = c.invMass3;

            const W = wb0 + wb1 + 2 * wv;
            const invW = stiffness / W;

            d.x = v.x - (1 / 3) * (b0.x + v.x + b1.x);
            d.y = v.y - (1 / 3) * (b0.y + v.y + b1.y);

            b0.addScaled(2 * wb0 * invW, d);
            v.addScaled(-4 * wv * invW, d);
            b1.addScaled(2 * wb1 * invW, d);
        }
    }

    private applyBendForces(dt: number): void {
        // omega = 2 * pi * hz
        const omega = 2 * Math.PI * this.tuning.bendHertz;

        const { d1, d2, Jd1, Jd2, J1, J2 } = temp;
        for (let i = 0; i < this.bendCount; ++i) {
            const c = this.bendConstraints[i];

            const p1 = this.ps[c.i1];
            const p2 = this.ps[c.i2];
            const p3 = this.ps[c.i3];

            const v1 = this.vs[c.i1];
            const v2 = this.vs[c.i2];
            const v3 = this.vs[c.i3];

            Vec2.subtract(p2, p1, d1);
            Vec2.subtract(p3, p2, d2);

            let L1sqr: number;
            let L2sqr: number;

            if (this.tuning.isometric) {
                L1sqr = c.L1 * c.L1;
                L2sqr = c.L2 * c.L2;
            } else {
                L1sqr = d1.lengthSquared();
                L2sqr = d2.lengthSquared();
            }

            if (L1sqr * L2sqr === 0) {
                continue;
            }

            const a = Vec2.cross(d1, d2);
            const b = Vec2.dot(d1, d2);

            const angle = Math.atan2(a, b);

            Vec2.skew(d1, Jd1).scale(-1 / L1sqr);
            Vec2.skew(d2, Jd2).scale(1 / L2sqr);

            Vec2.negate(Jd1, J1);
            Vec2.subtract(Jd1, Jd2, J2);
            const J3 = Jd2;

            let sum: number;
            if (this.tuning.fixedEffectiveMass) {
                sum = c.invEffectiveMass;
            } else {
                sum = c.invMass1 * Vec2.dot(J1, J1) + c.invMass2 * Vec2.dot(J2, J2) + c.invMass3 * Vec2.dot(J3, J3);
            }

            if (sum === 0) {
                continue;
            }

            const mass = 1 / sum;

            const spring = mass * omega * omega;
            const damper = 2 * mass * this.tuning.bendDamping * omega;

            const C = angle;
            const Cdot = Vec2.dot(J1, v1) + Vec2.dot(J2, v2) + Vec2.dot(J3, v3);

            const impulse = -dt * (spring * C + damper * Cdot);

            this.vs[c.i1].addScaled(c.invMass1 * impulse, J1);
            this.vs[c.i2].addScaled(c.invMass2 * impulse, J2);
            this.vs[c.i3].addScaled(c.invMass3 * impulse, J3);
        }
    }

    public draw(draw: Draw): void {
        for (let i = 0; i < this.count - 1; ++i) {
            draw.drawSegment(this.ps[i], this.ps[i + 1], debugColors.rope);

            const pc: Readonly<Color> = this.invMasses[i] > 0 ? debugColors.ropePointD : debugColors.ropePointG;
            draw.drawPoint(this.ps[i], 5, pc);
        }

        const pc: Readonly<Color> =
            this.invMasses[this.count - 1] > 0 ? debugColors.ropePointD : debugColors.ropePointG;
        draw.drawPoint(this.ps[this.count - 1], 5, pc);
    }
}
