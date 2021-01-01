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

// DEBUG: import { assert } from "../common/b2_common";
import { EPSILON, LINEAR_SLOP, MAX_LINEAR_CORRECTION, makeNumberArray, makeArray } from "../common/b2_common";
import { Vec2, XY } from "../common/b2_math";
import { Joint, JointDef, JointType, IJointDef } from "./b2_joint";
import { DistanceJoint, DistanceJointDef } from "./b2_distance_joint";
import { SolverData } from "./b2_time_step";
import type { Body } from "./b2_body";

export interface IAreaJointDef extends IJointDef {
    bodies: Body[];

    stiffness?: number;

    damping?: number;
}

export class AreaJointDef extends JointDef implements IAreaJointDef {
    public bodies: Body[] = [];

    public stiffness = 0;

    public damping = 0;

    public constructor() {
        super(JointType.Area);
    }

    public addBody(body: Body): void {
        this.bodies.push(body);

        if (this.bodies.length === 1) {
            this.bodyA = body;
        } else if (this.bodies.length === 2) {
            this.bodyB = body;
        }
    }
}

export class AreaJoint extends Joint {
    public m_bodies: Body[];

    public m_stiffness = 0;

    public m_damping = 0;

    // Solver shared
    public m_impulse = 0;

    // Solver temp
    public readonly m_targetLengths: number[];

    public m_targetArea = 0;

    public readonly m_normals: Vec2[];

    public readonly m_joints: DistanceJoint[] = [];

    public readonly m_deltas: Vec2[];

    public readonly m_delta = new Vec2();

    public constructor(def: IAreaJointDef) {
        super(def);

        // DEBUG: assert(def.bodies.length >= 3, "You cannot create an area joint with less than three bodies.");

        this.m_bodies = def.bodies;
        this.m_stiffness = def.stiffness ?? 0;
        this.m_damping = def.damping ?? 0;

        this.m_targetLengths = makeNumberArray(def.bodies.length);
        this.m_normals = makeArray(def.bodies.length, Vec2);
        this.m_deltas = makeArray(def.bodies.length, Vec2);

        const djd = new DistanceJointDef();
        djd.stiffness = this.m_stiffness;
        djd.damping = this.m_damping;

        this.m_targetArea = 0;

        for (let i = 0; i < this.m_bodies.length; ++i) {
            const body = this.m_bodies[i];
            const next = this.m_bodies[(i + 1) % this.m_bodies.length];

            const body_c = body.getWorldCenter();
            const next_c = next.getWorldCenter();

            this.m_targetLengths[i] = Vec2.distance(body_c, next_c);

            this.m_targetArea += Vec2.cross(body_c, next_c);

            djd.initialize(body, next, body_c, next_c);
            this.m_joints[i] = body.getWorld().createJoint(djd);
        }

        this.m_targetArea *= 0.5;
    }

    public getAnchorA<T extends XY>(out: T): T {
        return out;
    }

    public getAnchorB<T extends XY>(out: T): T {
        return out;
    }

    public getReactionForce<T extends XY>(inv_dt: number, out: T): T {
        return out;
    }

    public getReactionTorque(_inv_dt: number): number {
        return 0;
    }

    public setStiffness(stiffness: number): void {
        this.m_stiffness = stiffness;

        for (const joint of this.m_joints) {
            joint.setStiffness(stiffness);
        }
    }

    public getStiffness() {
        return this.m_stiffness;
    }

    public setDamping(damping: number): void {
        this.m_damping = damping;

        for (const joint of this.m_joints) {
            joint.setDamping(damping);
        }
    }

    public getDamping() {
        return this.m_damping;
    }

    public initVelocityConstraints(data: SolverData): void {
        for (let i = 0; i < this.m_bodies.length; ++i) {
            const prev = this.m_bodies[(i + this.m_bodies.length - 1) % this.m_bodies.length];
            const next = this.m_bodies[(i + 1) % this.m_bodies.length];
            const prev_c = data.positions[prev.m_islandIndex].c;
            const next_c = data.positions[next.m_islandIndex].c;
            const delta = this.m_deltas[i];

            Vec2.subtract(next_c, prev_c, delta);
        }

        if (data.step.warmStarting) {
            this.m_impulse *= data.step.dtRatio;

            for (let i = 0; i < this.m_bodies.length; ++i) {
                const body = this.m_bodies[i];
                const body_v = data.velocities[body.m_islandIndex].v;
                const delta = this.m_deltas[i];

                body_v.x += body.m_invMass * delta.y * 0.5 * this.m_impulse;
                body_v.y += body.m_invMass * -delta.x * 0.5 * this.m_impulse;
            }
        } else {
            this.m_impulse = 0;
        }
    }

    public solveVelocityConstraints(data: SolverData): void {
        let dotMassSum = 0;
        let crossMassSum = 0;

        for (let i = 0; i < this.m_bodies.length; ++i) {
            const body = this.m_bodies[i];
            const body_v = data.velocities[body.m_islandIndex].v;
            const delta = this.m_deltas[i];

            dotMassSum += delta.lengthSquared() / body.getMass();
            crossMassSum += Vec2.cross(body_v, delta);
        }

        const lambda = (-2 * crossMassSum) / dotMassSum;
        // lambda = clamp(lambda, -MAX_LINEAR_CORRECTION, MAX_LINEAR_CORRECTION);

        this.m_impulse += lambda;

        for (let i = 0; i < this.m_bodies.length; ++i) {
            const body = this.m_bodies[i];
            const body_v = data.velocities[body.m_islandIndex].v;
            const delta = this.m_deltas[i];

            body_v.x += body.m_invMass * delta.y * 0.5 * lambda;
            body_v.y += body.m_invMass * -delta.x * 0.5 * lambda;
        }
    }

    public solvePositionConstraints(data: SolverData): boolean {
        let perimeter = 0;
        let area = 0;

        for (let i = 0; i < this.m_bodies.length; ++i) {
            const body = this.m_bodies[i];
            const next = this.m_bodies[(i + 1) % this.m_bodies.length];
            const body_c = data.positions[body.m_islandIndex].c;
            const next_c = data.positions[next.m_islandIndex].c;

            const delta = Vec2.subtract(next_c, body_c, this.m_delta);

            let dist = delta.length();
            if (dist < EPSILON) {
                dist = 1;
            }

            this.m_normals[i].x = delta.y / dist;
            this.m_normals[i].y = -delta.x / dist;

            perimeter += dist;

            area += Vec2.cross(body_c, next_c);
        }

        area *= 0.5;

        const deltaArea = this.m_targetArea - area;
        const toExtrude = (0.5 * deltaArea) / perimeter;
        let done = true;

        for (let i = 0; i < this.m_bodies.length; ++i) {
            const body = this.m_bodies[i];
            const body_c = data.positions[body.m_islandIndex].c;
            const next_i = (i + 1) % this.m_bodies.length;

            const delta = Vec2.add(this.m_normals[i], this.m_normals[next_i], this.m_delta);
            delta.scale(toExtrude);

            const norm_sq = delta.lengthSquared();
            if (norm_sq > MAX_LINEAR_CORRECTION ** 2) {
                delta.scale(MAX_LINEAR_CORRECTION / Math.sqrt(norm_sq));
            }
            if (norm_sq > LINEAR_SLOP ** 2) {
                done = false;
            }

            body_c.add(delta);
        }

        return done;
    }
}
