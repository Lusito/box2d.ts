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
    public bodies: Body[];

    public stiffness = 0;

    public damping = 0;

    // Solver shared
    public impulse = 0;

    // Solver temp
    public readonly targetLengths: number[];

    public targetArea = 0;

    public readonly normals: Vec2[];

    public readonly joints: DistanceJoint[] = [];

    public readonly deltas: Vec2[];

    public readonly delta = new Vec2();

    public constructor(def: IAreaJointDef) {
        super(def);

        // DEBUG: assert(def.bodies.length >= 3, "You cannot create an area joint with less than three bodies.");

        this.bodies = def.bodies;
        this.stiffness = def.stiffness ?? 0;
        this.damping = def.damping ?? 0;

        this.targetLengths = makeNumberArray(def.bodies.length);
        this.normals = makeArray(def.bodies.length, Vec2);
        this.deltas = makeArray(def.bodies.length, Vec2);

        const djd = new DistanceJointDef();
        djd.stiffness = this.stiffness;
        djd.damping = this.damping;

        this.targetArea = 0;

        for (let i = 0; i < this.bodies.length; ++i) {
            const body = this.bodies[i];
            const next = this.bodies[(i + 1) % this.bodies.length];

            const body_c = body.getWorldCenter();
            const next_c = next.getWorldCenter();

            this.targetLengths[i] = Vec2.distance(body_c, next_c);

            this.targetArea += Vec2.cross(body_c, next_c);

            djd.initialize(body, next, body_c, next_c);
            this.joints[i] = body.getWorld().createJoint(djd);
        }

        this.targetArea *= 0.5;
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
        this.stiffness = stiffness;

        for (const joint of this.joints) {
            joint.setStiffness(stiffness);
        }
    }

    public getStiffness() {
        return this.stiffness;
    }

    public setDamping(damping: number): void {
        this.damping = damping;

        for (const joint of this.joints) {
            joint.setDamping(damping);
        }
    }

    public getDamping() {
        return this.damping;
    }

    public initVelocityConstraints(data: SolverData): void {
        for (let i = 0; i < this.bodies.length; ++i) {
            const prev = this.bodies[(i + this.bodies.length - 1) % this.bodies.length];
            const next = this.bodies[(i + 1) % this.bodies.length];
            const prev_c = data.positions[prev.islandIndex].c;
            const next_c = data.positions[next.islandIndex].c;
            const delta = this.deltas[i];

            Vec2.subtract(next_c, prev_c, delta);
        }

        if (data.step.warmStarting) {
            this.impulse *= data.step.dtRatio;

            for (let i = 0; i < this.bodies.length; ++i) {
                const body = this.bodies[i];
                const body_v = data.velocities[body.islandIndex].v;
                const delta = this.deltas[i];

                body_v.x += body.invMass * delta.y * 0.5 * this.impulse;
                body_v.y += body.invMass * -delta.x * 0.5 * this.impulse;
            }
        } else {
            this.impulse = 0;
        }
    }

    public solveVelocityConstraints(data: SolverData): void {
        let dotMassSum = 0;
        let crossMassSum = 0;

        for (let i = 0; i < this.bodies.length; ++i) {
            const body = this.bodies[i];
            const body_v = data.velocities[body.islandIndex].v;
            const delta = this.deltas[i];

            dotMassSum += delta.lengthSquared() / body.getMass();
            crossMassSum += Vec2.cross(body_v, delta);
        }

        const lambda = (-2 * crossMassSum) / dotMassSum;
        // lambda = clamp(lambda, -MAX_LINEAR_CORRECTION, MAX_LINEAR_CORRECTION);

        this.impulse += lambda;

        for (let i = 0; i < this.bodies.length; ++i) {
            const body = this.bodies[i];
            const body_v = data.velocities[body.islandIndex].v;
            const delta = this.deltas[i];

            body_v.x += body.invMass * delta.y * 0.5 * lambda;
            body_v.y += body.invMass * -delta.x * 0.5 * lambda;
        }
    }

    public solvePositionConstraints(data: SolverData): boolean {
        let perimeter = 0;
        let area = 0;

        for (let i = 0; i < this.bodies.length; ++i) {
            const body = this.bodies[i];
            const next = this.bodies[(i + 1) % this.bodies.length];
            const body_c = data.positions[body.islandIndex].c;
            const next_c = data.positions[next.islandIndex].c;

            const delta = Vec2.subtract(next_c, body_c, this.delta);

            let dist = delta.length();
            if (dist < EPSILON) {
                dist = 1;
            }

            this.normals[i].x = delta.y / dist;
            this.normals[i].y = -delta.x / dist;

            perimeter += dist;

            area += Vec2.cross(body_c, next_c);
        }

        area *= 0.5;

        const deltaArea = this.targetArea - area;
        const toExtrude = (0.5 * deltaArea) / perimeter;
        let done = true;

        for (let i = 0; i < this.bodies.length; ++i) {
            const body = this.bodies[i];
            const body_c = data.positions[body.islandIndex].c;
            const next_i = (i + 1) % this.bodies.length;

            const delta = Vec2.add(this.normals[i], this.normals[next_i], this.delta);
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
