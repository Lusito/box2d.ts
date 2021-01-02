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
import {
    MAX_FLOAT,
    TIME_TO_SLEEP,
    MAX_TRANSLATION,
    MAX_TRANSLATION_SQUARED,
    MAX_ROTATION,
    MAX_ROTATION_SQUARED,
    LINEAR_SLEEP_TOLERANCE,
    ANGULAR_SLEEP_TOLERANCE,
    makeArray,
} from "../common/b2_common";
import { Vec2 } from "../common/b2_math";
import { Timer } from "../common/b2_timer";
import { Contact } from "./b2_contact";
import { ContactSolver, ContactSolverDef, ContactVelocityConstraint } from "./b2_contact_solver";
import { Joint } from "./b2_joint";
import { Body, BodyType } from "./b2_body";
import { TimeStep, Profile, SolverData, Position, Velocity } from "./b2_time_step";
import { ContactImpulse, ContactListener } from "./b2_world_callbacks";

/*
Position Correction Notes
=========================
I tried the several algorithms for position correction of the 2D revolute joint.
I looked at these systems:
- simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
- suspension bridge with 30 1m long planks of length 1m.
- multi-link chain with 30 1m long links.

Here are the algorithms:

Baumgarte - A fraction of the position error is added to the velocity error. There is no
separate position solver.

Pseudo Velocities - After the velocity solver and position integration,
the position error, Jacobian, and effective mass are recomputed. Then
the velocity constraints are solved with pseudo velocities and a fraction
of the position error is added to the pseudo velocity error. The pseudo
velocities are initialized to zero and there is no warm-starting. After
the position solver, the pseudo velocities are added to the positions.
This is also called the First Order World method or the Position LCP method.

Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
position error is re-computed for each constraint and the positions are updated
after the constraint is solved. The radius vectors (aka Jacobians) are
re-computed too (otherwise the algorithm has horrible instability). The pseudo
velocity states are not needed because they are effectively zero at the beginning
of each iteration. Since we have the current position error, we allow the
iterations to terminate early if the error becomes smaller than LINEAR_SLOP.

Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
each time a constraint is solved.

Here are the results:
Baumgarte - this is the cheapest algorithm but it has some stability problems,
especially with the bridge. The chain links separate easily close to the root
and they jitter as they struggle to pull together. This is one of the most common
methods in the field. The big drawback is that the position correction artificially
affects the momentum, thus leading to instabilities and false bounce. I used a
bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
factor makes joints and contacts more spongy.

Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
stable. However, joints still separate with large angular velocities. Drag the
simple pendulum in a circle quickly and the joint will separate. The chain separates
easily and does not recover. I used a bias factor of 0.2. A larger value lead to
the bridge collapsing when a heavy cube drops on it.

Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
Velocities, but in other ways it is worse. The bridge and chain are much more
stable, but the simple pendulum goes unstable at high angular velocities.

Full NGS - stable in all tests. The joints display good stiffness. The bridge
still sags, but this is better than infinite forces.

Recommendations
Pseudo Velocities are not really worthwhile because the bridge and chain cannot
recover from joint separation. In other cases the benefit over Baumgarte is small.

Modified NGS is not a robust method for the revolute joint due to the violent
instability seen in the simple pendulum. Perhaps it is viable with other constraint
types, especially scalar constraints where the effective mass is a scalar.

This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
and is very fast. I don't think we can escape Baumgarte, especially in highly
demanding cases where high constraint fidelity is not needed.

Full NGS is robust and easy on the eyes. I recommend this as an option for
higher fidelity simulation and certainly for suspension bridges and long chains.
Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
joint separation can be problematic. The number of NGS iterations can be reduced
for better performance without harming robustness much.

Each joint in a can be handled differently in the position solver. So I recommend
a system where the user can select the algorithm on a per joint basis. I would
probably default to the slower Full NGS and let the user select the faster
Baumgarte method in performance critical scenarios.
*/

/*
Cache Performance

The Box2D solvers are dominated by cache misses. Data structures are designed
to increase the number of cache hits. Much of misses are due to random access
to body data. The constraint structures are iterated over linearly, which leads
to few cache misses.

The bodies are not accessed during iteration. Instead read only data, such as
the mass values are stored with the constraints. The mutable data are the constraint
impulses and the bodies velocities/positions. The impulses are held inside the
constraint structures. The body velocities/positions are held in compact, temporary
arrays to increase the number of cache hits. Linear and angular velocity are
stored in a single array since multiple arrays lead to multiple misses.
*/

/*
2D Rotation

R = [cos(theta) -sin(theta)]
    [sin(theta) cos(theta) ]

thetaDot = omega

Let q1 = cos(theta), q2 = sin(theta).
R = [q1 -q2]
    [q2  q1]

q1Dot = -thetaDot * q2
q2Dot = thetaDot * q1

q1_new = q1_old - dt * w * q2
q2_new = q2_old + dt * w * q1
then normalize.

This might be faster than computing sin+cos.
However, we can compute sin+cos of the same angle fast.
*/

/** @internal */
export class Island {
    public listener: ContactListener;

    public readonly bodies: Body[];

    public readonly contacts: Contact[];

    public readonly joints: Joint[];

    public readonly positions: Position[];

    public readonly velocities: Velocity[];

    public bodyCount = 0;

    public jointCount = 0;

    public contactCount = 0;

    public bodyCapacity: number;

    public constructor(
        bodyCapacity: number,
        contactCapacity: number,
        jointCapacity: number,
        listener: ContactListener,
    ) {
        this.bodyCapacity = bodyCapacity;

        this.listener = listener;

        this.bodies = new Array(bodyCapacity);
        this.contacts = new Array(contactCapacity);
        this.joints = new Array(jointCapacity);

        this.velocities = makeArray(bodyCapacity, Velocity);
        this.positions = makeArray(bodyCapacity, Position);

        this.resize(bodyCapacity);
    }

    public resize(bodyCapacity: number) {
        while (this.bodyCapacity < bodyCapacity) {
            this.velocities[this.bodyCapacity] = new Velocity();
            this.positions[this.bodyCapacity] = new Position();
            this.bodyCapacity++;
        }
    }

    public clear(): void {
        this.bodyCount = 0;
        this.contactCount = 0;
        this.jointCount = 0;
    }

    public addBody(body: Body): void {
        // DEBUG: assert(this.bodyCount < this.bodyCapacity);
        body.islandIndex = this.bodyCount;
        this.bodies[this.bodyCount] = body;
        ++this.bodyCount;
    }

    public addContact(contact: Contact): void {
        this.contacts[this.contactCount++] = contact;
    }

    public addJoint(joint: Joint): void {
        this.joints[this.jointCount++] = joint;
    }

    private static s_timer = new Timer();

    private static s_solverData = new SolverData();

    private static s_contactSolverDef = new ContactSolverDef();

    private static s_contactSolver = new ContactSolver();

    private static s_translation = new Vec2();

    public solve(profile: Profile, step: TimeStep, gravity: Vec2, allowSleep: boolean): void {
        const timer = Island.s_timer.reset();

        const h = step.dt;

        // Integrate velocities and apply damping. Initialize the body state.
        for (let i = 0; i < this.bodyCount; ++i) {
            const b = this.bodies[i];

            this.positions[i].c.copy(b.sweep.c);
            const { a } = b.sweep;
            const v = this.velocities[i].v.copy(b.linearVelocity);
            let w = b.angularVelocity;

            // Store positions for continuous collision.
            b.sweep.c0.copy(b.sweep.c);
            b.sweep.a0 = b.sweep.a;

            if (b.type === BodyType.Dynamic) {
                // Integrate velocities.
                v.x += h * b.invMass * (b.gravityScale * b.mass * gravity.x + b.force.x);
                v.y += h * b.invMass * (b.gravityScale * b.mass * gravity.y + b.force.y);
                w += h * b.invI * b.torque;

                // Apply damping.
                // ODE: dv/dt + c * v = 0
                // Solution: v(t) = v0 * exp(-c * t)
                // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
                // v2 = exp(-c * dt) * v1
                // Pade approximation:
                // v2 = v1 * 1 / (1 + c * dt)
                v.scale(1 / (1 + h * b.linearDamping));
                w *= 1 / (1 + h * b.angularDamping);
            }

            this.positions[i].a = a;
            this.velocities[i].w = w;
        }

        timer.reset();

        // Solver data
        const solverData = Island.s_solverData;
        solverData.step.copy(step);
        solverData.positions = this.positions;
        solverData.velocities = this.velocities;

        // Initialize velocity constraints.
        const contactSolverDef = Island.s_contactSolverDef;
        contactSolverDef.step.copy(step);
        contactSolverDef.contacts = this.contacts;
        contactSolverDef.count = this.contactCount;
        contactSolverDef.positions = this.positions;
        contactSolverDef.velocities = this.velocities;

        const contactSolver = Island.s_contactSolver.initialize(contactSolverDef);
        contactSolver.initializeVelocityConstraints();

        if (step.warmStarting) {
            contactSolver.warmStart();
        }

        for (let i = 0; i < this.jointCount; ++i) {
            this.joints[i].initVelocityConstraints(solverData);
        }

        profile.solveInit = timer.getMilliseconds();

        // Solve velocity constraints.
        timer.reset();
        for (let i = 0; i < step.config.velocityIterations; ++i) {
            for (let j = 0; j < this.jointCount; ++j) {
                this.joints[j].solveVelocityConstraints(solverData);
            }

            contactSolver.solveVelocityConstraints();
        }

        // Store impulses for warm starting
        contactSolver.storeImpulses();
        profile.solveVelocity = timer.getMilliseconds();

        // Integrate positions.
        for (let i = 0; i < this.bodyCount; ++i) {
            const { c } = this.positions[i];
            let { a } = this.positions[i];
            const { v } = this.velocities[i];
            let { w } = this.velocities[i];

            // Check for large velocities
            const translation = Vec2.scale(h, v, Island.s_translation);
            if (Vec2.dot(translation, translation) > MAX_TRANSLATION_SQUARED) {
                const ratio = MAX_TRANSLATION / translation.length();
                v.scale(ratio);
            }

            const rotation = h * w;
            if (rotation * rotation > MAX_ROTATION_SQUARED) {
                const ratio = MAX_ROTATION / Math.abs(rotation);
                w *= ratio;
            }

            // Integrate
            c.addScaled(h, v);
            a += h * w;

            this.positions[i].a = a;
            this.velocities[i].w = w;
        }

        // Solve position constraints
        timer.reset();
        let positionSolved = false;
        for (let i = 0; i < step.config.positionIterations; ++i) {
            const contactsOkay = contactSolver.solvePositionConstraints();

            let jointsOkay = true;
            for (let j = 0; j < this.jointCount; ++j) {
                const jointOkay = this.joints[j].solvePositionConstraints(solverData);
                jointsOkay = jointsOkay && jointOkay;
            }

            if (contactsOkay && jointsOkay) {
                // Exit early if the position errors are small.
                positionSolved = true;
                break;
            }
        }

        // Copy state buffers back to the bodies
        for (let i = 0; i < this.bodyCount; ++i) {
            const body = this.bodies[i];
            body.sweep.c.copy(this.positions[i].c);
            body.sweep.a = this.positions[i].a;
            body.linearVelocity.copy(this.velocities[i].v);
            body.angularVelocity = this.velocities[i].w;
            body.synchronizeTransform();
        }

        profile.solvePosition = timer.getMilliseconds();

        this.report(contactSolver.velocityConstraints);

        if (allowSleep) {
            let minSleepTime = MAX_FLOAT;

            const linTolSqr = LINEAR_SLEEP_TOLERANCE * LINEAR_SLEEP_TOLERANCE;
            const angTolSqr = ANGULAR_SLEEP_TOLERANCE * ANGULAR_SLEEP_TOLERANCE;

            for (let i = 0; i < this.bodyCount; ++i) {
                const b = this.bodies[i];
                if (b.getType() === BodyType.Static) {
                    continue;
                }

                if (
                    !b.autoSleepFlag ||
                    b.angularVelocity * b.angularVelocity > angTolSqr ||
                    Vec2.dot(b.linearVelocity, b.linearVelocity) > linTolSqr
                ) {
                    b.sleepTime = 0;
                    minSleepTime = 0;
                } else {
                    b.sleepTime += h;
                    minSleepTime = Math.min(minSleepTime, b.sleepTime);
                }
            }

            if (minSleepTime >= TIME_TO_SLEEP && positionSolved) {
                for (let i = 0; i < this.bodyCount; ++i) {
                    const b = this.bodies[i];
                    b.setAwake(false);
                }
            }
        }
    }

    public solveTOI(subStep: TimeStep, toiIndexA: number, toiIndexB: number): void {
        // DEBUG: assert(toiIndexA < this.bodyCount);
        // DEBUG: assert(toiIndexB < this.bodyCount);

        // Initialize the body state.
        for (let i = 0; i < this.bodyCount; ++i) {
            const b = this.bodies[i];
            this.positions[i].c.copy(b.sweep.c);
            this.positions[i].a = b.sweep.a;
            this.velocities[i].v.copy(b.linearVelocity);
            this.velocities[i].w = b.angularVelocity;
        }

        const contactSolverDef = Island.s_contactSolverDef;
        contactSolverDef.contacts = this.contacts;
        contactSolverDef.count = this.contactCount;
        contactSolverDef.step.copy(subStep);
        contactSolverDef.positions = this.positions;
        contactSolverDef.velocities = this.velocities;
        const contactSolver = Island.s_contactSolver.initialize(contactSolverDef);

        // Solve position constraints.
        for (let i = 0; i < subStep.config.positionIterations; ++i) {
            const contactsOkay = contactSolver.solveTOIPositionConstraints(toiIndexA, toiIndexB);
            if (contactsOkay) {
                break;
            }
        }

        // Leap of faith to new safe state.
        this.bodies[toiIndexA].sweep.c0.copy(this.positions[toiIndexA].c);
        this.bodies[toiIndexA].sweep.a0 = this.positions[toiIndexA].a;
        this.bodies[toiIndexB].sweep.c0.copy(this.positions[toiIndexB].c);
        this.bodies[toiIndexB].sweep.a0 = this.positions[toiIndexB].a;

        // No warm starting is needed for TOI events because warm
        // starting impulses were applied in the discrete solver.
        contactSolver.initializeVelocityConstraints();

        // Solve velocity constraints.
        for (let i = 0; i < subStep.config.velocityIterations; ++i) {
            contactSolver.solveVelocityConstraints();
        }

        // Don't store the TOI contact forces for warm starting
        // because they can be quite large.

        const h = subStep.dt;

        // Integrate positions
        for (let i = 0; i < this.bodyCount; ++i) {
            const { c } = this.positions[i];
            let { a } = this.positions[i];
            const { v } = this.velocities[i];
            let { w } = this.velocities[i];

            // Check for large velocities
            const translation = Vec2.scale(h, v, Island.s_translation);
            if (Vec2.dot(translation, translation) > MAX_TRANSLATION_SQUARED) {
                const ratio = MAX_TRANSLATION / translation.length();
                v.scale(ratio);
            }

            const rotation = h * w;
            if (rotation * rotation > MAX_ROTATION_SQUARED) {
                const ratio = MAX_ROTATION / Math.abs(rotation);
                w *= ratio;
            }

            // Integrate
            c.addScaled(h, v);
            a += h * w;

            this.positions[i].a = a;
            this.velocities[i].w = w;

            // Sync bodies
            const body = this.bodies[i];
            body.sweep.c.copy(c);
            body.sweep.a = a;
            body.linearVelocity.copy(v);
            body.angularVelocity = w;
            body.synchronizeTransform();
        }

        this.report(contactSolver.velocityConstraints);
    }

    private static s_impulse = new ContactImpulse();

    public report(constraints: ContactVelocityConstraint[]): void {
        for (let i = 0; i < this.contactCount; ++i) {
            const c = this.contacts[i];
            const vc = constraints[i];

            const impulse = Island.s_impulse;
            impulse.count = vc.pointCount;
            for (let j = 0; j < vc.pointCount; ++j) {
                impulse.normalImpulses[j] = vc.points[j].normalImpulse;
                impulse.tangentImpulses[j] = vc.points[j].tangentImpulse;
            }

            this.listener.postSolve(c, impulse);
        }
    }
}
