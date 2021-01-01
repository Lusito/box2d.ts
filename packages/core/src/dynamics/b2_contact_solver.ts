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
    LINEAR_SLOP,
    MAX_MANIFOLD_POINTS,
    MAX_LINEAR_CORRECTION,
    BAUMGARTE,
    TOI_BAUMGARTE,
    makeArray,
} from "../common/b2_common";
import { clamp, Vec2, Mat22, Rot, Transform } from "../common/b2_math";
import { WorldManifold, ManifoldType } from "../collision/b2_collision";
import { Contact } from "./b2_contact";
import { TimeStep, Position, Velocity } from "./b2_time_step";

let g_blockSolve = true;

export function setBlockSolve(value: boolean) {
    g_blockSolve = value;
}

export function getBlockSolve() {
    return g_blockSolve;
}

class VelocityConstraintPoint {
    public readonly rA = new Vec2();

    public readonly rB = new Vec2();

    public normalImpulse = 0;

    public tangentImpulse = 0;

    public normalMass = 0;

    public tangentMass = 0;

    public velocityBias = 0;
}

/** @internal */
export class ContactVelocityConstraint {
    public readonly points = makeArray(MAX_MANIFOLD_POINTS, VelocityConstraintPoint);

    public readonly normal = new Vec2();

    public readonly tangent = new Vec2();

    public readonly normalMass = new Mat22();

    public readonly K = new Mat22();

    public indexA = 0;

    public indexB = 0;

    public invMassA = 0;

    public invMassB = 0;

    public invIA = 0;

    public invIB = 0;

    public friction = 0;

    public restitution = 0;

    public threshold = 0;

    public tangentSpeed = 0;

    public pointCount = 0;

    public contactIndex = 0;
}

class ContactPositionConstraint {
    public readonly localPoints = makeArray(MAX_MANIFOLD_POINTS, Vec2);

    public readonly localNormal = new Vec2();

    public readonly localPoint = new Vec2();

    public indexA = 0;

    public indexB = 0;

    public invMassA = 0;

    public invMassB = 0;

    public readonly localCenterA = new Vec2();

    public readonly localCenterB = new Vec2();

    public invIA = 0;

    public invIB = 0;

    public type = ManifoldType.Circles;

    public radiusA = 0;

    public radiusB = 0;

    public pointCount = 0;
}

/** @internal */
export class ContactSolverDef {
    public readonly step = TimeStep.create();

    public contacts!: Contact[];

    public count = 0;

    public positions!: Position[];

    public velocities!: Velocity[];
}

class PositionSolverManifold {
    public readonly normal = new Vec2();

    public readonly point = new Vec2();

    public separation = 0;

    private static Initialize_s_pointA = new Vec2();

    private static Initialize_s_pointB = new Vec2();

    private static Initialize_s_planePoint = new Vec2();

    private static Initialize_s_clipPoint = new Vec2();

    public initialize(pc: ContactPositionConstraint, xfA: Transform, xfB: Transform, index: number): void {
        const pointA = PositionSolverManifold.Initialize_s_pointA;
        const pointB = PositionSolverManifold.Initialize_s_pointB;
        const planePoint = PositionSolverManifold.Initialize_s_planePoint;
        const clipPoint = PositionSolverManifold.Initialize_s_clipPoint;

        // DEBUG: assert(pc.pointCount > 0);

        switch (pc.type) {
            case ManifoldType.Circles:
                Transform.multiplyVec2(xfA, pc.localPoint, pointA);
                Transform.multiplyVec2(xfB, pc.localPoints[0], pointB);
                Vec2.subtract(pointB, pointA, this.normal).normalize();
                Vec2.mid(pointA, pointB, this.point);
                this.separation =
                    Vec2.dot(Vec2.subtract(pointB, pointA, Vec2.s_t0), this.normal) - pc.radiusA - pc.radiusB;
                break;

            case ManifoldType.FaceA:
                Rot.multiplyVec2(xfA.q, pc.localNormal, this.normal);
                Transform.multiplyVec2(xfA, pc.localPoint, planePoint);

                Transform.multiplyVec2(xfB, pc.localPoints[index], clipPoint);
                this.separation =
                    Vec2.dot(Vec2.subtract(clipPoint, planePoint, Vec2.s_t0), this.normal) - pc.radiusA - pc.radiusB;
                this.point.copy(clipPoint);
                break;

            case ManifoldType.FaceB:
                Rot.multiplyVec2(xfB.q, pc.localNormal, this.normal);
                Transform.multiplyVec2(xfB, pc.localPoint, planePoint);

                Transform.multiplyVec2(xfA, pc.localPoints[index], clipPoint);
                this.separation =
                    Vec2.dot(Vec2.subtract(clipPoint, planePoint, Vec2.s_t0), this.normal) - pc.radiusA - pc.radiusB;
                this.point.copy(clipPoint);

                // Ensure normal points from A to B
                this.normal.negate();
                break;
        }
    }
}

/** @internal */
export class ContactSolver {
    public readonly m_step = TimeStep.create();

    public m_positions!: Position[];

    public m_velocities!: Velocity[];

    public readonly m_positionConstraints = makeArray(1024, ContactPositionConstraint); // TODO: Settings

    public readonly m_velocityConstraints = makeArray(1024, ContactVelocityConstraint); // TODO: Settings

    public m_contacts!: Contact[];

    public m_count = 0;

    public initialize(def: ContactSolverDef): ContactSolver {
        this.m_step.copy(def.step);
        this.m_count = def.count;
        // TODO:
        if (this.m_positionConstraints.length < this.m_count) {
            const new_length = Math.max(this.m_positionConstraints.length * 2, this.m_count);
            while (this.m_positionConstraints.length < new_length) {
                this.m_positionConstraints[this.m_positionConstraints.length] = new ContactPositionConstraint();
            }
        }
        // TODO:
        if (this.m_velocityConstraints.length < this.m_count) {
            const new_length = Math.max(this.m_velocityConstraints.length * 2, this.m_count);
            while (this.m_velocityConstraints.length < new_length) {
                this.m_velocityConstraints[this.m_velocityConstraints.length] = new ContactVelocityConstraint();
            }
        }
        this.m_positions = def.positions;
        this.m_velocities = def.velocities;
        this.m_contacts = def.contacts;

        // Initialize position independent portions of the constraints.
        for (let i = 0; i < this.m_count; ++i) {
            const contact = this.m_contacts[i];

            const fixtureA = contact.m_fixtureA;
            const fixtureB = contact.m_fixtureB;
            const shapeA = fixtureA.getShape();
            const shapeB = fixtureB.getShape();
            const radiusA = shapeA.m_radius;
            const radiusB = shapeB.m_radius;
            const bodyA = fixtureA.getBody();
            const bodyB = fixtureB.getBody();
            const manifold = contact.getManifold();

            const { pointCount } = manifold;
            // DEBUG: assert(pointCount > 0);

            const vc = this.m_velocityConstraints[i];
            vc.friction = contact.m_friction;
            vc.restitution = contact.m_restitution;
            vc.threshold = contact.m_restitutionThreshold;
            vc.tangentSpeed = contact.m_tangentSpeed;
            vc.indexA = bodyA.m_islandIndex;
            vc.indexB = bodyB.m_islandIndex;
            vc.invMassA = bodyA.m_invMass;
            vc.invMassB = bodyB.m_invMass;
            vc.invIA = bodyA.m_invI;
            vc.invIB = bodyB.m_invI;
            vc.contactIndex = i;
            vc.pointCount = pointCount;
            vc.K.setZero();
            vc.normalMass.setZero();

            const pc = this.m_positionConstraints[i];
            pc.indexA = bodyA.m_islandIndex;
            pc.indexB = bodyB.m_islandIndex;
            pc.invMassA = bodyA.m_invMass;
            pc.invMassB = bodyB.m_invMass;
            pc.localCenterA.copy(bodyA.m_sweep.localCenter);
            pc.localCenterB.copy(bodyB.m_sweep.localCenter);
            pc.invIA = bodyA.m_invI;
            pc.invIB = bodyB.m_invI;
            pc.localNormal.copy(manifold.localNormal);
            pc.localPoint.copy(manifold.localPoint);
            pc.pointCount = pointCount;
            pc.radiusA = radiusA;
            pc.radiusB = radiusB;
            pc.type = manifold.type;

            for (let j = 0; j < pointCount; ++j) {
                const cp = manifold.points[j];
                const vcp = vc.points[j];

                if (this.m_step.warmStarting) {
                    vcp.normalImpulse = this.m_step.dtRatio * cp.normalImpulse;
                    vcp.tangentImpulse = this.m_step.dtRatio * cp.tangentImpulse;
                } else {
                    vcp.normalImpulse = 0;
                    vcp.tangentImpulse = 0;
                }

                vcp.rA.setZero();
                vcp.rB.setZero();
                vcp.normalMass = 0;
                vcp.tangentMass = 0;
                vcp.velocityBias = 0;

                pc.localPoints[j].copy(cp.localPoint);
            }
        }

        return this;
    }

    private static InitializeVelocityConstraints_s_xfA = new Transform();

    private static InitializeVelocityConstraints_s_xfB = new Transform();

    private static InitializeVelocityConstraints_s_worldManifold = new WorldManifold();

    public initializeVelocityConstraints(): void {
        const xfA = ContactSolver.InitializeVelocityConstraints_s_xfA;
        const xfB = ContactSolver.InitializeVelocityConstraints_s_xfB;
        const worldManifold = ContactSolver.InitializeVelocityConstraints_s_worldManifold;

        const k_maxConditionNumber = 1000;

        for (let i = 0; i < this.m_count; ++i) {
            const vc = this.m_velocityConstraints[i];
            const pc = this.m_positionConstraints[i];

            const { radiusA, radiusB, localCenterA, localCenterB } = pc;
            const manifold = this.m_contacts[vc.contactIndex].getManifold();

            const { indexA, indexB, tangent, pointCount } = vc;

            const mA = vc.invMassA;
            const mB = vc.invMassB;
            const iA = vc.invIA;
            const iB = vc.invIB;

            const cA = this.m_positions[indexA].c;
            const aA = this.m_positions[indexA].a;
            const vA = this.m_velocities[indexA].v;
            const wA = this.m_velocities[indexA].w;

            const cB = this.m_positions[indexB].c;
            const aB = this.m_positions[indexB].a;
            const vB = this.m_velocities[indexB].v;
            const wB = this.m_velocities[indexB].w;

            // DEBUG: assert(manifold.pointCount > 0);

            xfA.q.set(aA);
            xfB.q.set(aB);
            Vec2.subtract(cA, Rot.multiplyVec2(xfA.q, localCenterA, Vec2.s_t0), xfA.p);
            Vec2.subtract(cB, Rot.multiplyVec2(xfB.q, localCenterB, Vec2.s_t0), xfB.p);

            worldManifold.initialize(manifold, xfA, radiusA, xfB, radiusB);

            vc.normal.copy(worldManifold.normal);
            Vec2.crossVec2One(vc.normal, tangent); // compute from normal

            for (let j = 0; j < pointCount; ++j) {
                const vcp = vc.points[j];

                Vec2.subtract(worldManifold.points[j], cA, vcp.rA);
                Vec2.subtract(worldManifold.points[j], cB, vcp.rB);

                const rnA = Vec2.cross(vcp.rA, vc.normal);
                const rnB = Vec2.cross(vcp.rB, vc.normal);

                const kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

                vcp.normalMass = kNormal > 0 ? 1 / kNormal : 0;

                const rtA = Vec2.cross(vcp.rA, tangent);
                const rtB = Vec2.cross(vcp.rB, tangent);

                const kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

                vcp.tangentMass = kTangent > 0 ? 1 / kTangent : 0;

                // Setup a velocity bias for restitution.
                vcp.velocityBias = 0;
                const vRel = Vec2.dot(
                    vc.normal,
                    Vec2.subtract(
                        Vec2.addCrossScalarVec2(vB, wB, vcp.rB, Vec2.s_t0),
                        Vec2.addCrossScalarVec2(vA, wA, vcp.rA, Vec2.s_t1),
                        Vec2.s_t0,
                    ),
                );

                if (vRel < -vc.threshold) {
                    vcp.velocityBias = -vc.restitution * vRel;
                }
            }

            // If we have two points, then prepare the block solver.
            if (vc.pointCount === 2 && g_blockSolve) {
                const vcp1 = vc.points[0];
                const vcp2 = vc.points[1];

                const rn1A = Vec2.cross(vcp1.rA, vc.normal);
                const rn1B = Vec2.cross(vcp1.rB, vc.normal);
                const rn2A = Vec2.cross(vcp2.rA, vc.normal);
                const rn2B = Vec2.cross(vcp2.rB, vc.normal);

                const k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
                const k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
                const k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;

                // Ensure a reasonable condition number.
                if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)) {
                    // K is safe to invert.
                    vc.K.ex.set(k11, k12);
                    vc.K.ey.set(k12, k22);
                    vc.K.getInverse(vc.normalMass);
                } else {
                    // The constraints are redundant, just use one.
                    // TODO_ERIN use deepest?
                    vc.pointCount = 1;
                }
            }
        }
    }

    private static WarmStart_s_P = new Vec2();

    public warmStart(): void {
        const P = ContactSolver.WarmStart_s_P;

        // Warm start.
        for (let i = 0; i < this.m_count; ++i) {
            const vc = this.m_velocityConstraints[i];

            const { indexA, indexB, pointCount, normal, tangent } = vc;
            const mA = vc.invMassA;
            const iA = vc.invIA;
            const mB = vc.invMassB;
            const iB = vc.invIB;

            const vA = this.m_velocities[indexA].v;
            let wA = this.m_velocities[indexA].w;
            const vB = this.m_velocities[indexB].v;
            let wB = this.m_velocities[indexB].w;

            for (let j = 0; j < pointCount; ++j) {
                const vcp = vc.points[j];
                Vec2.add(
                    Vec2.scale(vcp.normalImpulse, normal, Vec2.s_t0),
                    Vec2.scale(vcp.tangentImpulse, tangent, Vec2.s_t1),
                    P,
                );
                wA -= iA * Vec2.cross(vcp.rA, P);
                vA.subtractScaled(mA, P);
                wB += iB * Vec2.cross(vcp.rB, P);
                vB.addScaled(mB, P);
            }

            this.m_velocities[indexA].w = wA;
            this.m_velocities[indexB].w = wB;
        }
    }

    private static SolveVelocityConstraints_s_dv = new Vec2();

    private static SolveVelocityConstraints_s_dv1 = new Vec2();

    private static SolveVelocityConstraints_s_dv2 = new Vec2();

    private static SolveVelocityConstraints_s_P = new Vec2();

    private static SolveVelocityConstraints_s_a = new Vec2();

    private static SolveVelocityConstraints_s_b = new Vec2();

    private static SolveVelocityConstraints_s_x = new Vec2();

    private static SolveVelocityConstraints_s_d = new Vec2();

    private static SolveVelocityConstraints_s_P1 = new Vec2();

    private static SolveVelocityConstraints_s_P2 = new Vec2();

    private static SolveVelocityConstraints_s_P1P2 = new Vec2();

    public solveVelocityConstraints(): void {
        const dv = ContactSolver.SolveVelocityConstraints_s_dv;
        const dv1 = ContactSolver.SolveVelocityConstraints_s_dv1;
        const dv2 = ContactSolver.SolveVelocityConstraints_s_dv2;
        const P = ContactSolver.SolveVelocityConstraints_s_P;
        const a = ContactSolver.SolveVelocityConstraints_s_a;
        const b = ContactSolver.SolveVelocityConstraints_s_b;
        const x = ContactSolver.SolveVelocityConstraints_s_x;
        const d = ContactSolver.SolveVelocityConstraints_s_d;
        const P1 = ContactSolver.SolveVelocityConstraints_s_P1;
        const P2 = ContactSolver.SolveVelocityConstraints_s_P2;
        const P1P2 = ContactSolver.SolveVelocityConstraints_s_P1P2;

        for (let i = 0; i < this.m_count; ++i) {
            const vc = this.m_velocityConstraints[i];

            const { indexA, indexB, pointCount, normal, tangent, friction } = vc;
            const mA = vc.invMassA;
            const iA = vc.invIA;
            const mB = vc.invMassB;
            const iB = vc.invIB;

            const vA = this.m_velocities[indexA].v;
            let wA = this.m_velocities[indexA].w;
            const vB = this.m_velocities[indexB].v;
            let wB = this.m_velocities[indexB].w;

            // DEBUG: assert(pointCount === 1 || pointCount === 2);

            // Solve tangent constraints first because non-penetration is more important
            // than friction.
            for (let j = 0; j < pointCount; ++j) {
                const vcp = vc.points[j];

                // Relative velocity at contact
                Vec2.subtract(
                    Vec2.addCrossScalarVec2(vB, wB, vcp.rB, Vec2.s_t0),
                    Vec2.addCrossScalarVec2(vA, wA, vcp.rA, Vec2.s_t1),
                    dv,
                );

                // Compute tangent force
                const vt = Vec2.dot(dv, tangent) - vc.tangentSpeed;
                let lambda = vcp.tangentMass * -vt;

                // Clamp the accumulated force
                const maxFriction = friction * vcp.normalImpulse;
                const newImpulse = clamp(vcp.tangentImpulse + lambda, -maxFriction, maxFriction);
                lambda = newImpulse - vcp.tangentImpulse;
                vcp.tangentImpulse = newImpulse;

                // Apply contact impulse
                Vec2.scale(lambda, tangent, P);

                vA.subtractScaled(mA, P);
                wA -= iA * Vec2.cross(vcp.rA, P);

                vB.addScaled(mB, P);
                wB += iB * Vec2.cross(vcp.rB, P);
            }

            // Solve normal constraints
            if (vc.pointCount === 1 || g_blockSolve === false) {
                for (let j = 0; j < pointCount; ++j) {
                    const vcp = vc.points[j];

                    // Relative velocity at contact
                    Vec2.subtract(
                        Vec2.addCrossScalarVec2(vB, wB, vcp.rB, Vec2.s_t0),
                        Vec2.addCrossScalarVec2(vA, wA, vcp.rA, Vec2.s_t1),
                        dv,
                    );

                    // Compute normal impulse
                    const vn = Vec2.dot(dv, normal);
                    let lambda = -vcp.normalMass * (vn - vcp.velocityBias);

                    // Clamp the accumulated impulse
                    const newImpulse = Math.max(vcp.normalImpulse + lambda, 0);
                    lambda = newImpulse - vcp.normalImpulse;
                    vcp.normalImpulse = newImpulse;

                    // Apply contact impulse
                    Vec2.scale(lambda, normal, P);
                    vA.subtractScaled(mA, P);
                    wA -= iA * Vec2.cross(vcp.rA, P);

                    vB.addScaled(mB, P);
                    wB += iB * Vec2.cross(vcp.rB, P);
                }
            } else {
                // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
                // Build the mini LCP for this contact patch
                //
                // vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
                //
                // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
                // b = vn0 - velocityBias
                //
                // The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
                // implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
                // vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
                // solution that satisfies the problem is chosen.
                //
                // In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
                // that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
                //
                // Substitute:
                //
                // x = a + d
                //
                // a := old total impulse
                // x := new total impulse
                // d := incremental impulse
                //
                // For the current iteration we extend the formula for the incremental impulse
                // to compute the new total impulse:
                //
                // vn = A * d + b
                //    = A * (x - a) + b
                //    = A * x + b - A * a
                //    = A * x + b'
                // b' = b - A * a;

                const cp1 = vc.points[0];
                const cp2 = vc.points[1];

                a.set(cp1.normalImpulse, cp2.normalImpulse);
                // DEBUG: assert(a.x >= 0 && a.y >= 0);

                // Relative velocity at contact
                Vec2.subtract(
                    Vec2.addCrossScalarVec2(vB, wB, cp1.rB, Vec2.s_t0),
                    Vec2.addCrossScalarVec2(vA, wA, cp1.rA, Vec2.s_t1),
                    dv1,
                );
                Vec2.subtract(
                    Vec2.addCrossScalarVec2(vB, wB, cp2.rB, Vec2.s_t0),
                    Vec2.addCrossScalarVec2(vA, wA, cp2.rA, Vec2.s_t1),
                    dv2,
                );

                // Compute normal velocity
                let vn1 = Vec2.dot(dv1, normal);
                let vn2 = Vec2.dot(dv2, normal);

                b.x = vn1 - cp1.velocityBias;
                b.y = vn2 - cp2.velocityBias;

                // Compute b'
                b.subtract(Mat22.multiplyVec2(vc.K, a, Vec2.s_t0));

                for (;;) {
                    //
                    // Case 1: vn = 0
                    //
                    // 0 = A * x + b'
                    //
                    // Solve for x:
                    //
                    // x = - inv(A) * b'
                    //
                    // Vec2 x = - Mul(vc->normalMass, b);
                    Mat22.multiplyVec2(vc.normalMass, b, x).negate();

                    if (x.x >= 0 && x.y >= 0) {
                        // Get the incremental impulse
                        // Vec2 d = x - a;
                        Vec2.subtract(x, a, d);

                        // Apply incremental impulse
                        Vec2.scale(d.x, normal, P1);
                        Vec2.scale(d.y, normal, P2);
                        Vec2.add(P1, P2, P1P2);
                        vA.subtractScaled(mA, P1P2);
                        wA -= iA * (Vec2.cross(cp1.rA, P1) + Vec2.cross(cp2.rA, P2));

                        vB.addScaled(mB, P1P2);
                        wB += iB * (Vec2.cross(cp1.rB, P1) + Vec2.cross(cp2.rB, P2));

                        // Accumulate
                        cp1.normalImpulse = x.x;
                        cp2.normalImpulse = x.y;
                        break;
                    }

                    //
                    // Case 2: vn1 = 0 and x2 = 0
                    //
                    //   0 = a11 * x1 + a12 * 0 + b1'
                    // vn2 = a21 * x1 + a22 * 0 + b2'
                    //
                    x.x = -cp1.normalMass * b.x;
                    x.y = 0;
                    vn1 = 0;
                    vn2 = vc.K.ex.y * x.x + b.y;

                    if (x.x >= 0 && vn2 >= 0) {
                        // Get the incremental impulse
                        // Vec2 d = x - a;
                        Vec2.subtract(x, a, d);

                        // Apply incremental impulse
                        Vec2.scale(d.x, normal, P1);
                        Vec2.scale(d.y, normal, P2);
                        Vec2.add(P1, P2, P1P2);
                        vA.subtractScaled(mA, P1P2);
                        wA -= iA * (Vec2.cross(cp1.rA, P1) + Vec2.cross(cp2.rA, P2));

                        vB.addScaled(mB, P1P2);
                        wB += iB * (Vec2.cross(cp1.rB, P1) + Vec2.cross(cp2.rB, P2));

                        // Accumulate
                        cp1.normalImpulse = x.x;
                        cp2.normalImpulse = x.y;
                        break;
                    }

                    //
                    // Case 3: vn2 = 0 and x1 = 0
                    //
                    // vn1 = a11 * 0 + a12 * x2 + b1'
                    //   0 = a21 * 0 + a22 * x2 + b2'
                    //
                    x.x = 0;
                    x.y = -cp2.normalMass * b.y;
                    vn1 = vc.K.ey.x * x.y + b.x;
                    vn2 = 0;

                    if (x.y >= 0 && vn1 >= 0) {
                        // Resubstitute for the incremental impulse
                        Vec2.subtract(x, a, d);

                        // Apply incremental impulse
                        Vec2.scale(d.x, normal, P1);
                        Vec2.scale(d.y, normal, P2);
                        Vec2.add(P1, P2, P1P2);
                        vA.subtractScaled(mA, P1P2);
                        wA -= iA * (Vec2.cross(cp1.rA, P1) + Vec2.cross(cp2.rA, P2));

                        vB.addScaled(mB, P1P2);
                        wB += iB * (Vec2.cross(cp1.rB, P1) + Vec2.cross(cp2.rB, P2));

                        // Accumulate
                        cp1.normalImpulse = x.x;
                        cp2.normalImpulse = x.y;
                        break;
                    }

                    //
                    // Case 4: x1 = 0 and x2 = 0
                    //
                    // vn1 = b1
                    // vn2 = b2;
                    x.x = 0;
                    x.y = 0;
                    vn1 = b.x;
                    vn2 = b.y;

                    if (vn1 >= 0 && vn2 >= 0) {
                        // Resubstitute for the incremental impulse
                        Vec2.subtract(x, a, d);

                        // Apply incremental impulse
                        Vec2.scale(d.x, normal, P1);
                        Vec2.scale(d.y, normal, P2);
                        Vec2.add(P1, P2, P1P2);
                        vA.subtractScaled(mA, P1P2);
                        wA -= iA * (Vec2.cross(cp1.rA, P1) + Vec2.cross(cp2.rA, P2));

                        vB.addScaled(mB, P1P2);
                        wB += iB * (Vec2.cross(cp1.rB, P1) + Vec2.cross(cp2.rB, P2));

                        // Accumulate
                        cp1.normalImpulse = x.x;
                        cp2.normalImpulse = x.y;

                        break;
                    }

                    // No solution, give up. This is hit sometimes, but it doesn't seem to matter.
                    break;
                }
            }

            this.m_velocities[indexA].w = wA;
            this.m_velocities[indexB].w = wB;
        }
    }

    public storeImpulses(): void {
        for (let i = 0; i < this.m_count; ++i) {
            const vc = this.m_velocityConstraints[i];
            const manifold = this.m_contacts[vc.contactIndex].getManifold();

            for (let j = 0; j < vc.pointCount; ++j) {
                manifold.points[j].normalImpulse = vc.points[j].normalImpulse;
                manifold.points[j].tangentImpulse = vc.points[j].tangentImpulse;
            }
        }
    }

    private static SolvePositionConstraints_s_xfA = new Transform();

    private static SolvePositionConstraints_s_xfB = new Transform();

    private static SolvePositionConstraints_s_psm = new PositionSolverManifold();

    private static SolvePositionConstraints_s_rA = new Vec2();

    private static SolvePositionConstraints_s_rB = new Vec2();

    private static SolvePositionConstraints_s_P = new Vec2();

    public solvePositionConstraints(): boolean {
        const xfA = ContactSolver.SolvePositionConstraints_s_xfA;
        const xfB = ContactSolver.SolvePositionConstraints_s_xfB;
        const psm = ContactSolver.SolvePositionConstraints_s_psm;
        const rA = ContactSolver.SolvePositionConstraints_s_rA;
        const rB = ContactSolver.SolvePositionConstraints_s_rB;
        const P = ContactSolver.SolvePositionConstraints_s_P;

        let minSeparation = 0;

        for (let i = 0; i < this.m_count; ++i) {
            const pc = this.m_positionConstraints[i];

            const { indexA, indexB, localCenterA, localCenterB, pointCount } = pc;
            const mA = pc.invMassA;
            const iA = pc.invIA;
            const mB = pc.invMassB;
            const iB = pc.invIB;

            const cA = this.m_positions[indexA].c;
            let aA = this.m_positions[indexA].a;

            const cB = this.m_positions[indexB].c;
            let aB = this.m_positions[indexB].a;

            // Solve normal constraints
            for (let j = 0; j < pointCount; ++j) {
                xfA.q.set(aA);
                xfB.q.set(aB);
                Vec2.subtract(cA, Rot.multiplyVec2(xfA.q, localCenterA, Vec2.s_t0), xfA.p);
                Vec2.subtract(cB, Rot.multiplyVec2(xfB.q, localCenterB, Vec2.s_t0), xfB.p);

                psm.initialize(pc, xfA, xfB, j);
                const { normal, point, separation } = psm;

                Vec2.subtract(point, cA, rA);
                Vec2.subtract(point, cB, rB);

                // Track max constraint error.
                minSeparation = Math.min(minSeparation, separation);

                // Prevent large corrections and allow slop.
                const C = clamp(BAUMGARTE * (separation + LINEAR_SLOP), -MAX_LINEAR_CORRECTION, 0);

                // Compute the effective mass.
                const rnA = Vec2.cross(rA, normal);
                const rnB = Vec2.cross(rB, normal);
                const K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

                // Compute normal impulse
                const impulse = K > 0 ? -C / K : 0;

                Vec2.scale(impulse, normal, P);

                cA.subtractScaled(mA, P);
                aA -= iA * Vec2.cross(rA, P);

                cB.addScaled(mB, P);
                aB += iB * Vec2.cross(rB, P);
            }

            this.m_positions[indexA].c.copy(cA);
            this.m_positions[indexA].a = aA;

            this.m_positions[indexB].c.copy(cB);
            this.m_positions[indexB].a = aB;
        }

        // We can't expect minSpeparation >= -LINEAR_SLOP because we don't
        // push the separation above -LINEAR_SLOP.
        return minSeparation >= -3 * LINEAR_SLOP;
    }

    private static SolveTOIPositionConstraints_s_xfA = new Transform();

    private static SolveTOIPositionConstraints_s_xfB = new Transform();

    private static SolveTOIPositionConstraints_s_psm = new PositionSolverManifold();

    private static SolveTOIPositionConstraints_s_rA = new Vec2();

    private static SolveTOIPositionConstraints_s_rB = new Vec2();

    private static SolveTOIPositionConstraints_s_P = new Vec2();

    public solveTOIPositionConstraints(toiIndexA: number, toiIndexB: number): boolean {
        const xfA = ContactSolver.SolveTOIPositionConstraints_s_xfA;
        const xfB = ContactSolver.SolveTOIPositionConstraints_s_xfB;
        const psm = ContactSolver.SolveTOIPositionConstraints_s_psm;
        const rA = ContactSolver.SolveTOIPositionConstraints_s_rA;
        const rB = ContactSolver.SolveTOIPositionConstraints_s_rB;
        const P = ContactSolver.SolveTOIPositionConstraints_s_P;

        let minSeparation = 0;

        for (let i = 0; i < this.m_count; ++i) {
            const pc = this.m_positionConstraints[i];

            const { indexA, indexB, localCenterA, localCenterB, pointCount } = pc;

            let mA = 0;
            let iA = 0;
            if (indexA === toiIndexA || indexA === toiIndexB) {
                mA = pc.invMassA;
                iA = pc.invIA;
            }

            let mB = 0;
            let iB = 0;
            if (indexB === toiIndexA || indexB === toiIndexB) {
                mB = pc.invMassB;
                iB = pc.invIB;
            }

            const cA = this.m_positions[indexA].c;
            let aA = this.m_positions[indexA].a;

            const cB = this.m_positions[indexB].c;
            let aB = this.m_positions[indexB].a;

            // Solve normal constraints
            for (let j = 0; j < pointCount; ++j) {
                xfA.q.set(aA);
                xfB.q.set(aB);
                Vec2.subtract(cA, Rot.multiplyVec2(xfA.q, localCenterA, Vec2.s_t0), xfA.p);
                Vec2.subtract(cB, Rot.multiplyVec2(xfB.q, localCenterB, Vec2.s_t0), xfB.p);

                psm.initialize(pc, xfA, xfB, j);
                const { normal, point, separation } = psm;

                Vec2.subtract(point, cA, rA);
                Vec2.subtract(point, cB, rB);

                // Track max constraint error.
                minSeparation = Math.min(minSeparation, separation);

                // Prevent large corrections and allow slop.
                const C = clamp(TOI_BAUMGARTE * (separation + LINEAR_SLOP), -MAX_LINEAR_CORRECTION, 0);

                // Compute the effective mass.
                const rnA = Vec2.cross(rA, normal);
                const rnB = Vec2.cross(rB, normal);
                const K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

                // Compute normal impulse
                const impulse = K > 0 ? -C / K : 0;

                Vec2.scale(impulse, normal, P);

                cA.subtractScaled(mA, P);
                aA -= iA * Vec2.cross(rA, P);

                cB.addScaled(mB, P);
                aB += iB * Vec2.cross(rB, P);
            }

            this.m_positions[indexA].a = aA;

            this.m_positions[indexB].a = aB;
        }

        // We can't expect minSpeparation >= -LINEAR_SLOP because we don't
        // push the separation above -LINEAR_SLOP.
        return minSeparation >= -1.5 * LINEAR_SLOP;
    }
}
