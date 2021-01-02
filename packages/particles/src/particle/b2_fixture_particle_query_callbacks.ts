import {
    ContactFilter,
    Fixture,
    RayCastInput,
    RayCastOutput,
    Rot,
    ShapeType,
    TimeStep,
    Transform,
    Vec2,
    verify,
    LINEAR_SLOP,
} from "@box2d/core";

import { computeDistance } from "./b2_compute_distance";
import { ParticleFlag } from "./b2_particle";
import type { ParticleSystem } from "./b2_particle_system";

export abstract class FixtureParticleQueryCallback {
    protected system: ParticleSystem;

    public constructor(system: ParticleSystem) {
        this.system = system;
    }

    public reportFixture(fixture: Fixture): boolean {
        if (fixture.isSensor()) {
            return true;
        }
        const shape = fixture.getShape();
        const childCount = shape.getChildCount();
        for (let childIndex = 0; childIndex < childCount; childIndex++) {
            const aabb = fixture.getAABB(childIndex);
            const enumerator = this.system.getInsideBoundsEnumerator(aabb);
            let index: number;
            // eslint-disable-next-line no-cond-assign
            while ((index = enumerator.getNext()) >= 0) {
                this.reportFixtureAndParticle(fixture, childIndex, index);
            }
        }
        return true;
    }

    public abstract reportFixtureAndParticle(_fixture: Fixture, _childIndex: number, _index: number): void;
}

export class ParticleSystem_UpdateBodyContactsCallback extends FixtureParticleQueryCallback {
    public contactFilter: ContactFilter | null = null;

    public shouldCollideFixtureParticle(fixture: Fixture, particleIndex: number): boolean {
        // Call the contact filter if it's set, to determine whether to
        // filter this contact.  Returns true if contact calculations should
        // be performed, false otherwise.
        if (this.contactFilter) {
            const flags = this.system.getFlagsBuffer();
            if (flags[particleIndex] & ParticleFlag.FixtureContactFilter) {
                return this.contactFilter.shouldCollideFixtureParticle(fixture, this.system, particleIndex);
            }
        }
        return true;
    }

    public reportFixtureAndParticle(fixture: Fixture, childIndex: number, a: number): void {
        const s_n = ParticleSystem_UpdateBodyContactsCallback.ReportFixtureAndParticle_s_n;
        const s_rp = ParticleSystem_UpdateBodyContactsCallback.ReportFixtureAndParticle_s_rp;
        const ap = this.system.positionBuffer.data[a];
        const n = s_n;

        const d = computeDistance(fixture.getShape(), fixture.getBody().getTransform(), ap, n, childIndex);
        if (d < this.system.particleDiameter && this.shouldCollideFixtureParticle(fixture, a)) {
            const b = fixture.getBody();
            const bp = b.getWorldCenter();
            const bm = b.getMass();
            const bI = b.getInertia() - bm * b.getLocalCenter().lengthSquared();
            const invBm = bm > 0 ? 1 / bm : 0;
            const invBI = bI > 0 ? 1 / bI : 0;
            const invAm = this.system.flagsBuffer.data[a] & ParticleFlag.Wall ? 0 : this.system.getParticleInvMass();

            const rp = Vec2.subtract(ap, bp, s_rp);
            const rpn = Vec2.cross(rp, n);
            const invM = invAm + invBm + invBI * rpn * rpn;

            const contact = this.system.bodyContactBuffer.data[this.system.bodyContactBuffer.append()];
            contact.index = a;
            contact.body = b;
            contact.fixture = fixture;
            contact.weight = 1 - d * this.system.inverseDiameter;

            contact.normal.copy(n.negate());
            contact.mass = invM > 0 ? 1 / invM : 0;
            this.system.detectStuckParticle(a);
        }
    }

    public static readonly ReportFixtureAndParticle_s_n = new Vec2();

    public static readonly ReportFixtureAndParticle_s_rp = new Vec2();
}

export class ParticleSystem_SolveCollisionCallback extends FixtureParticleQueryCallback {
    public step: TimeStep | null = null;

    public reportFixtureAndParticle(fixture: Fixture, childIndex: number, a: number): void {
        const s_p1 = ParticleSystem_SolveCollisionCallback.ReportFixtureAndParticle_s_p1;
        const s_output = ParticleSystem_SolveCollisionCallback.ReportFixtureAndParticle_s_output;
        const s_input = ParticleSystem_SolveCollisionCallback.ReportFixtureAndParticle_s_input;
        const s_p = ParticleSystem_SolveCollisionCallback.ReportFixtureAndParticle_s_p;
        const s_v = ParticleSystem_SolveCollisionCallback.ReportFixtureAndParticle_s_v;
        const s_f = ParticleSystem_SolveCollisionCallback.ReportFixtureAndParticle_s_f;

        const body = fixture.getBody();
        const ap = this.system.positionBuffer.data[a];
        const av = this.system.velocityBuffer.data[a];
        const output = s_output;
        const input = s_input;
        if (this.system.iterationIndex === 0) {
            const xf = body.getTransform();
            // Put 'ap' in the local space of the previous frame
            const p1 = Transform.transposeMultiplyVec2(body.xf0, ap, s_p1);
            if (fixture.getShape().getType() === ShapeType.Circle) {
                // Make relative to the center of the circle
                p1.subtract(body.getLocalCenter());
                // Re-apply rotation about the center of the circle
                Rot.multiplyVec2(body.xf0.q, p1, p1);
                // Subtract rotation of the current frame
                Rot.transposeMultiplyVec2(xf.q, p1, p1);
                // Return to local space
                p1.add(body.getLocalCenter());
            }
            // Return to global space and apply rotation of current frame
            Transform.multiplyVec2(xf, p1, input.p1);
        } else {
            input.p1.copy(ap);
        }

        const step = verify(this.step);
        Vec2.addScaled(ap, step.dt, av, input.p2);
        input.maxFraction = 1;
        if (fixture.rayCast(output, input, childIndex)) {
            const n = output.normal;
            const p = s_p;
            p.x = (1 - output.fraction) * input.p1.x + output.fraction * input.p2.x + LINEAR_SLOP * n.x;
            p.y = (1 - output.fraction) * input.p1.y + output.fraction * input.p2.y + LINEAR_SLOP * n.y;
            const v = s_v;
            v.x = step.inv_dt * (p.x - ap.x);
            v.y = step.inv_dt * (p.y - ap.y);
            this.system.velocityBuffer.data[a].copy(v);
            const f = s_f;
            f.x = step.inv_dt * this.system.getParticleMass() * (av.x - v.x);
            f.y = step.inv_dt * this.system.getParticleMass() * (av.y - v.y);
            this.system.particleApplyForce(a, f);
        }
    }

    public static readonly ReportFixtureAndParticle_s_p1 = new Vec2();

    public static readonly ReportFixtureAndParticle_s_output = new RayCastOutput();

    public static readonly ReportFixtureAndParticle_s_input = new RayCastInput();

    public static readonly ReportFixtureAndParticle_s_p = new Vec2();

    public static readonly ReportFixtureAndParticle_s_v = new Vec2();

    public static readonly ReportFixtureAndParticle_s_f = new Vec2();
}
