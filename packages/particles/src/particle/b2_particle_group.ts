/*
 * Copyright (c) 2013 Google, Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

import { XY, RGBA, b2Shape, b2Vec2, b2Color, b2Transform, b2Assert } from "@box2d/core";

import { b2ParticleFlag } from "./b2_particle";
import type { b2ParticleSystem } from "./b2_particle_system";

export enum b2ParticleGroupFlag {
    /**
     * Prevents overlapping or leaking.
     */
    b2_solidParticleGroup = 1 << 0,
    /**
     * Keeps its shape.
     */
    b2_rigidParticleGroup = 1 << 1,
    /**
     * Won't be destroyed if it gets empty.
     */
    b2_particleGroupCanBeEmpty = 1 << 2,
    /**
     * Will be destroyed on next simulation step.
     */
    b2_particleGroupWillBeDestroyed = 1 << 3,
    /**
     * Updates depth data on next simulation step.
     */
    b2_particleGroupNeedsUpdateDepth = 1 << 4,

    b2_particleGroupInternalMask = b2_particleGroupWillBeDestroyed | b2_particleGroupNeedsUpdateDepth,
}

export interface b2IParticleGroupDef {
    flags?: b2ParticleFlag;
    groupFlags?: number;
    position?: XY;
    angle?: number;
    linearVelocity?: XY;
    angularVelocity?: number;
    color?: RGBA;
    strength?: number;
    shape?: b2Shape;
    shapes?: b2Shape[];
    shapeCount?: number;
    stride?: number;
    particleCount?: number;
    positionData?: XY[];
    lifetime?: number;
    userData?: any;
    group?: b2ParticleGroup | null;
}

export class b2ParticleGroupDef implements b2IParticleGroupDef {
    public flags: b2ParticleFlag = 0;

    public groupFlags = 0;

    public readonly position = new b2Vec2();

    public angle = 0;

    public readonly linearVelocity = new b2Vec2();

    public angularVelocity = 0;

    public readonly color = new b2Color();

    public strength = 1;

    public shape?: b2Shape;

    public shapes?: b2Shape[];

    public shapeCount = 0;

    public stride = 0;

    public particleCount = 0;

    public positionData?: b2Vec2[];

    public lifetime = 0;

    public userData: any = null;

    public group: b2ParticleGroup | null = null;
}

export class b2ParticleGroup {
    public readonly m_system: b2ParticleSystem;

    public m_firstIndex = 0;

    public m_lastIndex = 0;

    public m_groupFlags = 0;

    public m_strength = 1;

    public m_prev: b2ParticleGroup | null = null;

    public m_next: b2ParticleGroup | null = null;

    public m_timestamp = -1;

    public m_mass = 0;

    public m_inertia = 0;

    public readonly m_center = new b2Vec2();

    public readonly m_linearVelocity = new b2Vec2();

    public m_angularVelocity = 0;

    public readonly m_transform = new b2Transform();

    public m_userData: any = null;

    public constructor(system: b2ParticleSystem) {
        this.m_system = system;
    }

    public GetNext(): b2ParticleGroup | null {
        return this.m_next;
    }

    public GetParticleSystem(): b2ParticleSystem {
        return this.m_system;
    }

    public GetParticleCount(): number {
        return this.m_lastIndex - this.m_firstIndex;
    }

    public GetBufferIndex(): number {
        return this.m_firstIndex;
    }

    public ContainsParticle(index: number): boolean {
        return this.m_firstIndex <= index && index < this.m_lastIndex;
    }

    public GetAllParticleFlags(): b2ParticleFlag {
        b2Assert(this.m_system.m_flagsBuffer.data !== null);
        let flags = 0;
        for (let i = this.m_firstIndex; i < this.m_lastIndex; i++) {
            flags |= this.m_system.m_flagsBuffer.data[i];
        }
        return flags;
    }

    public GetGroupFlags(): number {
        return this.m_groupFlags;
    }

    public SetGroupFlags(flags: number): void {
        // DEBUG: b2Assert((flags & b2ParticleGroupFlag.b2_particleGroupInternalMask) === 0);
        flags |= this.m_groupFlags & b2ParticleGroupFlag.b2_particleGroupInternalMask;
        this.m_system.SetGroupFlags(this, flags);
    }

    public GetMass(): number {
        this.UpdateStatistics();
        return this.m_mass;
    }

    public GetInertia(): number {
        this.UpdateStatistics();
        return this.m_inertia;
    }

    public GetCenter(): Readonly<b2Vec2> {
        this.UpdateStatistics();
        return this.m_center;
    }

    public GetLinearVelocity(): Readonly<b2Vec2> {
        this.UpdateStatistics();
        return this.m_linearVelocity;
    }

    public GetAngularVelocity(): number {
        this.UpdateStatistics();
        return this.m_angularVelocity;
    }

    public GetTransform(): Readonly<b2Transform> {
        return this.m_transform;
    }

    public GetPosition(): Readonly<b2Vec2> {
        return this.m_transform.p;
    }

    public GetAngle(): number {
        return this.m_transform.q.GetAngle();
    }

    public GetLinearVelocityFromWorldPoint<T extends XY>(worldPoint: XY, out: T): T {
        const s_t0 = b2ParticleGroup.GetLinearVelocityFromWorldPoint_s_t0;
        this.UpdateStatistics();

        return b2Vec2.AddCrossScalarVec2(
            this.m_linearVelocity,
            this.m_angularVelocity,
            b2Vec2.Subtract(worldPoint, this.m_center, s_t0),
            out,
        );
    }

    public static readonly GetLinearVelocityFromWorldPoint_s_t0 = new b2Vec2();

    public GetUserData(): void {
        return this.m_userData;
    }

    public SetUserData(data: any): void {
        this.m_userData = data;
    }

    public ApplyForce(force: XY): void {
        this.m_system.ApplyForce(this.m_firstIndex, this.m_lastIndex, force);
    }

    public ApplyLinearImpulse(impulse: XY): void {
        this.m_system.ApplyLinearImpulse(this.m_firstIndex, this.m_lastIndex, impulse);
    }

    public DestroyParticles(callDestructionListener: boolean): void {
        b2Assert(!this.m_system.m_world.IsLocked());

        for (let i = this.m_firstIndex; i < this.m_lastIndex; i++) {
            this.m_system.DestroyParticle(i, callDestructionListener);
        }
    }

    public UpdateStatistics(): void {
        b2Assert(this.m_system.m_positionBuffer.data !== null);
        b2Assert(this.m_system.m_velocityBuffer.data !== null);
        const p = new b2Vec2();
        const v = new b2Vec2();
        if (this.m_timestamp !== this.m_system.m_timestamp) {
            const m = this.m_system.GetParticleMass();
            this.m_mass = m * (this.m_lastIndex - this.m_firstIndex);
            this.m_center.SetZero();
            this.m_linearVelocity.SetZero();
            for (let i = this.m_firstIndex; i < this.m_lastIndex; i++) {
                this.m_center.AddScaled(m, this.m_system.m_positionBuffer.data[i]);
                this.m_linearVelocity.AddScaled(m, this.m_system.m_velocityBuffer.data[i]);
            }
            if (this.m_mass > 0) {
                const inv_mass = 1 / this.m_mass;
                this.m_center.Scale(inv_mass);
                this.m_linearVelocity.Scale(inv_mass);
            }
            this.m_inertia = 0;
            this.m_angularVelocity = 0;
            for (let i = this.m_firstIndex; i < this.m_lastIndex; i++) {
                b2Vec2.Subtract(this.m_system.m_positionBuffer.data[i], this.m_center, p);
                b2Vec2.Subtract(this.m_system.m_velocityBuffer.data[i], this.m_linearVelocity, v);
                this.m_inertia += m * b2Vec2.Dot(p, p);
                this.m_angularVelocity += m * b2Vec2.Cross(p, v);
            }
            if (this.m_inertia > 0) {
                this.m_angularVelocity *= 1 / this.m_inertia;
            }
            this.m_timestamp = this.m_system.m_timestamp;
        }
    }
}
