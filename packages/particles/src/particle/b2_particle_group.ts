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

import { XY, RGBA, Shape, Vec2, Color, Transform, Assert } from "@box2d/core";

import { ParticleFlag } from "./b2_particle";
import type { ParticleSystem } from "./b2_particle_system";

export enum ParticleGroupFlag {
    /**
     * Prevents overlapping or leaking.
     */
    Solid = 1 << 0,
    /**
     * Keeps its shape.
     */
    Rigid = 1 << 1,
    /**
     * Won't be destroyed if it gets empty.
     */
    CanBeEmpty = 1 << 2,
    /**
     * Will be destroyed on next simulation step.
     */
    WillBeDestroyed = 1 << 3,
    /**
     * Updates depth data on next simulation step.
     */
    NeedsUpdateDepth = 1 << 4,

    InternalMask = WillBeDestroyed | NeedsUpdateDepth,
}

export interface IParticleGroupDef {
    flags?: ParticleFlag;
    groupFlags?: ParticleGroupFlag;
    position?: XY;
    angle?: number;
    linearVelocity?: XY;
    angularVelocity?: number;
    color?: RGBA;
    strength?: number;
    shape?: Shape;
    shapes?: Shape[];
    shapeCount?: number;
    stride?: number;
    particleCount?: number;
    positionData?: XY[];
    lifetime?: number;
    userData?: any;
    group?: ParticleGroup | null;
}

export class ParticleGroupDef implements IParticleGroupDef {
    public flags: ParticleFlag = 0;

    public groupFlags: ParticleGroupFlag = 0;

    public readonly position = new Vec2();

    public angle = 0;

    public readonly linearVelocity = new Vec2();

    public angularVelocity = 0;

    public readonly color = new Color();

    public strength = 1;

    public shape?: Shape;

    public shapes?: Shape[];

    public shapeCount = 0;

    public stride = 0;

    public particleCount = 0;

    public positionData?: Vec2[];

    public lifetime = 0;

    public userData: any = null;

    public group: ParticleGroup | null = null;
}

export class ParticleGroup {
    public readonly m_system: ParticleSystem;

    public m_firstIndex = 0;

    public m_lastIndex = 0;

    public m_groupFlags: ParticleGroupFlag = 0;

    public m_strength = 1;

    public m_prev: ParticleGroup | null = null;

    public m_next: ParticleGroup | null = null;

    public m_timestamp = -1;

    public m_mass = 0;

    public m_inertia = 0;

    public readonly m_center = new Vec2();

    public readonly m_linearVelocity = new Vec2();

    public m_angularVelocity = 0;

    public readonly m_transform = new Transform();

    public m_userData: any = null;

    public constructor(system: ParticleSystem) {
        this.m_system = system;
    }

    public GetNext(): ParticleGroup | null {
        return this.m_next;
    }

    public GetParticleSystem(): ParticleSystem {
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

    public GetAllParticleFlags(): ParticleFlag {
        Assert(this.m_system.m_flagsBuffer.data !== null);
        let flags = 0;
        for (let i = this.m_firstIndex; i < this.m_lastIndex; i++) {
            flags |= this.m_system.m_flagsBuffer.data[i];
        }
        return flags;
    }

    public GetGroupFlags(): ParticleGroupFlag {
        return this.m_groupFlags;
    }

    public SetGroupFlags(flags: number): void {
        // DEBUG: Assert((flags & ParticleGroupFlag.InternalMask) === 0);
        flags |= this.m_groupFlags & ParticleGroupFlag.InternalMask;
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

    public GetCenter(): Readonly<Vec2> {
        this.UpdateStatistics();
        return this.m_center;
    }

    public GetLinearVelocity(): Readonly<Vec2> {
        this.UpdateStatistics();
        return this.m_linearVelocity;
    }

    public GetAngularVelocity(): number {
        this.UpdateStatistics();
        return this.m_angularVelocity;
    }

    public GetTransform(): Readonly<Transform> {
        return this.m_transform;
    }

    public GetPosition(): Readonly<Vec2> {
        return this.m_transform.p;
    }

    public GetAngle(): number {
        return this.m_transform.q.GetAngle();
    }

    public GetLinearVelocityFromWorldPoint<T extends XY>(worldPoint: XY, out: T): T {
        const s_t0 = ParticleGroup.GetLinearVelocityFromWorldPoint_s_t0;
        this.UpdateStatistics();

        return Vec2.AddCrossScalarVec2(
            this.m_linearVelocity,
            this.m_angularVelocity,
            Vec2.Subtract(worldPoint, this.m_center, s_t0),
            out,
        );
    }

    public static readonly GetLinearVelocityFromWorldPoint_s_t0 = new Vec2();

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
        Assert(!this.m_system.m_world.IsLocked());

        for (let i = this.m_firstIndex; i < this.m_lastIndex; i++) {
            this.m_system.DestroyParticle(i, callDestructionListener);
        }
    }

    public UpdateStatistics(): void {
        Assert(this.m_system.m_positionBuffer.data !== null);
        Assert(this.m_system.m_velocityBuffer.data !== null);
        const p = new Vec2();
        const v = new Vec2();
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
                Vec2.Subtract(this.m_system.m_positionBuffer.data[i], this.m_center, p);
                Vec2.Subtract(this.m_system.m_velocityBuffer.data[i], this.m_linearVelocity, v);
                this.m_inertia += m * Vec2.Dot(p, p);
                this.m_angularVelocity += m * Vec2.Cross(p, v);
            }
            if (this.m_inertia > 0) {
                this.m_angularVelocity *= 1 / this.m_inertia;
            }
            this.m_timestamp = this.m_system.m_timestamp;
        }
    }
}
