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

import { XY, RGBA, Shape, Vec2, Color, Transform, assert } from "@box2d/core";

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

    public getNext(): ParticleGroup | null {
        return this.m_next;
    }

    public getParticleSystem(): ParticleSystem {
        return this.m_system;
    }

    public getParticleCount(): number {
        return this.m_lastIndex - this.m_firstIndex;
    }

    public getBufferIndex(): number {
        return this.m_firstIndex;
    }

    public containsParticle(index: number): boolean {
        return this.m_firstIndex <= index && index < this.m_lastIndex;
    }

    public getAllParticleFlags(): ParticleFlag {
        assert(this.m_system.m_flagsBuffer.data !== null);
        let flags = 0;
        for (let i = this.m_firstIndex; i < this.m_lastIndex; i++) {
            flags |= this.m_system.m_flagsBuffer.data[i];
        }
        return flags;
    }

    public getGroupFlags(): ParticleGroupFlag {
        return this.m_groupFlags;
    }

    public setGroupFlags(flags: number): void {
        // DEBUG: assert((flags & ParticleGroupFlag.InternalMask) === 0);
        flags |= this.m_groupFlags & ParticleGroupFlag.InternalMask;
        this.m_system.setGroupFlags(this, flags);
    }

    public getMass(): number {
        this.updateStatistics();
        return this.m_mass;
    }

    public getInertia(): number {
        this.updateStatistics();
        return this.m_inertia;
    }

    public getCenter(): Readonly<Vec2> {
        this.updateStatistics();
        return this.m_center;
    }

    public getLinearVelocity(): Readonly<Vec2> {
        this.updateStatistics();
        return this.m_linearVelocity;
    }

    public getAngularVelocity(): number {
        this.updateStatistics();
        return this.m_angularVelocity;
    }

    public getTransform(): Readonly<Transform> {
        return this.m_transform;
    }

    public getPosition(): Readonly<Vec2> {
        return this.m_transform.p;
    }

    public getAngle(): number {
        return this.m_transform.q.getAngle();
    }

    public getLinearVelocityFromWorldPoint<T extends XY>(worldPoint: XY, out: T): T {
        const s_t0 = ParticleGroup.GetLinearVelocityFromWorldPoint_s_t0;
        this.updateStatistics();

        return Vec2.addCrossScalarVec2(
            this.m_linearVelocity,
            this.m_angularVelocity,
            Vec2.subtract(worldPoint, this.m_center, s_t0),
            out,
        );
    }

    public static readonly GetLinearVelocityFromWorldPoint_s_t0 = new Vec2();

    public getUserData(): void {
        return this.m_userData;
    }

    public setUserData(data: any): void {
        this.m_userData = data;
    }

    public applyForce(force: XY): void {
        this.m_system.applyForce(this.m_firstIndex, this.m_lastIndex, force);
    }

    public applyLinearImpulse(impulse: XY): void {
        this.m_system.applyLinearImpulse(this.m_firstIndex, this.m_lastIndex, impulse);
    }

    public destroyParticles(callDestructionListener: boolean): void {
        assert(!this.m_system.m_world.isLocked());

        for (let i = this.m_firstIndex; i < this.m_lastIndex; i++) {
            this.m_system.destroyParticle(i, callDestructionListener);
        }
    }

    public updateStatistics(): void {
        assert(this.m_system.m_positionBuffer.data !== null);
        assert(this.m_system.m_velocityBuffer.data !== null);
        const p = new Vec2();
        const v = new Vec2();
        if (this.m_timestamp !== this.m_system.m_timestamp) {
            const m = this.m_system.getParticleMass();
            this.m_mass = m * (this.m_lastIndex - this.m_firstIndex);
            this.m_center.setZero();
            this.m_linearVelocity.setZero();
            for (let i = this.m_firstIndex; i < this.m_lastIndex; i++) {
                this.m_center.addScaled(m, this.m_system.m_positionBuffer.data[i]);
                this.m_linearVelocity.addScaled(m, this.m_system.m_velocityBuffer.data[i]);
            }
            if (this.m_mass > 0) {
                const inv_mass = 1 / this.m_mass;
                this.m_center.scale(inv_mass);
                this.m_linearVelocity.scale(inv_mass);
            }
            this.m_inertia = 0;
            this.m_angularVelocity = 0;
            for (let i = this.m_firstIndex; i < this.m_lastIndex; i++) {
                Vec2.subtract(this.m_system.m_positionBuffer.data[i], this.m_center, p);
                Vec2.subtract(this.m_system.m_velocityBuffer.data[i], this.m_linearVelocity, v);
                this.m_inertia += m * Vec2.dot(p, p);
                this.m_angularVelocity += m * Vec2.cross(p, v);
            }
            if (this.m_inertia > 0) {
                this.m_angularVelocity *= 1 / this.m_inertia;
            }
            this.m_timestamp = this.m_system.m_timestamp;
        }
    }
}
