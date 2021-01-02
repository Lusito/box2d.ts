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
    public readonly system: ParticleSystem;

    public firstIndex = 0;

    public lastIndex = 0;

    public groupFlags: ParticleGroupFlag = 0;

    public strength = 1;

    public prev: ParticleGroup | null = null;

    public next: ParticleGroup | null = null;

    public timestamp = -1;

    public mass = 0;

    public inertia = 0;

    public readonly center = new Vec2();

    public readonly linearVelocity = new Vec2();

    public angularVelocity = 0;

    public readonly transform = new Transform();

    public userData: any = null;

    public constructor(system: ParticleSystem) {
        this.system = system;
    }

    public getNext(): ParticleGroup | null {
        return this.next;
    }

    public getParticleSystem(): ParticleSystem {
        return this.system;
    }

    public getParticleCount(): number {
        return this.lastIndex - this.firstIndex;
    }

    public getBufferIndex(): number {
        return this.firstIndex;
    }

    public containsParticle(index: number): boolean {
        return this.firstIndex <= index && index < this.lastIndex;
    }

    public getAllParticleFlags(): ParticleFlag {
        assert(this.system.flagsBuffer.data !== null);
        let flags = 0;
        for (let i = this.firstIndex; i < this.lastIndex; i++) {
            flags |= this.system.flagsBuffer.data[i];
        }
        return flags;
    }

    public getGroupFlags(): ParticleGroupFlag {
        return this.groupFlags;
    }

    public setGroupFlags(flags: number): void {
        // DEBUG: assert((flags & ParticleGroupFlag.InternalMask) === 0);
        flags |= this.groupFlags & ParticleGroupFlag.InternalMask;
        this.system.setGroupFlags(this, flags);
    }

    public getMass(): number {
        this.updateStatistics();
        return this.mass;
    }

    public getInertia(): number {
        this.updateStatistics();
        return this.inertia;
    }

    public getCenter(): Readonly<Vec2> {
        this.updateStatistics();
        return this.center;
    }

    public getLinearVelocity(): Readonly<Vec2> {
        this.updateStatistics();
        return this.linearVelocity;
    }

    public getAngularVelocity(): number {
        this.updateStatistics();
        return this.angularVelocity;
    }

    public getTransform(): Readonly<Transform> {
        return this.transform;
    }

    public getPosition(): Readonly<Vec2> {
        return this.transform.p;
    }

    public getAngle(): number {
        return this.transform.q.getAngle();
    }

    public getLinearVelocityFromWorldPoint<T extends XY>(worldPoint: XY, out: T): T {
        const s_t0 = ParticleGroup.GetLinearVelocityFromWorldPoint_s_t0;
        this.updateStatistics();

        return Vec2.addCrossScalarVec2(
            this.linearVelocity,
            this.angularVelocity,
            Vec2.subtract(worldPoint, this.center, s_t0),
            out,
        );
    }

    public static readonly GetLinearVelocityFromWorldPoint_s_t0 = new Vec2();

    public getUserData(): void {
        return this.userData;
    }

    public setUserData(data: any): void {
        this.userData = data;
    }

    public applyForce(force: XY): void {
        this.system.applyForce(this.firstIndex, this.lastIndex, force);
    }

    public applyLinearImpulse(impulse: XY): void {
        this.system.applyLinearImpulse(this.firstIndex, this.lastIndex, impulse);
    }

    public destroyParticles(callDestructionListener: boolean): void {
        assert(!this.system.world.isLocked());

        for (let i = this.firstIndex; i < this.lastIndex; i++) {
            this.system.destroyParticle(i, callDestructionListener);
        }
    }

    public updateStatistics(): void {
        assert(this.system.positionBuffer.data !== null);
        assert(this.system.velocityBuffer.data !== null);
        const p = new Vec2();
        const v = new Vec2();
        if (this.timestamp !== this.system.timestamp) {
            const m = this.system.getParticleMass();
            this.mass = m * (this.lastIndex - this.firstIndex);
            this.center.setZero();
            this.linearVelocity.setZero();
            for (let i = this.firstIndex; i < this.lastIndex; i++) {
                this.center.addScaled(m, this.system.positionBuffer.data[i]);
                this.linearVelocity.addScaled(m, this.system.velocityBuffer.data[i]);
            }
            if (this.mass > 0) {
                const inv_mass = 1 / this.mass;
                this.center.scale(inv_mass);
                this.linearVelocity.scale(inv_mass);
            }
            this.inertia = 0;
            this.angularVelocity = 0;
            for (let i = this.firstIndex; i < this.lastIndex; i++) {
                Vec2.subtract(this.system.positionBuffer.data[i], this.center, p);
                Vec2.subtract(this.system.velocityBuffer.data[i], this.linearVelocity, v);
                this.inertia += m * Vec2.dot(p, p);
                this.angularVelocity += m * Vec2.cross(p, v);
            }
            if (this.inertia > 0) {
                this.angularVelocity *= 1 / this.inertia;
            }
            this.timestamp = this.system.timestamp;
        }
    }
}
