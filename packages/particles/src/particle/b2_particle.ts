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

import { XY, RGBA, Vec2, Clamp, Color } from "@box2d/core";

import { b2_invalidParticleIndex } from "./b2_settings";
import type { ParticleGroup } from "./b2_particle_group";

/**
 * The particle type. Can be combined with the | operator.
 */
export enum ParticleFlag {
    /**
     * Water particle.
     */
    Water = 0,
    /**
     * Removed after next simulation step.
     */
    Zombie = 1 << 1,
    /**
     * Zero velocity.
     */
    Wall = 1 << 2,
    /**
     * With restitution from stretching.
     */
    Spring = 1 << 3,
    /**
     * With restitution from deformation.
     */
    Elastic = 1 << 4,
    /**
     * With viscosity.
     */
    Viscous = 1 << 5,
    /**
     * Without isotropic pressure.
     */
    Powder = 1 << 6,
    /**
     * With surface tension.
     */
    Tensile = 1 << 7,
    /**
     * Mix color between contacting particles.
     */
    ColorMixing = 1 << 8,
    /**
     * Call DestructionListener on destruction.
     */
    DestructionListener = 1 << 9,
    /**
     * Prevents other particles from leaking.
     */
    Barrier = 1 << 10,
    /**
     * Less compressibility.
     */
    StaticPressure = 1 << 11,
    /**
     * Makes pairs or triads with other particles.
     */
    Reactive = 1 << 12,
    /**
     * With high repulsive force.
     */
    Repulsive = 1 << 13,
    /**
     * Call ContactListener when this particle is about to interact with
     * a rigid body or stops interacting with a rigid body.
     * This results in an expensive operation compared to using
     * FixtureContactFilter to detect collisions between
     * particles.
     */
    FixtureContactListener = 1 << 14,
    /**
     * Call ContactListener when this particle is about to interact with
     * another particle or stops interacting with another particle.
     * This results in an expensive operation compared to using
     * ParticleContactFilter to detect collisions between
     * particles.
     */
    ParticleContactListener = 1 << 15,
    /**
     * Call ContactFilter when this particle interacts with rigid bodies.
     */
    FixtureContactFilter = 1 << 16,
    /**
     * Call ContactFilter when this particle interacts with other
     * particles.
     */
    ParticleContactFilter = 1 << 17,
}

export interface IParticleDef {
    flags?: ParticleFlag;
    position?: XY;
    velocity?: XY;
    color?: RGBA;
    lifetime?: number;
    userData?: any;
    group?: ParticleGroup | null;
}

export class ParticleDef implements IParticleDef {
    public flags: ParticleFlag = 0;

    public readonly position = new Vec2();

    public readonly velocity = new Vec2();

    public readonly color = new Color(0, 0, 0, 0);

    public lifetime = 0;

    public userData: any = null;

    public group: ParticleGroup | null = null;
}

export function CalculateParticleIterations(gravity: number, radius: number, timeStep: number): number {
    // In some situations you may want more particle iterations than this,
    // but to avoid excessive cycle cost, don't recommend more than this.
    const B2_MAX_RECOMMENDED_PARTICLE_ITERATIONS = 8;
    const B2_RADIUS_THRESHOLD = 0.01;
    const iterations = Math.ceil(Math.sqrt(gravity / (B2_RADIUS_THRESHOLD * radius)) * timeStep);
    return Clamp(iterations, 1, B2_MAX_RECOMMENDED_PARTICLE_ITERATIONS);
}

export class ParticleHandle {
    public m_index = b2_invalidParticleIndex;

    public GetIndex(): number {
        return this.m_index;
    }

    public SetIndex(index: number): void {
        this.m_index = index;
    }
}
