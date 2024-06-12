/*
 * Copyright (c) 2014 Google, Inc.
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

import { b2Vec2, b2Color, b2Assert } from "@box2d/core";
import {
    b2ParticleSystem,
    b2ParticleFlag,
    b2ParticleGroup,
    b2ParticleGroupFlag,
    b2ParticleDef,
} from "@box2d/particles";

export class EmittedParticleCallback {
    /**
     * Called for each created particle.
     */
    public ParticleCreated(_system: b2ParticleSystem, _particleIndex: number): void {}
}

/**
 * Emit particles from a circular region.
 */
export class RadialEmitter {
    /** Pointer to global world */
    public m_particleSystem: b2ParticleSystem | null = null;

    /** Called for each created particle. */
    public m_callback: EmittedParticleCallback | null = null;

    /** Center of particle emitter */
    public m_origin = new b2Vec2();

    /** Launch direction. */
    public m_startingVelocity = new b2Vec2();

    /** Speed particles are emitted */
    public m_speed = 0;

    /** Half width / height of particle emitter */
    public m_halfSize = new b2Vec2();

    /** Particles per second */
    public m_emitRate = 1;

    /** Initial color of particle emitted. */
    public m_color = new b2Color();

    /** Number particles to emit on the next frame */
    public m_emitRemainder = 0;

    /** Flags for created particles, see b2ParticleFlag. */
    public m_flags: b2ParticleFlag = b2ParticleFlag.b2_waterParticle;

    /** Group to put newly created particles in. */
    public m_group: b2ParticleGroup | null = null;

    public Destroy(): void {
        this.SetGroup(null);
    }

    /**
     * Set the center of the emitter.
     */
    public SetPosition(origin: b2Vec2): void {
        this.m_origin.Copy(origin);
    }

    /**
     * Get the center of the emitter.
     */
    public GetPosition(out: b2Vec2): b2Vec2 {
        return out.Copy(this.m_origin);
    }

    /**
     * Set the size of the circle which emits particles.
     */
    public SetSize(size: b2Vec2): void {
        b2Vec2.Scale(0.5, size, this.m_halfSize);
    }

    /**
     * Get the size of the circle which emits particles.
     */
    public GetSize(out: b2Vec2): b2Vec2 {
        return b2Vec2.Scale(2, this.m_halfSize, out);
    }

    /**
     * Set the starting velocity of emitted particles.
     */
    public SetVelocity(velocity: b2Vec2): void {
        this.m_startingVelocity.Copy(velocity);
    }

    /**
     * Get the starting velocity.
     */
    public GetVelocity(out: b2Vec2): b2Vec2 {
        return out.Copy(this.m_startingVelocity);
    }

    /**
     * Set the speed of particles along the direction from the
     * center of the emitter.
     */
    public SetSpeed(speed: number): void {
        this.m_speed = speed;
    }

    /**
     * Get the speed of particles along the direction from the
     * center of the emitter.
     */
    public GetSpeed(): number {
        return this.m_speed;
    }

    /**
     * Set the flags for created particles.
     */
    public SetParticleFlags(flags: b2ParticleFlag): void {
        this.m_flags = flags;
    }

    /**
     * Get the flags for created particles.
     */
    public GetParticleFlags(): b2ParticleFlag {
        return this.m_flags;
    }

    /**
     * Set the color of particles.
     */
    public SetColor(color: b2Color): void {
        this.m_color.Copy(color);
    }

    /**
     * Get the color of particles emitter.
     */
    public GetColor(out: b2Color): b2Color {
        return out.Copy(this.m_color);
    }

    /**
     * Set the emit rate in particles per second.
     */
    public SetEmitRate(emitRate: number): void {
        this.m_emitRate = emitRate;
    }

    /**
     * Get the current emit rate.
     */
    public GetEmitRate(): number {
        return this.m_emitRate;
    }

    /**
     * Set the particle system this emitter is adding particles to.
     */
    public SetParticleSystem(particleSystem: b2ParticleSystem): void {
        this.m_particleSystem = particleSystem;
    }

    /**
     * Get the particle system this emitter is adding particle to.
     */
    public GetParticleSystem(): b2ParticleSystem | null {
        return this.m_particleSystem;
    }

    /**
     * Set the callback that is called on the creation of each
     * particle.
     */
    public SetCallback(callback: EmittedParticleCallback): void {
        this.m_callback = callback;
    }

    /**
     * Get the callback that is called on the creation of each
     * particle.
     */
    public GetCallback(): EmittedParticleCallback | null {
        return this.m_callback;
    }

    /**
     * This class sets the group flags to b2_particleGroupCanBeEmpty
     * so that it isn't destroyed and clears the
     * b2_particleGroupCanBeEmpty on the group when the emitter no
     * longer references it so that the group can potentially be
     * cleaned up.
     */
    public SetGroup(group: b2ParticleGroup | null): void {
        if (this.m_group) {
            this.m_group.SetGroupFlags(this.m_group.GetGroupFlags() & ~b2ParticleGroupFlag.b2_particleGroupCanBeEmpty);
        }
        this.m_group = group;
        if (this.m_group) {
            this.m_group.SetGroupFlags(this.m_group.GetGroupFlags() | b2ParticleGroupFlag.b2_particleGroupCanBeEmpty);
        }
    }

    /**
     * Get the group particles should be created within.
     */
    public GetGroup(): b2ParticleGroup | null {
        return this.m_group;
    }

    /**
     * dt is seconds that have passed, particleIndices is an
     * optional pointer to an array which tracks which particles
     * have been created and particleIndicesCount is the size of the
     * particleIndices array. This function returns the number of
     * particles created during this simulation step.
     */
    public Step(
        dt: number,
        particleIndices?: number[],
        particleIndicesCount = particleIndices ? particleIndices.length : 0,
    ): number {
        b2Assert(this.m_particleSystem !== null);
        let numberOfParticlesCreated = 0;
        // How many (fractional) particles should we have emitted this frame?
        this.m_emitRemainder += this.m_emitRate * dt;

        const pd = new b2ParticleDef();
        pd.color.Copy(this.m_color);
        pd.flags = this.m_flags;
        pd.group = this.m_group;

        // Keep emitting particles on this frame until we only have a
        // fractional particle left.
        while (this.m_emitRemainder > 1) {
            this.m_emitRemainder -= 1;

            // Randomly pick a position within the emitter's radius.
            const angle = Math.random() * 2 * Math.PI;
            // Distance from the center of the circle.
            const distance = Math.random();
            const positionOnUnitCircle = new b2Vec2(Math.sin(angle), Math.cos(angle));

            // Initial position.
            pd.position.Set(
                this.m_origin.x + positionOnUnitCircle.x * distance * this.m_halfSize.x,
                this.m_origin.y + positionOnUnitCircle.y * distance * this.m_halfSize.y,
            );
            // Send it flying
            pd.velocity.Copy(this.m_startingVelocity);
            if (this.m_speed !== 0) {
                pd.velocity.AddScaled(this.m_speed, positionOnUnitCircle);
            }

            const particleIndex = this.m_particleSystem.CreateParticle(pd);
            if (this.m_callback) {
                this.m_callback.ParticleCreated(this.m_particleSystem, particleIndex);
            }
            if (particleIndices && numberOfParticlesCreated < particleIndicesCount) {
                particleIndices[numberOfParticlesCreated] = particleIndex;
            }
            ++numberOfParticlesCreated;
        }
        return numberOfParticlesCreated;
    }
}
