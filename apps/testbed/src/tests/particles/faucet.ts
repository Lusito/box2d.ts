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

import { b2PolygonShape, b2Vec2, b2Color, XY } from "@box2d/core";
import { b2ParticleSystem, b2ParticleFlag } from "@box2d/particles";

import { registerTest, TestContext } from "../../test";
import { Settings } from "../../settings";
import { EmittedParticleCallback, RadialEmitter } from "../../utils/particles/particle_emitter";
import { HotKey, hotKeyPress } from "../../utils/hotkeys";
import { AbstractParticleTestWithControls, particleColors } from "./abstract_particle_test";
import { baseParticleTypes } from "../../utils/particles/particle_parameter";

/**
 * Selection of particle types for this test.
 */
const particleTypes = {
    ...baseParticleTypes,
    "color mixing": b2ParticleFlag.b2_colorMixingParticle,
};

class ParticleLifetimeRandomizer extends EmittedParticleCallback {
    public m_minLifetime = 0;

    public m_maxLifetime = 0;

    public constructor(minLifetime: number, maxLifetime: number) {
        super();
        this.m_minLifetime = minLifetime;
        this.m_maxLifetime = maxLifetime;
    }

    /**
     * Called for each created particle.
     */
    public ParticleCreated(system: b2ParticleSystem, particleIndex: number): void {
        system.SetParticleLifetime(
            particleIndex,
            Math.random() * (this.m_maxLifetime - this.m_minLifetime) + this.m_minLifetime,
        );
    }
}

/**
 * Faucet test creates a container from boxes and continually
 * spawning particles with finite lifetimes that pour into the
 * box.
 */
class Faucet extends AbstractParticleTestWithControls {
    /** Used to cycle through particle colors. */
    public m_particleColorOffset = 0;

    /** Particle emitter. */
    public m_emitter: RadialEmitter;

    /** Callback which sets the lifetime of emitted particles. */
    public m_lifetimeRandomizer: ParticleLifetimeRandomizer;

    /** Minimum lifetime of particles in seconds. */
    public static readonly k_particleLifetimeMin = 30;

    /** Maximum lifetime of particles in seconds. */
    public static readonly k_particleLifetimeMax = 50;

    /** Height of the container. */
    public static readonly k_containerHeight = 0.2;

    /** Width of the container. */
    public static readonly k_containerWidth = 1;

    /** Thickness of the container's walls and bottom. */
    public static readonly k_containerThickness = 0.05;

    /** Width of the faucet relative to the container width. */
    public static readonly k_faucetWidth = 0.1;

    /**
     * Height of the faucet relative to the base as a fraction of
     * the container height.
     */
    public static readonly k_faucetHeight = 15;

    /** Length of the faucet as a fraction of the particle diameter. */
    public static readonly k_faucetLength = 2;

    /**
     * Spout height as a fraction of the faucet length.  This should
     * be greater than 1  ).
     */
    public static readonly k_spoutLength = 2;

    /**
     * Spout width as a fraction of the *faucet* width.  This should
     * be greater than 1).
     */
    public static readonly k_spoutWidth = 1.1;

    /** Maximum number of particles in the system. */
    public static readonly k_maxParticleCount = 1000;

    /**
     * Factor that is used to increase / decrease the emit rate.
     * This should be greater than 1.
     */
    public static readonly k_emitRateChangeFactor = 1.05;

    /** Minimum emit rate of the faucet in particles per second. */
    public static readonly k_emitRateMin = 1;

    /** Maximum emit rate of the faucet in particles per second. */
    public static readonly k_emitRateMax = 240;

    public constructor({ particleParameter }: TestContext) {
        super(particleParameter); // base class constructor

        this.m_emitter = new RadialEmitter();
        this.m_lifetimeRandomizer = new ParticleLifetimeRandomizer(
            Faucet.k_particleLifetimeMin,
            Faucet.k_particleLifetimeMax,
        );

        // Configure particle system parameters.
        this.m_particleSystem.SetRadius(0.035);
        this.m_particleSystem.SetMaxParticleCount(Faucet.k_maxParticleCount);
        this.m_particleSystem.SetDestructionByAge(true);

        const ground = this.m_world.CreateBody();

        // Create the container / trough style sink.
        {
            const shape = new b2PolygonShape();
            const height = Faucet.k_containerHeight + Faucet.k_containerThickness;
            shape.SetAsBox(
                Faucet.k_containerWidth - Faucet.k_containerThickness,
                Faucet.k_containerThickness,
                new b2Vec2(),
                0,
            );
            ground.CreateFixture({ shape });
            shape.SetAsBox(
                Faucet.k_containerThickness,
                height,
                new b2Vec2(-Faucet.k_containerWidth, Faucet.k_containerHeight),
                0,
            );
            ground.CreateFixture({ shape });
            shape.SetAsBox(
                Faucet.k_containerThickness,
                height,
                new b2Vec2(Faucet.k_containerWidth, Faucet.k_containerHeight),
                0,
            );
            ground.CreateFixture({ shape });
        }

        // Create ground under the container to catch overflow.
        {
            const shape = new b2PolygonShape();
            shape.SetAsBox(
                Faucet.k_containerWidth * 5,
                Faucet.k_containerThickness,
                new b2Vec2(0, Faucet.k_containerThickness * -2),
                0,
            );
            ground.CreateFixture({ shape });
        }

        // Create the faucet spout.
        {
            const shape = new b2PolygonShape();
            const particleDiameter = this.m_particleSystem.GetRadius() * 2;
            const faucetLength = Faucet.k_faucetLength * particleDiameter;
            // Dimensions of the faucet in world units.
            const length = faucetLength * Faucet.k_spoutLength;
            const width = Faucet.k_containerWidth * Faucet.k_faucetWidth * Faucet.k_spoutWidth;
            // Height from the bottom of the container.
            const height = Faucet.k_containerHeight * Faucet.k_faucetHeight + length * 0.5;

            shape.SetAsBox(particleDiameter, length, new b2Vec2(-width, height), 0);
            ground.CreateFixture({ shape });
            shape.SetAsBox(particleDiameter, length, new b2Vec2(width, height), 0);
            ground.CreateFixture({ shape });
            shape.SetAsBox(
                width - particleDiameter,
                particleDiameter,
                new b2Vec2(0, height + length - particleDiameter),
                0,
            );
            ground.CreateFixture({ shape });
        }

        // Initialize the particle emitter.
        {
            const faucetLength = this.m_particleSystem.GetRadius() * 2 * Faucet.k_faucetLength;
            this.m_emitter.SetParticleSystem(this.m_particleSystem);
            this.m_emitter.SetCallback(this.m_lifetimeRandomizer);
            this.m_emitter.SetPosition(
                new b2Vec2(
                    Faucet.k_containerWidth * Faucet.k_faucetWidth,
                    Faucet.k_containerHeight * Faucet.k_faucetHeight + faucetLength * 0.5,
                ),
            );
            this.m_emitter.SetVelocity(new b2Vec2());
            this.m_emitter.SetSize(new b2Vec2(0, faucetLength));
            this.m_emitter.SetColor(new b2Color(1, 1, 1, 1));
            this.m_emitter.SetEmitRate(120);
            this.m_emitter.SetParticleFlags(particleParameter.GetValue());
        }

        // Don't restart the test when changing particle types.
        particleParameter.SetRestartOnChange(false);
        // Limit the set of particle types.
        particleParameter.SetValues(particleTypes, "water");
    }

    public Step(settings: Settings, timeStep: number): void {
        let dt = settings.m_hertz > 0 ? 1 / settings.m_hertz : 0;

        if (settings.m_pause && !settings.m_singleStep) {
            dt = 0;
        }

        super.Step(settings, timeStep);
        this.m_particleColorOffset += dt;
        // Keep m_particleColorOffset in the range 0..k_ParticleColorsCount.
        if (this.m_particleColorOffset >= particleColors.length) {
            this.m_particleColorOffset -= particleColors.length;
        }

        // Propagate the currently selected particle flags.
        this.m_emitter.SetParticleFlags(this.particleParameter.GetValue());

        // If this is a color mixing particle, add some color.
        if (this.m_emitter.GetParticleFlags() & b2ParticleFlag.b2_colorMixingParticle) {
            // Each second, select a different color.
            this.m_emitter.SetColor(particleColors[Math.floor(this.m_particleColorOffset) % particleColors.length]);
        } else {
            this.m_emitter.SetColor(new b2Color(1, 1, 1, 1));
        }

        // Create the particles.
        this.m_emitter.Step(dt);
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("m", "Increase Flow", () =>
                this.m_emitter.SetEmitRate(
                    Math.max(Faucet.k_emitRateMin, this.m_emitter.GetEmitRate() * Faucet.k_emitRateChangeFactor),
                ),
            ),
            hotKeyPress("n", "Decrease Flow", () =>
                this.m_emitter.SetEmitRate(
                    Math.min(Faucet.k_emitRateMax, this.m_emitter.GetEmitRate() / Faucet.k_emitRateChangeFactor),
                ),
            ),
        ];
    }

    public GetDefaultViewZoom(): number {
        return 250;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 2,
        };
    }
}

registerTest("Particles", "Faucet", Faucet);
