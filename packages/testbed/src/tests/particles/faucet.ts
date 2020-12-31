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

import { PolygonShape, Vec2, Color, XY } from "@box2d/core";
import { ParticleSystem, ParticleFlag } from "@box2d/particles";

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
    "color mixing": ParticleFlag.ColorMixing,
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
    public ParticleCreated(system: ParticleSystem, particleIndex: number): void {
        system.SetParticleLifetime(
            particleIndex,
            Math.random() * (this.m_maxLifetime - this.m_minLifetime) + this.m_minLifetime,
        );
    }
}

/**
 * FaucetTest test creates a container from boxes and continually
 * spawning particles with finite lifetimes that pour into the
 * box.
 */
class FaucetTest extends AbstractParticleTestWithControls {
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
            FaucetTest.k_particleLifetimeMin,
            FaucetTest.k_particleLifetimeMax,
        );

        // Configure particle system parameters.
        this.m_particleSystem.SetRadius(0.035);
        this.m_particleSystem.SetMaxParticleCount(FaucetTest.k_maxParticleCount);
        this.m_particleSystem.SetDestructionByAge(true);

        const ground = this.m_world.CreateBody();

        // Create the container / trough style sink.
        {
            const shape = new PolygonShape();
            const height = FaucetTest.k_containerHeight + FaucetTest.k_containerThickness;
            shape.SetAsBox(
                FaucetTest.k_containerWidth - FaucetTest.k_containerThickness,
                FaucetTest.k_containerThickness,
                new Vec2(),
                0,
            );
            ground.CreateFixture({ shape });
            shape.SetAsBox(
                FaucetTest.k_containerThickness,
                height,
                new Vec2(-FaucetTest.k_containerWidth, FaucetTest.k_containerHeight),
                0,
            );
            ground.CreateFixture({ shape });
            shape.SetAsBox(
                FaucetTest.k_containerThickness,
                height,
                new Vec2(FaucetTest.k_containerWidth, FaucetTest.k_containerHeight),
                0,
            );
            ground.CreateFixture({ shape });
        }

        // Create ground under the container to catch overflow.
        {
            const shape = new PolygonShape();
            shape.SetAsBox(
                FaucetTest.k_containerWidth * 5,
                FaucetTest.k_containerThickness,
                new Vec2(0, FaucetTest.k_containerThickness * -2),
                0,
            );
            ground.CreateFixture({ shape });
        }

        // Create the faucet spout.
        {
            const shape = new PolygonShape();
            const particleDiameter = this.m_particleSystem.GetRadius() * 2;
            const faucetLength = FaucetTest.k_faucetLength * particleDiameter;
            // Dimensions of the faucet in world units.
            const length = faucetLength * FaucetTest.k_spoutLength;
            const width = FaucetTest.k_containerWidth * FaucetTest.k_faucetWidth * FaucetTest.k_spoutWidth;
            // Height from the bottom of the container.
            const height = FaucetTest.k_containerHeight * FaucetTest.k_faucetHeight + length * 0.5;

            shape.SetAsBox(particleDiameter, length, new Vec2(-width, height), 0);
            ground.CreateFixture({ shape });
            shape.SetAsBox(particleDiameter, length, new Vec2(width, height), 0);
            ground.CreateFixture({ shape });
            shape.SetAsBox(
                width - particleDiameter,
                particleDiameter,
                new Vec2(0, height + length - particleDiameter),
                0,
            );
            ground.CreateFixture({ shape });
        }

        // Initialize the particle emitter.
        {
            const faucetLength = this.m_particleSystem.GetRadius() * 2 * FaucetTest.k_faucetLength;
            this.m_emitter.SetParticleSystem(this.m_particleSystem);
            this.m_emitter.SetCallback(this.m_lifetimeRandomizer);
            this.m_emitter.SetPosition(
                new Vec2(
                    FaucetTest.k_containerWidth * FaucetTest.k_faucetWidth,
                    FaucetTest.k_containerHeight * FaucetTest.k_faucetHeight + faucetLength * 0.5,
                ),
            );
            this.m_emitter.SetVelocity(new Vec2());
            this.m_emitter.SetSize(new Vec2(0, faucetLength));
            this.m_emitter.SetColor(new Color(1, 1, 1, 1));
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
        if (this.m_emitter.GetParticleFlags() & ParticleFlag.ColorMixing) {
            // Each second, select a different color.
            this.m_emitter.SetColor(particleColors[Math.floor(this.m_particleColorOffset) % particleColors.length]);
        } else {
            this.m_emitter.SetColor(new Color(1, 1, 1, 1));
        }

        // Create the particles.
        this.m_emitter.Step(dt);
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("m", "Increase Flow", () =>
                this.m_emitter.SetEmitRate(
                    Math.max(
                        FaucetTest.k_emitRateMin,
                        this.m_emitter.GetEmitRate() * FaucetTest.k_emitRateChangeFactor,
                    ),
                ),
            ),
            hotKeyPress("n", "Decrease Flow", () =>
                this.m_emitter.SetEmitRate(
                    Math.min(
                        FaucetTest.k_emitRateMax,
                        this.m_emitter.GetEmitRate() / FaucetTest.k_emitRateChangeFactor,
                    ),
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

registerTest("Particles", "Faucet", FaucetTest);
