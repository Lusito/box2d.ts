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

import { Body, Vec2, ChainShape, PolygonShape, Clamp, XY, RandomFloat } from "@box2d/core";
import { ParticleGroup, ParticleGroupDef, ParticleFlag } from "@box2d/particles";

import { registerTest } from "../../test";
import { Settings } from "../../settings";
import { HotKey, hotKeyPress } from "../../utils/hotkeys";
import { AbstractParticleTest } from "./abstract_particle_test";

/**
 * Game which adds some fun to MaxwellTest's demon.
 *
 * http://en.wikipedia.org/wiki/MaxwellTest's_demon
 *
 * The user's goal is to try to catch as many particles as
 * possible in the bottom half of the container by splitting the
 * container using a barrier with the 'a' key.
 *
 * See MaxwellTest::getHotKeys() for other controls.
 */

class MaxwellTest extends AbstractParticleTest {
    public m_density = MaxwellTest.k_densityDefault;

    public m_position = MaxwellTest.k_containerHalfHeight;

    public m_temperature = MaxwellTest.k_temperatureDefault;

    public m_barrierBody: Body | null = null;

    public m_particleGroup: ParticleGroup | null = null;

    public static readonly k_containerWidth = 2;

    public static readonly k_containerHeight = 4;

    public static readonly k_containerHalfWidth = MaxwellTest.k_containerWidth / 2;

    public static readonly k_containerHalfHeight = MaxwellTest.k_containerHeight / 2;

    public static readonly k_barrierHeight = MaxwellTest.k_containerHalfHeight / 100;

    public static readonly k_barrierMovementIncrement = MaxwellTest.k_containerHalfHeight * 0.1;

    public static readonly k_densityStep = 1.25;

    public static readonly k_densityMin = 0.01;

    public static readonly k_densityMax = 0.8;

    public static readonly k_densityDefault = 0.25;

    public static readonly k_temperatureStep = 0.2;

    public static readonly k_temperatureMin = 0.4;

    public static readonly k_temperatureMax = 10;

    public static readonly k_temperatureDefault = 5;

    public constructor() {
        super(Vec2.ZERO);

        // Create the container.
        {
            const ground = this.m_world.CreateBody();
            const shape = new ChainShape();
            const vertices = [
                new Vec2(-MaxwellTest.k_containerHalfWidth, 0),
                new Vec2(MaxwellTest.k_containerHalfWidth, 0),
                new Vec2(MaxwellTest.k_containerHalfWidth, MaxwellTest.k_containerHeight),
                new Vec2(-MaxwellTest.k_containerHalfWidth, MaxwellTest.k_containerHeight),
            ];
            shape.CreateLoop(vertices, 4);
            ground.CreateFixture({
                shape,
                density: 0,
                restitution: 1,
            });
        }

        // Enable the barrier.
        this.EnableBarrier();
        // Create the particles.
        this.ResetParticles();
    }

    /**
     * Disable the barrier.
     */
    public DisableBarrier() {
        if (this.m_barrierBody) {
            this.m_world.DestroyBody(this.m_barrierBody);
            this.m_barrierBody = null;
        }
    }

    /**
     * Enable the barrier.
     */
    public EnableBarrier() {
        if (!this.m_barrierBody) {
            this.m_barrierBody = this.m_world.CreateBody();
            const barrierShape = new PolygonShape();
            barrierShape.SetAsBox(
                MaxwellTest.k_containerHalfWidth,
                MaxwellTest.k_barrierHeight,
                new Vec2(0, this.m_position),
                0,
            );
            this.m_barrierBody.CreateFixture({
                shape: barrierShape,
                density: 0,
                restitution: 1,
            });
        }
    }

    /**
     * Enable / disable the barrier.
     */
    public ToggleBarrier() {
        if (this.m_barrierBody) {
            this.DisableBarrier();
        } else {
            this.EnableBarrier();
        }
    }

    /**
     * Destroy and recreate all particles.
     */
    public ResetParticles() {
        if (this.m_particleGroup !== null) {
            this.m_particleGroup.DestroyParticles(false);
            this.m_particleGroup = null;
        }

        this.m_particleSystem.SetRadius(MaxwellTest.k_containerHalfWidth / 20);
        {
            const shape = new PolygonShape();
            shape.SetAsBox(
                this.m_density * MaxwellTest.k_containerHalfWidth,
                this.m_density * MaxwellTest.k_containerHalfHeight,
                new Vec2(0, MaxwellTest.k_containerHalfHeight),
                0,
            );
            const pd = new ParticleGroupDef();
            pd.flags = ParticleFlag.Powder;
            pd.shape = shape;
            this.m_particleGroup = this.m_particleSystem.CreateParticleGroup(pd);
            const velocities = this.m_particleSystem.GetVelocityBuffer();
            const index = this.m_particleGroup.GetBufferIndex();

            for (let i = 0; i < this.m_particleGroup.GetParticleCount(); ++i) {
                const v = velocities[index + i];
                v.Set(RandomFloat(-1, 1) + 1, RandomFloat(-1, 1) + 1);
                v.Normalize();
                v.Scale(this.m_temperature);
            }
        }
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("a", "Toggle Barrier", () => this.ToggleBarrier()),
            hotKeyPress("m", "Increase the Particle Density", () => {
                this.m_density = Math.min(this.m_density * MaxwellTest.k_densityStep, MaxwellTest.k_densityMax);
                this.Reset();
            }),
            hotKeyPress("n", "Reduce the Particle Density", () => {
                this.m_density = Math.max(this.m_density / MaxwellTest.k_densityStep, MaxwellTest.k_densityMin);
                this.Reset();
            }),
            hotKeyPress("w", "Move the location of the divider up", () =>
                this.MoveDivider(this.m_position + MaxwellTest.k_barrierMovementIncrement),
            ),
            hotKeyPress("s", "Move the location of the divider down", () =>
                this.MoveDivider(this.m_position - MaxwellTest.k_barrierMovementIncrement),
            ),
            hotKeyPress("h", "Reduce the temperature (velocity of particles)", () => {
                this.m_temperature = Math.max(
                    this.m_temperature - MaxwellTest.k_temperatureStep,
                    MaxwellTest.k_temperatureMin,
                );
                this.Reset();
            }),
            hotKeyPress("j", "Increase the temperature (velocity of particles)", () => {
                this.m_temperature = Math.min(
                    this.m_temperature + MaxwellTest.k_temperatureStep,
                    MaxwellTest.k_temperatureMax,
                );
                this.Reset();
            }),
        ];
    }

    /**
     * Determine whether a point is in the container.
     */
    public InContainer(p: Vec2) {
        return (
            p.x >= -MaxwellTest.k_containerHalfWidth &&
            p.x <= MaxwellTest.k_containerHalfWidth &&
            p.y >= 0 &&
            p.y <= MaxwellTest.k_containerHalfHeight * 2.0
        );
    }

    public MouseDown(p: Vec2) {
        if (!this.InContainer(p)) {
            super.MouseDown(p);
        }
    }

    public MouseUp(p: Vec2) {
        // If the pointer is in the container.
        if (this.InContainer(p)) {
            // Enable / disable the barrier.
            this.ToggleBarrier();
        } else {
            // Move the barrier to the touch position.
            this.MoveDivider(p.y);

            super.MouseUp(p);
        }
    }

    public Step(settings: Settings, timeStep: number) {
        super.Step(settings, timeStep);

        // Number of particles above (top) and below (bottom) the barrier.
        let top = 0;
        let bottom = 0;

        if (this.m_particleGroup) {
            const index = this.m_particleGroup.GetBufferIndex();
            const velocities = this.m_particleSystem.GetVelocityBuffer();
            const positions = this.m_particleSystem.GetPositionBuffer();

            for (let i = 0; i < this.m_particleGroup.GetParticleCount(); i++) {
                // Add energy to particles based upon the temperature.
                const v = velocities[index + i];
                v.Normalize();
                v.Scale(this.m_temperature);

                // Keep track of the number of particles above / below the
                // divider / barrier position.
                const p = positions[index + i];
                if (p.y > this.m_position) {
                    top++;
                } else {
                    bottom++;
                }
            }
        }

        // Calculate a score based upon the difference in pressure between the
        // upper and lower divisions of the container.
        const topPressure = top / (MaxwellTest.k_containerHeight - this.m_position);
        const botPressure = bottom / this.m_position;
        this.addDebug("Score", topPressure > 0 ? botPressure / topPressure - 1 : 0);
    }

    /**
     * Reset the particles and the barrier.
     */
    public Reset() {
        this.DisableBarrier();
        this.ResetParticles();
        this.EnableBarrier();
    }

    /**
     * Move the divider / barrier.
     */
    public MoveDivider(newPosition: number) {
        this.m_position = Clamp(
            newPosition,
            MaxwellTest.k_barrierMovementIncrement,
            MaxwellTest.k_containerHeight - MaxwellTest.k_barrierMovementIncrement,
        );
        this.Reset();
    }

    public GetDefaultViewZoom() {
        return 250;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 2,
        };
    }
}

registerTest("Particles", "Maxwell", MaxwellTest);
