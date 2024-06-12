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

import { b2Body, b2Vec2, b2ChainShape, b2PolygonShape, b2Clamp, XY, b2RandomFloat } from "@box2d/core";
import { b2ParticleGroup, b2ParticleGroupDef, b2ParticleFlag } from "@box2d/particles";

import { registerTest } from "../../test";
import { Settings } from "../../settings";
import { HotKey, hotKeyPress } from "../../utils/hotkeys";
import { AbstractParticleTest } from "./abstract_particle_test";

/**
 * Game which adds some fun to Maxwell's demon.
 *
 * http://en.wikipedia.org/wiki/Maxwell's_demon
 *
 * The user's goal is to try to catch as many particles as
 * possible in the bottom half of the container by splitting the
 * container using a barrier with the 'a' key.
 *
 * See Maxwell::getHotKeys() for other controls.
 */

class Maxwell extends AbstractParticleTest {
    public m_density = Maxwell.k_densityDefault;

    public m_position = Maxwell.k_containerHalfHeight;

    public m_temperature = Maxwell.k_temperatureDefault;

    public m_barrierBody: b2Body | null = null;

    public m_particleGroup: b2ParticleGroup | null = null;

    public static readonly k_containerWidth = 2;

    public static readonly k_containerHeight = 4;

    public static readonly k_containerHalfWidth = Maxwell.k_containerWidth / 2;

    public static readonly k_containerHalfHeight = Maxwell.k_containerHeight / 2;

    public static readonly k_barrierHeight = Maxwell.k_containerHalfHeight / 100;

    public static readonly k_barrierMovementIncrement = Maxwell.k_containerHalfHeight * 0.1;

    public static readonly k_densityStep = 1.25;

    public static readonly k_densityMin = 0.01;

    public static readonly k_densityMax = 0.8;

    public static readonly k_densityDefault = 0.25;

    public static readonly k_temperatureStep = 0.2;

    public static readonly k_temperatureMin = 0.4;

    public static readonly k_temperatureMax = 10;

    public static readonly k_temperatureDefault = 5;

    public constructor() {
        super(b2Vec2.ZERO);

        // Create the container.
        {
            const ground = this.m_world.CreateBody();
            const shape = new b2ChainShape();
            const vertices = [
                new b2Vec2(-Maxwell.k_containerHalfWidth, 0),
                new b2Vec2(Maxwell.k_containerHalfWidth, 0),
                new b2Vec2(Maxwell.k_containerHalfWidth, Maxwell.k_containerHeight),
                new b2Vec2(-Maxwell.k_containerHalfWidth, Maxwell.k_containerHeight),
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
            const barrierShape = new b2PolygonShape();
            barrierShape.SetAsBox(
                Maxwell.k_containerHalfWidth,
                Maxwell.k_barrierHeight,
                new b2Vec2(0, this.m_position),
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

        this.m_particleSystem.SetRadius(Maxwell.k_containerHalfWidth / 20);
        {
            const shape = new b2PolygonShape();
            shape.SetAsBox(
                this.m_density * Maxwell.k_containerHalfWidth,
                this.m_density * Maxwell.k_containerHalfHeight,
                new b2Vec2(0, Maxwell.k_containerHalfHeight),
                0,
            );
            const pd = new b2ParticleGroupDef();
            pd.flags = b2ParticleFlag.b2_powderParticle;
            pd.shape = shape;
            this.m_particleGroup = this.m_particleSystem.CreateParticleGroup(pd);
            const velocities = this.m_particleSystem.GetVelocityBuffer();
            const index = this.m_particleGroup.GetBufferIndex();

            for (let i = 0; i < this.m_particleGroup.GetParticleCount(); ++i) {
                const v = velocities[index + i];
                v.Set(b2RandomFloat(-1, 1) + 1, b2RandomFloat(-1, 1) + 1);
                v.Normalize();
                v.Scale(this.m_temperature);
            }
        }
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("a", "Toggle Barrier", () => this.ToggleBarrier()),
            hotKeyPress("m", "Increase the Particle Density", () => {
                this.m_density = Math.min(this.m_density * Maxwell.k_densityStep, Maxwell.k_densityMax);
                this.Reset();
            }),
            hotKeyPress("n", "Reduce the Particle Density", () => {
                this.m_density = Math.max(this.m_density / Maxwell.k_densityStep, Maxwell.k_densityMin);
                this.Reset();
            }),
            hotKeyPress("w", "Move the location of the divider up", () =>
                this.MoveDivider(this.m_position + Maxwell.k_barrierMovementIncrement),
            ),
            hotKeyPress("s", "Move the location of the divider down", () =>
                this.MoveDivider(this.m_position - Maxwell.k_barrierMovementIncrement),
            ),
            hotKeyPress("h", "Reduce the temperature (velocity of particles)", () => {
                this.m_temperature = Math.max(this.m_temperature - Maxwell.k_temperatureStep, Maxwell.k_temperatureMin);
                this.Reset();
            }),
            hotKeyPress("j", "Increase the temperature (velocity of particles)", () => {
                this.m_temperature = Math.min(this.m_temperature + Maxwell.k_temperatureStep, Maxwell.k_temperatureMax);
                this.Reset();
            }),
        ];
    }

    /**
     * Determine whether a point is in the container.
     */
    public InContainer(p: b2Vec2) {
        return (
            p.x >= -Maxwell.k_containerHalfWidth &&
            p.x <= Maxwell.k_containerHalfWidth &&
            p.y >= 0 &&
            p.y <= Maxwell.k_containerHalfHeight * 2.0
        );
    }

    public MouseDown(p: b2Vec2) {
        if (!this.InContainer(p)) {
            super.MouseDown(p);
        }
    }

    public MouseUp(p: b2Vec2) {
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
        const topPressure = top / (Maxwell.k_containerHeight - this.m_position);
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
        this.m_position = b2Clamp(
            newPosition,
            Maxwell.k_barrierMovementIncrement,
            Maxwell.k_containerHeight - Maxwell.k_barrierMovementIncrement,
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

registerTest("Particles", "Maxwell", Maxwell);
