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

import { Body, Vec2, ChainShape, PolygonShape, clamp, XY, randomFloat } from "@box2d/core";
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
    public density = MaxwellTest.k_densityDefault;

    public position = MaxwellTest.k_containerHalfHeight;

    public temperature = MaxwellTest.k_temperatureDefault;

    public barrierBody: Body | null = null;

    public particleGroup: ParticleGroup | null = null;

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
            const ground = this.world.createBody();
            const shape = new ChainShape();
            const vertices = [
                new Vec2(-MaxwellTest.k_containerHalfWidth, 0),
                new Vec2(MaxwellTest.k_containerHalfWidth, 0),
                new Vec2(MaxwellTest.k_containerHalfWidth, MaxwellTest.k_containerHeight),
                new Vec2(-MaxwellTest.k_containerHalfWidth, MaxwellTest.k_containerHeight),
            ];
            shape.createLoop(vertices, 4);
            ground.createFixture({
                shape,
                density: 0,
                restitution: 1,
            });
        }

        // Enable the barrier.
        this.enableBarrier();
        // Create the particles.
        this.resetParticles();
    }

    /**
     * Disable the barrier.
     */
    public disableBarrier() {
        if (this.barrierBody) {
            this.world.destroyBody(this.barrierBody);
            this.barrierBody = null;
        }
    }

    /**
     * Enable the barrier.
     */
    public enableBarrier() {
        if (!this.barrierBody) {
            this.barrierBody = this.world.createBody();
            const barrierShape = new PolygonShape();
            barrierShape.setAsBox(
                MaxwellTest.k_containerHalfWidth,
                MaxwellTest.k_barrierHeight,
                new Vec2(0, this.position),
                0,
            );
            this.barrierBody.createFixture({
                shape: barrierShape,
                density: 0,
                restitution: 1,
            });
        }
    }

    /**
     * Enable / disable the barrier.
     */
    public toggleBarrier() {
        if (this.barrierBody) {
            this.disableBarrier();
        } else {
            this.enableBarrier();
        }
    }

    /**
     * Destroy and recreate all particles.
     */
    public resetParticles() {
        if (this.particleGroup !== null) {
            this.particleGroup.destroyParticles(false);
            this.particleGroup = null;
        }

        this.particleSystem.setRadius(MaxwellTest.k_containerHalfWidth / 20);
        {
            const shape = new PolygonShape();
            shape.setAsBox(
                this.density * MaxwellTest.k_containerHalfWidth,
                this.density * MaxwellTest.k_containerHalfHeight,
                new Vec2(0, MaxwellTest.k_containerHalfHeight),
                0,
            );
            const pd = new ParticleGroupDef();
            pd.flags = ParticleFlag.Powder;
            pd.shape = shape;
            this.particleGroup = this.particleSystem.createParticleGroup(pd);
            const velocities = this.particleSystem.getVelocityBuffer();
            const index = this.particleGroup.getBufferIndex();

            for (let i = 0; i < this.particleGroup.getParticleCount(); ++i) {
                const v = velocities[index + i];
                v.set(randomFloat(-1, 1) + 1, randomFloat(-1, 1) + 1);
                v.normalize();
                v.scale(this.temperature);
            }
        }
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("a", "Toggle Barrier", () => this.toggleBarrier()),
            hotKeyPress("m", "Increase the Particle Density", () => {
                this.density = Math.min(this.density * MaxwellTest.k_densityStep, MaxwellTest.k_densityMax);
                this.reset();
            }),
            hotKeyPress("n", "Reduce the Particle Density", () => {
                this.density = Math.max(this.density / MaxwellTest.k_densityStep, MaxwellTest.k_densityMin);
                this.reset();
            }),
            hotKeyPress("w", "Move the location of the divider up", () =>
                this.moveDivider(this.position + MaxwellTest.k_barrierMovementIncrement),
            ),
            hotKeyPress("s", "Move the location of the divider down", () =>
                this.moveDivider(this.position - MaxwellTest.k_barrierMovementIncrement),
            ),
            hotKeyPress("h", "Reduce the temperature (velocity of particles)", () => {
                this.temperature = Math.max(
                    this.temperature - MaxwellTest.k_temperatureStep,
                    MaxwellTest.k_temperatureMin,
                );
                this.reset();
            }),
            hotKeyPress("j", "Increase the temperature (velocity of particles)", () => {
                this.temperature = Math.min(
                    this.temperature + MaxwellTest.k_temperatureStep,
                    MaxwellTest.k_temperatureMax,
                );
                this.reset();
            }),
        ];
    }

    /**
     * Determine whether a point is in the container.
     */
    public inContainer(p: Vec2) {
        return (
            p.x >= -MaxwellTest.k_containerHalfWidth &&
            p.x <= MaxwellTest.k_containerHalfWidth &&
            p.y >= 0 &&
            p.y <= MaxwellTest.k_containerHalfHeight * 2.0
        );
    }

    public mouseDown(p: Vec2) {
        if (!this.inContainer(p)) {
            super.mouseDown(p);
        }
    }

    public mouseUp(p: Vec2) {
        // If the pointer is in the container.
        if (this.inContainer(p)) {
            // Enable / disable the barrier.
            this.toggleBarrier();
        } else {
            // Move the barrier to the touch position.
            this.moveDivider(p.y);

            super.mouseUp(p);
        }
    }

    public step(settings: Settings, timeStep: number) {
        super.step(settings, timeStep);

        // Number of particles above (top) and below (bottom) the barrier.
        let top = 0;
        let bottom = 0;

        if (this.particleGroup) {
            const index = this.particleGroup.getBufferIndex();
            const velocities = this.particleSystem.getVelocityBuffer();
            const positions = this.particleSystem.getPositionBuffer();

            for (let i = 0; i < this.particleGroup.getParticleCount(); i++) {
                // Add energy to particles based upon the temperature.
                const v = velocities[index + i];
                v.normalize();
                v.scale(this.temperature);

                // Keep track of the number of particles above / below the
                // divider / barrier position.
                const p = positions[index + i];
                if (p.y > this.position) {
                    top++;
                } else {
                    bottom++;
                }
            }
        }

        // Calculate a score based upon the difference in pressure between the
        // upper and lower divisions of the container.
        const topPressure = top / (MaxwellTest.k_containerHeight - this.position);
        const botPressure = bottom / this.position;
        this.addDebug("Score", topPressure > 0 ? botPressure / topPressure - 1 : 0);
    }

    /**
     * Reset the particles and the barrier.
     */
    public reset() {
        this.disableBarrier();
        this.resetParticles();
        this.enableBarrier();
    }

    /**
     * Move the divider / barrier.
     */
    public moveDivider(newPosition: number) {
        this.position = clamp(
            newPosition,
            MaxwellTest.k_barrierMovementIncrement,
            MaxwellTest.k_containerHeight - MaxwellTest.k_barrierMovementIncrement,
        );
        this.reset();
    }

    public getDefaultViewZoom() {
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
