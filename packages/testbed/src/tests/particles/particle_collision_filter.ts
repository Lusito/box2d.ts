/*
 * Copyright (c) 2015 Google, Inc.
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

import { ContactFilter, Vec2, ChainShape, PolygonShape, XY, randomFloat } from "@box2d/core";
import { ParticleGroupDef, ParticleFlag, ParticleGroup } from "@box2d/particles";

import { registerTest } from "../../test";
import { Settings } from "../../settings";
import { hotKeyPress, HotKey } from "../../utils/hotkeys";
import { AbstractParticleTest } from "./abstract_particle_test";

// Optionally disables particle / fixture and particle / particle contacts.
class ParticleContactDisabler extends ContactFilter {
    public enableFixtureParticleCollisions = true;

    public enableParticleParticleCollisions = true;

    // Blindly enable / disable collisions between fixtures and particles.
    public shouldCollideFixtureParticle(): boolean {
        return this.enableFixtureParticleCollisions;
    }

    // Blindly enable / disable collisions between particles.
    public shouldCollideParticleParticle(): boolean {
        return this.enableParticleParticleCollisions;
    }
}

class ParticleCollisionFilterTest extends AbstractParticleTest {
    public constructor() {
        super(Vec2.ZERO);

        // must also set ParticleContactFilter and
        // FixtureContactFilter flags for particle group
        this.world.setContactFilter(this.contactDisabler);

        // Create the container.
        {
            const ground = this.world.createBody();
            const shape = new ChainShape();
            const vertices = [
                new Vec2(
                    -ParticleCollisionFilterTest.kBoxSize,
                    -ParticleCollisionFilterTest.kBoxSize + ParticleCollisionFilterTest.kOffset,
                ),
                new Vec2(
                    ParticleCollisionFilterTest.kBoxSize,
                    -ParticleCollisionFilterTest.kBoxSize + ParticleCollisionFilterTest.kOffset,
                ),
                new Vec2(
                    ParticleCollisionFilterTest.kBoxSize,
                    ParticleCollisionFilterTest.kBoxSize + ParticleCollisionFilterTest.kOffset,
                ),
                new Vec2(
                    -ParticleCollisionFilterTest.kBoxSize,
                    ParticleCollisionFilterTest.kBoxSize + ParticleCollisionFilterTest.kOffset,
                ),
            ];
            shape.createLoop(vertices);
            ground.createFixture({
                shape,
                density: 0,
                restitution: 1,
            });
        }

        // create the particles
        this.particleSystem.setRadius(0.5);
        {
            // PolygonShape shape;
            const shape = new PolygonShape();
            // shape.setAsBox(1.5, 1.5, Vec2(kBoxSizeHalf, kBoxSizeHalf + kOffset), 0);
            shape.setAsBox(
                1.5,
                1.5,
                new Vec2(
                    ParticleCollisionFilterTest.kBoxSizeHalf,
                    ParticleCollisionFilterTest.kBoxSizeHalf + ParticleCollisionFilterTest.kOffset,
                ),
                0,
            );
            // ParticleGroupDef pd;
            const pd = new ParticleGroupDef();
            // pd.shape = &shape;
            pd.shape = shape;
            // pd.flags = Powder
            // 		| ParticleContactFilter
            // 		| FixtureContactFilter;
            pd.flags = ParticleFlag.Powder | ParticleFlag.ParticleContactFilter | ParticleFlag.FixtureContactFilter;
            // particleGroup =
            // 	particleSystem.createParticleGroup(pd);
            this.particleGroup = this.particleSystem.createParticleGroup(pd);

            // Vec2* velocities =
            // 	particleSystem.getVelocityBuffer() +
            // 	particleGroup.getBufferIndex();
            const velocities = this.particleSystem.getVelocityBuffer();
            const index = this.particleGroup.getBufferIndex();
            // for (int i = 0; i < particleGroup.getParticleCount(); ++i) {
            // 	Vec2& v = *(velocities + i);
            // 	v.set(randomFloat(-1, 1), randomFloat(-1, 1));
            // 	v.normalize();
            // 	v *= kSpeedup;
            // }
            for (let i = 0; i < this.particleGroup.getParticleCount(); ++i) {
                const v = velocities[index + i];
                v.set(randomFloat(-1, 1), randomFloat(-1, 1));
                v.normalize();
                v.scale(ParticleCollisionFilterTest.kSpeedup);
            }
        }
    }

    public step(settings: Settings, timeStep: number): void {
        super.step(settings, timeStep);

        // const int32 index = particleGroup.getBufferIndex();
        const index = this.particleGroup.getBufferIndex();
        // Vec2* const velocities =
        // 	particleSystem.getVelocityBuffer() + index;
        const velocities = this.particleSystem.getVelocityBuffer();
        // for (int32 i = 0; i < particleGroup.getParticleCount(); i++) {
        // 	// Add energy to particles based upon the temperature.
        // 	Vec2& v = velocities[i];
        // 	v.normalize();
        // 	v *= kSpeedup;
        // }
        for (let i = 0; i < this.particleGroup.getParticleCount(); ++i) {
            const v = velocities[index + i];
            v.normalize();
            v.scale(ParticleCollisionFilterTest.kSpeedup);
        }
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 20,
        };
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("a", "Toggle Fixture Collisions", () => this.toggleFixtureCollisions()),
            hotKeyPress("s", "Toggle Particle Collisions", () => this.toggleParticleCollisions()),
        ];
    }

    public toggleFixtureCollisions(): void {
        this.contactDisabler.enableFixtureParticleCollisions = !this.contactDisabler.enableFixtureParticleCollisions;
    }

    public toggleParticleCollisions(): void {
        this.contactDisabler.enableParticleParticleCollisions = !this.contactDisabler.enableParticleParticleCollisions;
    }

    public contactDisabler = new ParticleContactDisabler();

    public particleGroup: ParticleGroup;

    public static readonly kBoxSize = 10;

    public static readonly kBoxSizeHalf = ParticleCollisionFilterTest.kBoxSize / 2;

    public static readonly kOffset = 20;

    public static readonly kParticlesContainerSize = ParticleCollisionFilterTest.kOffset + 0.5;

    public static readonly kSpeedup = 8;
}

registerTest("Particles", "Particle Collisions", ParticleCollisionFilterTest);
