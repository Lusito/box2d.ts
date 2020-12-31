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

import { ContactFilter, Vec2, ChainShape, PolygonShape, XY, RandomFloat } from "@box2d/core";
import { ParticleGroupDef, ParticleFlag, ParticleGroup } from "@box2d/particles";

import { registerTest } from "../../test";
import { Settings } from "../../settings";
import { hotKeyPress, HotKey } from "../../utils/hotkeys";
import { AbstractParticleTest } from "./abstract_particle_test";

// Optionally disables particle / fixture and particle / particle contacts.
class ParticleContactDisabler extends ContactFilter {
    public m_enableFixtureParticleCollisions = true;

    public m_enableParticleParticleCollisions = true;

    // Blindly enable / disable collisions between fixtures and particles.
    public ShouldCollideFixtureParticle(): boolean {
        return this.m_enableFixtureParticleCollisions;
    }

    // Blindly enable / disable collisions between particles.
    public ShouldCollideParticleParticle(): boolean {
        return this.m_enableParticleParticleCollisions;
    }
}

class ParticleCollisionFilterTest extends AbstractParticleTest {
    public constructor() {
        super(Vec2.ZERO);

        // must also set ParticleContactFilter and
        // FixtureContactFilter flags for particle group
        this.m_world.SetContactFilter(this.m_contactDisabler);

        // Create the container.
        {
            const ground = this.m_world.CreateBody();
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
            shape.CreateLoop(vertices);
            ground.CreateFixture({
                shape,
                density: 0,
                restitution: 1,
            });
        }

        // create the particles
        this.m_particleSystem.SetRadius(0.5);
        {
            // PolygonShape shape;
            const shape = new PolygonShape();
            // shape.SetAsBox(1.5, 1.5, Vec2(kBoxSizeHalf, kBoxSizeHalf + kOffset), 0);
            shape.SetAsBox(
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
            // m_particleGroup =
            // 	m_particleSystem.CreateParticleGroup(pd);
            this.m_particleGroup = this.m_particleSystem.CreateParticleGroup(pd);

            // Vec2* velocities =
            // 	m_particleSystem.GetVelocityBuffer() +
            // 	m_particleGroup.GetBufferIndex();
            const velocities = this.m_particleSystem.GetVelocityBuffer();
            const index = this.m_particleGroup.GetBufferIndex();
            // for (int i = 0; i < m_particleGroup.GetParticleCount(); ++i) {
            // 	Vec2& v = *(velocities + i);
            // 	v.Set(RandomFloat(-1, 1), RandomFloat(-1, 1));
            // 	v.Normalize();
            // 	v *= kSpeedup;
            // }
            for (let i = 0; i < this.m_particleGroup.GetParticleCount(); ++i) {
                const v = velocities[index + i];
                v.Set(RandomFloat(-1, 1), RandomFloat(-1, 1));
                v.Normalize();
                v.Scale(ParticleCollisionFilterTest.kSpeedup);
            }
        }
    }

    public Step(settings: Settings, timeStep: number): void {
        super.Step(settings, timeStep);

        // const int32 index = m_particleGroup.GetBufferIndex();
        const index = this.m_particleGroup.GetBufferIndex();
        // Vec2* const velocities =
        // 	m_particleSystem.GetVelocityBuffer() + index;
        const velocities = this.m_particleSystem.GetVelocityBuffer();
        // for (int32 i = 0; i < m_particleGroup.GetParticleCount(); i++) {
        // 	// Add energy to particles based upon the temperature.
        // 	Vec2& v = velocities[i];
        // 	v.Normalize();
        // 	v *= kSpeedup;
        // }
        for (let i = 0; i < this.m_particleGroup.GetParticleCount(); ++i) {
            const v = velocities[index + i];
            v.Normalize();
            v.Scale(ParticleCollisionFilterTest.kSpeedup);
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
            hotKeyPress("a", "Toggle Fixture Collisions", () => this.ToggleFixtureCollisions()),
            hotKeyPress("s", "Toggle Particle Collisions", () => this.ToggleParticleCollisions()),
        ];
    }

    public ToggleFixtureCollisions(): void {
        this.m_contactDisabler.m_enableFixtureParticleCollisions = !this.m_contactDisabler
            .m_enableFixtureParticleCollisions;
    }

    public ToggleParticleCollisions(): void {
        this.m_contactDisabler.m_enableParticleParticleCollisions = !this.m_contactDisabler
            .m_enableParticleParticleCollisions;
    }

    public m_contactDisabler = new ParticleContactDisabler();

    public m_particleGroup: ParticleGroup;

    public static readonly kBoxSize = 10;

    public static readonly kBoxSizeHalf = ParticleCollisionFilterTest.kBoxSize / 2;

    public static readonly kOffset = 20;

    public static readonly kParticlesContainerSize = ParticleCollisionFilterTest.kOffset + 0.5;

    public static readonly kSpeedup = 8;
}

registerTest("Particles", "Particle Collisions", ParticleCollisionFilterTest);
