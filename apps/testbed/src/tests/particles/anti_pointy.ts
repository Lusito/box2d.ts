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

import { b2PolygonShape, b2Vec2, XY } from "@box2d/core";
import { b2ParticleFlag, b2ParticleDef } from "@box2d/particles";

import { registerTest, TestContext } from "../../test";
import { Settings } from "../../settings";
import { AbstractParticleTestWithControls } from "./abstract_particle_test";
import { baseParticleTypes } from "../../utils/particles/particle_parameter";

/**
 * Test the behavior of particles falling onto a concave
 * ambiguous Body contact fixture junction.
 */

class AntiPointy extends AbstractParticleTestWithControls {
    public m_particlesToCreate = 300;

    public constructor({ particleParameter }: TestContext) {
        super(particleParameter);

        {
            const ground = this.m_world.CreateBody();

            // Construct a valley out of many polygons to ensure there's no
            // issue with particles falling directly on an ambiguous set of
            // fixture corners.

            const step = 1;

            for (let i = -10; i < 10; i += step) {
                const shape = new b2PolygonShape();
                const vertices = [new b2Vec2(i, -10), new b2Vec2(i + step, -10), new b2Vec2(0, 15)];
                shape.Set(vertices, 3);
                ground.CreateFixture({ shape });
            }
            for (let i = -10; i < 35; i += step) {
                const shape = new b2PolygonShape();
                const vertices = [new b2Vec2(-10, i), new b2Vec2(-10, i + step), new b2Vec2(0, 15)];
                shape.Set(vertices, 3);
                ground.CreateFixture({ shape });

                const vertices2 = [new b2Vec2(10, i), new b2Vec2(10, i + step), new b2Vec2(0, 15)];
                shape.Set(vertices2, 3);
                ground.CreateFixture({ shape });
            }
        }

        // Cap the number of generated particles or we'll fill forever
        this.m_particlesToCreate = 300;

        this.m_particleSystem.SetRadius(0.25 * 2); // HACK: increase particle radius
        const particleType = particleParameter.GetValue();
        if (particleType === b2ParticleFlag.b2_waterParticle) {
            this.m_particleSystem.SetDamping(0.2);
        }
        particleParameter.SetValues(baseParticleTypes, "water");
    }

    public Step(settings: Settings, timeStep: number) {
        super.Step(settings, timeStep);

        if (this.m_particlesToCreate <= 0) {
            return;
        }

        --this.m_particlesToCreate;

        const flags = this.particleParameter.GetValue();
        const pd = new b2ParticleDef();

        pd.position.Set(0, 40);
        pd.velocity.Set(0, -1);
        pd.flags = flags;

        if (flags & (b2ParticleFlag.b2_springParticle | b2ParticleFlag.b2_elasticParticle)) {
            const count = this.m_particleSystem.GetParticleCount();
            pd.velocity.Set(count & 1 ? -1 : 1, -5);
            pd.flags |= b2ParticleFlag.b2_reactiveParticle;
        }

        this.m_particleSystem.CreateParticle(pd);
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 20,
        };
    }
}

registerTest("Particles", "AntiPointy", AntiPointy);
