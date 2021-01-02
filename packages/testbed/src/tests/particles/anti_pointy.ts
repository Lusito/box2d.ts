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

import { PolygonShape, Vec2, XY } from "@box2d/core";
import { ParticleFlag, ParticleDef } from "@box2d/particles";

import { registerTest, TestContext } from "../../test";
import { Settings } from "../../settings";
import { AbstractParticleTestWithControls } from "./abstract_particle_test";
import { baseParticleTypes } from "../../utils/particles/particle_parameter";

/**
 * Test the behavior of particles falling onto a concave
 * ambiguous Body contact fixture junction.
 */

class AntiPointyTest extends AbstractParticleTestWithControls {
    public particlesToCreate = 300;

    public constructor({ particleParameter }: TestContext) {
        super(particleParameter);

        {
            const ground = this.world.createBody();

            // Construct a valley out of many polygons to ensure there's no
            // issue with particles falling directly on an ambiguous set of
            // fixture corners.

            const step = 1;

            for (let i = -10; i < 10; i += step) {
                const shape = new PolygonShape();
                const vertices = [new Vec2(i, -10), new Vec2(i + step, -10), new Vec2(0, 15)];
                shape.set(vertices, 3);
                ground.createFixture({ shape });
            }
            for (let i = -10; i < 35; i += step) {
                const shape = new PolygonShape();
                const vertices = [new Vec2(-10, i), new Vec2(-10, i + step), new Vec2(0, 15)];
                shape.set(vertices, 3);
                ground.createFixture({ shape });

                const vertices2 = [new Vec2(10, i), new Vec2(10, i + step), new Vec2(0, 15)];
                shape.set(vertices2, 3);
                ground.createFixture({ shape });
            }
        }

        // Cap the number of generated particles or we'll fill forever
        this.particlesToCreate = 300;

        this.particleSystem.setRadius(0.25 * 2); // HACK: increase particle radius
        const particleType = particleParameter.getValue();
        if (particleType === ParticleFlag.Water) {
            this.particleSystem.setDamping(0.2);
        }
        particleParameter.setValues(baseParticleTypes, "water");
    }

    public step(settings: Settings, timeStep: number) {
        super.step(settings, timeStep);

        if (this.particlesToCreate <= 0) {
            return;
        }

        --this.particlesToCreate;

        const flags = this.particleParameter.getValue();
        const pd = new ParticleDef();

        pd.position.set(0, 40);
        pd.velocity.set(0, -1);
        pd.flags = flags;

        if (flags & (ParticleFlag.Spring | ParticleFlag.Elastic)) {
            const count = this.particleSystem.getParticleCount();
            pd.velocity.set(count & 1 ? -1 : 1, -5);
            pd.flags |= ParticleFlag.Reactive;
        }

        this.particleSystem.createParticle(pd);
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 20,
        };
    }
}

registerTest("Particles", "AntiPointy", AntiPointyTest);
