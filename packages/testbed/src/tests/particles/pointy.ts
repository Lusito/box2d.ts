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

import { PolygonShape, Transform, Vec2, XY } from "@box2d/core";
import { ParticleFlag, ParticleDef } from "@box2d/particles";

import { registerTest, TestContext } from "../../test";
import { Settings } from "../../settings";
import { AbstractParticleTestWithControls } from "./abstract_particle_test";
import { baseParticleTypes } from "../../utils/particles/particle_parameter";

/**
 * Test behavior when particles fall on a convex ambigious Body
 * contact fixture junction.
 */

class PointyTest extends AbstractParticleTestWithControls {
    public killfieldShape = new PolygonShape();

    public killfieldTransform = new Transform();

    public constructor({ particleParameter }: TestContext) {
        super(particleParameter);

        particleParameter.setValues(baseParticleTypes, "water");

        {
            const ground = this.world.createBody();

            // Construct a triangle out of many polygons to ensure there's no
            // issue with particles falling directly on an ambiguous corner

            const xstep = 1;
            for (let x = -10; x < 10; x += xstep) {
                const shape = new PolygonShape();
                const vertices = [new Vec2(x, -10), new Vec2(x + xstep, -10), new Vec2(0, 25)];
                shape.set(vertices, 3);
                ground.createFixture({ shape });
            }
        }

        this.particleSystem.setRadius(0.25 * 2); // HACK: increase particle radius
        const particleType = particleParameter.getValue();
        if (particleType === ParticleFlag.Water) {
            this.particleSystem.setDamping(0.2);
        }

        // Create killfield shape and transform
        this.killfieldShape = new PolygonShape();
        this.killfieldShape.setAsBox(50, 1);

        // Put this at the bottom of the world
        this.killfieldTransform = new Transform();
        const loc = new Vec2(-25, 1);
        this.killfieldTransform.setPositionAngle(loc, 0);
    }

    public step(settings: Settings, timeStep: number) {
        super.step(settings, timeStep);

        const flags = this.particleParameter.getValue();
        const pd = new ParticleDef();

        pd.position.set(0, 33);
        pd.velocity.set(0, -1);
        pd.flags = flags;

        if (flags & (ParticleFlag.Spring | ParticleFlag.Elastic)) {
            const count = this.particleSystem.getParticleCount();
            pd.velocity.set(count & 1 ? -1 : 1, -5);
            pd.flags |= ParticleFlag.Reactive;
        }

        this.particleSystem.createParticle(pd);

        // kill every particle near the bottom of the screen
        this.particleSystem.destroyParticlesInShape(this.killfieldShape, this.killfieldTransform);
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 20,
        };
    }
}

registerTest("Particles", "Pointy", PointyTest);
