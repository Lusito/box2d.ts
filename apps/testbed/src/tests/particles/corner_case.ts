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

import { b2PolygonShape, b2Vec2, b2CircleShape, XY } from "@box2d/core";
import { b2ParticleGroupDef, b2ParticleFlag } from "@box2d/particles";

import { registerTest, TestContext } from "../../test";
import { AbstractParticleTestWithControls } from "./abstract_particle_test";

class CornerCase extends AbstractParticleTestWithControls {
    public constructor({ particleParameter }: TestContext) {
        super(particleParameter);

        {
            const ground = this.m_world.CreateBody();

            // Construct a pathological corner intersection out of many
            // polygons to ensure there's no issue with particle oscillation
            // from many fixture contact impulses at the corner

            // left edge
            {
                const shape = new b2PolygonShape();
                const vertices = [new b2Vec2(-20, 30), new b2Vec2(-20, 0), new b2Vec2(-25, 0), new b2Vec2(-25, 30)];
                shape.Set(vertices);
                ground.CreateFixture({ shape });
            }

            const yrange = 30;
            const ystep = yrange / 10;
            const xrange = 20;
            const xstep = xrange / 2;

            {
                const shape = new b2PolygonShape();
                const vertices = [new b2Vec2(-25, 0), new b2Vec2(20, 15), new b2Vec2(25, 0)];
                shape.Set(vertices);
                ground.CreateFixture({ shape });
            }

            for (let x = -xrange; x < xrange; x += xstep) {
                const shape = new b2PolygonShape();
                const vertices = [new b2Vec2(-25, 0), new b2Vec2(x, 15), new b2Vec2(x + xstep, 15)];
                shape.Set(vertices);
                ground.CreateFixture({ shape });
            }

            for (let y = 0; y < yrange; y += ystep) {
                const shape = new b2PolygonShape();
                const vertices = [new b2Vec2(25, y), new b2Vec2(25, y + ystep), new b2Vec2(20, 15)];
                shape.Set(vertices);
                ground.CreateFixture({ shape });
            }
        }

        this.m_particleSystem.SetRadius(1);
        const particleType = particleParameter.GetValue();

        {
            const shape = new b2CircleShape();
            shape.m_p.Set(0, 35);
            shape.m_radius = 12;
            const pd = new b2ParticleGroupDef();
            pd.flags = particleType;
            pd.shape = shape;
            const group = this.m_particleSystem.CreateParticleGroup(pd);
            if (pd.flags & b2ParticleFlag.b2_colorMixingParticle) {
                this.ColorParticleGroup(group, 0);
            }
        }
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 20,
        };
    }
}

registerTest("Particles", "Corner Case", CornerCase);
