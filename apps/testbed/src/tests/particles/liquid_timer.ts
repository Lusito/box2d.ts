/*
 * Copyright (c) 2013 Google, Inc.
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

import { b2ChainShape, b2Vec2, b2PolygonShape, b2EdgeShape, XY } from "@box2d/core";
import { b2ParticleGroupDef, b2ParticleFlag } from "@box2d/particles";

import { registerTest, TestContext } from "../../test";
import { baseParticleTypes } from "../../utils/particles/particle_parameter";
import { AbstractParticleTestWithControls } from "./abstract_particle_test";

const particleTypes = {
    "tensile + viscous": b2ParticleFlag.b2_tensileParticle | b2ParticleFlag.b2_viscousParticle,
    ...baseParticleTypes,
};

class LiquidTimer extends AbstractParticleTestWithControls {
    public constructor({ particleParameter }: TestContext) {
        super(particleParameter);

        // Setup particle parameters.
        particleParameter.SetValues(particleTypes, "tensile + viscous");

        {
            const ground = this.m_world.CreateBody();

            const shape = new b2ChainShape();
            const vertices = [new b2Vec2(-2, 0), new b2Vec2(2, 0), new b2Vec2(2, 4), new b2Vec2(-2, 4)];
            shape.CreateLoop(vertices, 4);
            ground.CreateFixture({ shape });
        }

        this.m_particleSystem.SetRadius(0.025);
        {
            const shape = new b2PolygonShape();
            shape.SetAsBox(2, 0.4, new b2Vec2(0, 3.6), 0);
            const pd = new b2ParticleGroupDef();
            pd.flags = particleParameter.GetValue();
            pd.shape = shape;
            const group = this.m_particleSystem.CreateParticleGroup(pd);
            if (pd.flags & b2ParticleFlag.b2_colorMixingParticle) {
                this.ColorParticleGroup(group, 0);
            }
        }

        {
            const body = this.m_world.CreateBody();
            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(-2, 3.2), new b2Vec2(-1.2, 3.2));
            body.CreateFixture({ shape, density: 0.1 });
        }

        {
            const body = this.m_world.CreateBody();
            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(-1.1, 3.2), new b2Vec2(2, 3.2));
            body.CreateFixture({ shape, density: 0.1 });
        }

        {
            const body = this.m_world.CreateBody();
            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(-1.2, 3.2), new b2Vec2(-1.2, 2.8));
            body.CreateFixture({ shape, density: 0.1 });
        }

        {
            const body = this.m_world.CreateBody();
            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(-1.1, 3.2), new b2Vec2(-1.1, 2.8));
            body.CreateFixture({ shape, density: 0.1 });
        }

        {
            const body = this.m_world.CreateBody();
            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(-1.6, 2.4), new b2Vec2(0.8, 2));
            body.CreateFixture({ shape, density: 0.1 });
        }

        {
            const body = this.m_world.CreateBody();
            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(1.6, 1.6), new b2Vec2(-0.8, 1.2));
            body.CreateFixture({ shape, density: 0.1 });
        }

        {
            const body = this.m_world.CreateBody();
            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(-1.2, 0.8), new b2Vec2(-1.2, 0));
            body.CreateFixture({ shape, density: 0.1 });
        }

        {
            const body = this.m_world.CreateBody();
            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(-0.4, 0.8), new b2Vec2(-0.4, 0));
            body.CreateFixture({ shape, density: 0.1 });
        }

        {
            const body = this.m_world.CreateBody();
            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(0.4, 0.8), new b2Vec2(0.4, 0));
            body.CreateFixture({ shape, density: 0.1 });
        }

        {
            const body = this.m_world.CreateBody();
            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(1.2, 0.8), new b2Vec2(1.2, 0));
            body.CreateFixture({ shape, density: 0.1 });
        }
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

registerTest("Particles", "Liquid Timer", LiquidTimer);
