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

import { ChainShape, Vec2, PolygonShape, XY } from "@box2d/core";
import { ParticleGroupDef, ParticleFlag } from "@box2d/particles";

import { registerTest, TestContext } from "../../test";
import { AbstractParticleTestWithControls } from "./abstract_particle_test";

class DamBreakTest extends AbstractParticleTestWithControls {
    public constructor({ particleParameter }: TestContext) {
        super(particleParameter);

        {
            const ground = this.m_world.CreateBody();

            const shape = new ChainShape();
            const vertices = [new Vec2(-2, 0), new Vec2(2, 0), new Vec2(2, 4), new Vec2(-2, 4)];
            shape.CreateLoop(vertices, 4);
            ground.CreateFixture({ shape });
        }

        this.m_particleSystem.SetRadius(0.025 * 2); // HACK: increase particle radius
        this.m_particleSystem.SetDamping(0.2);

        {
            const shape = new PolygonShape();
            shape.SetAsBox(0.8, 1, new Vec2(-1.2, 1.01), 0);
            const pd = new ParticleGroupDef();
            pd.flags = particleParameter.GetValue();
            pd.shape = shape;
            const group = this.m_particleSystem.CreateParticleGroup(pd);
            if (pd.flags & ParticleFlag.ColorMixing) {
                this.ColorParticleGroup(group, 0);
            }
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

registerTest("Particles", "DamBreak", DamBreakTest);
