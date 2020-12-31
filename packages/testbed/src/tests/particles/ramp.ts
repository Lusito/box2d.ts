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

import { PolygonShape, Vec2, CircleShape, XY } from "@box2d/core";
import { ParticleFlag, ParticleGroupDef } from "@box2d/particles";

import { registerTest, TestContext } from "../../test";
import { AbstractParticleTestWithControls } from "./abstract_particle_test";

class RampTest extends AbstractParticleTestWithControls {
    public constructor({ particleParameter }: TestContext) {
        super(particleParameter);

        {
            const ground = this.m_world.CreateBody();

            // Construct a ramp out of many polygons to ensure there's no
            // issue with particles moving across vertices

            const xstep = 5;
            const ystep = 5;

            for (let y = 30; y > 0; y -= ystep) {
                const shape = new PolygonShape();
                const vertices = [new Vec2(-25, y), new Vec2(-25, y - ystep), new Vec2(0, 15)];
                shape.Set(vertices, 3);
                ground.CreateFixture({ shape });
            }

            for (let x = -25; x < 25; x += xstep) {
                const shape = new PolygonShape();
                const vertices = [new Vec2(x, 0), new Vec2(x + xstep, 0), new Vec2(0, 15)];
                shape.Set(vertices, 3);
                ground.CreateFixture({ shape });
            }
        }

        this.m_particleSystem.SetRadius(0.25);
        const particleType = particleParameter.GetValue();
        if (particleType === ParticleFlag.Water) {
            this.m_particleSystem.SetDamping(0.2);
        }

        {
            const shape = new CircleShape();
            shape.m_p.Set(-20, 33);
            shape.m_radius = 3;
            const pd = new ParticleGroupDef();
            pd.flags = particleType;
            pd.shape = shape;
            const group = this.m_particleSystem.CreateParticleGroup(pd);
            if (pd.flags & ParticleFlag.ColorMixing) {
                this.ColorParticleGroup(group, 0);
            }
        }
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 10,
        };
    }
}

registerTest("Particles", "Ramp", RampTest);
