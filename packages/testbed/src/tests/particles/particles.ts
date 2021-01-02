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

import { PolygonShape, Vec2, CircleShape, BodyType, XY } from "@box2d/core";
import { ParticleGroupDef, ParticleFlag } from "@box2d/particles";

import { registerTest, TestContext } from "../../test";
import { AbstractParticleTestWithControls } from "./abstract_particle_test";

class ParticlesTest extends AbstractParticleTestWithControls {
    public constructor({ particleParameter }: TestContext) {
        super(particleParameter);
        {
            const ground = this.world.createBody();

            {
                const shape = new PolygonShape();
                const vertices = [new Vec2(-4, -1), new Vec2(4, -1), new Vec2(4, 0), new Vec2(-4, 0)];
                shape.set(vertices, 4);
                ground.createFixture({ shape });
            }

            {
                const shape = new PolygonShape();
                const vertices = [new Vec2(-4, -0.1), new Vec2(-2, -0.1), new Vec2(-2, 2), new Vec2(-4, 3)];
                shape.set(vertices, 4);
                ground.createFixture({ shape });
            }

            {
                const shape = new PolygonShape();
                const vertices = [new Vec2(2, -0.1), new Vec2(4, -0.1), new Vec2(4, 3), new Vec2(2, 2)];
                shape.set(vertices, 4);
                ground.createFixture({ shape });
            }
        }

        this.particleSystem.setRadius(0.035 * 2); // HACK: increase particle radius
        const particleType = particleParameter.getValue();
        this.particleSystem.setDamping(0.2);

        {
            const shape = new CircleShape();
            shape.p.set(0, 3);
            shape.radius = 2;
            const pd = new ParticleGroupDef();
            pd.flags = particleType;
            pd.shape = shape;
            const group = this.particleSystem.createParticleGroup(pd);
            if (pd.flags & ParticleFlag.ColorMixing) {
                this.colorParticleGroup(group, 0);
            }
        }

        {
            const body = this.world.createBody({
                type: BodyType.Dynamic,
            });
            const shape = new CircleShape();
            shape.p.set(0, 8);
            shape.radius = 0.5;
            body.createFixture({ shape, density: 0.5 });
        }
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

registerTest("Particles", "Particles", ParticlesTest);
