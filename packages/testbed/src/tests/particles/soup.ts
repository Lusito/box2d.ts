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

import { Body, PolygonShape, Vec2, BodyType, CircleShape, EdgeShape, MassData, XY } from "@box2d/core";
import { ParticleFlag, ParticleGroupDef } from "@box2d/particles";

import { registerTest, TestContext } from "../../test";
import { baseParticleTypes } from "../../utils/particles/particle_parameter";
import { AbstractParticleTestWithControls } from "./abstract_particle_test";

export const particleTypes = {
    ...baseParticleTypes,
    "color mixing": ParticleFlag.ColorMixing,
};

export class SoupTest extends AbstractParticleTestWithControls {
    public m_ground: Body;

    public constructor({ particleParameter }: TestContext) {
        super(particleParameter);

        particleParameter.SetValues(particleTypes, "water");

        this.m_ground = this.m_world.CreateBody();

        {
            const shape = new PolygonShape();
            const vertices = [new Vec2(-4, -1), new Vec2(4, -1), new Vec2(4, 0), new Vec2(-4, 0)];
            shape.Set(vertices, 4);
            this.m_ground.CreateFixture({ shape });
        }

        {
            const shape = new PolygonShape();
            const vertices = [new Vec2(-4, -0.1), new Vec2(-2, -0.1), new Vec2(-2, 2), new Vec2(-4, 3)];
            shape.Set(vertices, 4);
            this.m_ground.CreateFixture({ shape });
        }

        {
            const shape = new PolygonShape();
            const vertices = [new Vec2(2, -0.1), new Vec2(4, -0.1), new Vec2(4, 3), new Vec2(2, 2)];
            shape.Set(vertices, 4);
            this.m_ground.CreateFixture({ shape });
        }

        this.m_particleSystem.SetRadius(0.035 * 2); // HACK: increase particle radius
        {
            const shape = new PolygonShape();
            shape.SetAsBox(2, 1, new Vec2(0, 1), 0);
            const pd = new ParticleGroupDef();
            pd.shape = shape;
            pd.flags = particleParameter.GetValue();
            const group = this.m_particleSystem.CreateParticleGroup(pd);
            if (pd.flags & ParticleFlag.ColorMixing) {
                this.ColorParticleGroup(group, 0);
            }
        }

        {
            const body = this.m_world.CreateBody({
                type: BodyType.Dynamic,
            });
            const shape = new CircleShape();
            shape.m_p.Set(0, 0.5);
            shape.m_radius = 0.1;
            body.CreateFixture({ shape, density: 0.1 });
            this.m_particleSystem.DestroyParticlesInShape(shape, body.GetTransform());
        }

        {
            const body = this.m_world.CreateBody({
                type: BodyType.Dynamic,
            });
            const shape = new PolygonShape();
            shape.SetAsBox(0.1, 0.1, new Vec2(-1, 0.5), 0);
            body.CreateFixture({ shape, density: 0.1 });
            this.m_particleSystem.DestroyParticlesInShape(shape, body.GetTransform());
        }

        {
            const body = this.m_world.CreateBody({
                type: BodyType.Dynamic,
            });
            const shape = new PolygonShape();
            shape.SetAsBox(0.1, 0.1, new Vec2(1, 0.5), 0.5);
            body.CreateFixture({ shape, density: 0.1 });
            this.m_particleSystem.DestroyParticlesInShape(shape, body.GetTransform());
        }

        {
            const body = this.m_world.CreateBody({
                type: BodyType.Dynamic,
            });
            const shape = new EdgeShape();
            shape.SetTwoSided(new Vec2(0, 2), new Vec2(0.1, 2.1));
            body.CreateFixture({ shape, density: 1 });
            const massData = new MassData();
            massData.mass = 0.1;
            massData.center.x = 0.5 * shape.m_vertex1.x + shape.m_vertex2.x;
            massData.center.y = 0.5 * shape.m_vertex1.y + shape.m_vertex2.y;
            massData.I = 0;
            body.SetMassData(massData);
        }

        {
            const body = this.m_world.CreateBody({
                type: BodyType.Dynamic,
            });
            const shape = new EdgeShape();
            shape.SetTwoSided(new Vec2(0.3, 2), new Vec2(0.4, 2.1));
            body.CreateFixture({ shape, density: 1 });
            const massData = new MassData();
            massData.mass = 0.1;
            massData.center.x = 0.5 * shape.m_vertex1.x + shape.m_vertex2.x;
            massData.center.y = 0.5 * shape.m_vertex1.y + shape.m_vertex2.y;
            massData.I = 0;
            body.SetMassData(massData);
        }

        {
            const body = this.m_world.CreateBody({
                type: BodyType.Dynamic,
            });
            const shape = new EdgeShape();
            shape.SetTwoSided(new Vec2(-0.3, 2.1), new Vec2(-0.2, 2));
            body.CreateFixture({ shape, density: 1 });
            const massData = new MassData();
            massData.mass = 0.1;
            massData.center.x = 0.5 * shape.m_vertex1.x + shape.m_vertex2.x;
            massData.center.y = 0.5 * shape.m_vertex1.y + shape.m_vertex2.y;
            massData.I = 0;
            body.SetMassData(massData);
        }
    }

    public GetDefaultViewZoom() {
        return 250;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 1,
        };
    }
}

registerTest("Particles", "Soup", SoupTest);
