/*
 * Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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

import { b2PolygonShape, b2Vec2, b2CircleShape, b2BodyType, XY } from "@box2d/core";
import { b2ParticleGroupDef, b2ParticleFlag, b2ParticleGroupFlag } from "@box2d/particles";

import { registerTest } from "../../test";
import { AbstractParticleTest } from "./abstract_particle_test";

class ElasticParticles extends AbstractParticleTest {
    public constructor() {
        super();
        {
            const ground = this.m_world.CreateBody();

            {
                const shape = new b2PolygonShape();
                const vertices = [new b2Vec2(-4, -1), new b2Vec2(4, -1), new b2Vec2(4, 0), new b2Vec2(-4, 0)];
                shape.Set(vertices, 4);
                ground.CreateFixture({ shape });
            }

            {
                const shape = new b2PolygonShape();
                const vertices = [new b2Vec2(-4, -0.1), new b2Vec2(-2, -0.1), new b2Vec2(-2, 2), new b2Vec2(-4, 2)];
                shape.Set(vertices, 4);
                ground.CreateFixture({ shape });
            }

            {
                const shape = new b2PolygonShape();
                const vertices = [new b2Vec2(2, -0.1), new b2Vec2(4, -0.1), new b2Vec2(4, 2), new b2Vec2(2, 2)];
                shape.Set(vertices, 4);
                ground.CreateFixture({ shape });
            }
        }

        this.m_particleSystem.SetRadius(0.035 * 2); // HACK: increase particle radius

        {
            const shape = new b2CircleShape();
            shape.m_p.Set(0, 3);
            shape.m_radius = 0.5;
            const pd = new b2ParticleGroupDef();
            pd.flags = b2ParticleFlag.b2_springParticle;
            pd.groupFlags = b2ParticleGroupFlag.b2_solidParticleGroup;
            pd.shape = shape;
            pd.color.SetRGBA(1, 0, 0, 1);
            this.m_particleSystem.CreateParticleGroup(pd);
        }

        {
            const shape = new b2CircleShape();
            shape.m_p.Set(-1, 3);
            shape.m_radius = 0.5;
            const pd = new b2ParticleGroupDef();
            pd.flags = b2ParticleFlag.b2_elasticParticle;
            pd.groupFlags = b2ParticleGroupFlag.b2_solidParticleGroup;
            pd.shape = shape;
            pd.color.SetRGBA(0, 1, 0, 1);
            this.m_particleSystem.CreateParticleGroup(pd);
        }

        {
            const shape = new b2PolygonShape();
            shape.SetAsBox(1, 0.5);
            const pd = new b2ParticleGroupDef();
            pd.flags = b2ParticleFlag.b2_elasticParticle;
            pd.groupFlags = b2ParticleGroupFlag.b2_solidParticleGroup;
            pd.position.Set(1, 4);
            pd.angle = -0.5;
            pd.angularVelocity = 2;
            pd.shape = shape;
            pd.color.SetRGBA(0, 0, 1, 1);
            this.m_particleSystem.CreateParticleGroup(pd);
        }

        {
            const body = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
            });
            const shape = new b2CircleShape();
            shape.m_p.Set(0, 8);
            shape.m_radius = 0.5;
            body.CreateFixture({ shape, density: 0.5 });
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

    public getParticleSelectionRadius() {
        return 0.4;
    }
}

registerTest("Particles", "Elastic Particles", ElasticParticles);
