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

import { PolygonShape, Vec2, CircleShape, BodyType, XY } from "@box2d/core";
import { ParticleGroupDef, ParticleFlag, ParticleGroupFlag } from "@box2d/particles";

import { registerTest } from "../../test";
import { AbstractParticleTest } from "./abstract_particle_test";

class ElasticParticlesTest extends AbstractParticleTest {
    public constructor() {
        super();
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
                const vertices = [new Vec2(-4, -0.1), new Vec2(-2, -0.1), new Vec2(-2, 2), new Vec2(-4, 2)];
                shape.set(vertices, 4);
                ground.createFixture({ shape });
            }

            {
                const shape = new PolygonShape();
                const vertices = [new Vec2(2, -0.1), new Vec2(4, -0.1), new Vec2(4, 2), new Vec2(2, 2)];
                shape.set(vertices, 4);
                ground.createFixture({ shape });
            }
        }

        this.particleSystem.setRadius(0.035 * 2); // HACK: increase particle radius

        {
            const shape = new CircleShape();
            shape.p.set(0, 3);
            shape.radius = 0.5;
            const pd = new ParticleGroupDef();
            pd.flags = ParticleFlag.Spring;
            pd.groupFlags = ParticleGroupFlag.Solid;
            pd.shape = shape;
            pd.color.setRGBA(1, 0, 0, 1);
            this.particleSystem.createParticleGroup(pd);
        }

        {
            const shape = new CircleShape();
            shape.p.set(-1, 3);
            shape.radius = 0.5;
            const pd = new ParticleGroupDef();
            pd.flags = ParticleFlag.Elastic;
            pd.groupFlags = ParticleGroupFlag.Solid;
            pd.shape = shape;
            pd.color.setRGBA(0, 1, 0, 1);
            this.particleSystem.createParticleGroup(pd);
        }

        {
            const shape = new PolygonShape();
            shape.setAsBox(1, 0.5);
            const pd = new ParticleGroupDef();
            pd.flags = ParticleFlag.Elastic;
            pd.groupFlags = ParticleGroupFlag.Solid;
            pd.position.set(1, 4);
            pd.angle = -0.5;
            pd.angularVelocity = 2;
            pd.shape = shape;
            pd.color.setRGBA(0, 0, 1, 1);
            this.particleSystem.createParticleGroup(pd);
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
            y: 1,
        };
    }

    public getParticleSelectionRadius() {
        return 0.4;
    }
}

registerTest("Particles", "Elastic Particles", ElasticParticlesTest);
