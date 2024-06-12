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

import { b2PolygonShape, b2Vec2, b2CircleShape, b2Transform, XY } from "@box2d/core";
import { b2ParticleGroup, b2ParticleFlag, b2ParticleGroupFlag, b2ParticleGroupDef } from "@box2d/particles";

import { registerTest, TestContext } from "../../test";
import { Settings } from "../../settings";
import { AbstractParticleTestWithControls, particleColors } from "./abstract_particle_test";
import { defaultParticleTypes } from "../../utils/particles/particle_parameter";

const particleTypes = {
    ...defaultParticleTypes,
    erase: b2ParticleFlag.b2_zombieParticle,
    rigid: b2ParticleFlag.b2_waterParticle,
    "rigid barrier": b2ParticleFlag.b2_barrierParticle,
    "elastic barrier": b2ParticleFlag.b2_barrierParticle | b2ParticleFlag.b2_elasticParticle,
    "spring barrier": b2ParticleFlag.b2_barrierParticle | b2ParticleFlag.b2_springParticle,
    "repulsive wall": b2ParticleFlag.b2_repulsiveParticle | b2ParticleFlag.b2_wallParticle,
};

const groupFlagsByKey: Record<string, number> = {
    elastic: b2ParticleGroupFlag.b2_solidParticleGroup,
    rigid: b2ParticleGroupFlag.b2_rigidParticleGroup | b2ParticleGroupFlag.b2_solidParticleGroup,
    spring: b2ParticleGroupFlag.b2_solidParticleGroup,
    wall: b2ParticleGroupFlag.b2_solidParticleGroup,
    "rigid barrier": b2ParticleGroupFlag.b2_rigidParticleGroup,
    "elastic barrier": b2ParticleGroupFlag.b2_solidParticleGroup,
    "spring barrier": b2ParticleGroupFlag.b2_solidParticleGroup,
    "repulsive wall": b2ParticleGroupFlag.b2_solidParticleGroup,
};

const reactiveParticleFlags =
    b2ParticleFlag.b2_wallParticle | b2ParticleFlag.b2_springParticle | b2ParticleFlag.b2_elasticParticle;

class DrawingParticles extends AbstractParticleTestWithControls {
    public m_lastGroup: b2ParticleGroup | null;

    public m_colorIndex = 0;

    public constructor({ particleParameter }: TestContext) {
        super(particleParameter);

        {
            const ground = this.m_world.CreateBody();

            {
                const shape = new b2PolygonShape();
                const vertices = [new b2Vec2(-4, -2), new b2Vec2(4, -2), new b2Vec2(4, 0), new b2Vec2(-4, 0)];
                shape.Set(vertices, 4);
                ground.CreateFixture({ shape });
            }

            {
                const shape = new b2PolygonShape();
                const vertices = [new b2Vec2(-4, -2), new b2Vec2(-2, -2), new b2Vec2(-2, 6), new b2Vec2(-4, 6)];
                shape.Set(vertices, 4);
                ground.CreateFixture({ shape });
            }

            {
                const shape = new b2PolygonShape();
                const vertices = [new b2Vec2(2, -2), new b2Vec2(4, -2), new b2Vec2(4, 6), new b2Vec2(2, 6)];
                shape.Set(vertices, 4);
                ground.CreateFixture({ shape });
            }

            {
                const shape = new b2PolygonShape();
                const vertices = [new b2Vec2(-4, 4), new b2Vec2(4, 4), new b2Vec2(4, 6), new b2Vec2(-4, 6)];
                shape.Set(vertices, 4);
                ground.CreateFixture({ shape });
            }
        }

        this.m_colorIndex = 0;
        this.m_particleSystem.SetRadius(0.05 * 2);
        this.m_lastGroup = null;

        particleParameter.SetValues(particleTypes, "water");
        particleParameter.SetRestartOnChange(false);
    }

    public getGroupFlags() {
        return groupFlagsByKey[this.particleParameter.GetSelectedKey()] ?? 0;
    }

    public MouseMove(p: b2Vec2, leftDrag: boolean) {
        super.MouseMove(p, leftDrag);
        if (leftDrag) {
            const parameterValue = this.particleParameter.GetValue();
            const shape = new b2CircleShape();
            shape.m_p.Copy(p);
            shape.m_radius = 0.2;

            this.m_particleSystem.DestroyParticlesInShape(shape, b2Transform.IDENTITY);

            const groupFlags = this.getGroupFlags();

            const joinGroup = this.m_lastGroup && groupFlags === this.m_lastGroup.GetGroupFlags();
            if (!joinGroup) this.m_colorIndex = (this.m_colorIndex + 1) % particleColors.length;

            const pd = new b2ParticleGroupDef();
            pd.shape = shape;
            pd.flags = parameterValue;
            if (parameterValue & reactiveParticleFlags) pd.flags |= b2ParticleFlag.b2_reactiveParticle;
            pd.groupFlags = groupFlags;
            pd.color.Copy(particleColors[this.m_colorIndex]);
            pd.group = this.m_lastGroup;
            this.m_lastGroup = this.m_particleSystem.CreateParticleGroup(pd);
            this.m_mouseTracing = false;
        }
    }

    public MouseUp(p: b2Vec2) {
        super.MouseUp(p);
        this.m_lastGroup = null;
    }

    public ParticleGroupDestroyed(group: b2ParticleGroup) {
        super.ParticleGroupDestroyed(group);
        if (group === this.m_lastGroup) {
            this.m_lastGroup = null;
        }
    }

    public SplitParticleGroups() {
        for (let group = this.m_particleSystem.GetParticleGroupList(); group; group = group.GetNext()) {
            if (
                group !== this.m_lastGroup &&
                group.GetGroupFlags() & b2ParticleGroupFlag.b2_rigidParticleGroup &&
                group.GetAllParticleFlags() & b2ParticleFlag.b2_zombieParticle
            ) {
                // Split a rigid particle group which may be disconnected
                // by destroying particles.
                this.m_particleSystem.SplitParticleGroup(group);
            }
        }
    }

    public Step(settings: Settings, timeStep: number) {
        if (this.m_particleSystem.GetAllParticleFlags() & b2ParticleFlag.b2_zombieParticle) {
            this.SplitParticleGroups();
        }

        super.Step(settings, timeStep);
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

registerTest("Particles", "Particle Drawing", DrawingParticles);
