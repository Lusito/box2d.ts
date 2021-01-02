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

import { PolygonShape, Vec2, CircleShape, Transform, XY } from "@box2d/core";
import { ParticleGroup, ParticleFlag, ParticleGroupFlag, ParticleGroupDef } from "@box2d/particles";

import { registerTest, TestContext } from "../../test";
import { Settings } from "../../settings";
import { AbstractParticleTestWithControls, particleColors } from "./abstract_particle_test";
import { defaultParticleTypes } from "../../utils/particles/particle_parameter";

const particleTypes = {
    ...defaultParticleTypes,
    erase: ParticleFlag.Zombie,
    rigid: ParticleFlag.Water,
    "rigid barrier": ParticleFlag.Barrier,
    "elastic barrier": ParticleFlag.Barrier | ParticleFlag.Elastic,
    "spring barrier": ParticleFlag.Barrier | ParticleFlag.Spring,
    "repulsive wall": ParticleFlag.Repulsive | ParticleFlag.Wall,
};

const groupFlagsByKey: Record<string, number> = {
    elastic: ParticleGroupFlag.Solid,
    rigid: ParticleGroupFlag.Rigid | ParticleGroupFlag.Solid,
    spring: ParticleGroupFlag.Solid,
    wall: ParticleGroupFlag.Solid,
    "rigid barrier": ParticleGroupFlag.Rigid,
    "elastic barrier": ParticleGroupFlag.Solid,
    "spring barrier": ParticleGroupFlag.Solid,
    "repulsive wall": ParticleGroupFlag.Solid,
};

const reactiveParticleFlags = ParticleFlag.Wall | ParticleFlag.Spring | ParticleFlag.Elastic;

class DrawingParticlesTest extends AbstractParticleTestWithControls {
    public lastGroup: ParticleGroup | null;

    public colorIndex = 0;

    public constructor({ particleParameter }: TestContext) {
        super(particleParameter);

        {
            const ground = this.world.createBody();

            {
                const shape = new PolygonShape();
                const vertices = [new Vec2(-4, -2), new Vec2(4, -2), new Vec2(4, 0), new Vec2(-4, 0)];
                shape.set(vertices, 4);
                ground.createFixture({ shape });
            }

            {
                const shape = new PolygonShape();
                const vertices = [new Vec2(-4, -2), new Vec2(-2, -2), new Vec2(-2, 6), new Vec2(-4, 6)];
                shape.set(vertices, 4);
                ground.createFixture({ shape });
            }

            {
                const shape = new PolygonShape();
                const vertices = [new Vec2(2, -2), new Vec2(4, -2), new Vec2(4, 6), new Vec2(2, 6)];
                shape.set(vertices, 4);
                ground.createFixture({ shape });
            }

            {
                const shape = new PolygonShape();
                const vertices = [new Vec2(-4, 4), new Vec2(4, 4), new Vec2(4, 6), new Vec2(-4, 6)];
                shape.set(vertices, 4);
                ground.createFixture({ shape });
            }
        }

        this.colorIndex = 0;
        this.particleSystem.setRadius(0.05 * 2);
        this.lastGroup = null;

        particleParameter.setValues(particleTypes, "water");
        particleParameter.setRestartOnChange(false);
    }

    public getGroupFlags() {
        return groupFlagsByKey[this.particleParameter.getSelectedKey()] ?? 0;
    }

    public mouseMove(p: Vec2, leftDrag: boolean) {
        super.mouseMove(p, leftDrag);
        if (leftDrag) {
            const parameterValue = this.particleParameter.getValue();
            const shape = new CircleShape();
            shape.p.copy(p);
            shape.radius = 0.2;

            this.particleSystem.destroyParticlesInShape(shape, Transform.IDENTITY);

            const groupFlags = this.getGroupFlags();

            const joinGroup = this.lastGroup && groupFlags === this.lastGroup.getGroupFlags();
            if (!joinGroup) this.colorIndex = (this.colorIndex + 1) % particleColors.length;

            const pd = new ParticleGroupDef();
            pd.shape = shape;
            pd.flags = parameterValue;
            if (parameterValue & reactiveParticleFlags) pd.flags |= ParticleFlag.Reactive;
            pd.groupFlags = groupFlags;
            pd.color.copy(particleColors[this.colorIndex]);
            pd.group = this.lastGroup;
            this.lastGroup = this.particleSystem.createParticleGroup(pd);
            this.mouseTracing = false;
        }
    }

    public mouseUp(p: Vec2) {
        super.mouseUp(p);
        this.lastGroup = null;
    }

    public particleGroupDestroyed(group: ParticleGroup) {
        super.particleGroupDestroyed(group);
        if (group === this.lastGroup) {
            this.lastGroup = null;
        }
    }

    public splitParticleGroups() {
        for (let group = this.particleSystem.getParticleGroupList(); group; group = group.getNext()) {
            if (
                group !== this.lastGroup &&
                group.getGroupFlags() & ParticleGroupFlag.Rigid &&
                group.getAllParticleFlags() & ParticleFlag.Zombie
            ) {
                // Split a rigid particle group which may be disconnected
                // by destroying particles.
                this.particleSystem.splitParticleGroup(group);
            }
        }
    }

    public step(settings: Settings, timeStep: number) {
        if (this.particleSystem.getAllParticleFlags() & ParticleFlag.Zombie) {
            this.splitParticleGroups();
        }

        super.step(settings, timeStep);
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

registerTest("Particles", "Particle Drawing", DrawingParticlesTest);
