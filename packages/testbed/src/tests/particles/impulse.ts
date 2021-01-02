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

import { Vec2, ChainShape, PolygonShape, XY, assert } from "@box2d/core";
import { ParticleGroupDef, ParticleFlag } from "@box2d/particles";

import { HotKey, hotKeyPress } from "../../utils/hotkeys";
import { registerTest, TestContext } from "../../test";
import { AbstractParticleTestWithControls } from "./abstract_particle_test";

class ImpulseTest extends AbstractParticleTestWithControls {
    public static readonly kBoxLeft = -2;

    public static readonly kBoxRight = 2;

    public static readonly kBoxBottom = 0;

    public static readonly kBoxTop = 4;

    public useLinearImpulse = false;

    public constructor({ particleParameter }: TestContext) {
        super(particleParameter);

        // Create the containing box.
        {
            const ground = this.world.createBody();

            const box = [
                new Vec2(ImpulseTest.kBoxLeft, ImpulseTest.kBoxBottom),
                new Vec2(ImpulseTest.kBoxRight, ImpulseTest.kBoxBottom),
                new Vec2(ImpulseTest.kBoxRight, ImpulseTest.kBoxTop),
                new Vec2(ImpulseTest.kBoxLeft, ImpulseTest.kBoxTop),
            ];
            const shape = new ChainShape();
            shape.createLoop(box, box.length);
            ground.createFixture({ shape });
        }

        this.particleSystem.setRadius(0.025 * 2); // HACK: increase particle radius
        this.particleSystem.setDamping(0.2);

        // Create the particles.
        {
            const shape = new PolygonShape();
            shape.setAsBox(0.8, 1, new Vec2(0, 1.01), 0);
            const pd = new ParticleGroupDef();
            pd.flags = particleParameter.getValue();
            pd.shape = shape;
            const group = this.particleSystem.createParticleGroup(pd);
            if (pd.flags & ParticleFlag.ColorMixing) {
                this.colorParticleGroup(group, 0);
            }
        }
    }

    public mouseUp(p: Vec2) {
        super.mouseUp(p);

        // Apply an impulse to the particles.
        const isInsideBox =
            ImpulseTest.kBoxLeft <= p.x &&
            p.x <= ImpulseTest.kBoxRight &&
            ImpulseTest.kBoxBottom <= p.y &&
            p.y <= ImpulseTest.kBoxTop;
        if (isInsideBox) {
            const kBoxCenter = new Vec2(
                0.5 * (ImpulseTest.kBoxLeft + ImpulseTest.kBoxRight),
                0.5 * (ImpulseTest.kBoxBottom + ImpulseTest.kBoxTop),
            );
            const direction = Vec2.subtract(p, kBoxCenter, new Vec2());
            direction.normalize();
            this.applyImpulseOrForce(direction);
        }
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("l", "Use Linear ImpulseTest", () => {
                this.useLinearImpulse = true;
            }),
            hotKeyPress("f", "Use Force", () => {
                this.useLinearImpulse = false;
            }),
        ];
    }

    public applyImpulseOrForce(direction: Vec2) {
        const particleSystem = this.world.getParticleSystemList();
        assert(particleSystem !== null);
        const particleGroup = particleSystem.getParticleGroupList();
        assert(particleGroup !== null);
        const numParticles = particleGroup.getParticleCount();

        if (this.useLinearImpulse) {
            const kImpulseMagnitude = 0.005;
            const impulse = Vec2.scale(kImpulseMagnitude * numParticles, direction, new Vec2());
            particleGroup.applyLinearImpulse(impulse);
        } else {
            const kForceMagnitude = 1;
            const force = Vec2.scale(kForceMagnitude * numParticles, direction, new Vec2());
            particleGroup.applyForce(force);
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

registerTest("Particles", "Impulse", ImpulseTest);
