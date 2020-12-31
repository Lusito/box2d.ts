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

import { Vec2, ChainShape, PolygonShape, XY, Assert } from "@box2d/core";
import { ParticleGroupDef, ParticleFlag } from "@box2d/particles";

import { HotKey, hotKeyPress } from "../../utils/hotkeys";
import { registerTest, TestContext } from "../../test";
import { AbstractParticleTestWithControls } from "./abstract_particle_test";

class ImpulseTest extends AbstractParticleTestWithControls {
    public static readonly kBoxLeft = -2;

    public static readonly kBoxRight = 2;

    public static readonly kBoxBottom = 0;

    public static readonly kBoxTop = 4;

    public m_useLinearImpulse = false;

    public constructor({ particleParameter }: TestContext) {
        super(particleParameter);

        // Create the containing box.
        {
            const ground = this.m_world.CreateBody();

            const box = [
                new Vec2(ImpulseTest.kBoxLeft, ImpulseTest.kBoxBottom),
                new Vec2(ImpulseTest.kBoxRight, ImpulseTest.kBoxBottom),
                new Vec2(ImpulseTest.kBoxRight, ImpulseTest.kBoxTop),
                new Vec2(ImpulseTest.kBoxLeft, ImpulseTest.kBoxTop),
            ];
            const shape = new ChainShape();
            shape.CreateLoop(box, box.length);
            ground.CreateFixture({ shape });
        }

        this.m_particleSystem.SetRadius(0.025 * 2); // HACK: increase particle radius
        this.m_particleSystem.SetDamping(0.2);

        // Create the particles.
        {
            const shape = new PolygonShape();
            shape.SetAsBox(0.8, 1, new Vec2(0, 1.01), 0);
            const pd = new ParticleGroupDef();
            pd.flags = particleParameter.GetValue();
            pd.shape = shape;
            const group = this.m_particleSystem.CreateParticleGroup(pd);
            if (pd.flags & ParticleFlag.ColorMixing) {
                this.ColorParticleGroup(group, 0);
            }
        }
    }

    public MouseUp(p: Vec2) {
        super.MouseUp(p);

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
            const direction = Vec2.Subtract(p, kBoxCenter, new Vec2());
            direction.Normalize();
            this.ApplyImpulseOrForce(direction);
        }
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("l", "Use Linear ImpulseTest", () => {
                this.m_useLinearImpulse = true;
            }),
            hotKeyPress("f", "Use Force", () => {
                this.m_useLinearImpulse = false;
            }),
        ];
    }

    public ApplyImpulseOrForce(direction: Vec2) {
        const particleSystem = this.m_world.GetParticleSystemList();
        Assert(particleSystem !== null);
        const particleGroup = particleSystem.GetParticleGroupList();
        Assert(particleGroup !== null);
        const numParticles = particleGroup.GetParticleCount();

        if (this.m_useLinearImpulse) {
            const kImpulseMagnitude = 0.005;
            const impulse = Vec2.Scale(kImpulseMagnitude * numParticles, direction, new Vec2());
            particleGroup.ApplyLinearImpulse(impulse);
        } else {
            const kForceMagnitude = 1;
            const force = Vec2.Scale(kForceMagnitude * numParticles, direction, new Vec2());
            particleGroup.ApplyForce(force);
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

registerTest("Particles", "Impulse", ImpulseTest);
