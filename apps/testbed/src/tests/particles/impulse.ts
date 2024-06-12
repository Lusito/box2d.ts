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

import { b2Vec2, b2ChainShape, b2PolygonShape, XY, b2Assert } from "@box2d/core";
import { b2ParticleGroupDef, b2ParticleFlag } from "@box2d/particles";

import { HotKey, hotKeyPress } from "../../utils/hotkeys";
import { registerTest, TestContext } from "../../test";
import { AbstractParticleTestWithControls } from "./abstract_particle_test";

class Impulse extends AbstractParticleTestWithControls {
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
                new b2Vec2(Impulse.kBoxLeft, Impulse.kBoxBottom),
                new b2Vec2(Impulse.kBoxRight, Impulse.kBoxBottom),
                new b2Vec2(Impulse.kBoxRight, Impulse.kBoxTop),
                new b2Vec2(Impulse.kBoxLeft, Impulse.kBoxTop),
            ];
            const shape = new b2ChainShape();
            shape.CreateLoop(box, box.length);
            ground.CreateFixture({ shape });
        }

        this.m_particleSystem.SetRadius(0.025 * 2); // HACK: increase particle radius
        this.m_particleSystem.SetDamping(0.2);

        // Create the particles.
        {
            const shape = new b2PolygonShape();
            shape.SetAsBox(0.8, 1, new b2Vec2(0, 1.01), 0);
            const pd = new b2ParticleGroupDef();
            pd.flags = particleParameter.GetValue();
            pd.shape = shape;
            const group = this.m_particleSystem.CreateParticleGroup(pd);
            if (pd.flags & b2ParticleFlag.b2_colorMixingParticle) {
                this.ColorParticleGroup(group, 0);
            }
        }
    }

    public MouseUp(p: b2Vec2) {
        super.MouseUp(p);

        // Apply an impulse to the particles.
        const isInsideBox =
            Impulse.kBoxLeft <= p.x && p.x <= Impulse.kBoxRight && Impulse.kBoxBottom <= p.y && p.y <= Impulse.kBoxTop;
        if (isInsideBox) {
            const kBoxCenter = new b2Vec2(
                0.5 * (Impulse.kBoxLeft + Impulse.kBoxRight),
                0.5 * (Impulse.kBoxBottom + Impulse.kBoxTop),
            );
            const direction = b2Vec2.Subtract(p, kBoxCenter, new b2Vec2());
            direction.Normalize();
            this.ApplyImpulseOrForce(direction);
        }
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("l", "Use Linear Impulse", () => {
                this.m_useLinearImpulse = true;
            }),
            hotKeyPress("f", "Use Force", () => {
                this.m_useLinearImpulse = false;
            }),
        ];
    }

    public ApplyImpulseOrForce(direction: b2Vec2) {
        const particleSystem = this.m_world.GetParticleSystemList();
        b2Assert(particleSystem !== null);
        const particleGroup = particleSystem.GetParticleGroupList();
        b2Assert(particleGroup !== null);
        const numParticles = particleGroup.GetParticleCount();

        if (this.m_useLinearImpulse) {
            const kImpulseMagnitude = 0.005;
            const impulse = b2Vec2.Scale(kImpulseMagnitude * numParticles, direction, new b2Vec2());
            particleGroup.ApplyLinearImpulse(impulse);
        } else {
            const kForceMagnitude = 1;
            const force = b2Vec2.Scale(kForceMagnitude * numParticles, direction, new b2Vec2());
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

registerTest("Particles", "Impulse", Impulse);
