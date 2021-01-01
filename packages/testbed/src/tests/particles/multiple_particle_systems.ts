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

import { Vec2, Color, PolygonShape, BodyType, MassData, XY } from "@box2d/core";
import { ParticleSystem, ParticleSystemDef } from "@box2d/particles";

import { registerTest } from "../../test";
import { Settings } from "../../settings";
import { RadialEmitter } from "../../utils/particles/particle_emitter";
import { AbstractParticleTest } from "./abstract_particle_test";

class MultipleParticleSystemsTest extends AbstractParticleTest {
    public m_particleSystem2: ParticleSystem;

    public m_emitters: RadialEmitter[];

    /** Maximum number of particles per system. */
    public static readonly k_maxParticleCount = 500;

    /** Size of the box which is pushed around by particles. */
    public static readonly k_dynamicBoxSize = new Vec2(0.5, 0.5);

    /** Mass of the box. */
    public static readonly k_boxMass = 1;

    /** Emit rate of the emitters in particles per second. */
    public static readonly k_emitRate = 100;

    /**
     * Location of the left emitter (the position of the right one
     * is mirrored along the y-axis).
     */
    public static readonly k_emitterPosition = new Vec2(-5, 4);

    /**
     * Starting velocity of particles from the left emitter (the
     * velocity of particles from the right emitter are mirrored
     * along the y-axis).
     */
    public static readonly k_emitterVelocity = new Vec2(7, -4);

    /** Size of particle emitters. */
    public static readonly k_emitterSize = new Vec2(1, 1);

    /** Color of the left emitter's particles. */
    public static readonly k_leftEmitterColor = new Color().setByteRGBA(0x22, 0x33, 0xff, 0xff);

    /** Color of the right emitter's particles. */
    public static readonly k_rightEmitterColor = new Color().setByteRGBA(0xff, 0x22, 0x11, 0xff);

    public constructor() {
        super();

        this.m_emitters = [new RadialEmitter(), new RadialEmitter()];

        // Configure the default particle system's parameters.
        this.m_particleSystem.setRadius(0.05);
        this.m_particleSystem.setMaxParticleCount(MultipleParticleSystemsTest.k_maxParticleCount);
        this.m_particleSystem.setDestructionByAge(true);

        // Create a secondary particle system.
        const particleSystemDef = new ParticleSystemDef();
        particleSystemDef.radius = this.m_particleSystem.getRadius();
        particleSystemDef.destroyByAge = true;
        this.m_particleSystem2 = this.m_world.createParticleSystem(particleSystemDef);
        this.m_particleSystem2.setMaxParticleCount(MultipleParticleSystemsTest.k_maxParticleCount);

        // Create the ground.
        {
            const ground = this.m_world.createBody();
            const shape = new PolygonShape();
            shape.setAsBox(5, 0.1);
            ground.createFixture({ shape });
        }

        // Create a dynamic body to push around.
        {
            const body = this.m_world.createBody({
                type: BodyType.Dynamic,
            });
            const shape = new PolygonShape();
            const center = new Vec2(0, 1.2);
            shape.setAsBox(
                MultipleParticleSystemsTest.k_dynamicBoxSize.x,
                MultipleParticleSystemsTest.k_dynamicBoxSize.y,
                center,
                0,
            );
            body.createFixture({ shape });
            const massData = new MassData();
            massData.mass = MultipleParticleSystemsTest.k_boxMass;
            massData.center.copy(center);
            massData.I = 0;
            body.setMassData(massData);
        }

        // Initialize the emitters.
        for (let i = 0; i < this.m_emitters.length; ++i) {
            const mirrorAlongY = i & 1 ? -1 : 1;
            const emitter = this.m_emitters[i];
            emitter.setPosition(
                new Vec2(
                    MultipleParticleSystemsTest.k_emitterPosition.x * mirrorAlongY,
                    MultipleParticleSystemsTest.k_emitterPosition.y,
                ),
            );
            emitter.setSize(MultipleParticleSystemsTest.k_emitterSize);
            emitter.setVelocity(
                new Vec2(
                    MultipleParticleSystemsTest.k_emitterVelocity.x * mirrorAlongY,
                    MultipleParticleSystemsTest.k_emitterVelocity.y,
                ),
            );
            emitter.setEmitRate(MultipleParticleSystemsTest.k_emitRate);
            emitter.setColor(
                i & 1
                    ? MultipleParticleSystemsTest.k_rightEmitterColor
                    : MultipleParticleSystemsTest.k_leftEmitterColor,
            );
            emitter.setParticleSystem(i & 1 ? this.m_particleSystem2 : this.m_particleSystem);
        }
    }

    public step(settings: Settings, timeStep: number) {
        let dt = settings.m_hertz > 0 ? 1 / settings.m_hertz : 0;
        if (settings.m_pause && !settings.m_singleStep) {
            dt = 0;
        }

        this.m_particleSystem2.setStrictContactCheck(AbstractParticleTest.m_strictContacts);

        super.step(settings, timeStep);

        for (const emitter of this.m_emitters) {
            emitter.step(dt);
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
}

registerTest("Particles", "Multiple Systems", MultipleParticleSystemsTest);
