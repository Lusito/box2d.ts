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

import { AABB, CircleShape, Color, Transform, XY } from "@box2d/core";
import { ParticleSystem, ParticleSystemDef, ParticleGroup } from "@box2d/particles";

import { Test } from "../../test";
import { Settings } from "../../settings";
import { ParticleParameter } from "../../utils/particles/particle_parameter";
import { checkboxDef } from "../../ui/controls/Checkbox";

export const particleColors = [
    new Color().setByteRGBA(0xff, 0x00, 0x00, 0xff), // red
    new Color().setByteRGBA(0x00, 0xff, 0x00, 0xff), // green
    new Color().setByteRGBA(0x00, 0x00, 0xff, 0xff), // blue
    new Color().setByteRGBA(0xff, 0x8c, 0x00, 0xff), // orange
    new Color().setByteRGBA(0x00, 0xce, 0xd1, 0xff), // turquoise
    new Color().setByteRGBA(0xff, 0x00, 0xff, 0xff), // magenta
    new Color().setByteRGBA(0xff, 0xd7, 0x00, 0xff), // gold
    new Color().setByteRGBA(0x00, 0xff, 0xff, 0xff), // cyan
];

export class AbstractParticleTest extends Test {
    public particleSystem: ParticleSystem;

    protected static strictContacts = false;

    public constructor(gravity: XY = { x: 0, y: -10 }) {
        super(gravity);

        const particleSystemDef = new ParticleSystemDef();

        this.particleSystem = this.world.createParticleSystem(particleSystemDef);

        this.particleSystem.setGravityScale(0.4);
        this.particleSystem.setDensity(1.2);
    }

    public setupControls() {
        this.addTestControlGroup("Particles", [this.createStrictContactsCheckbox()]);
    }

    protected createStrictContactsCheckbox() {
        return checkboxDef("Strict Particle/Body Contacts", AbstractParticleTest.strictContacts, (value) => {
            AbstractParticleTest.strictContacts = value;
        });
    }

    public step(settings: Settings, timeStep: number) {
        super.step(settings, timeStep);

        this.particleSystem.setStrictContactCheck(AbstractParticleTest.strictContacts);

        if (settings.drawStats) {
            this.addStatistic("Particles", this.particleSystem.getParticleCount());
            this.addStatistic("Groups", this.particleSystem.getParticleGroupCount());
            this.addStatistic("Pairs", this.particleSystem.getPairCount());
            this.addStatistic("Triads", this.particleSystem.getTriadCount());
        }

        if (this.mouseTracing && !this.mouseJoint) {
            const shape = new CircleShape();
            shape.p.copy(this.mouseTracerPosition);
            shape.radius = this.getParticleSelectionRadius();
            const aabb = new AABB();
            const xf = new Transform();
            xf.setIdentity();
            shape.computeAABB(aabb, xf, 0);
            this.particleSystem.queryAABB(aabb, (index) => {
                const p = this.particleSystem.getPositionBuffer()[index];
                if (shape.testPoint(Transform.IDENTITY, p)) {
                    const v = this.particleSystem.getVelocityBuffer()[index];
                    v.copy(this.mouseTracerVelocity);
                }
                return true;
            });
        }
    }

    /**
     * Apply a preset range of colors to a particle group.
     *
     * A different color out of k_ParticleColors is applied to each
     * particlesPerColor particles in the specified group.
     *
     * If particlesPerColor is 0, the particles in the group are
     * divided into particleColors.length equal sets of colored
     * particles.
     */
    public colorParticleGroup(group: ParticleGroup, particlesPerColor: number) {
        // DEBUG: assert(group !== null);
        const colorBuffer = this.particleSystem.getColorBuffer();
        const particleCount = group.getParticleCount();
        const groupStart = group.getBufferIndex();
        const groupEnd = particleCount + groupStart;
        const colorCount = particleColors.length;
        if (!particlesPerColor) {
            particlesPerColor = Math.floor(particleCount / colorCount);
            if (!particlesPerColor) {
                particlesPerColor = 1;
            }
        }
        for (let i = groupStart; i < groupEnd; i++) {
            colorBuffer[i] = particleColors[Math.floor(i / particlesPerColor) % colorCount].clone();
        }
    }

    public getParticleSelectionRadius() {
        return 40 / this.getDefaultViewZoom();
    }
}

export class AbstractParticleTestWithControls extends AbstractParticleTest {
    protected particleParameter: ParticleParameter;

    public constructor(particleParameter: ParticleParameter, gravity: XY = { x: 0, y: -10 }) {
        super(gravity);
        this.particleParameter = particleParameter;
    }

    public setupControls() {
        this.addTestControlGroup("Particles", [
            this.createStrictContactsCheckbox(),
            this.particleParameter.getControl(),
        ]);
    }
}
