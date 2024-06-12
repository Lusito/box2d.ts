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

import { b2AABB, b2CircleShape, b2Color, b2Transform, XY } from "@box2d/core";
import { b2ParticleSystem, b2ParticleSystemDef, b2ParticleGroup } from "@box2d/particles";

import { Test } from "../../test";
import { Settings } from "../../settings";
import { ParticleParameter } from "../../utils/particles/particle_parameter";
import { checkboxDef } from "../../ui/controls/Checkbox";

export const particleColors = [
    new b2Color().SetByteRGBA(0xff, 0x00, 0x00, 0xff), // red
    new b2Color().SetByteRGBA(0x00, 0xff, 0x00, 0xff), // green
    new b2Color().SetByteRGBA(0x00, 0x00, 0xff, 0xff), // blue
    new b2Color().SetByteRGBA(0xff, 0x8c, 0x00, 0xff), // orange
    new b2Color().SetByteRGBA(0x00, 0xce, 0xd1, 0xff), // turquoise
    new b2Color().SetByteRGBA(0xff, 0x00, 0xff, 0xff), // magenta
    new b2Color().SetByteRGBA(0xff, 0xd7, 0x00, 0xff), // gold
    new b2Color().SetByteRGBA(0x00, 0xff, 0xff, 0xff), // cyan
];

export class AbstractParticleTest extends Test {
    public m_particleSystem: b2ParticleSystem;

    protected static m_strictContacts = false;

    public constructor(gravity: XY = { x: 0, y: -10 }) {
        super(gravity);

        const particleSystemDef = new b2ParticleSystemDef();

        this.m_particleSystem = this.m_world.CreateParticleSystem(particleSystemDef);

        this.m_particleSystem.SetGravityScale(0.4);
        this.m_particleSystem.SetDensity(1.2);
    }

    public setupControls() {
        this.addTestControlGroup("Particles", [this.createStrictContactsCheckbox()]);
    }

    protected createStrictContactsCheckbox() {
        return checkboxDef("Strict Particle/Body Contacts", AbstractParticleTest.m_strictContacts, (value) => {
            AbstractParticleTest.m_strictContacts = value;
        });
    }

    public Step(settings: Settings, timeStep: number) {
        super.Step(settings, timeStep);

        this.m_particleSystem.SetStrictContactCheck(AbstractParticleTest.m_strictContacts);

        if (settings.m_drawStats) {
            this.addStatistic("Particles", this.m_particleSystem.GetParticleCount());
            this.addStatistic("Groups", this.m_particleSystem.GetParticleGroupCount());
            this.addStatistic("Pairs", this.m_particleSystem.GetPairCount());
            this.addStatistic("Triads", this.m_particleSystem.GetTriadCount());
        }

        if (this.m_mouseTracing && !this.m_mouseJoint) {
            const shape = new b2CircleShape();
            shape.m_p.Copy(this.m_mouseTracerPosition);
            shape.m_radius = this.getParticleSelectionRadius();
            const aabb = new b2AABB();
            const xf = new b2Transform();
            xf.SetIdentity();
            shape.ComputeAABB(aabb, xf, 0);
            this.m_particleSystem.QueryAABB(aabb, (index) => {
                const p = this.m_particleSystem.GetPositionBuffer()[index];
                if (shape.TestPoint(b2Transform.IDENTITY, p)) {
                    const v = this.m_particleSystem.GetVelocityBuffer()[index];
                    v.Copy(this.m_mouseTracerVelocity);
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
    public ColorParticleGroup(group: b2ParticleGroup, particlesPerColor: number) {
        // DEBUG: b2Assert(group !== null);
        const colorBuffer = this.m_particleSystem.GetColorBuffer();
        const particleCount = group.GetParticleCount();
        const groupStart = group.GetBufferIndex();
        const groupEnd = particleCount + groupStart;
        const colorCount = particleColors.length;
        if (!particlesPerColor) {
            particlesPerColor = Math.floor(particleCount / colorCount);
            if (!particlesPerColor) {
                particlesPerColor = 1;
            }
        }
        for (let i = groupStart; i < groupEnd; i++) {
            colorBuffer[i] = particleColors[Math.floor(i / particlesPerColor) % colorCount].Clone();
        }
    }

    public getParticleSelectionRadius() {
        return 40 / this.GetDefaultViewZoom();
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
            this.particleParameter.GetControl(),
        ]);
    }
}
