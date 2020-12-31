/*
 * Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

import { XY, RGBA, Draw, World } from "@box2d/core";

import { ParticleSystem } from "./b2_particle_system";

declare module "@box2d/core" {
    interface Draw {
        DrawParticles(centers: XY[], radius: number, colors: RGBA[] | null, count: number): void;
    }
}

function DrawParticleSystem(draw: Draw, system: ParticleSystem): void {
    const particleCount = system.GetParticleCount();
    if (particleCount) {
        const radius = system.GetRadius();
        const positionBuffer = system.GetPositionBuffer();
        if (system.m_colorBuffer.data) {
            const colorBuffer = system.GetColorBuffer();
            draw.DrawParticles(positionBuffer, radius, colorBuffer, particleCount);
        } else {
            draw.DrawParticles(positionBuffer, radius, null, particleCount);
        }
    }
}

export function DrawParticleSystems(draw: Draw, world: World) {
    for (let p = world.GetParticleSystemList(); p; p = p.m_next) {
        DrawParticleSystem(draw, p);
    }
}
