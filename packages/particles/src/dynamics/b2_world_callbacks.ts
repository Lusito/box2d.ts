/*
 * Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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
import { DestructionListener, Fixture, ContactFilter, ContactListener } from "@box2d/core";

import { ParticleGroup } from "../particle/b2_particle_group";
import { ParticleSystem, ParticleBodyContact, ParticleContact } from "../particle/b2_particle_system";

// Declaration merging
declare module "@box2d/core" {
    interface DestructionListener {
        /**
         * Called when any particle group is about to be destroyed.
         */
        sayGoodbyeParticleGroup(group: ParticleGroup): void;

        /**
         * Called when a particle is about to be destroyed.
         * The index can be used in conjunction with
         * ParticleSystem::GetUserDataBuffer() or
         * ParticleSystem::GetParticleHandleFromIndex() to determine which
         * particle has been destroyed.
         */
        sayGoodbyeParticle(system: ParticleSystem, index: number): void;
    }

    interface ContactFilter {
        shouldCollideFixtureParticle(fixture: Fixture, system: ParticleSystem, index: number): boolean;
        shouldCollideParticleParticle(system: ParticleSystem, indexA: number, indexB: number): boolean;
    }

    interface ContactListener {
        beginContactFixtureParticle(system: ParticleSystem, contact: ParticleBodyContact): void;
        endContactFixtureParticle(system: ParticleSystem, contact: ParticleBodyContact): void;
        beginContactParticleParticle(system: ParticleSystem, contact: ParticleContact): void;
        endContactParticleParticle(system: ParticleSystem, contact: ParticleContact): void;
    }
}

// Default implementations
Object.assign(DestructionListener.prototype, {
    sayGoodbyeParticleGroup() {},
    sayGoodbyeParticle() {},
});

Object.assign(ContactFilter.prototype, {
    sayGoodbyeParticleGroup() {
        return true;
    },
    sayGoodbyeParticle() {
        return true;
    },
});

Object.assign(ContactListener.prototype, {
    beginContactFixtureParticle() {},
    endContactFixtureParticle() {},
    beginContactParticleParticle() {},
    endContactParticleParticle() {},
});
