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

import { World, MAX_FLOAT, Transform, Body, augment, Writeable, assert } from "@box2d/core";

import { calculateParticleIterations } from "../particle/b2_particle";
import { ParticleSystem, ParticleSystemDef } from "../particle/b2_particle_system";

declare module "@box2d/core" {
    interface World {
        particleSystemList: ParticleSystem | null;

        createParticleSystem(def: ParticleSystemDef): ParticleSystem;
        destroyParticleSystem(p: ParticleSystem): void;
        getParticleSystemList(): ParticleSystem | null;
        calculateReasonableParticleIterations(timeStep: number): number;
    }
}

augment(World, {
    create(original, gravity) {
        const world = original(gravity);
        world.particleSystemList = null;
        return world;
    },
});

function getSmallestRadius(world: World): number {
    let smallestRadius = MAX_FLOAT;
    for (let system = world.getParticleSystemList(); system !== null; system = system.next) {
        smallestRadius = Math.min(smallestRadius, system.getRadius());
    }
    return smallestRadius;
}
Object.assign(World.prototype, {
    createParticleSystem(this: World, def: ParticleSystemDef): ParticleSystem {
        assert(!this.isLocked());

        const p = new ParticleSystem(def, this);

        // Add to world doubly linked list.
        p.prev = null;
        p.next = this.particleSystemList;
        if (this.particleSystemList) {
            this.particleSystemList.prev = p;
        }
        this.particleSystemList = p;

        return p;
    },
    destroyParticleSystem(this: World, p: ParticleSystem): void {
        assert(!this.isLocked());

        // Remove world particleSystem list.
        if (p.prev) {
            p.prev.next = p.next;
        }

        if (p.next) {
            p.next.prev = p.prev;
        }

        if (p === this.particleSystemList) {
            this.particleSystemList = p.next;
        }
    },
    getParticleSystemList(this: World): ParticleSystem | null {
        return this.particleSystemList;
    },
    calculateReasonableParticleIterations(this: World, timeStep: number): number {
        if (this.particleSystemList === null) {
            return 1;
        }

        // Use the smallest radius, since that represents the worst-case.
        return calculateParticleIterations(this.getGravity().length(), getSmallestRadius(this), timeStep);
    },
});

augment(World.prototype, {
    createBody(this: World, original, def = {}) {
        const body = original(def);
        (body as Writeable<Body>).xf0 = new Transform();
        return body;
    },

    // eslint-disable-next-line @typescript-eslint/ban-ts-comment
    // @ts-ignore
    solve(this: World, original: (step: TimeStep) => void, step: TimeStep) {
        for (let p = this.particleSystemList; p; p = p.next) {
            p.solve(step); // Particle Simulation
        }
        // update previous transforms
        for (let b = this.getBodyList(); b; b = b.getNext()) {
            b.xf0.copy(b.getTransform());
        }

        original(step);
    },
});
