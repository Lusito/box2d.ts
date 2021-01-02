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

import {
    LINEAR_SLOP,
    Fixture,
    Vec2,
    Body,
    Color,
    World,
    Shape,
    Transform,
    AABB,
    XY,
    MAX_FLOAT,
    EdgeShape,
    ShapeType,
    ChainShape,
    TimeStep,
    clamp,
    Rot,
    ContactFilter,
    ContactListener,
    RayCastOutput,
    RayCastInput,
    MassData,
    DistanceProxy,
    assert,
} from "@box2d/core";

// DEBUG: import { assert, b2_maxParticleIndex } from "../common/b2_settings";
import {
    INVALID_PARTICLE_INDEX,
    MIN_PARTICLE_SYSTEM_BUFFER_CAPACITY,
    MAX_TRIAD_DISTANCE_SQUARED,
    BARRIER_COLLISION_TIME,
    MAX_PARTICLE_PRESSURE,
    MIN_PARTICLE_WEIGHT,
    MAX_PARTICLE_FORCE,
    PARTICLE_STRIDE,
} from "./b2_settings";
import { ParticleFlag, ParticleDef, ParticleHandle, IParticleDef } from "./b2_particle";
import { ParticleGroupFlag, ParticleGroupDef, ParticleGroup, IParticleGroupDef } from "./b2_particle_group";
import { VoronoiDiagram } from "./b2_voronoi_diagram";
import { computeDistance } from "./b2_compute_distance";
import {
    ParticleSystem_SolveCollisionCallback,
    ParticleSystem_UpdateBodyContactsCallback,
} from "./b2_fixture_particle_query_callbacks";

export type ParticleQueryCallback = (index: number) => boolean;
export type ParticleRayCastCallback = (index: number, point: Vec2, normal: Vec2, fraction: number) => number;

function std_iter_swap<T>(array: T[], a: number, b: number): void {
    const tmp = array[a];
    array[a] = array[b];
    array[b] = tmp;
}

function default_compare<T>(a: T, b: T): boolean {
    return a < b;
}

function std_sort<T>(
    array: T[],
    first = 0,
    len = array.length - first,
    cmp: (a: T, b: T) => boolean = default_compare,
): T[] {
    let left = first;
    const stack: number[] = [];
    let pos = 0;

    for (;;) {
        /* outer loop */
        for (; left + 1 < len; len++) {
            /* sort left to len-1 */
            const pivot = array[left + Math.floor(Math.random() * (len - left))]; /* pick random pivot */
            stack[pos++] = len; /* sort right part later */
            for (let right = left - 1; ; ) {
                /* inner loop: partitioning */
                while (cmp(array[++right], pivot)) continue; /* look for greater element */
                while (cmp(pivot, array[--len])) continue; /* look for smaller element */
                if (right >= len) {
                    break;
                } /* partition point found? */
                std_iter_swap(array, right, len); /* the only swap */
            } /* partitioned, continue left part */
        }
        if (pos === 0) {
            break;
        } /* stack empty? */
        left = len; /* left to right is sorted */
        len = stack[--pos]; /* get next range to sort */
    }

    return array;
}

function std_stable_sort<T>(
    array: T[],
    first = 0,
    len = array.length - first,
    cmp: (a: T, b: T) => boolean = default_compare,
): T[] {
    return std_sort(array, first, len, cmp);
}

function std_remove_if<T>(array: T[], predicate: (value: T) => boolean, length = array.length) {
    let l = 0;

    for (let c = 0; c < length; ++c) {
        // if we can be collapsed, keep l where it is.
        if (predicate(array[c])) {
            continue;
        }

        // this node can't be collapsed; push it back as far as we can.
        if (c === l) {
            ++l;
            continue; // quick exit if we're already in the right spot
        }

        // array[l++] = array[c];
        std_iter_swap(array, l++, c);
    }

    return l;
}

function std_lower_bound<A, B>(array: A[], first: number, last: number, val: B, cmp: (a: A, b: B) => boolean): number {
    let count = last - first;
    while (count > 0) {
        const step = Math.floor(count / 2);
        let it = first + step;

        if (cmp(array[it], val)) {
            first = ++it;
            count -= step + 1;
        } else {
            count = step;
        }
    }
    return first;
}

function std_upper_bound<A, B>(array: B[], first: number, last: number, val: A, cmp: (a: A, b: B) => boolean): number {
    let count = last - first;
    while (count > 0) {
        const step = Math.floor(count / 2);
        let it = first + step;

        if (!cmp(val, array[it])) {
            first = ++it;
            count -= step + 1;
        } else {
            count = step;
        }
    }
    return first;
}

function std_rotate<T>(array: T[], first: number, n_first: number, last: number): void {
    let next = n_first;
    while (first !== next) {
        std_iter_swap(array, first++, next++);
        if (next === last) {
            next = n_first;
        } else if (first === n_first) {
            n_first = next;
        }
    }
}

function std_unique<T>(array: T[], first: number, last: number, cmp: (a: T, b: T) => boolean): number {
    if (first === last) {
        return last;
    }
    let result = first;
    while (++first !== last) {
        if (!cmp(array[result], array[first])) {
            std_iter_swap(array, ++result, first);
        }
    }
    return ++result;
}

export class GrowableBuffer<T> {
    public data: T[] = [];

    public count = 0;

    public capacity = 0;

    public allocator: () => T;

    public constructor(allocator: () => T) {
        this.allocator = allocator;
    }

    public append(): number {
        if (this.count >= this.capacity) {
            this.grow();
        }
        return this.count++;
    }

    public reserve(newCapacity: number): void {
        if (this.capacity >= newCapacity) {
            return;
        }

        // DEBUG: assert(this.capacity === this.data.length);
        for (let i = this.capacity; i < newCapacity; ++i) {
            this.data[i] = this.allocator();
        }
        this.capacity = newCapacity;
    }

    public grow(): void {
        // Double the capacity.
        const newCapacity = this.capacity ? 2 * this.capacity : MIN_PARTICLE_SYSTEM_BUFFER_CAPACITY;
        // DEBUG: assert(newCapacity > this.capacity);
        this.reserve(newCapacity);
    }

    public free(): void {
        if (this.data.length === 0) {
            return;
        }

        this.data = [];
        this.capacity = 0;
        this.count = 0;
    }

    public shorten(_newEnd: number): void {
        // DEBUG: assert(false);
    }

    public getData(): T[] {
        return this.data;
    }

    public getCount(): number {
        return this.count;
    }

    public setCount(newCount: number): void {
        // DEBUG: assert(0 <= newCount && newCount <= this.capacity);
        this.count = newCount;
    }

    public getCapacity(): number {
        return this.capacity;
    }

    public removeIf(pred: (t: T) => boolean): void {
        // DEBUG: let count = 0;
        // DEBUG: for (let i = 0; i < this.count; ++i) {
        // DEBUG:   if (!pred(this.data[i])) {
        // DEBUG:     count++;
        // DEBUG:   }
        // DEBUG: }

        this.count = std_remove_if(this.data, pred, this.count);

        // DEBUG: assert(count === this.count);
    }

    public unique(pred: (a: T, b: T) => boolean): void {
        this.count = std_unique(this.data, 0, this.count, pred);
    }
}

export class ParticleSystem_UserOverridableBuffer<T> {
    public data!: T[]; // HACK: may be undefined

    public userSuppliedCapacity = 0;
}

export type ParticleIndex = number;

export class ParticleContact {
    public indexA = 0;

    public indexB = 0;

    public weight = 0;

    public normal = new Vec2();

    public flags: ParticleFlag = 0;

    public setIndices(a: number, b: number): void {
        // DEBUG: assert(a <= b2_maxParticleIndex && b <= b2_maxParticleIndex);
        this.indexA = a;
        this.indexB = b;
    }

    public setWeight(w: number): void {
        this.weight = w;
    }

    public setNormal(n: Vec2): void {
        this.normal.copy(n);
    }

    public setFlags(f: ParticleFlag): void {
        this.flags = f;
    }

    public getIndexA(): number {
        return this.indexA;
    }

    public getIndexB(): number {
        return this.indexB;
    }

    public getWeight(): number {
        return this.weight;
    }

    public getNormal(): Vec2 {
        return this.normal;
    }

    public getFlags(): ParticleFlag {
        return this.flags;
    }

    public isEqual(rhs: ParticleContact): boolean {
        return (
            this.indexA === rhs.indexA &&
            this.indexB === rhs.indexB &&
            this.flags === rhs.flags &&
            this.weight === rhs.weight &&
            this.normal.x === rhs.normal.x &&
            this.normal.y === rhs.normal.y
        );
    }

    public isNotEqual(rhs: ParticleContact): boolean {
        return !this.isEqual(rhs);
    }

    public approximatelyEqual(rhs: ParticleContact): boolean {
        const MAX_WEIGHT_DIFF = 0.01; // Weight 0 ~ 1, so about 1%
        const MAX_NORMAL_DIFF_SQ = 0.01 * 0.01; // Normal length = 1, so 1%
        return (
            this.indexA === rhs.indexA &&
            this.indexB === rhs.indexB &&
            this.flags === rhs.flags &&
            Math.abs(this.weight - rhs.weight) < MAX_WEIGHT_DIFF &&
            Vec2.distanceSquared(this.normal, rhs.normal) < MAX_NORMAL_DIFF_SQ
        );
    }
}

export class ParticleBodyContact {
    public index = 0; // Index of the particle making contact.

    public body!: Body; // The body making contact.

    public fixture!: Fixture; // The specific fixture making contact

    public weight = 0; // Weight of the contact. A value between 0   and 1  .

    public normal = new Vec2(); // The normalized direction from the particle to the body.

    public mass = 0; // The effective mass used in calculating force.
}

export class ParticlePair {
    public indexA = 0; // Indices of the respective particles making pair.

    public indexB = 0;

    public flags: ParticleFlag = 0; // The logical sum of the particle flags. See the ParticleFlag enum.

    public strength = 0; // The strength of cohesion among the particles.

    public distance = 0; // The initial distance of the particles.
}

export class ParticleTriad {
    public indexA = 0; // Indices of the respective particles making triad.

    public indexB = 0;

    public indexC = 0;

    public flags: ParticleFlag = 0; // The logical sum of the particle flags. See the ParticleFlag enum.

    public strength = 0; // The strength of cohesion among the particles.

    public pa = new Vec2(); // Values used for calculation.

    public pb = new Vec2();

    public pc = new Vec2();

    public ka = 0;

    public kb = 0;

    public kc = 0;

    public s = 0;
}

export class ParticleSystemDef {
    // Initialize physical coefficients to the maximum values that
    // maintain numerical stability.

    /**
     * Enable strict Particle/Body contact check.
     * See SetStrictContactCheck for details.
     */
    public strictContactCheck = false;

    /**
     * Set the particle density.
     * See SetDensity for details.
     */
    public density = 1;

    /**
     * Change the particle gravity scale. Adjusts the effect of the
     * global gravity vector on particles. Default value is 1  .
     */
    public gravityScale = 1;

    /** Particles behave as circles with this radius. In Box2D units. */
    public radius = 1;

    /**
     * Set the maximum number of particles.
     * By default, there is no maximum. The particle buffers can
     * continue to grow while World's block allocator still has
     * memory.
     * See SetMaxParticleCount for details.
     */
    public maxCount = 0;

    /**
     * Increases pressure in response to compression
     * Smaller values allow more compression
     */
    public pressureStrength = 0.005;

    /**
     * Reduces velocity along the collision normal
     * Smaller value reduces less
     */
    public dampingStrength = 1;

    /**
     * Restores shape of elastic particle groups
     * Larger values increase elastic particle velocity
     */
    public elasticStrength = 0.25;

    /**
     * Restores length of spring particle groups
     * Larger values increase spring particle velocity
     */
    public springStrength = 0.25;

    /**
     * Reduces relative velocity of viscous particles
     * Larger values slow down viscous particles more
     */
    public viscousStrength = 0.25;

    /**
     * Produces pressure on tensile particles
     * 0~0.2. Larger values increase the amount of surface tension.
     */
    public surfaceTensionPressureStrength = 0.2;

    /**
     * Smoothes outline of tensile particles
     * 0~0.2. Larger values result in rounder, smoother,
     * water-drop-like clusters of particles.
     */
    public surfaceTensionNormalStrength = 0.2;

    /**
     * Produces additional pressure on repulsive particles
     * Larger values repulse more
     * Negative values mean attraction. The range where particles
     * behave stably is about -0.2 to 2.
     */
    public repulsiveStrength = 1;

    /**
     * Produces repulsion between powder particles
     * Larger values repulse more
     */
    public powderStrength = 0.5;

    /**
     * Pushes particles out of solid particle group
     * Larger values repulse more
     */
    public ejectionStrength = 0.5;

    /**
     * Produces static pressure
     * Larger values increase the pressure on neighboring partilces
     * For a description of static pressure, see
     * http://en.wikipedia.org/wiki/Static_pressure#Static_pressure_in_fluid_dynamics
     */
    public staticPressureStrength = 0.2;

    /**
     * Reduces instability in static pressure calculation
     * Larger values make stabilize static pressure with fewer
     * iterations
     */
    public staticPressureRelaxation = 0.2;

    /**
     * Computes static pressure more precisely
     * See SetStaticPressureIterations for details
     */
    public staticPressureIterations = 8;

    /**
     * Determines how fast colors are mixed
     * 1   ==> mixed immediately
     * 0.5  ==> mixed half way each simulation step (see
     * World::Step())
     */
    public colorMixingStrength = 0.5;

    /**
     * Whether to destroy particles by age when no more particles
     * can be created.  See #ParticleSystem::SetDestructionByAge()
     * for more information.
     */
    public destroyByAge = true;

    /**
     * Granularity of particle lifetimes in seconds.  By default
     * this is set to (1   / 60  ) seconds.  ParticleSystem uses
     * a 32-bit signed value to track particle lifetimes so the
     * maximum lifetime of a particle is (2^32 - 1) / (1   /
     * lifetimeGranularity) seconds. With the value set to 1/60 the
     * maximum lifetime or age of a particle is 2.27 years.
     */
    public lifetimeGranularity = 1 / 60;

    public copy(def: ParticleSystemDef): ParticleSystemDef {
        this.strictContactCheck = def.strictContactCheck;
        this.density = def.density;
        this.gravityScale = def.gravityScale;
        this.radius = def.radius;
        this.maxCount = def.maxCount;
        this.pressureStrength = def.pressureStrength;
        this.dampingStrength = def.dampingStrength;
        this.elasticStrength = def.elasticStrength;
        this.springStrength = def.springStrength;
        this.viscousStrength = def.viscousStrength;
        this.surfaceTensionPressureStrength = def.surfaceTensionPressureStrength;
        this.surfaceTensionNormalStrength = def.surfaceTensionNormalStrength;
        this.repulsiveStrength = def.repulsiveStrength;
        this.powderStrength = def.powderStrength;
        this.ejectionStrength = def.ejectionStrength;
        this.staticPressureStrength = def.staticPressureStrength;
        this.staticPressureRelaxation = def.staticPressureRelaxation;
        this.staticPressureIterations = def.staticPressureIterations;
        this.colorMixingStrength = def.colorMixingStrength;
        this.destroyByAge = def.destroyByAge;
        this.lifetimeGranularity = def.lifetimeGranularity;
        return this;
    }

    public clone(): ParticleSystemDef {
        return new ParticleSystemDef().copy(this);
    }
}

export class ParticleSystem {
    public paused = false;

    public timestamp = 0;

    public allParticleFlags: ParticleFlag = 0;

    public needsUpdateAllParticleFlags = false;

    public allGroupFlags: ParticleGroupFlag = 0;

    public needsUpdateAllGroupFlags = false;

    public hasForce = false;

    public iterationIndex = 0;

    public inverseDensity = 0;

    public particleDiameter = 0;

    public inverseDiameter = 0;

    public squaredDiameter = 0;

    public count = 0;

    public internalAllocatedCapacity = 0;

    /** Maps particle indices to handles. */
    public handleIndexBuffer: ParticleSystem_UserOverridableBuffer<ParticleHandle | null> = new ParticleSystem_UserOverridableBuffer<ParticleHandle | null>();

    public flagsBuffer: ParticleSystem_UserOverridableBuffer<ParticleFlag> = new ParticleSystem_UserOverridableBuffer<
        ParticleFlag
    >();

    public positionBuffer: ParticleSystem_UserOverridableBuffer<Vec2> = new ParticleSystem_UserOverridableBuffer<
        Vec2
    >();

    public velocityBuffer: ParticleSystem_UserOverridableBuffer<Vec2> = new ParticleSystem_UserOverridableBuffer<
        Vec2
    >();

    public forceBuffer: Vec2[] = [];

    /**
     * this.weightBuffer is populated in ComputeWeight and used in
     * ComputeDepth(), SolveStaticPressure() and SolvePressure().
     */
    public weightBuffer: number[] = [];

    /**
     * When any particles have the flag StaticPressure,
     * this.staticPressureBuffer is first allocated and used in
     * SolveStaticPressure() and SolvePressure().  It will be
     * reallocated on subsequent CreateParticle() calls.
     */
    public staticPressureBuffer: number[] = [];

    /**
     * this.accumulationBuffer is used in many functions as a temporary
     * buffer for scalar values.
     */
    public accumulationBuffer: number[] = [];

    /**
     * When any particles have the flag Tensile,
     * this.accumulation2Buffer is first allocated and used in
     * SolveTensile() as a temporary buffer for vector values.  It
     * will be reallocated on subsequent CreateParticle() calls.
     */
    public accumulation2Buffer: Vec2[] = [];

    /**
     * When any particle groups have the flag Solid,
     * this.depthBuffer is first allocated and populated in
     * ComputeDepth() and used in SolveSolid(). It will be
     * reallocated on subsequent CreateParticle() calls.
     */
    public depthBuffer: number[] = [];

    public colorBuffer = new ParticleSystem_UserOverridableBuffer<Color>();

    public groupBuffer: Array<ParticleGroup | null> = [];

    public userDataBuffer = new ParticleSystem_UserOverridableBuffer<any>();

    /** Stuck particle detection parameters and record keeping */
    public stuckThreshold = 0;

    public lastBodyContactStepBuffer: ParticleSystem_UserOverridableBuffer<
        number
    > = new ParticleSystem_UserOverridableBuffer<number>();

    public bodyContactCountBuffer: ParticleSystem_UserOverridableBuffer<
        number
    > = new ParticleSystem_UserOverridableBuffer<number>();

    public consecutiveContactStepsBuffer: ParticleSystem_UserOverridableBuffer<
        number
    > = new ParticleSystem_UserOverridableBuffer<number>();

    public stuckParticleBuffer = new GrowableBuffer<number>(() => 0);

    public proxyBuffer = new GrowableBuffer<ParticleSystem_Proxy>(() => new ParticleSystem_Proxy());

    public contactBuffer = new GrowableBuffer<ParticleContact>(() => new ParticleContact());

    public bodyContactBuffer = new GrowableBuffer<ParticleBodyContact>(() => new ParticleBodyContact());

    public pairBuffer = new GrowableBuffer<ParticlePair>(() => new ParticlePair());

    public triadBuffer = new GrowableBuffer<ParticleTriad>(() => new ParticleTriad());

    /**
     * Time each particle should be destroyed relative to the last
     * time this.timeElapsed was initialized.  Each unit of time
     * corresponds to ParticleSystemDef::lifetimeGranularity
     * seconds.
     */
    public expirationTimeBuffer: ParticleSystem_UserOverridableBuffer<
        number
    > = new ParticleSystem_UserOverridableBuffer<number>();

    /**
     * List of particle indices sorted by expiration time.
     */
    public indexByExpirationTimeBuffer: ParticleSystem_UserOverridableBuffer<
        number
    > = new ParticleSystem_UserOverridableBuffer<number>();

    /**
     * Time elapsed in 32:32 fixed point.  Each non-fractional unit
     * of time corresponds to
     * ParticleSystemDef::lifetimeGranularity seconds.
     */
    public timeElapsed = 0;

    /**
     * Whether the expiration time buffer has been modified and
     * needs to be resorted.
     */
    public expirationTimeBufferRequiresSorting = false;

    public groupCount = 0;

    public groupList: ParticleGroup | null = null;

    public def = new ParticleSystemDef();

    public world: World;

    public prev: ParticleSystem | null = null;

    public next: ParticleSystem | null = null;

    public static readonly xTruncBits = 12;

    public static readonly yTruncBits = 12;

    public static readonly tagBits = 8 * 4; // 8u * sizeof(uint32);

    public static readonly yOffset = 1 << (ParticleSystem.yTruncBits - 1);

    public static readonly yShift = ParticleSystem.tagBits - ParticleSystem.yTruncBits;

    public static readonly xShift = ParticleSystem.tagBits - ParticleSystem.yTruncBits - ParticleSystem.xTruncBits;

    public static readonly xScale = 1 << ParticleSystem.xShift;

    public static readonly xOffset = ParticleSystem.xScale * (1 << (ParticleSystem.xTruncBits - 1));

    public static readonly yMask = ((1 << ParticleSystem.yTruncBits) - 1) << ParticleSystem.yShift;

    public static readonly xMask = ~ParticleSystem.yMask;

    public static computeTag(x: number, y: number): number {
        return (
            ((((y + ParticleSystem.yOffset) >>> 0) << ParticleSystem.yShift) +
                ((ParticleSystem.xScale * x + ParticleSystem.xOffset) >>> 0)) >>>
            0
        );
    }

    public static computeRelativeTag(tag: number, x: number, y: number): number {
        return (tag + (y << ParticleSystem.yShift) + (x << ParticleSystem.xShift)) >>> 0;
    }

    public constructor(def: ParticleSystemDef, world: World) {
        this.setStrictContactCheck(def.strictContactCheck);
        this.setDensity(def.density);
        this.setGravityScale(def.gravityScale);
        this.setRadius(def.radius);
        this.setMaxParticleCount(def.maxCount);
        // DEBUG: assert(def.lifetimeGranularity > 0);
        this.def = def.clone();
        this.world = world;
        this.setDestructionByAge(this.def.destroyByAge);
    }

    public drop(): void {
        while (this.groupList) {
            this.destroyParticleGroup(this.groupList);
        }

        this.freeUserOverridableBuffer(this.handleIndexBuffer);
        this.freeUserOverridableBuffer(this.flagsBuffer);
        this.freeUserOverridableBuffer(this.lastBodyContactStepBuffer);
        this.freeUserOverridableBuffer(this.bodyContactCountBuffer);
        this.freeUserOverridableBuffer(this.consecutiveContactStepsBuffer);
        this.freeUserOverridableBuffer(this.positionBuffer);
        this.freeUserOverridableBuffer(this.velocityBuffer);
        this.freeUserOverridableBuffer(this.colorBuffer);
        this.freeUserOverridableBuffer(this.userDataBuffer);
        this.freeUserOverridableBuffer(this.expirationTimeBuffer);
        this.freeUserOverridableBuffer(this.indexByExpirationTimeBuffer);
        this.freeBuffer(this.forceBuffer);
        this.freeBuffer(this.weightBuffer);
        this.freeBuffer(this.staticPressureBuffer);
        this.freeBuffer(this.accumulationBuffer);
        this.freeBuffer(this.accumulation2Buffer);
        this.freeBuffer(this.depthBuffer);
        this.freeBuffer(this.groupBuffer);
    }

    /**
     * Create a particle whose properties have been defined.
     *
     * No reference to the definition is retained.
     *
     * A simulation step must occur before it's possible to interact
     * with a newly created particle.  For example,
     * DestroyParticleInShape() will not destroy a particle until
     * World::Step() has been called.
     *
     * warning: This function is locked during callbacks.
     */
    public createParticle(def: IParticleDef): number {
        assert(!this.world.isLocked());

        if (this.count >= this.internalAllocatedCapacity) {
            // Double the particle capacity.
            const capacity = this.count ? 2 * this.count : MIN_PARTICLE_SYSTEM_BUFFER_CAPACITY;
            this.reallocateInternalAllocatedBuffers(capacity);
        }
        if (this.count >= this.internalAllocatedCapacity) {
            // If the oldest particle should be destroyed...
            if (this.def.destroyByAge) {
                this.destroyOldestParticle(0, false);
                // Need to destroy this particle *now* so that it's possible to
                // create a new particle.
                this.solveZombie();
            } else {
                return INVALID_PARTICLE_INDEX;
            }
        }
        const index = this.count++;
        this.flagsBuffer.data[index] = 0;
        if (this.lastBodyContactStepBuffer.data) {
            this.lastBodyContactStepBuffer.data[index] = 0;
        }
        if (this.bodyContactCountBuffer.data) {
            this.bodyContactCountBuffer.data[index] = 0;
        }
        if (this.consecutiveContactStepsBuffer.data) {
            this.consecutiveContactStepsBuffer.data[index] = 0;
        }
        this.positionBuffer.data[index] = (this.positionBuffer.data[index] || new Vec2()).copy(
            def.position ?? Vec2.ZERO,
        );
        this.velocityBuffer.data[index] = (this.velocityBuffer.data[index] || new Vec2()).copy(
            def.velocity ?? Vec2.ZERO,
        );
        this.weightBuffer[index] = 0;
        this.forceBuffer[index] = (this.forceBuffer[index] || new Vec2()).setZero();
        if (this.staticPressureBuffer) {
            this.staticPressureBuffer[index] = 0;
        }
        if (this.depthBuffer) {
            this.depthBuffer[index] = 0;
        }
        const color = new Color().copy(def.color ?? Color.ZERO);
        if (this.colorBuffer.data || !color.isZero()) {
            this.colorBuffer.data = this.requestBuffer(this.colorBuffer.data);
            this.colorBuffer.data[index] = (this.colorBuffer.data[index] || new Color()).copy(color);
        }
        if (this.userDataBuffer.data || def.userData) {
            this.userDataBuffer.data = this.requestBuffer(this.userDataBuffer.data);
            this.userDataBuffer.data[index] = def.userData;
        }
        if (this.handleIndexBuffer.data) {
            this.handleIndexBuffer.data[index] = null;
        }

        const proxy = this.proxyBuffer.data[this.proxyBuffer.append()];

        // If particle lifetimes are enabled or the lifetime is set in the particle
        // definition, initialize the lifetime.
        const lifetime = def.lifetime ?? 0;
        const finiteLifetime = lifetime > 0;
        if (this.expirationTimeBuffer.data || finiteLifetime) {
            this.setParticleLifetime(
                index,
                finiteLifetime ? lifetime : this.expirationTimeToLifetime(-this.getQuantizedTimeElapsed()),
            );
            // Add a reference to the newly added particle to the end of the
            // queue.
            this.indexByExpirationTimeBuffer.data[index] = index;
        }

        proxy.index = index;
        const group = def.group ?? null;
        this.groupBuffer[index] = group;
        if (group) {
            if (group.firstIndex < group.lastIndex) {
                // Move particles in the group just before the new particle.
                this.rotateBuffer(group.firstIndex, group.lastIndex, index);
                // DEBUG: assert(group.lastIndex === index);
                // Update the index range of the group to contain the new particle.
                group.lastIndex = index + 1;
            } else {
                // If the group is empty, reset the index range to contain only the
                // new particle.
                group.firstIndex = index;
                group.lastIndex = index + 1;
            }
        }
        this.setParticleFlags(index, def.flags ?? 0);
        return index;
    }

    /**
     * Retrieve a handle to the particle at the specified index.
     *
     * Please see #ParticleHandle for why you might want a handle.
     */
    public getParticleHandleFromIndex(index: number): ParticleHandle {
        // DEBUG: assert(index >= 0 && index < this.getParticleCount() && index !== b2_invalidParticleIndex);
        this.handleIndexBuffer.data = this.requestBuffer(this.handleIndexBuffer.data);
        let handle = this.handleIndexBuffer.data[index];
        if (handle) {
            return handle;
        }
        // Create a handle.
        handle = new ParticleHandle();
        // DEBUG: assert(handle !== null);
        handle.setIndex(index);
        this.handleIndexBuffer.data[index] = handle;
        return handle;
    }

    /**
     * Destroy a particle.
     *
     * The particle is removed after the next simulation step (see
     * World::Step()).
     *
     * @param index Index of the particle to destroy.
     * @param callDestructionListener Whether to call the
     *      destruction listener just before the particle is
     *      destroyed.
     */
    public destroyParticle(index: number, callDestructionListener = false): void {
        let flags = ParticleFlag.Zombie;
        if (callDestructionListener) {
            flags |= ParticleFlag.DestructionListener;
        }
        this.setParticleFlags(index, this.flagsBuffer.data[index] | flags);
    }

    /**
     * Destroy the Nth oldest particle in the system.
     *
     * The particle is removed after the next World::Step().
     *
     * @param index Index of the Nth oldest particle to
     *      destroy, 0 will destroy the oldest particle in the
     *      system, 1 will destroy the next oldest particle etc.
     * @param callDestructionListener Whether to call the
     *      destruction listener just before the particle is
     *      destroyed.
     */
    public destroyOldestParticle(index: number, callDestructionListener = false): void {
        const particleCount = this.getParticleCount();
        // DEBUG: assert(index >= 0 && index < particleCount);
        // Make sure particle lifetime tracking is enabled.
        // DEBUG: assert(this.indexByExpirationTimeBuffer.data !== null);
        // Destroy the oldest particle (preferring to destroy finite
        // lifetime particles first) to free a slot in the buffer.
        const oldestFiniteLifetimeParticle = this.indexByExpirationTimeBuffer.data[particleCount - (index + 1)];
        const oldestInfiniteLifetimeParticle = this.indexByExpirationTimeBuffer.data[index];
        this.destroyParticle(
            this.expirationTimeBuffer.data[oldestFiniteLifetimeParticle] > 0.0
                ? oldestFiniteLifetimeParticle
                : oldestInfiniteLifetimeParticle,
            callDestructionListener,
        );
    }

    /**
     * Destroy particles inside a shape.
     *
     * warning: This function is locked during callbacks.
     *
     * In addition, this function immediately destroys particles in
     * the shape in constrast to DestroyParticle() which defers the
     * destruction until the next simulation step.
     *
     * @returns Number of particles destroyed.
     * @param shape Shape which encloses particles
     *      that should be destroyed.
     * @param xf Transform applied to the shape.
     * @param callDestructionListener Whether to call the
     *      world DestructionListener for each particle
     *      destroyed.
     */
    public destroyParticlesInShape(shape: Shape, xf: Transform, callDestructionListener = false): number {
        const s_aabb = ParticleSystem.DestroyParticlesInShape_s_aabb;
        assert(!this.world.isLocked());

        const aabb = s_aabb;
        shape.computeAABB(aabb, xf, 0);
        let destroyed = 0;
        this.queryAABB(aabb, (index) => {
            // DEBUG: assert(index >= 0 && index < this.system.count);
            if (shape.testPoint(xf, this.positionBuffer.data[index])) {
                this.destroyParticle(index, callDestructionListener);
                destroyed++;
            }
            return true;
        });
        return destroyed;
    }

    public static readonly DestroyParticlesInShape_s_aabb = new AABB();

    /**
     * Create a particle group whose properties have been defined.
     *
     * No reference to the definition is retained.
     *
     * warning: This function is locked during callbacks.
     */
    public createParticleGroup(groupDef: IParticleGroupDef): ParticleGroup {
        const s_transform = ParticleSystem.CreateParticleGroup_s_transform;

        assert(!this.world.isLocked());

        const transform = s_transform;
        transform.setPositionAngle(groupDef.position ?? Vec2.ZERO, groupDef.angle ?? 0);
        const firstIndex = this.count;
        if (groupDef.shape) {
            this.createParticlesWithShapeForGroup(groupDef.shape, groupDef, transform);
        }
        if (groupDef.shapes) {
            this.createParticlesWithShapesForGroup(
                groupDef.shapes,
                groupDef.shapeCount ?? groupDef.shapes.length,
                groupDef,
                transform,
            );
        }
        if (groupDef.positionData) {
            const count = groupDef.particleCount ?? groupDef.positionData.length;
            for (let i = 0; i < count; i++) {
                const p = groupDef.positionData[i];
                this.createParticleForGroup(groupDef, transform, p);
            }
        }
        const lastIndex = this.count;

        let group = new ParticleGroup(this);
        group.firstIndex = firstIndex;
        group.lastIndex = lastIndex;
        group.strength = groupDef.strength ?? 1;
        group.userData = groupDef.userData;
        group.transform.copy(transform);
        group.prev = null;
        group.next = this.groupList;
        if (this.groupList) {
            this.groupList.prev = group;
        }
        this.groupList = group;
        ++this.groupCount;
        for (let i = firstIndex; i < lastIndex; i++) {
            this.groupBuffer[i] = group;
        }
        this.setGroupFlags(group, groupDef.groupFlags ?? 0);

        // Create pairs and triads between particles in the group.
        const filter = new ParticleSystem_ConnectionFilter();
        this.updateContacts(true);
        this.updatePairsAndTriads(firstIndex, lastIndex, filter);

        if (groupDef.group) {
            this.joinParticleGroups(groupDef.group, group);
            group = groupDef.group;
        }

        return group;
    }

    public static readonly CreateParticleGroup_s_transform = new Transform();

    /**
     * Join two particle groups.
     *
     * warning: This function is locked during callbacks.
     *
     * @param groupA The first group. Expands to encompass the second group.
     * @param groupB The second group. It is destroyed.
     */
    public joinParticleGroups(groupA: ParticleGroup, groupB: ParticleGroup): void {
        assert(!this.world.isLocked());

        // DEBUG: assert(groupA !== groupB);
        this.rotateBuffer(groupB.firstIndex, groupB.lastIndex, this.count);
        // DEBUG: assert(groupB.lastIndex === this.count);
        this.rotateBuffer(groupA.firstIndex, groupA.lastIndex, groupB.firstIndex);
        // DEBUG: assert(groupA.lastIndex === groupB.firstIndex);

        // Create pairs and triads connecting groupA and groupB.
        const filter = new ParticleSystem_JoinParticleGroupsFilter(groupB.firstIndex);
        this.updateContacts(true);
        this.updatePairsAndTriads(groupA.firstIndex, groupB.lastIndex, filter);

        for (let i = groupB.firstIndex; i < groupB.lastIndex; i++) {
            this.groupBuffer[i] = groupA;
        }
        const groupFlags = groupA.groupFlags | groupB.groupFlags;
        this.setGroupFlags(groupA, groupFlags);
        groupA.lastIndex = groupB.lastIndex;
        groupB.firstIndex = groupB.lastIndex;
        this.destroyParticleGroup(groupB);
    }

    /**
     * Split particle group into multiple disconnected groups.
     *
     * warning: This function is locked during callbacks.
     *
     * @param group The group to be split.
     */
    public splitParticleGroup(group: ParticleGroup): void {
        this.updateContacts(true);
        const particleCount = group.getParticleCount();
        // We create several linked lists. Each list represents a set of connected particles.
        const nodeBuffer = new Array<ParticleSystem_ParticleListNode>(particleCount);
        for (let i = 0; i < particleCount; i++) nodeBuffer[i] = new ParticleSystem_ParticleListNode();
        ParticleSystem.initializeParticleLists(group, nodeBuffer);
        this.mergeParticleListsInContact(group, nodeBuffer);
        const survivingList = ParticleSystem.findLongestParticleList(group, nodeBuffer);
        this.mergeZombieParticleListNodes(group, nodeBuffer, survivingList);
        this.createParticleGroupsFromParticleList(group, nodeBuffer, survivingList);
        this.updatePairsAndTriadsWithParticleList(group, nodeBuffer);
    }

    /**
     * Get the world particle group list. With the returned group,
     * use ParticleGroup::GetNext to get the next group in the
     * world list.
     *
     * A null group indicates the end of the list.
     *
     * @returns The head of the world particle group list.
     */
    public getParticleGroupList(): ParticleGroup | null {
        return this.groupList;
    }

    /**
     * Get the number of particle groups.
     */
    public getParticleGroupCount(): number {
        return this.groupCount;
    }

    /**
     * Get the number of particles.
     */
    public getParticleCount(): number {
        return this.count;
    }

    /**
     * Get the maximum number of particles.
     */
    public getMaxParticleCount(): number {
        return this.def.maxCount;
    }

    /**
     * Set the maximum number of particles.
     *
     * A value of 0 means there is no maximum. The particle buffers
     * can continue to grow while World's block allocator still
     * has memory.
     *
     * Note: If you try to CreateParticle() with more than this
     * count, b2_invalidParticleIndex is returned unless
     * SetDestructionByAge() is used to enable the destruction of
     * the oldest particles in the system.
     */
    public setMaxParticleCount(count: number): void {
        // DEBUG: assert(this.count <= count);
        this.def.maxCount = count;
    }

    /**
     * Get all existing particle flags.
     */
    public getAllParticleFlags(): ParticleFlag {
        return this.allParticleFlags;
    }

    /**
     * Get all existing particle group flags.
     */
    public getAllGroupFlags(): ParticleGroupFlag {
        return this.allGroupFlags;
    }

    /**
     * Pause or unpause the particle system. When paused,
     * World::Step() skips over this particle system. All
     * ParticleSystem function calls still work.
     *
     * @param paused Paused is true to pause, false to un-pause.
     */
    public setPaused(paused: boolean): void {
        this.paused = paused;
    }

    /**
     * Initially, true, then, the last value passed into
     * SetPaused().
     *
     * @returns true if the particle system is being updated in World::Step().
     */
    public getPaused(): boolean {
        return this.paused;
    }

    /**
     * Change the particle density.
     *
     * Particle density affects the mass of the particles, which in
     * turn affects how the particles interact with Bodies. Note
     * that the density does not affect how the particles interact
     * with each other.
     */
    public setDensity(density: number): void {
        this.def.density = density;
        this.inverseDensity = 1 / this.def.density;
    }

    /**
     * Get the particle density.
     */
    public getDensity(): number {
        return this.def.density;
    }

    /**
     * Change the particle gravity scale. Adjusts the effect of the
     * global gravity vector on particles.
     */
    public setGravityScale(gravityScale: number): void {
        this.def.gravityScale = gravityScale;
    }

    /**
     * Get the particle gravity scale.
     */
    public getGravityScale(): number {
        return this.def.gravityScale;
    }

    /**
     * Damping is used to reduce the velocity of particles. The
     * damping parameter can be larger than 1   but the damping
     * effect becomes sensitive to the time step when the damping
     * parameter is large.
     */
    public setDamping(damping: number): void {
        this.def.dampingStrength = damping;
    }

    /**
     * Get damping for particles
     */
    public getDamping(): number {
        return this.def.dampingStrength;
    }

    /**
     * Change the number of iterations when calculating the static
     * pressure of particles. By default, 8 iterations. You can
     * reduce the number of iterations down to 1 in some situations,
     * but this may cause instabilities when many particles come
     * together. If you see particles popping away from each other
     * like popcorn, you may have to increase the number of
     * iterations.
     *
     * For a description of static pressure, see
     * http://en.wikipedia.org/wiki/Static_pressure#Static_pressure_in_fluid_dynamics
     */
    public setStaticPressureIterations(iterations: number): void {
        this.def.staticPressureIterations = iterations;
    }

    /**
     * Get the number of iterations for static pressure of
     * particles.
     */
    public getStaticPressureIterations(): number {
        return this.def.staticPressureIterations;
    }

    /**
     * Change the particle radius.
     *
     * You should set this only once, on world start.
     * If you change the radius during execution, existing particles
     * may explode, shrink, or behave unexpectedly.
     */
    public setRadius(radius: number): void {
        this.particleDiameter = 2 * radius;
        this.squaredDiameter = this.particleDiameter * this.particleDiameter;
        this.inverseDiameter = 1 / this.particleDiameter;
    }

    /**
     * Get the particle radius.
     */
    public getRadius(): number {
        return this.particleDiameter / 2;
    }

    /**
     * Get the position of each particle
     *
     * Array is length GetParticleCount()
     *
     * @returns The pointer to the head of the particle positions array.
     */
    public getPositionBuffer(): Vec2[] {
        return this.positionBuffer.data;
    }

    /**
     * Get the velocity of each particle
     *
     * Array is length GetParticleCount()
     *
     * @returns The pointer to the head of the particle velocities array.
     */
    public getVelocityBuffer(): Vec2[] {
        return this.velocityBuffer.data;
    }

    /**
     * Get the color of each particle
     *
     * Array is length GetParticleCount()
     *
     * @returns The pointer to the head of the particle colors array.
     */
    public getColorBuffer(): Color[] {
        this.colorBuffer.data = this.requestBuffer(this.colorBuffer.data);
        return this.colorBuffer.data;
    }

    /**
     * Get the particle-group of each particle.
     *
     * Array is length GetParticleCount()
     *
     * @returns The pointer to the head of the particle group array.
     */
    public getGroupBuffer(): Array<ParticleGroup | null> {
        return this.groupBuffer;
    }

    /**
     * Get the weight of each particle
     *
     * Array is length GetParticleCount()
     *
     * @returns The pointer to the head of the particle positions array.
     */
    public getWeightBuffer(): number[] {
        return this.weightBuffer;
    }

    /**
     * Get the user-specified data of each particle.
     *
     * Array is length GetParticleCount()
     *
     * @returns The pointer to the head of the particle user-data array.
     */
    public getUserDataBuffer<T>(): T[] {
        this.userDataBuffer.data = this.requestBuffer(this.userDataBuffer.data);
        return this.userDataBuffer.data;
    }

    /**
     * Get the flags for each particle. See the ParticleFlag enum.
     *
     * Array is length GetParticleCount()
     *
     * @returns The pointer to the head of the particle-flags array.
     */
    public getFlagsBuffer(): ParticleFlag[] {
        return this.flagsBuffer.data;
    }

    /**
     * Set flags for a particle. See the ParticleFlag enum.
     */
    public setParticleFlags(index: number, newFlags: ParticleFlag): void {
        const oldFlags = this.flagsBuffer.data[index];
        if (oldFlags & ~newFlags) {
            // If any flags might be removed
            this.needsUpdateAllParticleFlags = true;
        }
        if (~this.allParticleFlags & newFlags) {
            // If any flags were added
            if (newFlags & ParticleFlag.Tensile) {
                this.accumulation2Buffer = this.requestBuffer(this.accumulation2Buffer);
            }
            if (newFlags & ParticleFlag.ColorMixing) {
                this.colorBuffer.data = this.requestBuffer(this.colorBuffer.data);
            }
            this.allParticleFlags |= newFlags;
        }
        this.flagsBuffer.data[index] = newFlags;
    }

    /**
     * Get flags for a particle. See the ParticleFlag enum.
     */
    public getParticleFlags(index: number): ParticleFlag {
        return this.flagsBuffer.data[index];
    }

    /**
     * Set an external buffer for particle data.
     *
     * Normally, the World's block allocator is used for particle
     * data. However, sometimes you may have an OpenGL or Java
     * buffer for particle data. To avoid data duplication, you may
     * supply this external buffer.
     *
     * Note that, when World's block allocator is used, the
     * particle data buffers can grow as required. However, when
     * external buffers are used, the maximum number of particles is
     * clamped to the size of the smallest external buffer.
     *
     * @param buffer A pointer to a block of memory.
     * @param capacity The number of values in the block.
     */
    public setFlagsBuffer(buffer: ParticleFlag[]): void {
        this.setUserOverridableBuffer(this.flagsBuffer, buffer);
    }

    public setPositionBuffer(buffer: Vec2[]): void {
        this.setUserOverridableBuffer(this.positionBuffer, buffer);
    }

    public setVelocityBuffer(buffer: Vec2[]): void {
        this.setUserOverridableBuffer(this.velocityBuffer, buffer);
    }

    public setColorBuffer(buffer: Color[]): void {
        this.setUserOverridableBuffer(this.colorBuffer, buffer);
    }

    public setUserDataBuffer<T>(buffer: T[]): void {
        this.setUserOverridableBuffer(this.userDataBuffer, buffer);
    }

    /**
     * Get contacts between particles
     * Contact data can be used for many reasons, for example to
     * trigger rendering or audio effects.
     */
    public getContacts(): ParticleContact[] {
        return this.contactBuffer.data;
    }

    public getContactCount(): number {
        return this.contactBuffer.count;
    }

    /**
     * Get contacts between particles and bodies
     *
     * Contact data can be used for many reasons, for example to
     * trigger rendering or audio effects.
     */
    public getBodyContacts(): ParticleBodyContact[] {
        return this.bodyContactBuffer.data;
    }

    public getBodyContactCount(): number {
        return this.bodyContactBuffer.count;
    }

    /**
     * Get array of particle pairs. The particles in a pair:
     *   (1) are contacting,
     *   (2) are in the same particle group,
     *   (3) are part of a rigid particle group, or are spring, elastic,
     *       or wall particles.
     *   (4) have at least one particle that is a spring or barrier
     *       particle (i.e. one of the types in k_pairFlags),
     *   (5) have at least one particle that returns true for
     *       ConnectionFilter::IsNecessary,
     *   (6) are not zombie particles.
     *
     * Essentially, this is an array of spring or barrier particles
     * that are interacting. The array is sorted by ParticlePair's
     * indexA, and then indexB. There are no duplicate entries.
     */
    public getPairs(): ParticlePair[] {
        return this.pairBuffer.data;
    }

    public getPairCount(): number {
        return this.pairBuffer.count;
    }

    /**
     * Get array of particle triads. The particles in a triad:
     *   (1) are in the same particle group,
     *   (2) are in a Voronoi triangle together,
     *   (3) are within b2_maxTriadDistance particle diameters of each
     *       other,
     *   (4) return true for ConnectionFilter::ShouldCreateTriad
     *   (5) have at least one particle of type elastic (i.e. one of the
     *       types in k_triadFlags),
     *   (6) are part of a rigid particle group, or are spring, elastic,
     *       or wall particles.
     *   (7) are not zombie particles.
     *
     * Essentially, this is an array of elastic particles that are
     * interacting. The array is sorted by ParticleTriad's indexA,
     * then indexB, then indexC. There are no duplicate entries.
     */
    public getTriads(): ParticleTriad[] {
        return this.triadBuffer.data;
    }

    public getTriadCount(): number {
        return this.triadBuffer.count;
    }

    /**
     * Set an optional threshold for the maximum number of
     * consecutive particle iterations that a particle may contact
     * multiple bodies before it is considered a candidate for being
     * "stuck". Setting to zero or less disables.
     */
    public setStuckThreshold(steps: number): void {
        this.stuckThreshold = steps;

        if (steps > 0) {
            this.lastBodyContactStepBuffer.data = this.requestBuffer(this.lastBodyContactStepBuffer.data);
            this.bodyContactCountBuffer.data = this.requestBuffer(this.bodyContactCountBuffer.data);
            this.consecutiveContactStepsBuffer.data = this.requestBuffer(this.consecutiveContactStepsBuffer.data);
        }
    }

    /**
     * Get potentially stuck particles from the last step; the user
     * must decide if they are stuck or not, and if so, delete or
     * move them
     */
    public getStuckCandidates(): number[] {
        return this.stuckParticleBuffer.getData();
    }

    /**
     * Get the number of stuck particle candidates from the last
     * step.
     */
    public getStuckCandidateCount(): number {
        return this.stuckParticleBuffer.getCount();
    }

    /**
     * Compute the kinetic energy that can be lost by damping force
     */
    public computeCollisionEnergy(): number {
        const s_v = ParticleSystem.ComputeCollisionEnergy_s_v;
        const vel_data = this.velocityBuffer.data;
        let sum_v2 = 0;
        for (let k = 0; k < this.contactBuffer.count; k++) {
            const contact = this.contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            const n = contact.normal;
            const v = Vec2.subtract(vel_data[b], vel_data[a], s_v);
            const vn = Vec2.dot(v, n);
            if (vn < 0) {
                sum_v2 += vn * vn;
            }
        }
        return 0.5 * this.getParticleMass() * sum_v2;
    }

    public static readonly ComputeCollisionEnergy_s_v = new Vec2();

    /**
     * Set strict Particle/Body contact check.
     *
     * This is an option that will help ensure correct behavior if
     * there are corners in the world model where Particle/Body
     * contact is ambiguous. This option scales at n*log(n) of the
     * number of Particle/Body contacts, so it is best to only
     * enable if it is necessary for your geometry. Enable if you
     * see strange particle behavior around Body intersections.
     */
    public setStrictContactCheck(enabled: boolean): void {
        this.def.strictContactCheck = enabled;
    }

    /**
     * Get the status of the strict contact check.
     */
    public getStrictContactCheck(): boolean {
        return this.def.strictContactCheck;
    }

    /**
     * Set the lifetime (in seconds) of a particle relative to the
     * current time.  A lifetime of less than or equal to 0
     * results in the particle living forever until it's manually
     * destroyed by the application.
     */
    public setParticleLifetime(index: number, lifetime: number): void {
        // DEBUG: assert(this.validateParticleIndex(index));
        const initializeExpirationTimes = this.indexByExpirationTimeBuffer.data === null;
        this.expirationTimeBuffer.data = this.requestBuffer(this.expirationTimeBuffer.data);
        this.indexByExpirationTimeBuffer.data = this.requestBuffer(this.indexByExpirationTimeBuffer.data);

        // Initialize the inverse mapping buffer.
        if (initializeExpirationTimes) {
            const particleCount = this.getParticleCount();
            for (let i = 0; i < particleCount; ++i) {
                this.indexByExpirationTimeBuffer.data[i] = i;
            }
        }

        const quantizedLifetime = lifetime / this.def.lifetimeGranularity;
        // Use a negative lifetime so that it's possible to track which
        // of the infinite lifetime particles are older.
        const newExpirationTime =
            quantizedLifetime > 0 ? this.getQuantizedTimeElapsed() + quantizedLifetime : quantizedLifetime;
        if (newExpirationTime !== this.expirationTimeBuffer.data[index]) {
            this.expirationTimeBuffer.data[index] = newExpirationTime;
            this.expirationTimeBufferRequiresSorting = true;
        }
    }

    /**
     * Get the lifetime (in seconds) of a particle relative to the
     * current time.  A value > 0   is returned if the particle is
     * scheduled to be destroyed in the future, values <= 0
     * indicate the particle has an infinite lifetime.
     */
    public getParticleLifetime(index: number): number {
        // DEBUG: assert(this.validateParticleIndex(index));
        return this.expirationTimeToLifetime(this.getExpirationTimeBuffer()[index]);
    }

    /**
     * Enable / disable destruction of particles in CreateParticle()
     * when no more particles can be created due to a prior call to
     * SetMaxParticleCount().  When this is enabled, the oldest
     * particle is destroyed in CreateParticle() favoring the
     * destruction of particles with a finite lifetime over
     * particles with infinite lifetimes. This feature is enabled by
     * default when particle lifetimes are tracked.  Explicitly
     * enabling this feature using this function enables particle
     * lifetime tracking.
     */
    public setDestructionByAge(enable: boolean): void {
        if (enable) {
            this.getExpirationTimeBuffer();
        }
        this.def.destroyByAge = enable;
    }

    /**
     * Get whether the oldest particle will be destroyed in
     * CreateParticle() when the maximum number of particles are
     * present in the system.
     */
    public getDestructionByAge(): boolean {
        return this.def.destroyByAge;
    }

    /**
     * Get the array of particle expiration times indexed by
     * particle index.
     *
     * GetParticleCount() items are in the returned array.
     */
    public getExpirationTimeBuffer(): number[] {
        this.expirationTimeBuffer.data = this.requestBuffer(this.expirationTimeBuffer.data);
        return this.expirationTimeBuffer.data;
    }

    /**
     * Convert a expiration time value in returned by
     * GetExpirationTimeBuffer() to a time in seconds relative to
     * the current simulation time.
     */
    public expirationTimeToLifetime(expirationTime: number): number {
        return (
            (expirationTime > 0 ? expirationTime - this.getQuantizedTimeElapsed() : expirationTime) *
            this.def.lifetimeGranularity
        );
    }

    /**
     * Get the array of particle indices ordered by reverse
     * lifetime. The oldest particle indexes are at the end of the
     * array with the newest at the start.  Particles with infinite
     * lifetimes (i.e expiration times less than or equal to 0) are
     * placed at the start of the array.
     * ExpirationTimeToLifetime(GetExpirationTimeBuffer()[index]) is
     * equivalent to GetParticleLifetime(index).
     *
     * GetParticleCount() items are in the returned array.
     */
    public getIndexByExpirationTimeBuffer(): number[] {
        // If particles are present, initialize / reinitialize the lifetime buffer.
        if (this.getParticleCount()) {
            this.setParticleLifetime(0, this.getParticleLifetime(0));
        } else {
            this.indexByExpirationTimeBuffer.data = this.requestBuffer(this.indexByExpirationTimeBuffer.data);
        }
        return this.indexByExpirationTimeBuffer.data;
    }

    /**
     * Apply an impulse to one particle. This immediately modifies
     * the velocity. Similar to Body::ApplyLinearImpulse.
     *
     * @param index The particle that will be modified.
     * @param impulse Impulse the world impulse vector, usually in N-seconds or kg-m/s.
     */
    public particleApplyLinearImpulse(index: number, impulse: XY): void {
        this.applyLinearImpulse(index, index + 1, impulse);
    }

    /**
     * Apply an impulse to all particles between 'firstIndex' and
     * 'lastIndex'. This immediately modifies the velocity. Note
     * that the impulse is applied to the total mass of all
     * particles. So, calling ParticleApplyLinearImpulse(0, impulse)
     * and ParticleApplyLinearImpulse(1, impulse) will impart twice
     * as much velocity as calling just ApplyLinearImpulse(0, 1,
     * impulse).
     *
     * @param firstIndex The first particle to be modified.
     * @param lastIndex The last particle to be modified.
     * @param impulse The world impulse vector, usually in N-seconds or kg-m/s.
     */
    public applyLinearImpulse(firstIndex: number, lastIndex: number, impulse: XY): void {
        const vel_data = this.velocityBuffer.data;
        const numParticles = lastIndex - firstIndex;
        const totalMass = numParticles * this.getParticleMass();
        const velocityDelta = Vec2.scale(1 / totalMass, impulse, new Vec2());
        for (let i = firstIndex; i < lastIndex; i++) {
            vel_data[i].add(velocityDelta);
        }
    }

    public static isSignificantForce(force: XY): boolean {
        return force.x !== 0 || force.y !== 0;
    }

    /**
     * Apply a force to the center of a particle.
     *
     * @param index The particle that will be modified.
     * @param force The world force vector, usually in Newtons (N).
     */
    public particleApplyForce(index: number, force: XY): void {
        if (ParticleSystem.isSignificantForce(force) && this.forceCanBeApplied(this.flagsBuffer.data[index])) {
            this.prepareForceBuffer();
            this.forceBuffer[index].add(force);
        }
    }

    /**
     * Distribute a force across several particles. The particles
     * must not be wall particles. Note that the force is
     * distributed across all the particles, so calling this
     * function for indices 0..N is not the same as calling
     * ParticleApplyForce(i, force) for i in 0..N.
     *
     * @param firstIndex The first particle to be modified.
     * @param lastIndex The last particle to be modified.
     * @param force The world force vector, usually in Newtons (N).
     */
    public applyForce(firstIndex: number, lastIndex: number, force: XY): void {
        // Ensure we're not trying to apply force to particles that can't move,
        // such as wall particles.
        // DEBUG: let flags = 0;
        // DEBUG: for (let i = firstIndex; i < lastIndex; i++) {
        // DEBUG:   flags |= this.flagsBuffer.data[i];
        // DEBUG: }
        // DEBUG: assert(this.forceCanBeApplied(flags));

        // Early out if force does nothing (optimization).
        const distributedForce = Vec2.scale(1 / (lastIndex - firstIndex), force, new Vec2());
        if (ParticleSystem.isSignificantForce(distributedForce)) {
            this.prepareForceBuffer();

            // Distribute the force over all the particles.
            for (let i = firstIndex; i < lastIndex; i++) {
                this.forceBuffer[i].add(distributedForce);
            }
        }
    }

    /**
     * Get the next particle-system in the world's particle-system
     * list.
     */
    public getNext(): ParticleSystem | null {
        return this.next;
    }

    /**
     * Query the particle system for all particles that potentially
     * overlap the provided AABB.
     *
     * @param callback A user implemented callback class.
     * @param aabb The query box.
     */
    public queryAABB(aabb: AABB, callback: ParticleQueryCallback): void {
        if (this.proxyBuffer.count === 0) {
            return;
        }
        const beginProxy = 0;
        const endProxy = this.proxyBuffer.count;
        const firstProxy = std_lower_bound(
            this.proxyBuffer.data,
            beginProxy,
            endProxy,
            ParticleSystem.computeTag(
                this.inverseDiameter * aabb.lowerBound.x,
                this.inverseDiameter * aabb.lowerBound.y,
            ),
            ParticleSystem_Proxy.compareProxyTag,
        );
        const lastProxy = std_upper_bound(
            this.proxyBuffer.data,
            firstProxy,
            endProxy,
            ParticleSystem.computeTag(
                this.inverseDiameter * aabb.upperBound.x,
                this.inverseDiameter * aabb.upperBound.y,
            ),
            ParticleSystem_Proxy.compareTagProxy,
        );
        const pos_data = this.positionBuffer.data;
        for (let k = firstProxy; k < lastProxy; ++k) {
            const proxy = this.proxyBuffer.data[k];
            const i = proxy.index;
            const p = pos_data[i];
            if (
                aabb.lowerBound.x < p.x &&
                p.x < aabb.upperBound.x &&
                aabb.lowerBound.y < p.y &&
                p.y < aabb.upperBound.y
            ) {
                if (!callback(i)) {
                    break;
                }
            }
        }
    }

    /**
     * Query the particle system for all particles that potentially
     * overlap the provided shape's AABB. Calls QueryAABB
     * internally.
     *
     * @param callback A user implemented callback class.
     * @param shape The query shape
     * @param xf The transform of the AABB
     * @param childIndex
     */
    public queryShapeAABB(shape: Shape, xf: Transform, childIndex: number, callback: ParticleQueryCallback): void {
        const s_aabb = ParticleSystem.QueryShapeAABB_s_aabb;
        const aabb = s_aabb;
        shape.computeAABB(aabb, xf, childIndex);
        this.queryAABB(aabb, callback);
    }

    public static readonly QueryShapeAABB_s_aabb = new AABB();

    public queryPointAABB(point: XY, slop: number, callback: ParticleQueryCallback): void {
        const s_aabb = ParticleSystem.QueryPointAABB_s_aabb;
        const aabb = s_aabb;
        aabb.lowerBound.set(point.x - slop, point.y - slop);
        aabb.upperBound.set(point.x + slop, point.y + slop);
        this.queryAABB(aabb, callback);
    }

    public static readonly QueryPointAABB_s_aabb = new AABB();

    /**
     * Ray-cast the particle system for all particles in the path of
     * the ray. Your callback controls whether you get the closest
     * point, any point, or n-points. The ray-cast ignores particles
     * that contain the starting point.
     *
     * @param callback A user implemented callback class.
     * @param point1 The ray starting point
     * @param point2 The ray ending point
     */
    public rayCast(point1: XY, point2: XY, callback: ParticleRayCastCallback): void {
        const s_aabb = ParticleSystem.RayCast_s_aabb;
        const s_p = ParticleSystem.RayCast_s_p;
        const s_v = ParticleSystem.RayCast_s_v;
        const s_n = ParticleSystem.RayCast_s_n;
        const s_point = ParticleSystem.RayCast_s_point;
        if (this.proxyBuffer.count === 0) {
            return;
        }
        const pos_data = this.positionBuffer.data;
        const aabb = s_aabb;
        Vec2.min(point1, point2, aabb.lowerBound);
        Vec2.max(point1, point2, aabb.upperBound);
        let fraction = 1;
        // solving the following equation:
        // ((1-t)*point1+t*point2-position)^2=diameter^2
        // where t is a potential fraction
        const v = Vec2.subtract(point2, point1, s_v);
        const v2 = Vec2.dot(v, v);
        const enumerator = this.getInsideBoundsEnumerator(aabb);

        let i: number;
        // eslint-disable-next-line no-cond-assign
        while ((i = enumerator.getNext()) >= 0) {
            const p = Vec2.subtract(point1, pos_data[i], s_p);
            const pv = Vec2.dot(p, v);
            const p2 = Vec2.dot(p, p);
            const determinant = pv * pv - v2 * (p2 - this.squaredDiameter);
            if (determinant >= 0) {
                const sqrtDeterminant = Math.sqrt(determinant);
                // find a solution between 0 and fraction
                let t = (-pv - sqrtDeterminant) / v2;
                if (t > fraction) {
                    continue;
                }
                if (t < 0) {
                    t = (-pv + sqrtDeterminant) / v2;
                    if (t < 0 || t > fraction) {
                        continue;
                    }
                }

                const n = Vec2.addScaled(p, t, v, s_n);
                n.normalize();
                const f = callback(i, Vec2.addScaled(point1, t, v, s_point), n, t);
                fraction = Math.min(fraction, f);
                if (fraction <= 0) {
                    break;
                }
            }
        }
    }

    public static readonly RayCast_s_aabb = new AABB();

    public static readonly RayCast_s_p = new Vec2();

    public static readonly RayCast_s_v = new Vec2();

    public static readonly RayCast_s_n = new Vec2();

    public static readonly RayCast_s_point = new Vec2();

    /**
     * Compute the axis-aligned bounding box for all particles
     * contained within this particle system.
     * @param aabb Returns the axis-aligned bounding box of the system.
     */
    public computeAABB(aabb: AABB): void {
        const particleCount = this.getParticleCount();
        // DEBUG: assert(aabb !== null);
        aabb.lowerBound.x = +MAX_FLOAT;
        aabb.lowerBound.y = +MAX_FLOAT;
        aabb.upperBound.x = -MAX_FLOAT;
        aabb.upperBound.y = -MAX_FLOAT;

        const pos_data = this.positionBuffer.data;
        for (let i = 0; i < particleCount; i++) {
            const p = pos_data[i];
            Vec2.min(aabb.lowerBound, p, aabb.lowerBound);
            Vec2.max(aabb.upperBound, p, aabb.upperBound);
        }
        aabb.lowerBound.x -= this.particleDiameter;
        aabb.lowerBound.y -= this.particleDiameter;
        aabb.upperBound.x += this.particleDiameter;
        aabb.upperBound.y += this.particleDiameter;
    }

    /** All particle types that require creating pairs */
    public static readonly k_pairFlags = ParticleFlag.Spring;

    /** All particle types that require creating triads */
    public static readonly k_triadFlags = ParticleFlag.Elastic;

    /** All particle types that do not produce dynamic pressure */
    public static readonly k_noPressureFlags = ParticleFlag.Powder | ParticleFlag.Tensile;

    /** All particle types that apply extra damping force with bodies */
    public static readonly k_extraDampingFlags = ParticleFlag.StaticPressure;

    public static readonly k_barrierWallFlags = ParticleFlag.Barrier | ParticleFlag.Wall;

    public freeBuffer<T>(b: T[] | null): void {
        if (b === null) {
            return;
        }
        b.length = 0;
    }

    public freeUserOverridableBuffer<T>(b: ParticleSystem_UserOverridableBuffer<T>): void {
        if (b.userSuppliedCapacity === 0) {
            this.freeBuffer(b.data);
        }
    }

    /**
     * Reallocate a buffer
     */
    public reallocateBuffer3<T>(oldBuffer: T[] | null, oldCapacity: number, newCapacity: number): T[] {
        assert(newCapacity > oldCapacity);
        const newBuffer = oldBuffer ? oldBuffer.slice() : [];
        newBuffer.length = newCapacity;
        return newBuffer;
    }

    /**
     * Reallocate a buffer
     */
    public reallocateBuffer5<T>(
        buffer: T[] | null,
        userSuppliedCapacity: number,
        oldCapacity: number,
        newCapacity: number,
        deferred: boolean,
    ): T[] {
        assert(newCapacity > oldCapacity);
        // A 'deferred' buffer is reallocated only if it is not NULL.
        // If 'userSuppliedCapacity' is not zero, buffer is user supplied and must
        // be kept.
        assert(!userSuppliedCapacity || newCapacity <= userSuppliedCapacity);
        if ((!deferred || buffer) && !userSuppliedCapacity) {
            buffer = this.reallocateBuffer3(buffer, oldCapacity, newCapacity);
        }
        return buffer as T[]; // TODO: fix this
    }

    /**
     * Reallocate a buffer
     */
    public reallocateBuffer4<T>(
        buffer: ParticleSystem_UserOverridableBuffer<any>,
        oldCapacity: number,
        newCapacity: number,
        deferred: boolean,
    ): T[] {
        // DEBUG: assert(newCapacity > oldCapacity);
        return this.reallocateBuffer5(buffer.data, buffer.userSuppliedCapacity, oldCapacity, newCapacity, deferred);
    }

    public requestBuffer<T>(buffer: T[] | null): T[] {
        if (!buffer) {
            if (this.internalAllocatedCapacity === 0) {
                this.reallocateInternalAllocatedBuffers(MIN_PARTICLE_SYSTEM_BUFFER_CAPACITY);
            }

            buffer = [];
            buffer.length = this.internalAllocatedCapacity;
        }
        return buffer;
    }

    /**
     * Reallocate the handle / index map and schedule the allocation
     * of a new pool for handle allocation.
     */
    public reallocateHandleBuffers(newCapacity: number): void {
        // DEBUG: assert(newCapacity > this.internalAllocatedCapacity);
        // Reallocate a new handle / index map buffer, copying old handle pointers
        // is fine since they're kept around.
        this.handleIndexBuffer.data = this.reallocateBuffer4(
            this.handleIndexBuffer,
            this.internalAllocatedCapacity,
            newCapacity,
            true,
        );
        // Set the size of the next handle allocation.
        // this.handleAllocator.setItemsPerSlab(newCapacity - this.internalAllocatedCapacity);
    }

    public reallocateInternalAllocatedBuffers(capacity: number): void {
        function limitCapacity(maxCount: number): number {
            return maxCount && capacity > maxCount ? maxCount : capacity;
        }

        // Don't increase capacity beyond the smallest user-supplied buffer size.
        capacity = limitCapacity(this.def.maxCount);
        capacity = limitCapacity(this.flagsBuffer.userSuppliedCapacity);
        capacity = limitCapacity(this.positionBuffer.userSuppliedCapacity);
        capacity = limitCapacity(this.velocityBuffer.userSuppliedCapacity);
        capacity = limitCapacity(this.colorBuffer.userSuppliedCapacity);
        capacity = limitCapacity(this.userDataBuffer.userSuppliedCapacity);
        if (this.internalAllocatedCapacity < capacity) {
            this.reallocateHandleBuffers(capacity);
            this.flagsBuffer.data = this.reallocateBuffer4(
                this.flagsBuffer,
                this.internalAllocatedCapacity,
                capacity,
                false,
            );

            // Conditionally defer these as they are optional if the feature is
            // not enabled.
            const stuck = this.stuckThreshold > 0;
            this.lastBodyContactStepBuffer.data = this.reallocateBuffer4(
                this.lastBodyContactStepBuffer,
                this.internalAllocatedCapacity,
                capacity,
                stuck,
            );
            this.bodyContactCountBuffer.data = this.reallocateBuffer4(
                this.bodyContactCountBuffer,
                this.internalAllocatedCapacity,
                capacity,
                stuck,
            );
            this.consecutiveContactStepsBuffer.data = this.reallocateBuffer4(
                this.consecutiveContactStepsBuffer,
                this.internalAllocatedCapacity,
                capacity,
                stuck,
            );
            this.positionBuffer.data = this.reallocateBuffer4(
                this.positionBuffer,
                this.internalAllocatedCapacity,
                capacity,
                false,
            );
            this.velocityBuffer.data = this.reallocateBuffer4(
                this.velocityBuffer,
                this.internalAllocatedCapacity,
                capacity,
                false,
            );
            this.forceBuffer = this.reallocateBuffer5(
                this.forceBuffer,
                0,
                this.internalAllocatedCapacity,
                capacity,
                false,
            );
            this.weightBuffer = this.reallocateBuffer5(
                this.weightBuffer,
                0,
                this.internalAllocatedCapacity,
                capacity,
                false,
            );
            this.staticPressureBuffer = this.reallocateBuffer5(
                this.staticPressureBuffer,
                0,
                this.internalAllocatedCapacity,
                capacity,
                true,
            );
            this.accumulationBuffer = this.reallocateBuffer5(
                this.accumulationBuffer,
                0,
                this.internalAllocatedCapacity,
                capacity,
                false,
            );
            this.accumulation2Buffer = this.reallocateBuffer5(
                this.accumulation2Buffer,
                0,
                this.internalAllocatedCapacity,
                capacity,
                true,
            );
            this.depthBuffer = this.reallocateBuffer5(
                this.depthBuffer,
                0,
                this.internalAllocatedCapacity,
                capacity,
                true,
            );
            this.colorBuffer.data = this.reallocateBuffer4(
                this.colorBuffer,
                this.internalAllocatedCapacity,
                capacity,
                true,
            );
            this.groupBuffer = this.reallocateBuffer5(
                this.groupBuffer,
                0,
                this.internalAllocatedCapacity,
                capacity,
                false,
            );
            this.userDataBuffer.data = this.reallocateBuffer4(
                this.userDataBuffer,
                this.internalAllocatedCapacity,
                capacity,
                true,
            );
            this.expirationTimeBuffer.data = this.reallocateBuffer4(
                this.expirationTimeBuffer,
                this.internalAllocatedCapacity,
                capacity,
                true,
            );
            this.indexByExpirationTimeBuffer.data = this.reallocateBuffer4(
                this.indexByExpirationTimeBuffer,
                this.internalAllocatedCapacity,
                capacity,
                false,
            );
            this.internalAllocatedCapacity = capacity;
        }
    }

    public createParticleForGroup(groupDef: IParticleGroupDef, xf: Transform, p: XY): void {
        const particleDef = new ParticleDef();
        particleDef.flags = groupDef.flags ?? 0;
        Transform.multiplyVec2(xf, p, particleDef.position);
        Vec2.add(
            groupDef.linearVelocity ?? Vec2.ZERO,
            Vec2.crossScalarVec2(
                groupDef.angularVelocity ?? 0,
                Vec2.subtract(particleDef.position, groupDef.position ?? Vec2.ZERO, Vec2.s_t0),
                Vec2.s_t0,
            ),
            particleDef.velocity,
        );
        particleDef.color.copy(groupDef.color ?? Color.ZERO);
        particleDef.lifetime = groupDef.lifetime ?? 0;
        particleDef.userData = groupDef.userData;
        this.createParticle(particleDef);
    }

    public createParticlesStrokeShapeForGroup(shape: Shape, groupDef: IParticleGroupDef, xf: Transform): void {
        const s_edge = ParticleSystem.CreateParticlesStrokeShapeForGroup_s_edge;
        const s_d = ParticleSystem.CreateParticlesStrokeShapeForGroup_s_d;
        const s_p = ParticleSystem.CreateParticlesStrokeShapeForGroup_s_p;
        let stride = groupDef.stride ?? 0;
        if (stride === 0) {
            stride = this.getParticleStride();
        }
        let positionOnEdge = 0;
        const childCount = shape.getChildCount();
        for (let childIndex = 0; childIndex < childCount; childIndex++) {
            let edge: EdgeShape | null = null;
            if (shape.getType() === ShapeType.Edge) {
                edge = shape as EdgeShape;
            } else {
                // DEBUG: assert(shape.getType() === ShapeType.Chain);
                edge = s_edge;
                (shape as ChainShape).getChildEdge(edge, childIndex);
            }
            const d = Vec2.subtract(edge.vertex2, edge.vertex1, s_d);
            const edgeLength = d.length();

            while (positionOnEdge < edgeLength) {
                const p = Vec2.addScaled(edge.vertex1, positionOnEdge / edgeLength, d, s_p);
                this.createParticleForGroup(groupDef, xf, p);
                positionOnEdge += stride;
            }
            positionOnEdge -= edgeLength;
        }
    }

    public static readonly CreateParticlesStrokeShapeForGroup_s_edge = new EdgeShape();

    public static readonly CreateParticlesStrokeShapeForGroup_s_d = new Vec2();

    public static readonly CreateParticlesStrokeShapeForGroup_s_p = new Vec2();

    public createParticlesFillShapeForGroup(shape: Shape, groupDef: IParticleGroupDef, xf: Transform): void {
        const s_aabb = ParticleSystem.CreateParticlesFillShapeForGroup_s_aabb;
        const s_p = ParticleSystem.CreateParticlesFillShapeForGroup_s_p;
        let stride = groupDef.stride ?? 0;
        if (stride === 0) {
            stride = this.getParticleStride();
        }
        const identity = Transform.IDENTITY;
        const aabb = s_aabb;
        // DEBUG: assert(shape.getChildCount() === 1);
        shape.computeAABB(aabb, identity, 0);
        for (let y = Math.floor(aabb.lowerBound.y / stride) * stride; y < aabb.upperBound.y; y += stride) {
            for (let x = Math.floor(aabb.lowerBound.x / stride) * stride; x < aabb.upperBound.x; x += stride) {
                const p = s_p.set(x, y);
                if (shape.testPoint(identity, p)) {
                    this.createParticleForGroup(groupDef, xf, p);
                }
            }
        }
    }

    public static readonly CreateParticlesFillShapeForGroup_s_aabb = new AABB();

    public static readonly CreateParticlesFillShapeForGroup_s_p = new Vec2();

    public createParticlesWithShapeForGroup(shape: Shape, groupDef: IParticleGroupDef, xf: Transform): void {
        switch (shape.getType()) {
            case ShapeType.Edge:
            case ShapeType.Chain:
                this.createParticlesStrokeShapeForGroup(shape, groupDef, xf);
                break;
            case ShapeType.Polygon:
            case ShapeType.Circle:
                this.createParticlesFillShapeForGroup(shape, groupDef, xf);
                break;
            default:
                // DEBUG: assert(false);
                break;
        }
    }

    public createParticlesWithShapesForGroup(
        shapes: Shape[],
        shapeCount: number,
        groupDef: IParticleGroupDef,
        xf: Transform,
    ): void {
        const compositeShape = new ParticleSystem_CompositeShape(shapes, shapeCount);
        this.createParticlesFillShapeForGroup(compositeShape, groupDef, xf);
    }

    public cloneParticle(oldIndex: number, group: ParticleGroup): number {
        const def = new ParticleDef();
        def.flags = this.flagsBuffer.data[oldIndex];
        def.position.copy(this.positionBuffer.data[oldIndex]);
        def.velocity.copy(this.velocityBuffer.data[oldIndex]);
        if (this.colorBuffer.data) {
            def.color.copy(this.colorBuffer.data[oldIndex]);
        }
        if (this.userDataBuffer.data) {
            def.userData = this.userDataBuffer.data[oldIndex];
        }
        def.group = group;
        const newIndex = this.createParticle(def);
        if (this.handleIndexBuffer.data) {
            const handle = this.handleIndexBuffer.data[oldIndex];
            if (handle) {
                handle.setIndex(newIndex);
            }
            this.handleIndexBuffer.data[newIndex] = handle;
            this.handleIndexBuffer.data[oldIndex] = null;
        }
        if (this.lastBodyContactStepBuffer.data) {
            this.lastBodyContactStepBuffer.data[newIndex] = this.lastBodyContactStepBuffer.data[oldIndex];
        }
        if (this.bodyContactCountBuffer.data) {
            this.bodyContactCountBuffer.data[newIndex] = this.bodyContactCountBuffer.data[oldIndex];
        }
        if (this.consecutiveContactStepsBuffer.data) {
            this.consecutiveContactStepsBuffer.data[newIndex] = this.consecutiveContactStepsBuffer.data[oldIndex];
        }
        if (this.hasForce) {
            this.forceBuffer[newIndex].copy(this.forceBuffer[oldIndex]);
        }
        if (this.staticPressureBuffer) {
            this.staticPressureBuffer[newIndex] = this.staticPressureBuffer[oldIndex];
        }
        if (this.depthBuffer) {
            this.depthBuffer[newIndex] = this.depthBuffer[oldIndex];
        }
        if (this.expirationTimeBuffer.data) {
            this.expirationTimeBuffer.data[newIndex] = this.expirationTimeBuffer.data[oldIndex];
        }
        return newIndex;
    }

    public destroyParticlesInGroup(group: ParticleGroup, callDestructionListener = false): void {
        for (let i = group.firstIndex; i < group.lastIndex; i++) {
            this.destroyParticle(i, callDestructionListener);
        }
    }

    public destroyParticleGroup(group: ParticleGroup): void {
        // DEBUG: assert(this.groupCount > 0);
        // DEBUG: assert(group !== null);

        const destructionListener = this.world.getDestructionListener();
        destructionListener?.sayGoodbyeParticleGroup(group);

        this.setGroupFlags(group, 0);
        for (let i = group.firstIndex; i < group.lastIndex; i++) {
            this.groupBuffer[i] = null;
        }

        if (group.prev) {
            group.prev.next = group.next;
        }
        if (group.next) {
            group.next.prev = group.prev;
        }
        if (group === this.groupList) {
            this.groupList = group.next;
        }

        --this.groupCount;
    }

    public static particleCanBeConnected(flags: ParticleFlag, group: ParticleGroup | null): boolean {
        return (
            (flags & (ParticleFlag.Wall | ParticleFlag.Spring | ParticleFlag.Elastic)) !== 0 ||
            (group !== null && (group.getGroupFlags() & ParticleGroupFlag.Rigid) !== 0)
        );
    }

    public updatePairsAndTriads(firstIndex: number, lastIndex: number, filter: ParticleSystem_ConnectionFilter): void {
        const s_dab = ParticleSystem.UpdatePairsAndTriads_s_dab;
        const s_dbc = ParticleSystem.UpdatePairsAndTriads_s_dbc;
        const s_dca = ParticleSystem.UpdatePairsAndTriads_s_dca;
        const pos_data = this.positionBuffer.data;
        // Create pairs or triads.
        // All particles in each pair/triad should satisfy the following:
        // * firstIndex <= index < lastIndex
        // * don't have Zombie
        // * ParticleCanBeConnected returns true
        // * ShouldCreatePair/ShouldCreateTriad returns true
        // Any particles in each pair/triad should satisfy the following:
        // * filter.IsNeeded returns true
        // * have one of k_pairFlags/k_triadsFlags
        // DEBUG: assert(firstIndex <= lastIndex);
        let particleFlags = 0;
        for (let i = firstIndex; i < lastIndex; i++) {
            particleFlags |= this.flagsBuffer.data[i];
        }
        if (particleFlags & ParticleSystem.k_pairFlags) {
            for (let k = 0; k < this.contactBuffer.count; k++) {
                const contact = this.contactBuffer.data[k];
                const a = contact.indexA;
                const b = contact.indexB;
                const af = this.flagsBuffer.data[a];
                const bf = this.flagsBuffer.data[b];
                const groupA = this.groupBuffer[a];
                const groupB = this.groupBuffer[b];
                if (
                    a >= firstIndex &&
                    a < lastIndex &&
                    b >= firstIndex &&
                    b < lastIndex &&
                    !((af | bf) & ParticleFlag.Zombie) &&
                    (af | bf) & ParticleSystem.k_pairFlags &&
                    (filter.isNecessary(a) || filter.isNecessary(b)) &&
                    ParticleSystem.particleCanBeConnected(af, groupA) &&
                    ParticleSystem.particleCanBeConnected(bf, groupB) &&
                    filter.shouldCreatePair(a, b)
                ) {
                    const pair = this.pairBuffer.data[this.pairBuffer.append()];
                    pair.indexA = a;
                    pair.indexB = b;
                    pair.flags = contact.flags;
                    pair.strength = Math.min(groupA ? groupA.strength : 1, groupB ? groupB.strength : 1);
                    pair.distance = Vec2.distance(pos_data[a], pos_data[b]);
                }
                std_stable_sort(this.pairBuffer.data, 0, this.pairBuffer.count, ParticleSystem.comparePairIndices);
                this.pairBuffer.unique(ParticleSystem.matchPairIndices);
            }
        }
        if (particleFlags & ParticleSystem.k_triadFlags) {
            const diagram = new VoronoiDiagram(lastIndex - firstIndex);
            // let necessary_count = 0;
            for (let i = firstIndex; i < lastIndex; i++) {
                const flags = this.flagsBuffer.data[i];
                const group = this.groupBuffer[i];
                if (!(flags & ParticleFlag.Zombie) && ParticleSystem.particleCanBeConnected(flags, group)) {
                    // if (filter.isNecessary(i)) {
                    //     ++necessary_count;
                    // }
                    diagram.addGenerator(pos_data[i], i, filter.isNecessary(i));
                }
            }
            // if (necessary_count === 0) {
            //     debugger;
            //     for (let i = firstIndex; i < lastIndex; i++) {
            //         filter.isNecessary(i);
            //     }
            // }
            const stride = this.getParticleStride();
            diagram.generate(stride / 2, stride * 2);
            diagram.getNodes((a, b, c) => {
                const af = this.flagsBuffer.data[a];
                const bf = this.flagsBuffer.data[b];
                const cf = this.flagsBuffer.data[c];
                if ((af | bf | cf) & ParticleSystem.k_triadFlags && filter.shouldCreateTriad(a, b, c)) {
                    const pa = pos_data[a];
                    const pb = pos_data[b];
                    const pc = pos_data[c];
                    const dab = Vec2.subtract(pa, pb, s_dab);
                    const dbc = Vec2.subtract(pb, pc, s_dbc);
                    const dca = Vec2.subtract(pc, pa, s_dca);
                    const maxDistanceSquared = MAX_TRIAD_DISTANCE_SQUARED * this.squaredDiameter;
                    if (
                        Vec2.dot(dab, dab) > maxDistanceSquared ||
                        Vec2.dot(dbc, dbc) > maxDistanceSquared ||
                        Vec2.dot(dca, dca) > maxDistanceSquared
                    ) {
                        return;
                    }
                    const groupA = this.groupBuffer[a];
                    const groupB = this.groupBuffer[b];
                    const groupC = this.groupBuffer[c];
                    const triad = this.triadBuffer.data[this.triadBuffer.append()];
                    triad.indexA = a;
                    triad.indexB = b;
                    triad.indexC = c;
                    triad.flags = af | bf | cf;
                    triad.strength = Math.min(
                        Math.min(groupA ? groupA.strength : 1, groupB ? groupB.strength : 1),
                        groupC ? groupC.strength : 1,
                    );
                    const midPoint_x = (pa.x + pb.x + pc.x) / 3;
                    const midPoint_y = (pa.y + pb.y + pc.y) / 3;
                    triad.pa.x = pa.x - midPoint_x;
                    triad.pa.y = pa.y - midPoint_y;
                    triad.pb.x = pb.x - midPoint_x;
                    triad.pb.y = pb.y - midPoint_y;
                    triad.pc.x = pc.x - midPoint_x;
                    triad.pc.y = pc.y - midPoint_y;
                    triad.ka = -Vec2.dot(dca, dab);
                    triad.kb = -Vec2.dot(dab, dbc);
                    triad.kc = -Vec2.dot(dbc, dca);
                    triad.s = Vec2.cross(pa, pb) + Vec2.cross(pb, pc) + Vec2.cross(pc, pa);
                }
            });
            std_stable_sort(this.triadBuffer.data, 0, this.triadBuffer.count, ParticleSystem.compareTriadIndices);
            this.triadBuffer.unique(ParticleSystem.matchTriadIndices);
        }
    }

    private static UpdatePairsAndTriads_s_dab = new Vec2();

    private static UpdatePairsAndTriads_s_dbc = new Vec2();

    private static UpdatePairsAndTriads_s_dca = new Vec2();

    public updatePairsAndTriadsWithReactiveParticles(): void {
        const filter = new ParticleSystem_ReactiveFilter(this.flagsBuffer);
        this.updatePairsAndTriads(0, this.count, filter);

        for (let i = 0; i < this.count; i++) {
            this.flagsBuffer.data[i] &= ~ParticleFlag.Reactive;
        }
        this.allParticleFlags &= ~ParticleFlag.Reactive;
    }

    public static comparePairIndices(a: ParticlePair, b: ParticlePair): boolean {
        const diffA = a.indexA - b.indexA;
        if (diffA !== 0) {
            return diffA < 0;
        }
        return a.indexB < b.indexB;
    }

    public static matchPairIndices(a: ParticlePair, b: ParticlePair): boolean {
        return a.indexA === b.indexA && a.indexB === b.indexB;
    }

    public static compareTriadIndices(a: ParticleTriad, b: ParticleTriad): boolean {
        const diffA = a.indexA - b.indexA;
        if (diffA !== 0) {
            return diffA < 0;
        }
        const diffB = a.indexB - b.indexB;
        if (diffB !== 0) {
            return diffB < 0;
        }
        return a.indexC < b.indexC;
    }

    public static matchTriadIndices(a: ParticleTriad, b: ParticleTriad): boolean {
        return a.indexA === b.indexA && a.indexB === b.indexB && a.indexC === b.indexC;
    }

    public static initializeParticleLists(group: ParticleGroup, nodeBuffer: ParticleSystem_ParticleListNode[]): void {
        const bufferIndex = group.getBufferIndex();
        const particleCount = group.getParticleCount();
        for (let i = 0; i < particleCount; i++) {
            const node = nodeBuffer[i];
            node.list = node;
            node.next = null;
            node.count = 1;
            node.index = i + bufferIndex;
        }
    }

    public mergeParticleListsInContact(group: ParticleGroup, nodeBuffer: ParticleSystem_ParticleListNode[]): void {
        const bufferIndex = group.getBufferIndex();
        for (let k = 0; k < this.contactBuffer.count; k++) {
            /* const ParticleContact& */
            const contact = this.contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            if (!group.containsParticle(a) || !group.containsParticle(b)) {
                continue;
            }
            let listA = nodeBuffer[a - bufferIndex].list;
            let listB = nodeBuffer[b - bufferIndex].list;
            if (listA === listB) {
                continue;
            }
            // To minimize the cost of insertion, make sure listA is longer than
            // listB.
            if (listA.count < listB.count) {
                const tmp = listA;
                listA = listB;
                listB = tmp;
            }
            // DEBUG: assert(listA.count >= listB.count);
            ParticleSystem.mergeParticleLists(listA, listB);
        }
    }

    public static mergeParticleLists(
        listA: ParticleSystem_ParticleListNode,
        listB: ParticleSystem_ParticleListNode,
    ): void {
        // Insert listB between index 0 and 1 of listA
        // Example:
        //     listA => a1 => a2 => a3 => null
        //     listB => b1 => b2 => null
        // to
        //     listA => listB => b1 => b2 => a1 => a2 => a3 => null
        // DEBUG: assert(listA !== listB);
        for (let b = listB; ; ) {
            b.list = listA;
            const nextB: ParticleSystem_ParticleListNode | null = b.next;
            if (nextB) {
                b = nextB;
            } else {
                b.next = listA.next;
                break;
            }
        }
        listA.next = listB;
        listA.count += listB.count;
        listB.count = 0;
    }

    public static findLongestParticleList(
        group: ParticleGroup,
        nodeBuffer: ParticleSystem_ParticleListNode[],
    ): ParticleSystem_ParticleListNode {
        const particleCount = group.getParticleCount();
        let result = nodeBuffer[0];
        for (let i = 0; i < particleCount; i++) {
            const node = nodeBuffer[i];
            if (result.count < node.count) {
                result = node;
            }
        }
        return result;
    }

    public mergeZombieParticleListNodes(
        group: ParticleGroup,
        nodeBuffer: ParticleSystem_ParticleListNode[],
        survivingList: ParticleSystem_ParticleListNode,
    ): void {
        const particleCount = group.getParticleCount();
        for (let i = 0; i < particleCount; i++) {
            const node = nodeBuffer[i];
            if (node !== survivingList && this.flagsBuffer.data[node.index] & ParticleFlag.Zombie) {
                ParticleSystem.mergeParticleListAndNode(survivingList, node);
            }
        }
    }

    public static mergeParticleListAndNode(
        list: ParticleSystem_ParticleListNode,
        node: ParticleSystem_ParticleListNode,
    ): void {
        // Insert node between index 0 and 1 of list
        // Example:
        //     list => a1 => a2 => a3 => null
        //     node => null
        // to
        //     list => node => a1 => a2 => a3 => null
        // DEBUG: assert(node !== list);
        // DEBUG: assert(node.list === node);
        // DEBUG: assert(node.count === 1);
        node.list = list;
        node.next = list.next;
        list.next = node;
        list.count++;
        node.count = 0;
    }

    public createParticleGroupsFromParticleList(
        group: ParticleGroup,
        nodeBuffer: ParticleSystem_ParticleListNode[],
        survivingList: ParticleSystem_ParticleListNode,
    ): void {
        const particleCount = group.getParticleCount();
        const def = new ParticleGroupDef();
        def.groupFlags = group.getGroupFlags();
        def.userData = group.getUserData();
        for (let i = 0; i < particleCount; i++) {
            const list = nodeBuffer[i];
            if (!list.count || list === survivingList) {
                continue;
            }
            // DEBUG: assert(list.list === list);
            const newGroup = this.createParticleGroup(def);
            for (let node: ParticleSystem_ParticleListNode | null = list; node; node = node.next) {
                const oldIndex = node.index;
                // DEBUG: const flags = this.flagsBuffer.data[oldIndex];
                // DEBUG: assert(!(flags & ParticleFlag.Zombie));
                const newIndex = this.cloneParticle(oldIndex, newGroup);
                this.flagsBuffer.data[oldIndex] |= ParticleFlag.Zombie;
                node.index = newIndex;
            }
        }
    }

    public updatePairsAndTriadsWithParticleList(
        group: ParticleGroup,
        nodeBuffer: ParticleSystem_ParticleListNode[],
    ): void {
        const bufferIndex = group.getBufferIndex();
        // Update indices in pairs and triads. If an index belongs to the group,
        // replace it with the corresponding value in nodeBuffer.
        // Note that nodeBuffer is allocated only for the group and the index should
        // be shifted by bufferIndex.
        for (let k = 0; k < this.pairBuffer.count; k++) {
            const pair = this.pairBuffer.data[k];
            const a = pair.indexA;
            const b = pair.indexB;
            if (group.containsParticle(a)) {
                pair.indexA = nodeBuffer[a - bufferIndex].index;
            }
            if (group.containsParticle(b)) {
                pair.indexB = nodeBuffer[b - bufferIndex].index;
            }
        }
        for (let k = 0; k < this.triadBuffer.count; k++) {
            const triad = this.triadBuffer.data[k];
            const a = triad.indexA;
            const b = triad.indexB;
            const c = triad.indexC;
            if (group.containsParticle(a)) {
                triad.indexA = nodeBuffer[a - bufferIndex].index;
            }
            if (group.containsParticle(b)) {
                triad.indexB = nodeBuffer[b - bufferIndex].index;
            }
            if (group.containsParticle(c)) {
                triad.indexC = nodeBuffer[c - bufferIndex].index;
            }
        }
    }

    public computeDepth(): void {
        const contactGroups: ParticleContact[] = []; // TODO: static
        let contactGroupsCount = 0;
        for (let k = 0; k < this.contactBuffer.count; k++) {
            const contact = this.contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            const groupA = this.groupBuffer[a];
            const groupB = this.groupBuffer[b];
            if (groupA && groupA === groupB && groupA.groupFlags & ParticleGroupFlag.NeedsUpdateDepth) {
                contactGroups[contactGroupsCount++] = contact;
            }
        }
        const groupsToUpdate: ParticleGroup[] = []; // TODO: static
        let groupsToUpdateCount = 0;
        for (let group = this.groupList; group; group = group.getNext()) {
            if (group.groupFlags & ParticleGroupFlag.NeedsUpdateDepth) {
                groupsToUpdate[groupsToUpdateCount++] = group;
                this.setGroupFlags(group, group.groupFlags & ~ParticleGroupFlag.NeedsUpdateDepth);
                for (let i = group.firstIndex; i < group.lastIndex; i++) {
                    this.accumulationBuffer[i] = 0;
                }
            }
        }
        // Compute sum of weight of contacts except between different groups.
        for (let k = 0; k < contactGroupsCount; k++) {
            const contact = contactGroups[k];
            const a = contact.indexA;
            const b = contact.indexB;
            const w = contact.weight;
            this.accumulationBuffer[a] += w;
            this.accumulationBuffer[b] += w;
        }

        // DEBUG: assert(this.depthBuffer !== null);
        for (let i = 0; i < groupsToUpdateCount; i++) {
            const group = groupsToUpdate[i];
            for (let j = group.firstIndex; j < group.lastIndex; j++) {
                const w = this.accumulationBuffer[j];
                this.depthBuffer[j] = w < 0.8 ? 0 : MAX_FLOAT;
            }
        }
        // The number of iterations is equal to particle number from the deepest
        // particle to the nearest surface particle, and in general it is smaller
        // than sqrt of total particle number.
        const iterationCount = Math.sqrt(this.count) >> 0;
        for (let t = 0; t < iterationCount; t++) {
            let updated = false;
            for (let k = 0; k < contactGroupsCount; k++) {
                const contact = contactGroups[k];
                const a = contact.indexA;
                const b = contact.indexB;
                const r = 1 - contact.weight;
                const ap0 = this.depthBuffer[a];
                const bp0 = this.depthBuffer[b];
                const ap1 = bp0 + r;
                const bp1 = ap0 + r;
                if (ap0 > ap1) {
                    this.depthBuffer[a] = ap1;
                    updated = true;
                }
                if (bp0 > bp1) {
                    this.depthBuffer[b] = bp1;
                    updated = true;
                }
            }
            if (!updated) {
                break;
            }
        }
        for (let i = 0; i < groupsToUpdateCount; i++) {
            const group = groupsToUpdate[i];
            for (let j = group.firstIndex; j < group.lastIndex; j++) {
                if (this.depthBuffer[j] < MAX_FLOAT) {
                    this.depthBuffer[j] *= this.particleDiameter;
                } else {
                    this.depthBuffer[j] = 0;
                }
            }
        }
    }

    public getInsideBoundsEnumerator(aabb: Readonly<AABB>): ParticleSystem_InsideBoundsEnumerator {
        const lowerTag = ParticleSystem.computeTag(
            this.inverseDiameter * aabb.lowerBound.x - 1,
            this.inverseDiameter * aabb.lowerBound.y - 1,
        );
        const upperTag = ParticleSystem.computeTag(
            this.inverseDiameter * aabb.upperBound.x + 1,
            this.inverseDiameter * aabb.upperBound.y + 1,
        );
        const beginProxy = 0;
        const endProxy = this.proxyBuffer.count;
        const firstProxy = std_lower_bound(
            this.proxyBuffer.data,
            beginProxy,
            endProxy,
            lowerTag,
            ParticleSystem_Proxy.compareProxyTag,
        );
        const lastProxy = std_upper_bound(
            this.proxyBuffer.data,
            beginProxy,
            endProxy,
            upperTag,
            ParticleSystem_Proxy.compareTagProxy,
        );

        // DEBUG: assert(beginProxy <= firstProxy);
        // DEBUG: assert(firstProxy <= lastProxy);
        // DEBUG: assert(lastProxy <= endProxy);

        return new ParticleSystem_InsideBoundsEnumerator(this, lowerTag, upperTag, firstProxy, lastProxy);
    }

    public updateAllParticleFlags(): void {
        this.allParticleFlags = 0;
        for (let i = 0; i < this.count; i++) {
            this.allParticleFlags |= this.flagsBuffer.data[i];
        }
        this.needsUpdateAllParticleFlags = false;
    }

    public updateAllGroupFlags(): void {
        this.allGroupFlags = 0;
        for (let group = this.groupList; group; group = group.getNext()) {
            this.allGroupFlags |= group.groupFlags;
        }
        this.needsUpdateAllGroupFlags = false;
    }

    public addContact(a: number, b: number): void {
        // DEBUG: assert(contacts === this.contactBuffer);
        const flags_data = this.flagsBuffer.data;
        const pos_data = this.positionBuffer.data;
        const d = Vec2.subtract(pos_data[b], pos_data[a], ParticleSystem.AddContact_s_d);
        const distBtParticlesSq = Vec2.dot(d, d);
        if (distBtParticlesSq > 0 && distBtParticlesSq < this.squaredDiameter) {
            const invD = 1 / Math.sqrt(distBtParticlesSq);
            const contact = this.contactBuffer.data[this.contactBuffer.append()];
            contact.indexA = a;
            contact.indexB = b;
            contact.flags = flags_data[a] | flags_data[b];
            contact.weight = 1 - distBtParticlesSq * invD * this.inverseDiameter;
            contact.normal.x = invD * d.x;
            contact.normal.y = invD * d.y;
        }
    }

    public static readonly AddContact_s_d = new Vec2();

    public findContacts_Reference(): void {
        // DEBUG: assert(contacts === this.contactBuffer);
        const beginProxy = 0;
        const endProxy = this.proxyBuffer.count;

        this.contactBuffer.count = 0;
        for (let a = beginProxy, c = beginProxy; a < endProxy; a++) {
            const rightTag = ParticleSystem.computeRelativeTag(this.proxyBuffer.data[a].tag, 1, 0);
            for (let b = a + 1; b < endProxy; b++) {
                if (rightTag < this.proxyBuffer.data[b].tag) {
                    break;
                }
                this.addContact(this.proxyBuffer.data[a].index, this.proxyBuffer.data[b].index);
            }
            const bottomLeftTag = ParticleSystem.computeRelativeTag(this.proxyBuffer.data[a].tag, -1, 1);
            for (; c < endProxy; c++) {
                if (bottomLeftTag <= this.proxyBuffer.data[c].tag) {
                    break;
                }
            }
            const bottomRightTag = ParticleSystem.computeRelativeTag(this.proxyBuffer.data[a].tag, 1, 1);
            for (let b = c; b < endProxy; b++) {
                if (bottomRightTag < this.proxyBuffer.data[b].tag) {
                    break;
                }
                this.addContact(this.proxyBuffer.data[a].index, this.proxyBuffer.data[b].index);
            }
        }
    }

    public findContacts(): void {
        this.findContacts_Reference();
    }

    public updateProxies_Reference(): void {
        // DEBUG: assert(proxies === this.proxyBuffer);
        const pos_data = this.positionBuffer.data;
        const inv_diam = this.inverseDiameter;
        for (let k = 0; k < this.proxyBuffer.count; ++k) {
            const proxy = this.proxyBuffer.data[k];
            const i = proxy.index;
            const p = pos_data[i];
            proxy.tag = ParticleSystem.computeTag(inv_diam * p.x, inv_diam * p.y);
        }
    }

    public updateProxies(): void {
        this.updateProxies_Reference();
    }

    public sortProxies(): void {
        // DEBUG: assert(proxies === this.proxyBuffer);

        std_sort(this.proxyBuffer.data, 0, this.proxyBuffer.count, ParticleSystem_Proxy.compareProxyProxy);
    }

    public filterContacts(): void {
        // Optionally filter the contact.
        const contactFilter = this.getParticleContactFilter();
        if (contactFilter === null) {
            return;
        }

        // DEBUG: assert(contacts === this.contactBuffer);
        this.contactBuffer.removeIf(
            (contact) =>
                (contact.flags & ParticleFlag.ParticleContactFilter) !== 0 &&
                !contactFilter.shouldCollideParticleParticle(this, contact.indexA, contact.indexB),
        );
    }

    public notifyContactListenerPreContact(particlePairs: ParticlePairSet): void {
        const contactListener = this.getParticleContactListener();
        if (contactListener === null) {
            return;
        }

        particlePairs.initialize(this.contactBuffer, this.flagsBuffer);

        throw new Error(); // TODO: notify
    }

    public notifyContactListenerPostContact(particlePairs: ParticlePairSet): void {
        const contactListener = this.getParticleContactListener();
        if (contactListener === null) {
            return;
        }

        // Loop through all new contacts, reporting any new ones, and
        // "invalidating" the ones that still exist.
        for (let k = 0; k < this.contactBuffer.count; ++k) {
            const contact = this.contactBuffer.data[k];
            // ParticlePair pair;
            // pair.first = contact.getIndexA();
            // pair.second = contact.getIndexB();
            // const int32 itemIndex = particlePairs.find(pair);
            const itemIndex = -1; // TODO
            if (itemIndex >= 0) {
                // Already touching, ignore this contact.
                particlePairs.invalidate(itemIndex);
            } else {
                // Just started touching, inform the listener.
                contactListener.beginContactParticleParticle(this, contact);
            }
        }

        // Report particles that are no longer touching.
        // That is, any pairs that were not invalidated above.
        // const int32 pairCount = particlePairs.getCount();
        // const ParticlePair* const pairs = particlePairs.getBuffer();
        // const int8* const valid = particlePairs.getValidBuffer();
        // for (int32 i = 0; i < pairCount; ++i)
        // {
        //  if (valid[i])
        //  {
        //    contactListener.endContactParticleParticle(this, pairs[i].first, pairs[i].second);
        //  }
        // }

        throw new Error(); // TODO: notify
    }

    public static particleContactIsZombie(contact: ParticleContact): boolean {
        return (contact.flags & ParticleFlag.Zombie) === ParticleFlag.Zombie;
    }

    public updateContacts(exceptZombie: boolean): void {
        this.updateProxies();
        this.sortProxies();

        const particlePairs = new ParticlePairSet(); // TODO: static
        this.notifyContactListenerPreContact(particlePairs);

        this.findContacts();
        this.filterContacts();

        this.notifyContactListenerPostContact(particlePairs);

        if (exceptZombie) {
            this.contactBuffer.removeIf(ParticleSystem.particleContactIsZombie);
        }
    }

    public notifyBodyContactListenerPreContact(fixtureSet: ParticleSystem_FixtureParticleSet): void {
        const contactListener = this.getFixtureContactListener();
        if (contactListener === null) {
            return;
        }

        fixtureSet.initialize(this.bodyContactBuffer, this.flagsBuffer);

        throw new Error(); // TODO: notify
    }

    public notifyBodyContactListenerPostContact(fixtureSet: ParticleSystem_FixtureParticleSet): void {
        const contactListener = this.getFixtureContactListener();
        if (contactListener === null) {
            return;
        }

        // Loop through all new contacts, reporting any new ones, and
        // "invalidating" the ones that still exist.
        for (let k = 0; k < this.bodyContactBuffer.count; k++) {
            const contact = this.bodyContactBuffer.data[k];
            // DEBUG: assert(contact !== null);
            // FixtureParticle fixtureParticleToFind;
            // fixtureParticleToFind.first = contact.fixture;
            // fixtureParticleToFind.second = contact.index;
            // const int32 index = fixtureSet.find(fixtureParticleToFind);
            const index = -1; // TODO
            if (index >= 0) {
                // Already touching remove this from the set.
                fixtureSet.invalidate(index);
            } else {
                // Just started touching, report it!
                contactListener.beginContactFixtureParticle(this, contact);
            }
        }

        // If the contact listener is enabled, report all fixtures that are no
        // longer in contact with particles.
        // const FixtureParticle* const fixtureParticles = fixtureSet.getBuffer();
        // const int8* const fixtureParticlesValid = fixtureSet.getValidBuffer();
        // const int32 fixtureParticleCount = fixtureSet.getCount();
        // for (int32 i = 0; i < fixtureParticleCount; ++i)
        // {
        //  if (fixtureParticlesValid[i])
        //  {
        //    const FixtureParticle* const fixtureParticle = &fixtureParticles[i];
        //    contactListener.endContactFixtureParticle(fixtureParticle.first, this, fixtureParticle.second);
        //  }
        // }

        throw new Error(); // TODO: notify
    }

    public updateBodyContacts(): void {
        const s_aabb = ParticleSystem.UpdateBodyContacts_s_aabb;

        // If the particle contact listener is enabled, generate a set of
        // fixture / particle contacts.
        const fixtureSet = new ParticleSystem_FixtureParticleSet(); // TODO: static
        this.notifyBodyContactListenerPreContact(fixtureSet);

        if (this.stuckThreshold > 0) {
            const particleCount = this.getParticleCount();
            for (let i = 0; i < particleCount; i++) {
                // Detect stuck particles, see comment in
                // ParticleSystem::DetectStuckParticle()
                this.bodyContactCountBuffer.data[i] = 0;
                if (this.timestamp > this.lastBodyContactStepBuffer.data[i] + 1) {
                    this.consecutiveContactStepsBuffer.data[i] = 0;
                }
            }
        }
        this.bodyContactBuffer.setCount(0);
        this.stuckParticleBuffer.setCount(0);

        const aabb = s_aabb;
        this.computeAABB(aabb);

        const callback = this.UpdateBodyContacts_callback;
        callback.contactFilter = this.getFixtureContactFilter();
        this.world.queryAABB(aabb, (fixture) => callback.reportFixture(fixture));

        if (this.def.strictContactCheck) {
            this.removeSpuriousBodyContacts();
        }

        this.notifyBodyContactListenerPostContact(fixtureSet);
    }

    public static readonly UpdateBodyContacts_s_aabb = new AABB();

    private readonly UpdateBodyContacts_callback = new ParticleSystem_UpdateBodyContactsCallback(this);

    public solve(step: TimeStep): void {
        const s_subStep = ParticleSystem.Solve_s_subStep;
        if (this.count === 0) {
            return;
        }
        // If particle lifetimes are enabled, destroy particles that are too old.
        if (this.expirationTimeBuffer.data) {
            this.solveLifetimes(step);
        }
        if (this.allParticleFlags & ParticleFlag.Zombie) {
            this.solveZombie();
        }
        if (this.needsUpdateAllParticleFlags) {
            this.updateAllParticleFlags();
        }
        if (this.needsUpdateAllGroupFlags) {
            this.updateAllGroupFlags();
        }
        if (this.paused) {
            return;
        }
        const { particleIterations } = step.config;

        for (this.iterationIndex = 0; this.iterationIndex < particleIterations; this.iterationIndex++) {
            ++this.timestamp;
            const subStep = s_subStep.copy(step);
            subStep.dt /= particleIterations;
            subStep.inv_dt *= particleIterations;
            this.updateContacts(false);
            this.updateBodyContacts();
            this.computeWeight();
            if (this.allGroupFlags & ParticleGroupFlag.NeedsUpdateDepth) {
                this.computeDepth();
            }
            if (this.allParticleFlags & ParticleFlag.Reactive) {
                this.updatePairsAndTriadsWithReactiveParticles();
            }
            if (this.hasForce) {
                this.solveForce(subStep);
            }
            if (this.allParticleFlags & ParticleFlag.Viscous) {
                this.solveViscous();
            }
            if (this.allParticleFlags & ParticleFlag.Repulsive) {
                this.solveRepulsive(subStep);
            }
            if (this.allParticleFlags & ParticleFlag.Powder) {
                this.solvePowder(subStep);
            }
            if (this.allParticleFlags & ParticleFlag.Tensile) {
                this.solveTensile(subStep);
            }
            if (this.allGroupFlags & ParticleGroupFlag.Solid) {
                this.solveSolid(subStep);
            }
            if (this.allParticleFlags & ParticleFlag.ColorMixing) {
                this.solveColorMixing();
            }
            this.solveGravity(subStep);
            if (this.allParticleFlags & ParticleFlag.StaticPressure) {
                this.solveStaticPressure(subStep);
            }
            this.solvePressure(subStep);
            this.solveDamping(subStep);
            if (this.allParticleFlags & ParticleSystem.k_extraDampingFlags) {
                this.solveExtraDamping();
            }
            // SolveElastic and SolveSpring refer the current velocities for
            // numerical stability, they should be called as late as possible.
            if (this.allParticleFlags & ParticleFlag.Elastic) {
                this.solveElastic(subStep);
            }
            if (this.allParticleFlags & ParticleFlag.Spring) {
                this.solveSpring(subStep);
            }
            this.limitVelocity(subStep);
            if (this.allGroupFlags & ParticleGroupFlag.Rigid) {
                this.solveRigidDamping();
            }
            if (this.allParticleFlags & ParticleFlag.Barrier) {
                this.solveBarrier(subStep);
            }
            // SolveCollision, SolveRigid and SolveWall should be called after
            // other force functions because they may require particles to have
            // specific velocities.
            this.solveCollision(subStep);
            if (this.allGroupFlags & ParticleGroupFlag.Rigid) {
                this.solveRigid(subStep);
            }
            if (this.allParticleFlags & ParticleFlag.Wall) {
                this.solveWall();
            }
            // The particle positions can be updated only at the end of substep.
            for (let i = 0; i < this.count; i++) {
                this.positionBuffer.data[i].addScaled(subStep.dt, this.velocityBuffer.data[i]);
            }
        }
    }

    public static readonly Solve_s_subStep = TimeStep.create();

    public solveCollision(step: TimeStep): void {
        const s_aabb = ParticleSystem.SolveCollision_s_aabb;
        const pos_data = this.positionBuffer.data;
        const vel_data = this.velocityBuffer.data;

        // This function detects particles which are crossing boundary of bodies
        // and modifies velocities of them so that they will move just in front of
        // boundary. This function function also applies the reaction force to
        // bodies as precisely as the numerical stability is kept.
        const aabb = s_aabb;
        aabb.lowerBound.x = +MAX_FLOAT;
        aabb.lowerBound.y = +MAX_FLOAT;
        aabb.upperBound.x = -MAX_FLOAT;
        aabb.upperBound.y = -MAX_FLOAT;
        for (let i = 0; i < this.count; i++) {
            const v = vel_data[i];
            const p1 = pos_data[i];
            const p2_x = p1.x + step.dt * v.x;
            const p2_y = p1.y + step.dt * v.y;
            aabb.lowerBound.x = Math.min(aabb.lowerBound.x, Math.min(p1.x, p2_x));
            aabb.lowerBound.y = Math.min(aabb.lowerBound.y, Math.min(p1.y, p2_y));
            aabb.upperBound.x = Math.max(aabb.upperBound.x, Math.max(p1.x, p2_x));
            aabb.upperBound.y = Math.max(aabb.upperBound.y, Math.max(p1.y, p2_y));
        }
        const callback = this.SolveCollision_callback;
        callback.step = step;
        this.world.queryAABB(aabb, (fixture) => callback.reportFixture(fixture));
    }

    public static readonly SolveCollision_s_aabb = new AABB();

    private readonly SolveCollision_callback = new ParticleSystem_SolveCollisionCallback(this);

    public limitVelocity(step: TimeStep): void {
        const vel_data = this.velocityBuffer.data;
        const criticalVelocitySquared = this.getCriticalVelocitySquared(step);
        for (let i = 0; i < this.count; i++) {
            const v = vel_data[i];
            const v2 = Vec2.dot(v, v);
            if (v2 > criticalVelocitySquared) {
                v.scale(Math.sqrt(criticalVelocitySquared / v2));
            }
        }
    }

    public solveGravity(step: TimeStep): void {
        const s_gravity = ParticleSystem.SolveGravity_s_gravity;
        const vel_data = this.velocityBuffer.data;
        const gravity = Vec2.scale(step.dt * this.def.gravityScale, this.world.getGravity(), s_gravity);
        for (let i = 0; i < this.count; i++) {
            vel_data[i].add(gravity);
        }
    }

    public static readonly SolveGravity_s_gravity = new Vec2();

    public solveBarrier(step: TimeStep): void {
        const s_aabb = ParticleSystem.SolveBarrier_s_aabb;
        const s_va = ParticleSystem.SolveBarrier_s_va;
        const s_vb = ParticleSystem.SolveBarrier_s_vb;
        const s_pba = ParticleSystem.SolveBarrier_s_pba;
        const s_vba = ParticleSystem.SolveBarrier_s_vba;
        const s_vc = ParticleSystem.SolveBarrier_s_vc;
        const s_pca = ParticleSystem.SolveBarrier_s_pca;
        const s_vca = ParticleSystem.SolveBarrier_s_vca;
        const s_qba = ParticleSystem.SolveBarrier_s_qba;
        const s_qca = ParticleSystem.SolveBarrier_s_qca;
        const s_dv = ParticleSystem.SolveBarrier_s_dv;
        const s_f = ParticleSystem.SolveBarrier_s_f;
        const pos_data = this.positionBuffer.data;
        const vel_data = this.velocityBuffer.data;
        // If a particle is passing between paired barrier particles,
        // its velocity will be decelerated to avoid passing.
        for (let i = 0; i < this.count; i++) {
            const flags = this.flagsBuffer.data[i];
            if ((flags & ParticleSystem.k_barrierWallFlags) !== 0) {
                vel_data[i].setZero();
            }
        }
        const tmax = BARRIER_COLLISION_TIME * step.dt;
        const mass = this.getParticleMass();
        for (let k = 0; k < this.pairBuffer.count; k++) {
            const pair = this.pairBuffer.data[k];
            if (pair.flags & ParticleFlag.Barrier) {
                const a = pair.indexA;
                const b = pair.indexB;
                const pa = pos_data[a];
                const pb = pos_data[b];
                const aabb = s_aabb;
                Vec2.min(pa, pb, aabb.lowerBound);
                Vec2.max(pa, pb, aabb.upperBound);
                const aGroup = this.groupBuffer[a];
                const bGroup = this.groupBuffer[b];
                const va = this.getLinearVelocity(aGroup, a, pa, s_va);
                const vb = this.getLinearVelocity(bGroup, b, pb, s_vb);
                const pba = Vec2.subtract(pb, pa, s_pba);
                const vba = Vec2.subtract(vb, va, s_vba);
                const enumerator = this.getInsideBoundsEnumerator(aabb);
                let c: number;
                // eslint-disable-next-line no-cond-assign
                while ((c = enumerator.getNext()) >= 0) {
                    const pc = pos_data[c];
                    const cGroup = this.groupBuffer[c];
                    if (aGroup !== cGroup && bGroup !== cGroup) {
                        const vc = this.getLinearVelocity(cGroup, c, pc, s_vc);
                        // Solve the equation below:
                        //   (1-s)*(pa+t*va)+s*(pb+t*vb) = pc+t*vc
                        // which expresses that the particle c will pass a line
                        // connecting the particles a and b at the time of t.
                        // if s is between 0 and 1, c will pass between a and b.
                        const pca = Vec2.subtract(pc, pa, s_pca);
                        const vca = Vec2.subtract(vc, va, s_vca);
                        const e2 = Vec2.cross(vba, vca);
                        const e1 = Vec2.cross(pba, vca) - Vec2.cross(pca, vba);
                        const e0 = Vec2.cross(pba, pca);
                        let s: number;
                        let t: number;
                        const qba = s_qba;
                        const qca = s_qca;
                        if (e2 === 0) {
                            if (e1 === 0) {
                                continue;
                            }
                            t = -e0 / e1;
                            if (!(t >= 0 && t < tmax)) {
                                continue;
                            }
                            Vec2.addScaled(pba, t, vba, qba);
                            Vec2.addScaled(pca, t, vca, qca);
                            s = Vec2.dot(qba, qca) / Vec2.dot(qba, qba);
                            if (!(s >= 0 && s <= 1)) {
                                continue;
                            }
                        } else {
                            const det = e1 * e1 - 4 * e0 * e2;
                            if (det < 0) {
                                continue;
                            }
                            const sqrtDet = Math.sqrt(det);
                            let t1 = (-e1 - sqrtDet) / (2 * e2);
                            let t2 = (-e1 + sqrtDet) / (2 * e2);
                            if (t1 > t2) {
                                const tmp = t1;
                                t1 = t2;
                                t2 = tmp;
                            }
                            t = t1;
                            Vec2.addScaled(pba, t, vba, qba);
                            Vec2.addScaled(pca, t, vca, qca);
                            s = Vec2.dot(qba, qca) / Vec2.dot(qba, qba);
                            if (!(t >= 0 && t < tmax && s >= 0 && s <= 1)) {
                                t = t2;
                                if (!(t >= 0 && t < tmax)) {
                                    continue;
                                }

                                Vec2.addScaled(pba, t, vba, qba);
                                Vec2.addScaled(pca, t, vca, qca);
                                s = Vec2.dot(qba, qca) / Vec2.dot(qba, qba);
                                if (!(s >= 0 && s <= 1)) {
                                    continue;
                                }
                            }
                        }
                        // Apply a force to particle c so that it will have the
                        // interpolated velocity at the collision point on line ab.
                        const dv = s_dv;
                        dv.x = va.x + s * vba.x - vc.x;
                        dv.y = va.y + s * vba.y - vc.y;
                        const f = Vec2.scale(mass, dv, s_f);
                        if (cGroup && this.isRigidGroup(cGroup)) {
                            // If c belongs to a rigid group, the force will be
                            // distributed in the group.
                            const groupMass = cGroup.getMass();
                            const inertia = cGroup.getInertia();
                            if (groupMass > 0) {
                                cGroup.linearVelocity.addScaled(1 / groupMass, f);
                            }
                            if (inertia > 0) {
                                cGroup.angularVelocity +=
                                    Vec2.cross(Vec2.subtract(pc, cGroup.getCenter(), Vec2.s_t0), f) / inertia;
                            }
                        } else {
                            vel_data[c].add(dv);
                        }
                        // Apply a reversed force to particle c after particle
                        // movement so that momentum will be preserved.
                        this.particleApplyForce(c, f.scale(-step.inv_dt));
                    }
                }
            }
        }
    }

    public static readonly SolveBarrier_s_aabb = new AABB();

    public static readonly SolveBarrier_s_va = new Vec2();

    public static readonly SolveBarrier_s_vb = new Vec2();

    public static readonly SolveBarrier_s_pba = new Vec2();

    public static readonly SolveBarrier_s_vba = new Vec2();

    public static readonly SolveBarrier_s_vc = new Vec2();

    public static readonly SolveBarrier_s_pca = new Vec2();

    public static readonly SolveBarrier_s_vca = new Vec2();

    public static readonly SolveBarrier_s_qba = new Vec2();

    public static readonly SolveBarrier_s_qca = new Vec2();

    public static readonly SolveBarrier_s_dv = new Vec2();

    public static readonly SolveBarrier_s_f = new Vec2();

    public solveStaticPressure(step: TimeStep): void {
        this.staticPressureBuffer = this.requestBuffer(this.staticPressureBuffer);
        const criticalPressure = this.getCriticalPressure(step);
        const pressurePerWeight = this.def.staticPressureStrength * criticalPressure;
        const maxPressure = MAX_PARTICLE_PRESSURE * criticalPressure;
        const relaxation = this.def.staticPressureRelaxation;
        // Compute pressure satisfying the modified Poisson equation:
        //   Sum_for_j((p_i - p_j) * w_ij) + relaxation * p_i =
        //   pressurePerWeight * (w_i - b2_minParticleWeight)
        // by iterating the calculation:
        //   p_i = (Sum_for_j(p_j * w_ij) + pressurePerWeight *
        //         (w_i - b2_minParticleWeight)) / (w_i + relaxation)
        // where
        //   p_i and p_j are static pressure of particle i and j
        //   w_ij is contact weight between particle i and j
        //   w_i is sum of contact weight of particle i
        for (let t = 0; t < this.def.staticPressureIterations; t++) {
            for (let i = 0; i < this.count; i++) {
                this.accumulationBuffer[i] = 0;
            }
            for (let k = 0; k < this.contactBuffer.count; k++) {
                const contact = this.contactBuffer.data[k];
                if (contact.flags & ParticleFlag.StaticPressure) {
                    const a = contact.indexA;
                    const b = contact.indexB;
                    const w = contact.weight;
                    this.accumulationBuffer[a] += w * this.staticPressureBuffer[b]; // a <- b
                    this.accumulationBuffer[b] += w * this.staticPressureBuffer[a]; // b <- a
                }
            }
            for (let i = 0; i < this.count; i++) {
                const w = this.weightBuffer[i];
                if (this.flagsBuffer.data[i] & ParticleFlag.StaticPressure) {
                    const wh = this.accumulationBuffer[i];
                    const h = (wh + pressurePerWeight * (w - MIN_PARTICLE_WEIGHT)) / (w + relaxation);
                    this.staticPressureBuffer[i] = clamp(h, 0, maxPressure);
                } else {
                    this.staticPressureBuffer[i] = 0;
                }
            }
        }
    }

    public computeWeight(): void {
        // calculates the sum of contact-weights for each particle
        // that means dimensionless density
        for (let k = 0; k < this.count; k++) {
            this.weightBuffer[k] = 0;
        }
        for (let k = 0; k < this.bodyContactBuffer.count; k++) {
            const contact = this.bodyContactBuffer.data[k];
            const a = contact.index;
            const w = contact.weight;
            this.weightBuffer[a] += w;
        }
        for (let k = 0; k < this.contactBuffer.count; k++) {
            const contact = this.contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            const w = contact.weight;
            this.weightBuffer[a] += w;
            this.weightBuffer[b] += w;
        }
    }

    public solvePressure(step: TimeStep): void {
        const s_f = ParticleSystem.SolvePressure_s_f;
        const pos_data = this.positionBuffer.data;
        const vel_data = this.velocityBuffer.data;
        // calculates pressure as a linear function of density
        const criticalPressure = this.getCriticalPressure(step);
        const pressurePerWeight = this.def.pressureStrength * criticalPressure;
        const maxPressure = MAX_PARTICLE_PRESSURE * criticalPressure;
        for (let i = 0; i < this.count; i++) {
            const w = this.weightBuffer[i];
            const h = pressurePerWeight * Math.max(0, w - MIN_PARTICLE_WEIGHT);
            this.accumulationBuffer[i] = Math.min(h, maxPressure);
        }
        // ignores particles which have their own repulsive force
        if (this.allParticleFlags & ParticleSystem.k_noPressureFlags) {
            for (let i = 0; i < this.count; i++) {
                if (this.flagsBuffer.data[i] & ParticleSystem.k_noPressureFlags) {
                    this.accumulationBuffer[i] = 0;
                }
            }
        }
        // static pressure
        if (this.allParticleFlags & ParticleFlag.StaticPressure) {
            // DEBUG: assert(this.staticPressureBuffer !== null);
            for (let i = 0; i < this.count; i++) {
                if (this.flagsBuffer.data[i] & ParticleFlag.StaticPressure) {
                    this.accumulationBuffer[i] += this.staticPressureBuffer[i];
                }
            }
        }
        // applies pressure between each particles in contact
        const velocityPerPressure = step.dt / (this.def.density * this.particleDiameter);
        const inv_mass = this.getParticleInvMass();
        for (let k = 0; k < this.bodyContactBuffer.count; k++) {
            const contact = this.bodyContactBuffer.data[k];
            const a = contact.index;
            const b = contact.body;
            const w = contact.weight;
            const m = contact.mass;
            const n = contact.normal;
            const p = pos_data[a];
            const h = this.accumulationBuffer[a] + pressurePerWeight * w;
            const f = Vec2.scale(velocityPerPressure * w * m * h, n, s_f);
            vel_data[a].subtractScaled(inv_mass, f);
            b.applyLinearImpulse(f, p, true);
        }
        for (let k = 0; k < this.contactBuffer.count; k++) {
            const contact = this.contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            const w = contact.weight;
            const n = contact.normal;
            const h = this.accumulationBuffer[a] + this.accumulationBuffer[b];
            const f = Vec2.scale(velocityPerPressure * w * h, n, s_f);
            vel_data[a].subtract(f);
            vel_data[b].add(f);
        }
    }

    public static readonly SolvePressure_s_f = new Vec2();

    public solveDamping(step: TimeStep): void {
        const s_v = ParticleSystem.SolveDamping_s_v;
        const s_f = ParticleSystem.SolveDamping_s_f;
        const pos_data = this.positionBuffer.data;
        const vel_data = this.velocityBuffer.data;
        // reduces normal velocity of each contact
        const linearDamping = this.def.dampingStrength;
        const quadraticDamping = 1 / this.getCriticalVelocity(step);
        const inv_mass = this.getParticleInvMass();
        for (let k = 0; k < this.bodyContactBuffer.count; k++) {
            const contact = this.bodyContactBuffer.data[k];
            const a = contact.index;
            const b = contact.body;
            const w = contact.weight;
            const m = contact.mass;
            const n = contact.normal;
            const p = pos_data[a];
            const v = Vec2.subtract(b.getLinearVelocityFromWorldPoint(p, Vec2.s_t0), vel_data[a], s_v);
            const vn = Vec2.dot(v, n);
            if (vn < 0) {
                const damping = Math.max(linearDamping * w, Math.min(-quadraticDamping * vn, 0.5));
                const f = Vec2.scale(damping * m * vn, n, s_f);
                vel_data[a].addScaled(inv_mass, f);
                b.applyLinearImpulse(f.negate(), p, true);
            }
        }
        for (let k = 0; k < this.contactBuffer.count; k++) {
            const contact = this.contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            const w = contact.weight;
            const n = contact.normal;
            const v = Vec2.subtract(vel_data[b], vel_data[a], s_v);
            const vn = Vec2.dot(v, n);
            if (vn < 0) {
                const damping = Math.max(linearDamping * w, Math.min(-quadraticDamping * vn, 0.5));
                const f = Vec2.scale(damping * vn, n, s_f);
                vel_data[a].add(f);
                vel_data[b].subtract(f);
            }
        }
    }

    public static readonly SolveDamping_s_v = new Vec2();

    public static readonly SolveDamping_s_f = new Vec2();

    public solveRigidDamping(): void {
        const s_t0 = ParticleSystem.SolveRigidDamping_s_t0;
        const s_t1 = ParticleSystem.SolveRigidDamping_s_t1;
        const s_p = ParticleSystem.SolveRigidDamping_s_p;
        const s_v = ParticleSystem.SolveRigidDamping_s_v;
        const invMassA = [0];
        const invInertiaA = [0];
        const tangentDistanceA = [0]; // TODO: static
        const invMassB = [0];
        const invInertiaB = [0];
        const tangentDistanceB = [0]; // TODO: static
        // Apply impulse to rigid particle groups colliding with other objects
        // to reduce relative velocity at the colliding point.
        const pos_data = this.positionBuffer.data;
        const damping = this.def.dampingStrength;
        for (let k = 0; k < this.bodyContactBuffer.count; k++) {
            const contact = this.bodyContactBuffer.data[k];
            const a = contact.index;
            const aGroup = this.groupBuffer[a];
            if (aGroup && this.isRigidGroup(aGroup)) {
                const b = contact.body;
                const n = contact.normal;
                const w = contact.weight;
                const p = pos_data[a];
                const v = Vec2.subtract(
                    b.getLinearVelocityFromWorldPoint(p, s_t0),
                    aGroup.getLinearVelocityFromWorldPoint(p, s_t1),
                    s_v,
                );
                const vn = Vec2.dot(v, n);
                if (vn < 0) {
                    // The group's average velocity at particle position 'p' is pushing
                    // the particle into the body.
                    this.initDampingParameterWithRigidGroupOrParticle(
                        invMassA,
                        invInertiaA,
                        tangentDistanceA,
                        true,
                        aGroup,
                        a,
                        p,
                        n,
                    );
                    // Calculate b.I from public functions of Body.
                    this.initDampingParameter(
                        invMassB,
                        invInertiaB,
                        tangentDistanceB,
                        b.getMass(),
                        b.getInertia() - b.getMass() * b.getLocalCenter().lengthSquared(),
                        b.getWorldCenter(),
                        p,
                        n,
                    );
                    const f =
                        damping *
                        Math.min(w, 1) *
                        this.computeDampingImpulse(
                            invMassA[0],
                            invInertiaA[0],
                            tangentDistanceA[0],
                            invMassB[0],
                            invInertiaB[0],
                            tangentDistanceB[0],
                            vn,
                        );
                    this.applyDamping(invMassA[0], invInertiaA[0], tangentDistanceA[0], true, aGroup, a, f, n);
                    b.applyLinearImpulse(Vec2.scale(-f, n, Vec2.s_t0), p, true);
                }
            }
        }
        for (let k = 0; k < this.contactBuffer.count; k++) {
            const contact = this.contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            const n = contact.normal;
            const w = contact.weight;
            const aGroup = this.groupBuffer[a];
            const bGroup = this.groupBuffer[b];
            const aRigid = this.isRigidGroup(aGroup);
            const bRigid = this.isRigidGroup(bGroup);
            if (aGroup !== bGroup && (aRigid || bRigid)) {
                const p = Vec2.mid(pos_data[a], pos_data[b], s_p);
                const v = Vec2.subtract(
                    this.getLinearVelocity(bGroup, b, p, s_t0),
                    this.getLinearVelocity(aGroup, a, p, s_t1),
                    s_v,
                );
                const vn = Vec2.dot(v, n);
                if (vn < 0) {
                    this.initDampingParameterWithRigidGroupOrParticle(
                        invMassA,
                        invInertiaA,
                        tangentDistanceA,
                        aRigid,
                        aGroup,
                        a,
                        p,
                        n,
                    );
                    this.initDampingParameterWithRigidGroupOrParticle(
                        invMassB,
                        invInertiaB,
                        tangentDistanceB,
                        bRigid,
                        bGroup,
                        b,
                        p,
                        n,
                    );
                    const f =
                        damping *
                        w *
                        this.computeDampingImpulse(
                            invMassA[0],
                            invInertiaA[0],
                            tangentDistanceA[0],
                            invMassB[0],
                            invInertiaB[0],
                            tangentDistanceB[0],
                            vn,
                        );
                    this.applyDamping(invMassA[0], invInertiaA[0], tangentDistanceA[0], aRigid, aGroup, a, f, n);
                    this.applyDamping(invMassB[0], invInertiaB[0], tangentDistanceB[0], bRigid, bGroup, b, -f, n);
                }
            }
        }
    }

    public static readonly SolveRigidDamping_s_t0 = new Vec2();

    public static readonly SolveRigidDamping_s_t1 = new Vec2();

    public static readonly SolveRigidDamping_s_p = new Vec2();

    public static readonly SolveRigidDamping_s_v = new Vec2();

    public solveExtraDamping(): void {
        const s_v = ParticleSystem.SolveExtraDamping_s_v;
        const s_f = ParticleSystem.SolveExtraDamping_s_f;
        const vel_data = this.velocityBuffer.data;
        // Applies additional damping force between bodies and particles which can
        // produce strong repulsive force. Applying damping force multiple times
        // is effective in suppressing vibration.
        const pos_data = this.positionBuffer.data;
        const inv_mass = this.getParticleInvMass();
        for (let k = 0; k < this.bodyContactBuffer.count; k++) {
            const contact = this.bodyContactBuffer.data[k];
            const a = contact.index;
            if (this.flagsBuffer.data[a] & ParticleSystem.k_extraDampingFlags) {
                const b = contact.body;
                const m = contact.mass;
                const n = contact.normal;
                const p = pos_data[a];
                const v = Vec2.subtract(b.getLinearVelocityFromWorldPoint(p, Vec2.s_t0), vel_data[a], s_v);
                const vn = Vec2.dot(v, n);
                if (vn < 0) {
                    const f = Vec2.scale(0.5 * m * vn, n, s_f);
                    vel_data[a].addScaled(inv_mass, f);
                    b.applyLinearImpulse(f.negate(), p, true);
                }
            }
        }
    }

    public static readonly SolveExtraDamping_s_v = new Vec2();

    public static readonly SolveExtraDamping_s_f = new Vec2();

    public solveWall(): void {
        const vel_data = this.velocityBuffer.data;
        for (let i = 0; i < this.count; i++) {
            if (this.flagsBuffer.data[i] & ParticleFlag.Wall) {
                vel_data[i].setZero();
            }
        }
    }

    public solveRigid(step: TimeStep): void {
        const s_position = ParticleSystem.SolveRigid_s_position;
        const s_rotation = ParticleSystem.SolveRigid_s_rotation;
        const s_transform = ParticleSystem.SolveRigid_s_transform;
        const s_velocityTransform = ParticleSystem.SolveRigid_s_velocityTransform;
        const pos_data = this.positionBuffer.data;
        const vel_data = this.velocityBuffer.data;
        for (let group = this.groupList; group; group = group.getNext()) {
            if (group.groupFlags & ParticleGroupFlag.Rigid) {
                group.updateStatistics();
                const rotation = s_rotation;
                rotation.set(step.dt * group.angularVelocity);
                const position = Vec2.add(
                    group.center,
                    Vec2.subtract(
                        Vec2.scale(step.dt, group.linearVelocity, Vec2.s_t0),
                        Rot.multiplyVec2(rotation, group.center, Vec2.s_t1),
                        Vec2.s_t0,
                    ),
                    s_position,
                );
                const transform = s_transform;
                transform.setPositionRotation(position, rotation);
                Transform.multiply(transform, group.transform, group.transform);
                const velocityTransform = s_velocityTransform;
                velocityTransform.p.x = step.inv_dt * transform.p.x;
                velocityTransform.p.y = step.inv_dt * transform.p.y;
                velocityTransform.q.s = step.inv_dt * transform.q.s;
                velocityTransform.q.c = step.inv_dt * (transform.q.c - 1);
                for (let i = group.firstIndex; i < group.lastIndex; i++) {
                    Transform.multiplyVec2(velocityTransform, pos_data[i], vel_data[i]);
                }
            }
        }
    }

    public static readonly SolveRigid_s_position = new Vec2();

    public static readonly SolveRigid_s_rotation = new Rot();

    public static readonly SolveRigid_s_transform = new Transform();

    public static readonly SolveRigid_s_velocityTransform = new Transform();

    public solveElastic(step: TimeStep): void {
        const s_pa = ParticleSystem.SolveElastic_s_pa;
        const s_pb = ParticleSystem.SolveElastic_s_pb;
        const s_pc = ParticleSystem.SolveElastic_s_pc;
        const s_r = ParticleSystem.SolveElastic_s_r;
        const s_t0 = ParticleSystem.SolveElastic_s_t0;
        const pos_data = this.positionBuffer.data;
        const vel_data = this.velocityBuffer.data;
        const elasticStrength = step.inv_dt * this.def.elasticStrength;
        for (let k = 0; k < this.triadBuffer.count; k++) {
            const triad = this.triadBuffer.data[k];
            if (triad.flags & ParticleFlag.Elastic) {
                const a = triad.indexA;
                const b = triad.indexB;
                const c = triad.indexC;
                const oa = triad.pa;
                const ob = triad.pb;
                const oc = triad.pc;
                const pa = s_pa.copy(pos_data[a]);
                const pb = s_pb.copy(pos_data[b]);
                const pc = s_pc.copy(pos_data[c]);
                const va = vel_data[a];
                const vb = vel_data[b];
                const vc = vel_data[c];
                pa.addScaled(step.dt, va);
                pb.addScaled(step.dt, vb);
                pc.addScaled(step.dt, vc);
                const midPoint_x = (pa.x + pb.x + pc.x) / 3;
                const midPoint_y = (pa.y + pb.y + pc.y) / 3;
                pa.x -= midPoint_x;
                pa.y -= midPoint_y;
                pb.x -= midPoint_x;
                pb.y -= midPoint_y;
                pc.x -= midPoint_x;
                pc.y -= midPoint_y;
                const r = s_r;
                r.s = Vec2.cross(oa, pa) + Vec2.cross(ob, pb) + Vec2.cross(oc, pc);
                r.c = Vec2.dot(oa, pa) + Vec2.dot(ob, pb) + Vec2.dot(oc, pc);
                const r2 = r.s * r.s + r.c * r.c;
                let invR = 1 / Math.sqrt(r2);
                if (!Number.isFinite(invR)) {
                    invR = 1.98177537e19;
                }
                r.s *= invR;
                r.c *= invR;
                const strength = elasticStrength * triad.strength;
                Rot.multiplyVec2(r, oa, s_t0);
                Vec2.subtract(s_t0, pa, s_t0);
                Vec2.scale(strength, s_t0, s_t0);
                va.add(s_t0);
                Rot.multiplyVec2(r, ob, s_t0);
                Vec2.subtract(s_t0, pb, s_t0);
                Vec2.scale(strength, s_t0, s_t0);
                vb.add(s_t0);
                Rot.multiplyVec2(r, oc, s_t0);
                Vec2.subtract(s_t0, pc, s_t0);
                Vec2.scale(strength, s_t0, s_t0);
                vc.add(s_t0);
            }
        }
    }

    public static readonly SolveElastic_s_pa = new Vec2();

    public static readonly SolveElastic_s_pb = new Vec2();

    public static readonly SolveElastic_s_pc = new Vec2();

    public static readonly SolveElastic_s_r = new Rot();

    public static readonly SolveElastic_s_t0 = new Vec2();

    public solveSpring(step: TimeStep): void {
        const s_pa = ParticleSystem.SolveSpring_s_pa;
        const s_pb = ParticleSystem.SolveSpring_s_pb;
        const s_d = ParticleSystem.SolveSpring_s_d;
        const s_f = ParticleSystem.SolveSpring_s_f;
        const pos_data = this.positionBuffer.data;
        const vel_data = this.velocityBuffer.data;
        const springStrength = step.inv_dt * this.def.springStrength;
        for (let k = 0; k < this.pairBuffer.count; k++) {
            const pair = this.pairBuffer.data[k];
            if (pair.flags & ParticleFlag.Spring) {
                const a = pair.indexA;
                const b = pair.indexB;
                const pa = s_pa.copy(pos_data[a]);
                const pb = s_pb.copy(pos_data[b]);
                const va = vel_data[a];
                const vb = vel_data[b];
                pa.addScaled(step.dt, va);
                pb.addScaled(step.dt, vb);
                const d = Vec2.subtract(pb, pa, s_d);
                const r0 = pair.distance;
                const r1 = d.length();
                const strength = springStrength * pair.strength;
                const f = Vec2.scale((strength * (r0 - r1)) / r1, d, s_f);
                va.subtract(f);
                vb.add(f);
            }
        }
    }

    public static readonly SolveSpring_s_pa = new Vec2();

    public static readonly SolveSpring_s_pb = new Vec2();

    public static readonly SolveSpring_s_d = new Vec2();

    public static readonly SolveSpring_s_f = new Vec2();

    public solveTensile(step: TimeStep): void {
        const s_weightedNormal = ParticleSystem.SolveTensile_s_weightedNormal;
        const s_s = ParticleSystem.SolveTensile_s_s;
        const s_f = ParticleSystem.SolveTensile_s_f;
        const vel_data = this.velocityBuffer.data;
        // DEBUG: assert(this.accumulation2Buffer !== null);
        for (let i = 0; i < this.count; i++) {
            this.accumulation2Buffer[i] = new Vec2();
            this.accumulation2Buffer[i].setZero();
        }
        for (let k = 0; k < this.contactBuffer.count; k++) {
            const contact = this.contactBuffer.data[k];
            if (contact.flags & ParticleFlag.Tensile) {
                const a = contact.indexA;
                const b = contact.indexB;
                const w = contact.weight;
                const n = contact.normal;
                const weightedNormal = Vec2.scale((1 - w) * w, n, s_weightedNormal);
                this.accumulation2Buffer[a].subtract(weightedNormal);
                this.accumulation2Buffer[b].add(weightedNormal);
            }
        }
        const criticalVelocity = this.getCriticalVelocity(step);
        const pressureStrength = this.def.surfaceTensionPressureStrength * criticalVelocity;
        const normalStrength = this.def.surfaceTensionNormalStrength * criticalVelocity;
        const maxVelocityVariation = MAX_PARTICLE_FORCE * criticalVelocity;
        for (let k = 0; k < this.contactBuffer.count; k++) {
            const contact = this.contactBuffer.data[k];
            if (contact.flags & ParticleFlag.Tensile) {
                const a = contact.indexA;
                const b = contact.indexB;
                const w = contact.weight;
                const n = contact.normal;
                const h = this.weightBuffer[a] + this.weightBuffer[b];
                const s = Vec2.subtract(this.accumulation2Buffer[b], this.accumulation2Buffer[a], s_s);
                const fn =
                    Math.min(pressureStrength * (h - 2) + normalStrength * Vec2.dot(s, n), maxVelocityVariation) * w;
                const f = Vec2.scale(fn, n, s_f);
                vel_data[a].subtract(f);
                vel_data[b].add(f);
            }
        }
    }

    public static readonly SolveTensile_s_weightedNormal = new Vec2();

    public static readonly SolveTensile_s_s = new Vec2();

    public static readonly SolveTensile_s_f = new Vec2();

    public solveViscous(): void {
        const s_v = ParticleSystem.SolveViscous_s_v;
        const s_f = ParticleSystem.SolveViscous_s_f;
        const pos_data = this.positionBuffer.data;
        const vel_data = this.velocityBuffer.data;
        const { viscousStrength } = this.def;
        const inv_mass = this.getParticleInvMass();
        for (let k = 0; k < this.bodyContactBuffer.count; k++) {
            const contact = this.bodyContactBuffer.data[k];
            const a = contact.index;
            if (this.flagsBuffer.data[a] & ParticleFlag.Viscous) {
                const b = contact.body;
                const w = contact.weight;
                const m = contact.mass;
                const p = pos_data[a];
                const v = Vec2.subtract(b.getLinearVelocityFromWorldPoint(p, Vec2.s_t0), vel_data[a], s_v);
                const f = Vec2.scale(viscousStrength * m * w, v, s_f);
                vel_data[a].addScaled(inv_mass, f);
                b.applyLinearImpulse(f.negate(), p, true);
            }
        }
        for (let k = 0; k < this.contactBuffer.count; k++) {
            const contact = this.contactBuffer.data[k];
            if (contact.flags & ParticleFlag.Viscous) {
                const a = contact.indexA;
                const b = contact.indexB;
                const w = contact.weight;
                const v = Vec2.subtract(vel_data[b], vel_data[a], s_v);
                const f = Vec2.scale(viscousStrength * w, v, s_f);
                vel_data[a].add(f);
                vel_data[b].subtract(f);
            }
        }
    }

    public static readonly SolveViscous_s_v = new Vec2();

    public static readonly SolveViscous_s_f = new Vec2();

    public solveRepulsive(step: TimeStep): void {
        const s_f = ParticleSystem.SolveRepulsive_s_f;
        const vel_data = this.velocityBuffer.data;
        const repulsiveStrength = this.def.repulsiveStrength * this.getCriticalVelocity(step);
        for (let k = 0; k < this.contactBuffer.count; k++) {
            const contact = this.contactBuffer.data[k];
            if (contact.flags & ParticleFlag.Repulsive) {
                const a = contact.indexA;
                const b = contact.indexB;
                if (this.groupBuffer[a] !== this.groupBuffer[b]) {
                    const w = contact.weight;
                    const n = contact.normal;
                    const f = Vec2.scale(repulsiveStrength * w, n, s_f);
                    vel_data[a].subtract(f);
                    vel_data[b].add(f);
                }
            }
        }
    }

    public static readonly SolveRepulsive_s_f = new Vec2();

    public solvePowder(step: TimeStep): void {
        const s_f = ParticleSystem.SolvePowder_s_f;
        const pos_data = this.positionBuffer.data;
        const vel_data = this.velocityBuffer.data;
        const powderStrength = this.def.powderStrength * this.getCriticalVelocity(step);
        const minWeight = 1 - PARTICLE_STRIDE;
        const inv_mass = this.getParticleInvMass();
        for (let k = 0; k < this.bodyContactBuffer.count; k++) {
            const contact = this.bodyContactBuffer.data[k];
            const a = contact.index;
            if (this.flagsBuffer.data[a] & ParticleFlag.Powder) {
                const w = contact.weight;
                if (w > minWeight) {
                    const b = contact.body;
                    const m = contact.mass;
                    const p = pos_data[a];
                    const n = contact.normal;
                    const f = Vec2.scale(powderStrength * m * (w - minWeight), n, s_f);
                    vel_data[a].subtractScaled(inv_mass, f);
                    b.applyLinearImpulse(f, p, true);
                }
            }
        }
        for (let k = 0; k < this.contactBuffer.count; k++) {
            const contact = this.contactBuffer.data[k];
            if (contact.flags & ParticleFlag.Powder) {
                const w = contact.weight;
                if (w > minWeight) {
                    const a = contact.indexA;
                    const b = contact.indexB;
                    const n = contact.normal;
                    const f = Vec2.scale(powderStrength * (w - minWeight), n, s_f);
                    vel_data[a].subtract(f);
                    vel_data[b].add(f);
                }
            }
        }
    }

    public static readonly SolvePowder_s_f = new Vec2();

    public solveSolid(step: TimeStep): void {
        const s_f = ParticleSystem.SolveSolid_s_f;
        const vel_data = this.velocityBuffer.data;
        // applies extra repulsive force from solid particle groups
        this.depthBuffer = this.requestBuffer(this.depthBuffer);
        const ejectionStrength = step.inv_dt * this.def.ejectionStrength;
        for (let k = 0; k < this.contactBuffer.count; k++) {
            const contact = this.contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            if (this.groupBuffer[a] !== this.groupBuffer[b]) {
                const w = contact.weight;
                const n = contact.normal;
                const h = this.depthBuffer[a] + this.depthBuffer[b];
                const f = Vec2.scale(ejectionStrength * h * w, n, s_f);
                vel_data[a].subtract(f);
                vel_data[b].add(f);
            }
        }
    }

    public static readonly SolveSolid_s_f = new Vec2();

    public solveForce(step: TimeStep): void {
        const vel_data = this.velocityBuffer.data;
        const velocityPerForce = step.dt * this.getParticleInvMass();
        for (let i = 0; i < this.count; i++) {
            vel_data[i].addScaled(velocityPerForce, this.forceBuffer[i]);
        }
        this.hasForce = false;
    }

    public solveColorMixing(): void {
        // mixes color between contacting particles
        const colorMixing = 0.5 * this.def.colorMixingStrength;
        if (colorMixing) {
            for (let k = 0; k < this.contactBuffer.count; k++) {
                const contact = this.contactBuffer.data[k];
                const a = contact.indexA;
                const b = contact.indexB;
                if (this.flagsBuffer.data[a] & this.flagsBuffer.data[b] & ParticleFlag.ColorMixing) {
                    const colorA = this.colorBuffer.data[a];
                    const colorB = this.colorBuffer.data[b];
                    // Use the static method to ensure certain compilers inline
                    // this correctly.
                    Color.mix(colorA, colorB, colorMixing);
                }
            }
        }
    }

    public solveZombie(): void {
        // removes particles with zombie flag
        let newCount = 0;
        const newIndices: number[] = []; // TODO: static
        for (let i = 0; i < this.count; i++) {
            newIndices[i] = INVALID_PARTICLE_INDEX;
        }
        // DEBUG: assert(newIndices.length === this.count);
        let allParticleFlags = 0;
        const destructionListener = this.world.getDestructionListener();
        for (let i = 0; i < this.count; i++) {
            const flags = this.flagsBuffer.data[i];
            if (flags & ParticleFlag.Zombie) {
                if (flags & ParticleFlag.DestructionListener) {
                    destructionListener?.sayGoodbyeParticle(this, i);
                }
                // Destroy particle handle.
                if (this.handleIndexBuffer.data) {
                    const handle = this.handleIndexBuffer.data[i];
                    if (handle) {
                        handle.setIndex(INVALID_PARTICLE_INDEX);
                        this.handleIndexBuffer.data[i] = null;
                    }
                }
                newIndices[i] = INVALID_PARTICLE_INDEX;
            } else {
                newIndices[i] = newCount;
                if (i !== newCount) {
                    // Update handle to reference new particle index.
                    if (this.handleIndexBuffer.data) {
                        const handle = this.handleIndexBuffer.data[i];
                        if (handle) {
                            handle.setIndex(newCount);
                        }
                        this.handleIndexBuffer.data[newCount] = handle;
                    }
                    this.flagsBuffer.data[newCount] = this.flagsBuffer.data[i];
                    if (this.lastBodyContactStepBuffer.data) {
                        this.lastBodyContactStepBuffer.data[newCount] = this.lastBodyContactStepBuffer.data[i];
                    }
                    if (this.bodyContactCountBuffer.data) {
                        this.bodyContactCountBuffer.data[newCount] = this.bodyContactCountBuffer.data[i];
                    }
                    if (this.consecutiveContactStepsBuffer.data) {
                        this.consecutiveContactStepsBuffer.data[newCount] = this.consecutiveContactStepsBuffer.data[i];
                    }
                    this.positionBuffer.data[newCount].copy(this.positionBuffer.data[i]);
                    this.velocityBuffer.data[newCount].copy(this.velocityBuffer.data[i]);
                    this.groupBuffer[newCount] = this.groupBuffer[i];
                    if (this.hasForce) {
                        this.forceBuffer[newCount].copy(this.forceBuffer[i]);
                    }
                    if (this.staticPressureBuffer) {
                        this.staticPressureBuffer[newCount] = this.staticPressureBuffer[i];
                    }
                    if (this.depthBuffer) {
                        this.depthBuffer[newCount] = this.depthBuffer[i];
                    }
                    if (this.colorBuffer.data) {
                        this.colorBuffer.data[newCount].copy(this.colorBuffer.data[i]);
                    }
                    if (this.userDataBuffer.data) {
                        this.userDataBuffer.data[newCount] = this.userDataBuffer.data[i];
                    }
                    if (this.expirationTimeBuffer.data) {
                        this.expirationTimeBuffer.data[newCount] = this.expirationTimeBuffer.data[i];
                    }
                }
                newCount++;
                allParticleFlags |= flags;
            }
        }

        // predicate functions
        const test = {
            isProxyInvalid: (proxy: ParticleSystem_Proxy) => {
                return proxy.index < 0;
            },
            isContactInvalid: (contact: ParticleContact) => {
                return contact.indexA < 0 || contact.indexB < 0;
            },
            isBodyContactInvalid: (contact: ParticleBodyContact) => {
                return contact.index < 0;
            },
            isPairInvalid: (pair: ParticlePair) => {
                return pair.indexA < 0 || pair.indexB < 0;
            },
            isTriadInvalid: (triad: ParticleTriad) => {
                return triad.indexA < 0 || triad.indexB < 0 || triad.indexC < 0;
            },
        };

        // update proxies
        for (let k = 0; k < this.proxyBuffer.count; k++) {
            const proxy = this.proxyBuffer.data[k];
            proxy.index = newIndices[proxy.index];
        }
        this.proxyBuffer.removeIf(test.isProxyInvalid);

        // update contacts
        for (let k = 0; k < this.contactBuffer.count; k++) {
            const contact = this.contactBuffer.data[k];
            contact.indexA = newIndices[contact.indexA];
            contact.indexB = newIndices[contact.indexB];
        }
        this.contactBuffer.removeIf(test.isContactInvalid);

        // update particle-body contacts
        for (let k = 0; k < this.bodyContactBuffer.count; k++) {
            const contact = this.bodyContactBuffer.data[k];
            contact.index = newIndices[contact.index];
        }
        this.bodyContactBuffer.removeIf(test.isBodyContactInvalid);

        // update pairs
        for (let k = 0; k < this.pairBuffer.count; k++) {
            const pair = this.pairBuffer.data[k];
            pair.indexA = newIndices[pair.indexA];
            pair.indexB = newIndices[pair.indexB];
        }
        this.pairBuffer.removeIf(test.isPairInvalid);

        // update triads
        for (let k = 0; k < this.triadBuffer.count; k++) {
            const triad = this.triadBuffer.data[k];
            triad.indexA = newIndices[triad.indexA];
            triad.indexB = newIndices[triad.indexB];
            triad.indexC = newIndices[triad.indexC];
        }
        this.triadBuffer.removeIf(test.isTriadInvalid);

        // Update lifetime indices.
        if (this.indexByExpirationTimeBuffer.data) {
            let writeOffset = 0;
            for (let readOffset = 0; readOffset < this.count; readOffset++) {
                const newIndex = newIndices[this.indexByExpirationTimeBuffer.data[readOffset]];
                if (newIndex !== INVALID_PARTICLE_INDEX) {
                    this.indexByExpirationTimeBuffer.data[writeOffset++] = newIndex;
                }
            }
        }

        // update groups
        for (let group = this.groupList; group; group = group.getNext()) {
            let firstIndex = newCount;
            let lastIndex = 0;
            let modified = false;
            for (let i = group.firstIndex; i < group.lastIndex; i++) {
                const j = newIndices[i];
                if (j >= 0) {
                    firstIndex = Math.min(firstIndex, j);
                    lastIndex = Math.max(lastIndex, j + 1);
                } else {
                    modified = true;
                }
            }
            if (firstIndex < lastIndex) {
                group.firstIndex = firstIndex;
                group.lastIndex = lastIndex;
                if (modified) {
                    if (group.groupFlags & ParticleGroupFlag.Solid) {
                        this.setGroupFlags(group, group.groupFlags | ParticleGroupFlag.NeedsUpdateDepth);
                    }
                }
            } else {
                group.firstIndex = 0;
                group.lastIndex = 0;
                if (!(group.groupFlags & ParticleGroupFlag.CanBeEmpty)) {
                    this.setGroupFlags(group, group.groupFlags | ParticleGroupFlag.WillBeDestroyed);
                }
            }
        }

        // update particle count
        this.count = newCount;
        this.allParticleFlags = allParticleFlags;
        this.needsUpdateAllParticleFlags = false;

        // destroy bodies with no particles
        for (let group = this.groupList; group; ) {
            const next = group.getNext();
            if (group.groupFlags & ParticleGroupFlag.WillBeDestroyed) {
                this.destroyParticleGroup(group);
            }
            group = next;
        }
    }

    /**
     * Destroy all particles which have outlived their lifetimes set
     * by SetParticleLifetime().
     */
    public solveLifetimes(step: TimeStep): void {
        // Update the time elapsed.
        this.timeElapsed = this.lifetimeToExpirationTime(step.dt);
        // Get the floor (non-fractional component) of the elapsed time.
        const quantizedTimeElapsed = this.getQuantizedTimeElapsed();

        const expirationTimes = this.expirationTimeBuffer.data;
        const expirationTimeIndices = this.indexByExpirationTimeBuffer.data;
        const particleCount = this.getParticleCount();
        // Sort the lifetime buffer if it's required.
        if (this.expirationTimeBufferRequiresSorting) {
            /**
             * Compare the lifetime of particleIndexA and particleIndexB
             * returning true if the lifetime of A is greater than B for
             * particles that will expire.  If either particle's lifetime is
             * infinite (<= 0  ) this function return true if the lifetime
             * of A is lesser than B. When used with std::sort() this
             * results in an array of particle indicies sorted in reverse
             * order by particle lifetime.
             *
             * For example, the set of lifetimes
             * (1, 0.7, 0.3, 0, -1, 2)
             * would be sorted as
             * (0, 1, -2, 1, 0.7, 0.3)
             */
            std_sort(expirationTimeIndices, 0, particleCount, (particleIndexA, particleIndexB) => {
                const expirationTimeA = expirationTimes[particleIndexA];
                const expirationTimeB = expirationTimes[particleIndexB];
                const infiniteExpirationTimeA = expirationTimeA <= 0;
                const infiniteExpirationTimeB = expirationTimeB <= 0;
                return infiniteExpirationTimeA === infiniteExpirationTimeB
                    ? expirationTimeA > expirationTimeB
                    : infiniteExpirationTimeA;
            });

            this.expirationTimeBufferRequiresSorting = false;
        }

        // Destroy particles which have expired.
        for (let i = particleCount - 1; i >= 0; --i) {
            const particleIndex = expirationTimeIndices[i];
            const expirationTime = expirationTimes[particleIndex];
            // If no particles need to be destroyed, skip this.
            if (quantizedTimeElapsed < expirationTime || expirationTime <= 0) {
                break;
            }
            // Destroy this particle.
            this.destroyParticle(particleIndex);
        }
    }

    public rotateBuffer(start: number, mid: number, end: number): void {
        // move the particles assigned to the given group toward the end of array
        if (start === mid || mid === end) {
            return;
        }
        // DEBUG: assert(mid >= start && mid <= end);

        function newIndices(i: number): number {
            if (i < start) {
                return i;
            }
            if (i < mid) {
                return i + end - mid;
            }
            if (i < end) {
                return i + start - mid;
            }
            return i;
        }

        std_rotate(this.flagsBuffer.data, start, mid, end);
        if (this.lastBodyContactStepBuffer.data) {
            std_rotate(this.lastBodyContactStepBuffer.data, start, mid, end);
        }
        if (this.bodyContactCountBuffer.data) {
            std_rotate(this.bodyContactCountBuffer.data, start, mid, end);
        }
        if (this.consecutiveContactStepsBuffer.data) {
            std_rotate(this.consecutiveContactStepsBuffer.data, start, mid, end);
        }
        std_rotate(this.positionBuffer.data, start, mid, end);
        std_rotate(this.velocityBuffer.data, start, mid, end);
        std_rotate(this.groupBuffer, start, mid, end);
        if (this.hasForce) {
            std_rotate(this.forceBuffer, start, mid, end);
        }
        if (this.staticPressureBuffer) {
            std_rotate(this.staticPressureBuffer, start, mid, end);
        }
        if (this.depthBuffer) {
            std_rotate(this.depthBuffer, start, mid, end);
        }
        if (this.colorBuffer.data) {
            std_rotate(this.colorBuffer.data, start, mid, end);
        }
        if (this.userDataBuffer.data) {
            std_rotate(this.userDataBuffer.data, start, mid, end);
        }

        // Update handle indices.
        if (this.handleIndexBuffer.data) {
            std_rotate(this.handleIndexBuffer.data, start, mid, end);
            for (let i = start; i < end; ++i) {
                const handle = this.handleIndexBuffer.data[i];
                if (handle) {
                    handle.setIndex(newIndices(handle.getIndex()));
                }
            }
        }

        if (this.expirationTimeBuffer.data) {
            std_rotate(this.expirationTimeBuffer.data, start, mid, end);
            // Update expiration time buffer indices.
            const particleCount = this.getParticleCount();
            const indexByExpirationTime = this.indexByExpirationTimeBuffer.data;
            for (let i = 0; i < particleCount; ++i) {
                indexByExpirationTime[i] = newIndices(indexByExpirationTime[i]);
            }
        }

        // update proxies
        for (let k = 0; k < this.proxyBuffer.count; k++) {
            const proxy = this.proxyBuffer.data[k];
            proxy.index = newIndices(proxy.index);
        }

        // update contacts
        for (let k = 0; k < this.contactBuffer.count; k++) {
            const contact = this.contactBuffer.data[k];
            contact.indexA = newIndices(contact.indexA);
            contact.indexB = newIndices(contact.indexB);
        }

        // update particle-body contacts
        for (let k = 0; k < this.bodyContactBuffer.count; k++) {
            const contact = this.bodyContactBuffer.data[k];
            contact.index = newIndices(contact.index);
        }

        // update pairs
        for (let k = 0; k < this.pairBuffer.count; k++) {
            const pair = this.pairBuffer.data[k];
            pair.indexA = newIndices(pair.indexA);
            pair.indexB = newIndices(pair.indexB);
        }

        // update triads
        for (let k = 0; k < this.triadBuffer.count; k++) {
            const triad = this.triadBuffer.data[k];
            triad.indexA = newIndices(triad.indexA);
            triad.indexB = newIndices(triad.indexB);
            triad.indexC = newIndices(triad.indexC);
        }

        // update groups
        for (let group = this.groupList; group; group = group.getNext()) {
            group.firstIndex = newIndices(group.firstIndex);
            group.lastIndex = newIndices(group.lastIndex - 1) + 1;
        }
    }

    public getCriticalVelocity(step: TimeStep): number {
        return this.particleDiameter * step.inv_dt;
    }

    public getCriticalVelocitySquared(step: TimeStep): number {
        const velocity = this.getCriticalVelocity(step);
        return velocity * velocity;
    }

    public getCriticalPressure(step: TimeStep): number {
        return this.def.density * this.getCriticalVelocitySquared(step);
    }

    public getParticleStride(): number {
        return PARTICLE_STRIDE * this.particleDiameter;
    }

    public getParticleMass(): number {
        const stride = this.getParticleStride();
        return this.def.density * stride * stride;
    }

    public getParticleInvMass(): number {
        // mass = density * stride^2, so we take the inverse of this.
        const inverseStride = this.inverseDiameter * (1 / PARTICLE_STRIDE);
        return this.inverseDensity * inverseStride * inverseStride;
    }

    /**
     * Get the world's contact filter if any particles with the
     * ContactFilter flag are present in the system.
     */
    public getFixtureContactFilter(): ContactFilter | null {
        return this.allParticleFlags & ParticleFlag.FixtureContactFilter
            ? this.world.getContactManager().contactFilter
            : null;
    }

    /**
     * Get the world's contact filter if any particles with the
     * ParticleContactFilter flag are present in the
     * system.
     */
    public getParticleContactFilter(): ContactFilter | null {
        return this.allParticleFlags & ParticleFlag.ParticleContactFilter
            ? this.world.getContactManager().contactFilter
            : null;
    }

    /**
     * Get the world's contact listener if any particles with the
     * FixtureContactListener flag are present in the
     * system.
     */
    public getFixtureContactListener(): ContactListener | null {
        return this.allParticleFlags & ParticleFlag.FixtureContactListener
            ? this.world.getContactManager().contactListener
            : null;
    }

    /**
     * Get the world's contact listener if any particles with the
     * ParticleContactListener flag are present in the
     * system.
     */
    public getParticleContactListener(): ContactListener | null {
        return this.allParticleFlags & ParticleFlag.ParticleContactListener
            ? this.world.getContactManager().contactListener
            : null;
    }

    public setUserOverridableBuffer<T>(buffer: ParticleSystem_UserOverridableBuffer<T>, data: T[]): void {
        buffer.data = data;
        buffer.userSuppliedCapacity = data.length;
    }

    public setGroupFlags(group: ParticleGroup, newFlags: ParticleGroupFlag): void {
        const oldFlags = group.groupFlags;
        if ((oldFlags ^ newFlags) & ParticleGroupFlag.Solid) {
            // If the Solid flag changed schedule depth update.
            newFlags |= ParticleGroupFlag.NeedsUpdateDepth;
        }
        if (oldFlags & ~newFlags) {
            // If any flags might be removed
            this.needsUpdateAllGroupFlags = true;
        }
        if (~this.allGroupFlags & newFlags) {
            // If any flags were added
            if (newFlags & ParticleGroupFlag.Solid) {
                this.depthBuffer = this.requestBuffer(this.depthBuffer);
            }
            this.allGroupFlags |= newFlags;
        }
        group.groupFlags = newFlags;
    }

    public static bodyContactCompare(lhs: ParticleBodyContact, rhs: ParticleBodyContact): boolean {
        if (lhs.index === rhs.index) {
            // Subsort by weight, decreasing.
            return lhs.weight > rhs.weight;
        }
        return lhs.index < rhs.index;
    }

    public removeSpuriousBodyContacts(): void {
        // At this point we have a list of contact candidates based on AABB
        // overlap.The AABB query that  generated this returns all collidable
        // fixtures overlapping particle bounding boxes.  This breaks down around
        // vertices where two shapes intersect, such as a "ground" surface made
        // of multiple PolygonShapes; it potentially applies a lot of spurious
        // impulses from normals that should not actually contribute.  See the
        // Ramp example in Testbed.
        //
        // To correct for this, we apply this algorithm:
        //   * sort contacts by particle and subsort by weight (nearest to farthest)
        //   * for each contact per particle:
        //      - project a point at the contact distance along the inverse of the
        //        contact normal
        //      - if this intersects the fixture that generated the contact, apply
        //         it, otherwise discard as impossible
        //      - repeat for up to n nearest contacts, currently we get good results
        //        from n=3.
        std_sort(this.bodyContactBuffer.data, 0, this.bodyContactBuffer.count, ParticleSystem.bodyContactCompare);

        const s_n = ParticleSystem.RemoveSpuriousBodyContacts_s_n;
        const s_pos = ParticleSystem.RemoveSpuriousBodyContacts_s_pos;
        const s_normal = ParticleSystem.RemoveSpuriousBodyContacts_s_normal;

        // Max number of contacts processed per particle, from nearest to farthest.
        // This must be at least 2 for correctness with concave shapes; 3 was
        // experimentally arrived at as looking reasonable.
        const k_maxContactsPerPoint = 3;
        // Index of last particle processed.
        let lastIndex = -1;
        // Number of contacts processed for the current particle.
        let currentContacts = 0;
        // Output the number of discarded contacts.
        // let discarded = 0;
        const particleBodyContactRemovePredicate = (contact: ParticleBodyContact): boolean => {
            // This implements the selection criteria described in
            // RemoveSpuriousBodyContacts().
            // This functor is iterating through a list of Body contacts per
            // Particle, ordered from near to far.  For up to the maximum number of
            // contacts we allow per point per step, we verify that the contact
            // normal of the Body that genenerated the contact makes physical sense
            // by projecting a point back along that normal and seeing if it
            // intersects the fixture generating the contact.

            if (contact.index !== lastIndex) {
                currentContacts = 0;
                lastIndex = contact.index;
            }

            if (currentContacts++ > k_maxContactsPerPoint) {
                // ++discarded;
                return true;
            }

            // Project along inverse normal (as returned in the contact) to get the
            // point to check.
            const n = s_n.copy(contact.normal);
            // weight is 1-(inv(diameter) * distance)
            n.scale(this.particleDiameter * (1 - contact.weight));
            const pos = Vec2.add(this.positionBuffer.data[contact.index], n, s_pos);

            // pos is now a point projected back along the contact normal to the
            // contact distance. If the surface makes sense for a contact, pos will
            // now lie on or in the fixture generating
            if (!contact.fixture.testPoint(pos)) {
                const childCount = contact.fixture.getShape().getChildCount();
                for (let childIndex = 0; childIndex < childCount; childIndex++) {
                    const normal = s_normal;

                    const distance = computeDistance(
                        contact.fixture.getShape(),
                        contact.fixture.getBody().getTransform(),
                        pos,
                        normal,
                        childIndex,
                    );
                    if (distance < LINEAR_SLOP) {
                        return false;
                    }
                }
                // ++discarded;
                return true;
            }

            return false;
        };
        this.bodyContactBuffer.count = std_remove_if(
            this.bodyContactBuffer.data,
            particleBodyContactRemovePredicate,
            this.bodyContactBuffer.count,
        );
    }

    private static RemoveSpuriousBodyContacts_s_n = new Vec2();

    private static RemoveSpuriousBodyContacts_s_pos = new Vec2();

    private static RemoveSpuriousBodyContacts_s_normal = new Vec2();

    public detectStuckParticle(particle: number): void {
        // Detect stuck particles
        //
        // The basic algorithm is to allow the user to specify an optional
        // threshold where we detect whenever a particle is contacting
        // more than one fixture for more than threshold consecutive
        // steps. This is considered to be "stuck", and these are put
        // in a list the user can query per step, if enabled, to deal with
        // such particles.

        if (this.stuckThreshold <= 0) {
            return;
        }

        // Get the state variables for this particle.
        // int32 * const consecutiveCount = &consecutiveContactStepsBuffer.data[particle];
        // int32 * const lastStep = &lastBodyContactStepBuffer.data[particle];
        // int32 * const bodyCount = &bodyContactCountBuffer.data[particle];

        // This is only called when there is a body contact for this particle.
        // ++(*bodyCount);
        ++this.bodyContactCountBuffer.data[particle];

        // We want to only trigger detection once per step, the first time we
        // contact more than one fixture in a step for a given particle.
        // if (*bodyCount === 2)
        if (this.bodyContactCountBuffer.data[particle] === 2) {
            // ++(*consecutiveCount);
            ++this.consecutiveContactStepsBuffer.data[particle];
            // if (*consecutiveCount > stuckThreshold)
            if (this.consecutiveContactStepsBuffer.data[particle] > this.stuckThreshold) {
                // int32& newStuckParticle = stuckParticleBuffer.append();
                // newStuckParticle = particle;
                this.stuckParticleBuffer.data[this.stuckParticleBuffer.append()] = particle;
            }
        }
        // *lastStep = timestamp;
        this.lastBodyContactStepBuffer.data[particle] = this.timestamp;
    }

    /**
     * Determine whether a particle index is valid.
     */
    public validateParticleIndex(index: number): boolean {
        return index >= 0 && index < this.getParticleCount() && index !== INVALID_PARTICLE_INDEX;
    }

    /**
     * Get the time elapsed in
     * ParticleSystemDef::lifetimeGranularity.
     */
    public getQuantizedTimeElapsed(): number {
        // return (int32)(timeElapsed >> 32);
        return Math.floor(this.timeElapsed / 0x100000000);
    }

    /**
     * Convert a lifetime in seconds to an expiration time.
     */
    public lifetimeToExpirationTime(lifetime: number): number {
        return this.timeElapsed + Math.floor((lifetime / this.def.lifetimeGranularity) * 0x100000000);
    }

    public forceCanBeApplied(flags: ParticleFlag): boolean {
        return !(flags & ParticleFlag.Wall);
    }

    public prepareForceBuffer(): void {
        if (!this.hasForce) {
            for (let i = 0; i < this.count; i++) {
                this.forceBuffer[i].setZero();
            }
            this.hasForce = true;
        }
    }

    public isRigidGroup(group: ParticleGroup | null): boolean {
        return group !== null && (group.groupFlags & ParticleGroupFlag.Rigid) !== 0;
    }

    public getLinearVelocity(group: ParticleGroup | null, particleIndex: number, point: Vec2, out: Vec2): Vec2 {
        if (group && this.isRigidGroup(group)) {
            return group.getLinearVelocityFromWorldPoint(point, out);
        }
        return out.copy(this.velocityBuffer.data[particleIndex]);
    }

    public initDampingParameter(
        invMass: number[],
        invInertia: number[],
        tangentDistance: number[],
        mass: number,
        inertia: number,
        center: Vec2,
        point: Vec2,
        normal: Vec2,
    ): void {
        invMass[0] = mass > 0 ? 1 / mass : 0;
        invInertia[0] = inertia > 0 ? 1 / inertia : 0;
        tangentDistance[0] = Vec2.cross(Vec2.subtract(point, center, Vec2.s_t0), normal);
    }

    public initDampingParameterWithRigidGroupOrParticle(
        invMass: number[],
        invInertia: number[],
        tangentDistance: number[],
        isRigidGroup: boolean,
        group: ParticleGroup | null,
        particleIndex: number,
        point: Vec2,
        normal: Vec2,
    ): void {
        if (group && isRigidGroup) {
            this.initDampingParameter(
                invMass,
                invInertia,
                tangentDistance,
                group.getMass(),
                group.getInertia(),
                group.getCenter(),
                point,
                normal,
            );
        } else {
            const flags = this.flagsBuffer.data[particleIndex];
            this.initDampingParameter(
                invMass,
                invInertia,
                tangentDistance,
                flags & ParticleFlag.Wall ? 0 : this.getParticleMass(),
                0,
                point,
                point,
                normal,
            );
        }
    }

    public computeDampingImpulse(
        invMassA: number,
        invInertiaA: number,
        tangentDistanceA: number,
        invMassB: number,
        invInertiaB: number,
        tangentDistanceB: number,
        normalVelocity: number,
    ): number {
        const invMass =
            invMassA +
            invInertiaA * tangentDistanceA * tangentDistanceA +
            invMassB +
            invInertiaB * tangentDistanceB * tangentDistanceB;
        return invMass > 0 ? normalVelocity / invMass : 0;
    }

    public applyDamping(
        invMass: number,
        invInertia: number,
        tangentDistance: number,
        isRigidGroup: boolean,
        group: ParticleGroup | null,
        particleIndex: number,
        impulse: number,
        normal: Vec2,
    ): void {
        if (group && isRigidGroup) {
            group.linearVelocity.addScaled(impulse * invMass, normal);
            group.angularVelocity += impulse * tangentDistance * invInertia;
        } else {
            this.velocityBuffer.data[particleIndex].addScaled(impulse * invMass, normal);
        }
    }
}

export class ParticleSystem_Proxy {
    public index = INVALID_PARTICLE_INDEX;

    public tag = 0;

    public static compareProxyProxy(a: ParticleSystem_Proxy, b: ParticleSystem_Proxy): boolean {
        return a.tag < b.tag;
    }

    public static compareTagProxy(a: number, b: ParticleSystem_Proxy): boolean {
        return a < b.tag;
    }

    public static compareProxyTag(a: ParticleSystem_Proxy, b: number): boolean {
        return a.tag < b;
    }
}

export class ParticleSystem_InsideBoundsEnumerator {
    public system: ParticleSystem;

    public xLower: number;

    public xUpper: number;

    public yLower: number;

    public yUpper: number;

    public first: number;

    public last: number;

    /**
     * InsideBoundsEnumerator enumerates all particles inside the
     * given bounds.
     *
     * Construct an enumerator with bounds of tags and a range of
     * proxies.
     */
    public constructor(system: ParticleSystem, lower: number, upper: number, first: number, last: number) {
        this.system = system;
        this.xLower = (lower & ParticleSystem.xMask) >>> 0;
        this.xUpper = (upper & ParticleSystem.xMask) >>> 0;
        this.yLower = (lower & ParticleSystem.yMask) >>> 0;
        this.yUpper = (upper & ParticleSystem.yMask) >>> 0;
        this.first = first;
        this.last = last;
        // DEBUG: assert(this.first <= this.last);
    }

    /**
     * Get index of the next particle. Returns
     * b2_invalidParticleIndex if there are no more particles.
     */
    public getNext(): number {
        while (this.first < this.last) {
            const xTag = (this.system.proxyBuffer.data[this.first].tag & ParticleSystem.xMask) >>> 0;
            // #if B2_ASSERT_ENABLED
            // DEBUG: const yTag = (this.system.proxyBuffer.data[this.first].tag & ParticleSystem_yMask) >>> 0;
            // DEBUG: assert(yTag >= this.yLower);
            // DEBUG: assert(yTag <= this.yUpper);
            // #endif
            if (xTag >= this.xLower && xTag <= this.xUpper) {
                return this.system.proxyBuffer.data[this.first++].index;
            }
            this.first++;
        }
        return INVALID_PARTICLE_INDEX;
    }
}

export class ParticleSystem_ParticleListNode {
    /** The head of the list. */
    public list!: ParticleSystem_ParticleListNode;

    /** The next node in the list. */
    public next: ParticleSystem_ParticleListNode | null = null;

    /**
     * Number of entries in the list. Valid only for the node at the
     * head of the list.
     */
    public count = 0;

    /** Particle index. */
    public index = 0;
}

/**
 * @constructor
 */
export class ParticleSystem_FixedSetAllocator<T> {
    public allocate(_itemSize: number, count: number): number {
        // TODO
        return count;
    }

    public clear(): void {
        // TODO
    }

    public getCount(): number {
        // TODO
        return 0;
    }

    public invalidate(_itemIndex: number): void {
        // TODO
    }

    public getValidBuffer(): boolean[] {
        // TODO
        return [];
    }

    public getBuffer(): T[] {
        // TODO
        return [];
    }

    public setCount(_count: number): void {
        // TODO
    }
}

export class ParticleSystem_FixtureParticle {
    public first: Fixture;

    public second = INVALID_PARTICLE_INDEX;

    public constructor(fixture: Fixture, particle: number) {
        this.first = fixture;
        this.second = particle;
    }
}

export class ParticleSystem_FixtureParticleSet extends ParticleSystem_FixedSetAllocator<
    ParticleSystem_FixtureParticle
> {
    public initialize(
        _bodyContactBuffer: GrowableBuffer<ParticleBodyContact>,
        _flagsBuffer: ParticleSystem_UserOverridableBuffer<ParticleFlag>,
    ): void {
        // TODO
    }

    public find(_pair: ParticleSystem_FixtureParticle): number {
        // TODO
        return INVALID_PARTICLE_INDEX;
    }
}

export class ParticleSystem_ParticlePair {
    public first = INVALID_PARTICLE_INDEX;

    public second = INVALID_PARTICLE_INDEX;

    public constructor(particleA: number, particleB: number) {
        this.first = particleA;
        this.second = particleB;
    }
}

export class ParticlePairSet extends ParticleSystem_FixedSetAllocator<ParticleSystem_ParticlePair> {
    public initialize(
        _contactBuffer: GrowableBuffer<ParticleContact>,
        _flagsBuffer: ParticleSystem_UserOverridableBuffer<ParticleFlag>,
    ): void {
        // TODO
    }

    public find(_pair: ParticleSystem_ParticlePair): number {
        // TODO
        return INVALID_PARTICLE_INDEX;
    }
}

export class ParticleSystem_ConnectionFilter {
    /**
     * Is the particle necessary for connection?
     * A pair or a triad should contain at least one 'necessary'
     * particle.
     */
    public isNecessary(_index: number): boolean {
        return true;
    }

    /**
     * An additional condition for creating a pair.
     */
    public shouldCreatePair(_a: number, _b: number): boolean {
        return true;
    }

    /**
     * An additional condition for creating a triad.
     */
    public shouldCreateTriad(_a: number, _b: number, _c: number): boolean {
        return true;
    }
}

export class ParticleSystem_JoinParticleGroupsFilter extends ParticleSystem_ConnectionFilter {
    public threshold = 0;

    public constructor(threshold: number) {
        super();
        this.threshold = threshold;
    }

    /**
     * An additional condition for creating a pair.
     */
    public shouldCreatePair(a: number, b: number): boolean {
        return (a < this.threshold && this.threshold <= b) || (b < this.threshold && this.threshold <= a);
    }

    /**
     * An additional condition for creating a triad.
     */
    public shouldCreateTriad(a: number, b: number, c: number): boolean {
        return (
            (a < this.threshold || b < this.threshold || c < this.threshold) &&
            (this.threshold <= a || this.threshold <= b || this.threshold <= c)
        );
    }
}

export class ParticleSystem_CompositeShape extends Shape {
    public constructor(shapes: Shape[], shapeCount = shapes.length) {
        super(ShapeType.Unknown, 0);
        this.shapes = shapes;
        this.shapeCount = shapeCount;
    }

    public shapes: Shape[];

    public shapeCount = 0;

    public clone(): Shape {
        throw new Error();
    }

    public getChildCount(): number {
        return 1;
    }

    /**
     * @see Shape::TestPoint
     */
    public testPoint(xf: Transform, p: XY): boolean {
        for (let i = 0; i < this.shapeCount; i++) {
            if (this.shapes[i].testPoint(xf, p)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Implement Shape.
     */
    public rayCast(_output: RayCastOutput, _input: RayCastInput, _xf: Transform, _childIndex: number): boolean {
        // DEBUG: assert(false);
        return false;
    }

    /**
     * @see Shape::ComputeAABB
     */
    public computeAABB(aabb: AABB, xf: Transform, _childIndex: number): void {
        const s_subaabb = new AABB();
        aabb.lowerBound.x = +MAX_FLOAT;
        aabb.lowerBound.y = +MAX_FLOAT;
        aabb.upperBound.x = -MAX_FLOAT;
        aabb.upperBound.y = -MAX_FLOAT;
        // DEBUG: assert(childIndex === 0);
        for (let i = 0; i < this.shapeCount; i++) {
            const childCount = this.shapes[i].getChildCount();
            for (let j = 0; j < childCount; j++) {
                const subaabb = s_subaabb;
                this.shapes[i].computeAABB(subaabb, xf, j);
                aabb.combine1(subaabb);
            }
        }
    }

    /**
     * @see Shape::ComputeMass
     */
    public computeMass(_massData: MassData, _density: number): void {
        // DEBUG: assert(false);
    }

    public setupDistanceProxy(_proxy: DistanceProxy, _index: number): void {
        // DEBUG: assert(false);
    }

    public draw() {}
}

export class ParticleSystem_ReactiveFilter extends ParticleSystem_ConnectionFilter {
    public flagsBuffer: ParticleSystem_UserOverridableBuffer<ParticleFlag>;

    public constructor(flagsBuffer: ParticleSystem_UserOverridableBuffer<ParticleFlag>) {
        super();
        this.flagsBuffer = flagsBuffer;
    }

    public isNecessary(index: number): boolean {
        return (this.flagsBuffer.data[index] & ParticleFlag.Reactive) !== 0;
    }
}
