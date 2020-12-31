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
    Clamp,
    Rot,
    ContactFilter,
    ContactListener,
    RayCastOutput,
    RayCastInput,
    MassData,
    DistanceProxy,
    Assert,
} from "@box2d/core";

// DEBUG: import { Assert, b2_maxParticleIndex } from "../common/b2_settings";
import {
    b2_invalidParticleIndex,
    b2_minParticleSystemBufferCapacity,
    b2_maxTriadDistanceSquared,
    b2_barrierCollisionTime,
    b2_maxParticlePressure,
    b2_minParticleWeight,
    b2_maxParticleForce,
    b2_particleStride,
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

    public Append(): number {
        if (this.count >= this.capacity) {
            this.Grow();
        }
        return this.count++;
    }

    public Reserve(newCapacity: number): void {
        if (this.capacity >= newCapacity) {
            return;
        }

        // DEBUG: Assert(this.capacity === this.data.length);
        for (let i = this.capacity; i < newCapacity; ++i) {
            this.data[i] = this.allocator();
        }
        this.capacity = newCapacity;
    }

    public Grow(): void {
        // Double the capacity.
        const newCapacity = this.capacity ? 2 * this.capacity : b2_minParticleSystemBufferCapacity;
        // DEBUG: Assert(newCapacity > this.capacity);
        this.Reserve(newCapacity);
    }

    public Free(): void {
        if (this.data.length === 0) {
            return;
        }

        this.data = [];
        this.capacity = 0;
        this.count = 0;
    }

    public Shorten(_newEnd: number): void {
        // DEBUG: Assert(false);
    }

    public Data(): T[] {
        return this.data;
    }

    public GetCount(): number {
        return this.count;
    }

    public SetCount(newCount: number): void {
        // DEBUG: Assert(0 <= newCount && newCount <= this.capacity);
        this.count = newCount;
    }

    public GetCapacity(): number {
        return this.capacity;
    }

    public RemoveIf(pred: (t: T) => boolean): void {
        // DEBUG: let count = 0;
        // DEBUG: for (let i = 0; i < this.count; ++i) {
        // DEBUG:   if (!pred(this.data[i])) {
        // DEBUG:     count++;
        // DEBUG:   }
        // DEBUG: }

        this.count = std_remove_if(this.data, pred, this.count);

        // DEBUG: Assert(count === this.count);
    }

    public Unique(pred: (a: T, b: T) => boolean): void {
        this.count = std_unique(this.data, 0, this.count, pred);
    }
}

export class ParticleSystem_UserOverridableBuffer<T> {
    public m_data: T[] | null = null;

    public get data(): T[] {
        return this.m_data as T[];
    } // HACK: may return null

    public set data(value: T[]) {
        this.m_data = value;
    }

    public userSuppliedCapacity = 0;
}

export type ParticleIndex = number;

export class ParticleContact {
    public indexA = 0;

    public indexB = 0;

    public weight = 0;

    public normal = new Vec2();

    public flags: ParticleFlag = 0;

    public SetIndices(a: number, b: number): void {
        // DEBUG: Assert(a <= b2_maxParticleIndex && b <= b2_maxParticleIndex);
        this.indexA = a;
        this.indexB = b;
    }

    public SetWeight(w: number): void {
        this.weight = w;
    }

    public SetNormal(n: Vec2): void {
        this.normal.Copy(n);
    }

    public SetFlags(f: ParticleFlag): void {
        this.flags = f;
    }

    public GetIndexA(): number {
        return this.indexA;
    }

    public GetIndexB(): number {
        return this.indexB;
    }

    public GetWeight(): number {
        return this.weight;
    }

    public GetNormal(): Vec2 {
        return this.normal;
    }

    public GetFlags(): ParticleFlag {
        return this.flags;
    }

    public IsEqual(rhs: ParticleContact): boolean {
        return (
            this.indexA === rhs.indexA &&
            this.indexB === rhs.indexB &&
            this.flags === rhs.flags &&
            this.weight === rhs.weight &&
            this.normal.x === rhs.normal.x &&
            this.normal.y === rhs.normal.y
        );
    }

    public IsNotEqual(rhs: ParticleContact): boolean {
        return !this.IsEqual(rhs);
    }

    public ApproximatelyEqual(rhs: ParticleContact): boolean {
        const MAX_WEIGHT_DIFF = 0.01; // Weight 0 ~ 1, so about 1%
        const MAX_NORMAL_DIFF_SQ = 0.01 * 0.01; // Normal length = 1, so 1%
        return (
            this.indexA === rhs.indexA &&
            this.indexB === rhs.indexB &&
            this.flags === rhs.flags &&
            Math.abs(this.weight - rhs.weight) < MAX_WEIGHT_DIFF &&
            Vec2.DistanceSquared(this.normal, rhs.normal) < MAX_NORMAL_DIFF_SQ
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

    public Copy(def: ParticleSystemDef): ParticleSystemDef {
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

    public Clone(): ParticleSystemDef {
        return new ParticleSystemDef().Copy(this);
    }
}

export class ParticleSystem {
    public m_paused = false;

    public m_timestamp = 0;

    public m_allParticleFlags: ParticleFlag = 0;

    public m_needsUpdateAllParticleFlags = false;

    public m_allGroupFlags: ParticleGroupFlag = 0;

    public m_needsUpdateAllGroupFlags = false;

    public m_hasForce = false;

    public m_iterationIndex = 0;

    public m_inverseDensity = 0;

    public m_particleDiameter = 0;

    public m_inverseDiameter = 0;

    public m_squaredDiameter = 0;

    public m_count = 0;

    public m_internalAllocatedCapacity = 0;

    /** Maps particle indices to handles. */
    public m_handleIndexBuffer: ParticleSystem_UserOverridableBuffer<ParticleHandle | null> = new ParticleSystem_UserOverridableBuffer<ParticleHandle | null>();

    public m_flagsBuffer: ParticleSystem_UserOverridableBuffer<ParticleFlag> = new ParticleSystem_UserOverridableBuffer<
        ParticleFlag
    >();

    public m_positionBuffer: ParticleSystem_UserOverridableBuffer<Vec2> = new ParticleSystem_UserOverridableBuffer<
        Vec2
    >();

    public m_velocityBuffer: ParticleSystem_UserOverridableBuffer<Vec2> = new ParticleSystem_UserOverridableBuffer<
        Vec2
    >();

    public m_forceBuffer: Vec2[] = [];

    /**
     * this.m_weightBuffer is populated in ComputeWeight and used in
     * ComputeDepth(), SolveStaticPressure() and SolvePressure().
     */
    public m_weightBuffer: number[] = [];

    /**
     * When any particles have the flag StaticPressure,
     * this.m_staticPressureBuffer is first allocated and used in
     * SolveStaticPressure() and SolvePressure().  It will be
     * reallocated on subsequent CreateParticle() calls.
     */
    public m_staticPressureBuffer: number[] = [];

    /**
     * this.m_accumulationBuffer is used in many functions as a temporary
     * buffer for scalar values.
     */
    public m_accumulationBuffer: number[] = [];

    /**
     * When any particles have the flag Tensile,
     * this.m_accumulation2Buffer is first allocated and used in
     * SolveTensile() as a temporary buffer for vector values.  It
     * will be reallocated on subsequent CreateParticle() calls.
     */
    public m_accumulation2Buffer: Vec2[] = [];

    /**
     * When any particle groups have the flag Solid,
     * this.m_depthBuffer is first allocated and populated in
     * ComputeDepth() and used in SolveSolid(). It will be
     * reallocated on subsequent CreateParticle() calls.
     */
    public m_depthBuffer: number[] = [];

    public m_colorBuffer = new ParticleSystem_UserOverridableBuffer<Color>();

    public m_groupBuffer: Array<ParticleGroup | null> = [];

    public m_userDataBuffer = new ParticleSystem_UserOverridableBuffer<any>();

    /** Stuck particle detection parameters and record keeping */
    public m_stuckThreshold = 0;

    public m_lastBodyContactStepBuffer: ParticleSystem_UserOverridableBuffer<
        number
    > = new ParticleSystem_UserOverridableBuffer<number>();

    public m_bodyContactCountBuffer: ParticleSystem_UserOverridableBuffer<
        number
    > = new ParticleSystem_UserOverridableBuffer<number>();

    public m_consecutiveContactStepsBuffer: ParticleSystem_UserOverridableBuffer<
        number
    > = new ParticleSystem_UserOverridableBuffer<number>();

    public m_stuckParticleBuffer = new GrowableBuffer<number>(() => 0);

    public m_proxyBuffer = new GrowableBuffer<ParticleSystem_Proxy>(() => new ParticleSystem_Proxy());

    public m_contactBuffer = new GrowableBuffer<ParticleContact>(() => new ParticleContact());

    public m_bodyContactBuffer = new GrowableBuffer<ParticleBodyContact>(() => new ParticleBodyContact());

    public m_pairBuffer = new GrowableBuffer<ParticlePair>(() => new ParticlePair());

    public m_triadBuffer = new GrowableBuffer<ParticleTriad>(() => new ParticleTriad());

    /**
     * Time each particle should be destroyed relative to the last
     * time this.m_timeElapsed was initialized.  Each unit of time
     * corresponds to ParticleSystemDef::lifetimeGranularity
     * seconds.
     */
    public m_expirationTimeBuffer: ParticleSystem_UserOverridableBuffer<
        number
    > = new ParticleSystem_UserOverridableBuffer<number>();

    /**
     * List of particle indices sorted by expiration time.
     */
    public m_indexByExpirationTimeBuffer: ParticleSystem_UserOverridableBuffer<
        number
    > = new ParticleSystem_UserOverridableBuffer<number>();

    /**
     * Time elapsed in 32:32 fixed point.  Each non-fractional unit
     * of time corresponds to
     * ParticleSystemDef::lifetimeGranularity seconds.
     */
    public m_timeElapsed = 0;

    /**
     * Whether the expiration time buffer has been modified and
     * needs to be resorted.
     */
    public m_expirationTimeBufferRequiresSorting = false;

    public m_groupCount = 0;

    public m_groupList: ParticleGroup | null = null;

    public m_def = new ParticleSystemDef();

    public m_world: World;

    public m_prev: ParticleSystem | null = null;

    public m_next: ParticleSystem | null = null;

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
        this.SetStrictContactCheck(def.strictContactCheck);
        this.SetDensity(def.density);
        this.SetGravityScale(def.gravityScale);
        this.SetRadius(def.radius);
        this.SetMaxParticleCount(def.maxCount);
        // DEBUG: Assert(def.lifetimeGranularity > 0);
        this.m_def = def.Clone();
        this.m_world = world;
        this.SetDestructionByAge(this.m_def.destroyByAge);
    }

    public Drop(): void {
        while (this.m_groupList) {
            this.DestroyParticleGroup(this.m_groupList);
        }

        this.FreeUserOverridableBuffer(this.m_handleIndexBuffer);
        this.FreeUserOverridableBuffer(this.m_flagsBuffer);
        this.FreeUserOverridableBuffer(this.m_lastBodyContactStepBuffer);
        this.FreeUserOverridableBuffer(this.m_bodyContactCountBuffer);
        this.FreeUserOverridableBuffer(this.m_consecutiveContactStepsBuffer);
        this.FreeUserOverridableBuffer(this.m_positionBuffer);
        this.FreeUserOverridableBuffer(this.m_velocityBuffer);
        this.FreeUserOverridableBuffer(this.m_colorBuffer);
        this.FreeUserOverridableBuffer(this.m_userDataBuffer);
        this.FreeUserOverridableBuffer(this.m_expirationTimeBuffer);
        this.FreeUserOverridableBuffer(this.m_indexByExpirationTimeBuffer);
        this.FreeBuffer(this.m_forceBuffer);
        this.FreeBuffer(this.m_weightBuffer);
        this.FreeBuffer(this.m_staticPressureBuffer);
        this.FreeBuffer(this.m_accumulationBuffer);
        this.FreeBuffer(this.m_accumulation2Buffer);
        this.FreeBuffer(this.m_depthBuffer);
        this.FreeBuffer(this.m_groupBuffer);
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
    public CreateParticle(def: IParticleDef): number {
        Assert(!this.m_world.IsLocked());

        if (this.m_count >= this.m_internalAllocatedCapacity) {
            // Double the particle capacity.
            const capacity = this.m_count ? 2 * this.m_count : b2_minParticleSystemBufferCapacity;
            this.ReallocateInternalAllocatedBuffers(capacity);
        }
        if (this.m_count >= this.m_internalAllocatedCapacity) {
            // If the oldest particle should be destroyed...
            if (this.m_def.destroyByAge) {
                this.DestroyOldestParticle(0, false);
                // Need to destroy this particle *now* so that it's possible to
                // create a new particle.
                this.SolveZombie();
            } else {
                return b2_invalidParticleIndex;
            }
        }
        const index = this.m_count++;
        this.m_flagsBuffer.data[index] = 0;
        if (this.m_lastBodyContactStepBuffer.data) {
            this.m_lastBodyContactStepBuffer.data[index] = 0;
        }
        if (this.m_bodyContactCountBuffer.data) {
            this.m_bodyContactCountBuffer.data[index] = 0;
        }
        if (this.m_consecutiveContactStepsBuffer.data) {
            this.m_consecutiveContactStepsBuffer.data[index] = 0;
        }
        this.m_positionBuffer.data[index] = (this.m_positionBuffer.data[index] || new Vec2()).Copy(
            def.position ?? Vec2.ZERO,
        );
        this.m_velocityBuffer.data[index] = (this.m_velocityBuffer.data[index] || new Vec2()).Copy(
            def.velocity ?? Vec2.ZERO,
        );
        this.m_weightBuffer[index] = 0;
        this.m_forceBuffer[index] = (this.m_forceBuffer[index] || new Vec2()).SetZero();
        if (this.m_staticPressureBuffer) {
            this.m_staticPressureBuffer[index] = 0;
        }
        if (this.m_depthBuffer) {
            this.m_depthBuffer[index] = 0;
        }
        const color = new Color().Copy(def.color ?? Color.ZERO);
        if (this.m_colorBuffer.data || !color.IsZero()) {
            this.m_colorBuffer.data = this.RequestBuffer(this.m_colorBuffer.data);
            this.m_colorBuffer.data[index] = (this.m_colorBuffer.data[index] || new Color()).Copy(color);
        }
        if (this.m_userDataBuffer.data || def.userData) {
            this.m_userDataBuffer.data = this.RequestBuffer(this.m_userDataBuffer.data);
            this.m_userDataBuffer.data[index] = def.userData;
        }
        if (this.m_handleIndexBuffer.data) {
            this.m_handleIndexBuffer.data[index] = null;
        }

        const proxy = this.m_proxyBuffer.data[this.m_proxyBuffer.Append()];

        // If particle lifetimes are enabled or the lifetime is set in the particle
        // definition, initialize the lifetime.
        const lifetime = def.lifetime ?? 0;
        const finiteLifetime = lifetime > 0;
        if (this.m_expirationTimeBuffer.data || finiteLifetime) {
            this.SetParticleLifetime(
                index,
                finiteLifetime ? lifetime : this.ExpirationTimeToLifetime(-this.GetQuantizedTimeElapsed()),
            );
            // Add a reference to the newly added particle to the end of the
            // queue.
            this.m_indexByExpirationTimeBuffer.data[index] = index;
        }

        proxy.index = index;
        const group = def.group ?? null;
        this.m_groupBuffer[index] = group;
        if (group) {
            if (group.m_firstIndex < group.m_lastIndex) {
                // Move particles in the group just before the new particle.
                this.RotateBuffer(group.m_firstIndex, group.m_lastIndex, index);
                // DEBUG: Assert(group.m_lastIndex === index);
                // Update the index range of the group to contain the new particle.
                group.m_lastIndex = index + 1;
            } else {
                // If the group is empty, reset the index range to contain only the
                // new particle.
                group.m_firstIndex = index;
                group.m_lastIndex = index + 1;
            }
        }
        this.SetParticleFlags(index, def.flags ?? 0);
        return index;
    }

    /**
     * Retrieve a handle to the particle at the specified index.
     *
     * Please see #ParticleHandle for why you might want a handle.
     */
    public GetParticleHandleFromIndex(index: number): ParticleHandle {
        // DEBUG: Assert(index >= 0 && index < this.GetParticleCount() && index !== b2_invalidParticleIndex);
        this.m_handleIndexBuffer.data = this.RequestBuffer(this.m_handleIndexBuffer.data);
        let handle = this.m_handleIndexBuffer.data[index];
        if (handle) {
            return handle;
        }
        // Create a handle.
        handle = new ParticleHandle();
        // DEBUG: Assert(handle !== null);
        handle.SetIndex(index);
        this.m_handleIndexBuffer.data[index] = handle;
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
    public DestroyParticle(index: number, callDestructionListener = false): void {
        let flags = ParticleFlag.Zombie;
        if (callDestructionListener) {
            flags |= ParticleFlag.DestructionListener;
        }
        this.SetParticleFlags(index, this.m_flagsBuffer.data[index] | flags);
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
    public DestroyOldestParticle(index: number, callDestructionListener = false): void {
        const particleCount = this.GetParticleCount();
        // DEBUG: Assert(index >= 0 && index < particleCount);
        // Make sure particle lifetime tracking is enabled.
        // DEBUG: Assert(this.m_indexByExpirationTimeBuffer.data !== null);
        // Destroy the oldest particle (preferring to destroy finite
        // lifetime particles first) to free a slot in the buffer.
        const oldestFiniteLifetimeParticle = this.m_indexByExpirationTimeBuffer.data[particleCount - (index + 1)];
        const oldestInfiniteLifetimeParticle = this.m_indexByExpirationTimeBuffer.data[index];
        this.DestroyParticle(
            this.m_expirationTimeBuffer.data[oldestFiniteLifetimeParticle] > 0.0
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
    public DestroyParticlesInShape(shape: Shape, xf: Transform, callDestructionListener = false): number {
        const s_aabb = ParticleSystem.DestroyParticlesInShape_s_aabb;
        Assert(!this.m_world.IsLocked());

        const aabb = s_aabb;
        shape.ComputeAABB(aabb, xf, 0);
        let destroyed = 0;
        this.QueryAABB(aabb, (index) => {
            // DEBUG: Assert(index >= 0 && index < this.m_system.m_count);
            if (shape.TestPoint(xf, this.m_positionBuffer.data[index])) {
                this.DestroyParticle(index, callDestructionListener);
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
    public CreateParticleGroup(groupDef: IParticleGroupDef): ParticleGroup {
        const s_transform = ParticleSystem.CreateParticleGroup_s_transform;

        Assert(!this.m_world.IsLocked());

        const transform = s_transform;
        transform.SetPositionAngle(groupDef.position ?? Vec2.ZERO, groupDef.angle ?? 0);
        const firstIndex = this.m_count;
        if (groupDef.shape) {
            this.CreateParticlesWithShapeForGroup(groupDef.shape, groupDef, transform);
        }
        if (groupDef.shapes) {
            this.CreateParticlesWithShapesForGroup(
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
                this.CreateParticleForGroup(groupDef, transform, p);
            }
        }
        const lastIndex = this.m_count;

        let group = new ParticleGroup(this);
        group.m_firstIndex = firstIndex;
        group.m_lastIndex = lastIndex;
        group.m_strength = groupDef.strength ?? 1;
        group.m_userData = groupDef.userData;
        group.m_transform.Copy(transform);
        group.m_prev = null;
        group.m_next = this.m_groupList;
        if (this.m_groupList) {
            this.m_groupList.m_prev = group;
        }
        this.m_groupList = group;
        ++this.m_groupCount;
        for (let i = firstIndex; i < lastIndex; i++) {
            this.m_groupBuffer[i] = group;
        }
        this.SetGroupFlags(group, groupDef.groupFlags ?? 0);

        // Create pairs and triads between particles in the group.
        const filter = new ParticleSystem_ConnectionFilter();
        this.UpdateContacts(true);
        this.UpdatePairsAndTriads(firstIndex, lastIndex, filter);

        if (groupDef.group) {
            this.JoinParticleGroups(groupDef.group, group);
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
    public JoinParticleGroups(groupA: ParticleGroup, groupB: ParticleGroup): void {
        Assert(!this.m_world.IsLocked());

        // DEBUG: Assert(groupA !== groupB);
        this.RotateBuffer(groupB.m_firstIndex, groupB.m_lastIndex, this.m_count);
        // DEBUG: Assert(groupB.m_lastIndex === this.m_count);
        this.RotateBuffer(groupA.m_firstIndex, groupA.m_lastIndex, groupB.m_firstIndex);
        // DEBUG: Assert(groupA.m_lastIndex === groupB.m_firstIndex);

        // Create pairs and triads connecting groupA and groupB.
        const filter = new ParticleSystem_JoinParticleGroupsFilter(groupB.m_firstIndex);
        this.UpdateContacts(true);
        this.UpdatePairsAndTriads(groupA.m_firstIndex, groupB.m_lastIndex, filter);

        for (let i = groupB.m_firstIndex; i < groupB.m_lastIndex; i++) {
            this.m_groupBuffer[i] = groupA;
        }
        const groupFlags = groupA.m_groupFlags | groupB.m_groupFlags;
        this.SetGroupFlags(groupA, groupFlags);
        groupA.m_lastIndex = groupB.m_lastIndex;
        groupB.m_firstIndex = groupB.m_lastIndex;
        this.DestroyParticleGroup(groupB);
    }

    /**
     * Split particle group into multiple disconnected groups.
     *
     * warning: This function is locked during callbacks.
     *
     * @param group The group to be split.
     */
    public SplitParticleGroup(group: ParticleGroup): void {
        this.UpdateContacts(true);
        const particleCount = group.GetParticleCount();
        // We create several linked lists. Each list represents a set of connected particles.
        const nodeBuffer = new Array<ParticleSystem_ParticleListNode>(particleCount);
        for (let i = 0; i < particleCount; i++) nodeBuffer[i] = new ParticleSystem_ParticleListNode();
        ParticleSystem.InitializeParticleLists(group, nodeBuffer);
        this.MergeParticleListsInContact(group, nodeBuffer);
        const survivingList = ParticleSystem.FindLongestParticleList(group, nodeBuffer);
        this.MergeZombieParticleListNodes(group, nodeBuffer, survivingList);
        this.CreateParticleGroupsFromParticleList(group, nodeBuffer, survivingList);
        this.UpdatePairsAndTriadsWithParticleList(group, nodeBuffer);
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
    public GetParticleGroupList(): ParticleGroup | null {
        return this.m_groupList;
    }

    /**
     * Get the number of particle groups.
     */
    public GetParticleGroupCount(): number {
        return this.m_groupCount;
    }

    /**
     * Get the number of particles.
     */
    public GetParticleCount(): number {
        return this.m_count;
    }

    /**
     * Get the maximum number of particles.
     */
    public GetMaxParticleCount(): number {
        return this.m_def.maxCount;
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
    public SetMaxParticleCount(count: number): void {
        // DEBUG: Assert(this.m_count <= count);
        this.m_def.maxCount = count;
    }

    /**
     * Get all existing particle flags.
     */
    public GetAllParticleFlags(): ParticleFlag {
        return this.m_allParticleFlags;
    }

    /**
     * Get all existing particle group flags.
     */
    public GetAllGroupFlags(): ParticleGroupFlag {
        return this.m_allGroupFlags;
    }

    /**
     * Pause or unpause the particle system. When paused,
     * World::Step() skips over this particle system. All
     * ParticleSystem function calls still work.
     *
     * @param paused Paused is true to pause, false to un-pause.
     */
    public SetPaused(paused: boolean): void {
        this.m_paused = paused;
    }

    /**
     * Initially, true, then, the last value passed into
     * SetPaused().
     *
     * @returns true if the particle system is being updated in World::Step().
     */
    public GetPaused(): boolean {
        return this.m_paused;
    }

    /**
     * Change the particle density.
     *
     * Particle density affects the mass of the particles, which in
     * turn affects how the particles interact with Bodies. Note
     * that the density does not affect how the particles interact
     * with each other.
     */
    public SetDensity(density: number): void {
        this.m_def.density = density;
        this.m_inverseDensity = 1 / this.m_def.density;
    }

    /**
     * Get the particle density.
     */
    public GetDensity(): number {
        return this.m_def.density;
    }

    /**
     * Change the particle gravity scale. Adjusts the effect of the
     * global gravity vector on particles.
     */
    public SetGravityScale(gravityScale: number): void {
        this.m_def.gravityScale = gravityScale;
    }

    /**
     * Get the particle gravity scale.
     */
    public GetGravityScale(): number {
        return this.m_def.gravityScale;
    }

    /**
     * Damping is used to reduce the velocity of particles. The
     * damping parameter can be larger than 1   but the damping
     * effect becomes sensitive to the time step when the damping
     * parameter is large.
     */
    public SetDamping(damping: number): void {
        this.m_def.dampingStrength = damping;
    }

    /**
     * Get damping for particles
     */
    public GetDamping(): number {
        return this.m_def.dampingStrength;
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
    public SetStaticPressureIterations(iterations: number): void {
        this.m_def.staticPressureIterations = iterations;
    }

    /**
     * Get the number of iterations for static pressure of
     * particles.
     */
    public GetStaticPressureIterations(): number {
        return this.m_def.staticPressureIterations;
    }

    /**
     * Change the particle radius.
     *
     * You should set this only once, on world start.
     * If you change the radius during execution, existing particles
     * may explode, shrink, or behave unexpectedly.
     */
    public SetRadius(radius: number): void {
        this.m_particleDiameter = 2 * radius;
        this.m_squaredDiameter = this.m_particleDiameter * this.m_particleDiameter;
        this.m_inverseDiameter = 1 / this.m_particleDiameter;
    }

    /**
     * Get the particle radius.
     */
    public GetRadius(): number {
        return this.m_particleDiameter / 2;
    }

    /**
     * Get the position of each particle
     *
     * Array is length GetParticleCount()
     *
     * @returns The pointer to the head of the particle positions array.
     */
    public GetPositionBuffer(): Vec2[] {
        return this.m_positionBuffer.data;
    }

    /**
     * Get the velocity of each particle
     *
     * Array is length GetParticleCount()
     *
     * @returns The pointer to the head of the particle velocities array.
     */
    public GetVelocityBuffer(): Vec2[] {
        return this.m_velocityBuffer.data;
    }

    /**
     * Get the color of each particle
     *
     * Array is length GetParticleCount()
     *
     * @returns The pointer to the head of the particle colors array.
     */
    public GetColorBuffer(): Color[] {
        this.m_colorBuffer.data = this.RequestBuffer(this.m_colorBuffer.data);
        return this.m_colorBuffer.data;
    }

    /**
     * Get the particle-group of each particle.
     *
     * Array is length GetParticleCount()
     *
     * @returns The pointer to the head of the particle group array.
     */
    public GetGroupBuffer(): Array<ParticleGroup | null> {
        return this.m_groupBuffer;
    }

    /**
     * Get the weight of each particle
     *
     * Array is length GetParticleCount()
     *
     * @returns The pointer to the head of the particle positions array.
     */
    public GetWeightBuffer(): number[] {
        return this.m_weightBuffer;
    }

    /**
     * Get the user-specified data of each particle.
     *
     * Array is length GetParticleCount()
     *
     * @returns The pointer to the head of the particle user-data array.
     */
    public GetUserDataBuffer<T>(): T[] {
        this.m_userDataBuffer.data = this.RequestBuffer(this.m_userDataBuffer.data);
        return this.m_userDataBuffer.data;
    }

    /**
     * Get the flags for each particle. See the ParticleFlag enum.
     *
     * Array is length GetParticleCount()
     *
     * @returns The pointer to the head of the particle-flags array.
     */
    public GetFlagsBuffer(): ParticleFlag[] {
        return this.m_flagsBuffer.data;
    }

    /**
     * Set flags for a particle. See the ParticleFlag enum.
     */
    public SetParticleFlags(index: number, newFlags: ParticleFlag): void {
        const oldFlags = this.m_flagsBuffer.data[index];
        if (oldFlags & ~newFlags) {
            // If any flags might be removed
            this.m_needsUpdateAllParticleFlags = true;
        }
        if (~this.m_allParticleFlags & newFlags) {
            // If any flags were added
            if (newFlags & ParticleFlag.Tensile) {
                this.m_accumulation2Buffer = this.RequestBuffer(this.m_accumulation2Buffer);
            }
            if (newFlags & ParticleFlag.ColorMixing) {
                this.m_colorBuffer.data = this.RequestBuffer(this.m_colorBuffer.data);
            }
            this.m_allParticleFlags |= newFlags;
        }
        this.m_flagsBuffer.data[index] = newFlags;
    }

    /**
     * Get flags for a particle. See the ParticleFlag enum.
     */
    public GetParticleFlags(index: number): ParticleFlag {
        return this.m_flagsBuffer.data[index];
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
    public SetFlagsBuffer(buffer: ParticleFlag[]): void {
        this.SetUserOverridableBuffer(this.m_flagsBuffer, buffer);
    }

    public SetPositionBuffer(buffer: Vec2[]): void {
        this.SetUserOverridableBuffer(this.m_positionBuffer, buffer);
    }

    public SetVelocityBuffer(buffer: Vec2[]): void {
        this.SetUserOverridableBuffer(this.m_velocityBuffer, buffer);
    }

    public SetColorBuffer(buffer: Color[]): void {
        this.SetUserOverridableBuffer(this.m_colorBuffer, buffer);
    }

    public SetUserDataBuffer<T>(buffer: T[]): void {
        this.SetUserOverridableBuffer(this.m_userDataBuffer, buffer);
    }

    /**
     * Get contacts between particles
     * Contact data can be used for many reasons, for example to
     * trigger rendering or audio effects.
     */
    public GetContacts(): ParticleContact[] {
        return this.m_contactBuffer.data;
    }

    public GetContactCount(): number {
        return this.m_contactBuffer.count;
    }

    /**
     * Get contacts between particles and bodies
     *
     * Contact data can be used for many reasons, for example to
     * trigger rendering or audio effects.
     */
    public GetBodyContacts(): ParticleBodyContact[] {
        return this.m_bodyContactBuffer.data;
    }

    public GetBodyContactCount(): number {
        return this.m_bodyContactBuffer.count;
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
    public GetPairs(): ParticlePair[] {
        return this.m_pairBuffer.data;
    }

    public GetPairCount(): number {
        return this.m_pairBuffer.count;
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
    public GetTriads(): ParticleTriad[] {
        return this.m_triadBuffer.data;
    }

    public GetTriadCount(): number {
        return this.m_triadBuffer.count;
    }

    /**
     * Set an optional threshold for the maximum number of
     * consecutive particle iterations that a particle may contact
     * multiple bodies before it is considered a candidate for being
     * "stuck". Setting to zero or less disables.
     */
    public SetStuckThreshold(steps: number): void {
        this.m_stuckThreshold = steps;

        if (steps > 0) {
            this.m_lastBodyContactStepBuffer.data = this.RequestBuffer(this.m_lastBodyContactStepBuffer.data);
            this.m_bodyContactCountBuffer.data = this.RequestBuffer(this.m_bodyContactCountBuffer.data);
            this.m_consecutiveContactStepsBuffer.data = this.RequestBuffer(this.m_consecutiveContactStepsBuffer.data);
        }
    }

    /**
     * Get potentially stuck particles from the last step; the user
     * must decide if they are stuck or not, and if so, delete or
     * move them
     */
    public GetStuckCandidates(): number[] {
        return this.m_stuckParticleBuffer.Data();
    }

    /**
     * Get the number of stuck particle candidates from the last
     * step.
     */
    public GetStuckCandidateCount(): number {
        return this.m_stuckParticleBuffer.GetCount();
    }

    /**
     * Compute the kinetic energy that can be lost by damping force
     */
    public ComputeCollisionEnergy(): number {
        const s_v = ParticleSystem.ComputeCollisionEnergy_s_v;
        const vel_data = this.m_velocityBuffer.data;
        let sum_v2 = 0;
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            const n = contact.normal;
            const v = Vec2.Subtract(vel_data[b], vel_data[a], s_v);
            const vn = Vec2.Dot(v, n);
            if (vn < 0) {
                sum_v2 += vn * vn;
            }
        }
        return 0.5 * this.GetParticleMass() * sum_v2;
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
    public SetStrictContactCheck(enabled: boolean): void {
        this.m_def.strictContactCheck = enabled;
    }

    /**
     * Get the status of the strict contact check.
     */
    public GetStrictContactCheck(): boolean {
        return this.m_def.strictContactCheck;
    }

    /**
     * Set the lifetime (in seconds) of a particle relative to the
     * current time.  A lifetime of less than or equal to 0
     * results in the particle living forever until it's manually
     * destroyed by the application.
     */
    public SetParticleLifetime(index: number, lifetime: number): void {
        // DEBUG: Assert(this.ValidateParticleIndex(index));
        const initializeExpirationTimes = this.m_indexByExpirationTimeBuffer.data === null;
        this.m_expirationTimeBuffer.data = this.RequestBuffer(this.m_expirationTimeBuffer.data);
        this.m_indexByExpirationTimeBuffer.data = this.RequestBuffer(this.m_indexByExpirationTimeBuffer.data);

        // Initialize the inverse mapping buffer.
        if (initializeExpirationTimes) {
            const particleCount = this.GetParticleCount();
            for (let i = 0; i < particleCount; ++i) {
                this.m_indexByExpirationTimeBuffer.data[i] = i;
            }
        }

        const quantizedLifetime = lifetime / this.m_def.lifetimeGranularity;
        // Use a negative lifetime so that it's possible to track which
        // of the infinite lifetime particles are older.
        const newExpirationTime =
            quantizedLifetime > 0 ? this.GetQuantizedTimeElapsed() + quantizedLifetime : quantizedLifetime;
        if (newExpirationTime !== this.m_expirationTimeBuffer.data[index]) {
            this.m_expirationTimeBuffer.data[index] = newExpirationTime;
            this.m_expirationTimeBufferRequiresSorting = true;
        }
    }

    /**
     * Get the lifetime (in seconds) of a particle relative to the
     * current time.  A value > 0   is returned if the particle is
     * scheduled to be destroyed in the future, values <= 0
     * indicate the particle has an infinite lifetime.
     */
    public GetParticleLifetime(index: number): number {
        // DEBUG: Assert(this.ValidateParticleIndex(index));
        return this.ExpirationTimeToLifetime(this.GetExpirationTimeBuffer()[index]);
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
    public SetDestructionByAge(enable: boolean): void {
        if (enable) {
            this.GetExpirationTimeBuffer();
        }
        this.m_def.destroyByAge = enable;
    }

    /**
     * Get whether the oldest particle will be destroyed in
     * CreateParticle() when the maximum number of particles are
     * present in the system.
     */
    public GetDestructionByAge(): boolean {
        return this.m_def.destroyByAge;
    }

    /**
     * Get the array of particle expiration times indexed by
     * particle index.
     *
     * GetParticleCount() items are in the returned array.
     */
    public GetExpirationTimeBuffer(): number[] {
        this.m_expirationTimeBuffer.data = this.RequestBuffer(this.m_expirationTimeBuffer.data);
        return this.m_expirationTimeBuffer.data;
    }

    /**
     * Convert a expiration time value in returned by
     * GetExpirationTimeBuffer() to a time in seconds relative to
     * the current simulation time.
     */
    public ExpirationTimeToLifetime(expirationTime: number): number {
        return (
            (expirationTime > 0 ? expirationTime - this.GetQuantizedTimeElapsed() : expirationTime) *
            this.m_def.lifetimeGranularity
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
    public GetIndexByExpirationTimeBuffer(): number[] {
        // If particles are present, initialize / reinitialize the lifetime buffer.
        if (this.GetParticleCount()) {
            this.SetParticleLifetime(0, this.GetParticleLifetime(0));
        } else {
            this.m_indexByExpirationTimeBuffer.data = this.RequestBuffer(this.m_indexByExpirationTimeBuffer.data);
        }
        return this.m_indexByExpirationTimeBuffer.data;
    }

    /**
     * Apply an impulse to one particle. This immediately modifies
     * the velocity. Similar to Body::ApplyLinearImpulse.
     *
     * @param index The particle that will be modified.
     * @param impulse Impulse the world impulse vector, usually in N-seconds or kg-m/s.
     */
    public ParticleApplyLinearImpulse(index: number, impulse: XY): void {
        this.ApplyLinearImpulse(index, index + 1, impulse);
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
    public ApplyLinearImpulse(firstIndex: number, lastIndex: number, impulse: XY): void {
        const vel_data = this.m_velocityBuffer.data;
        const numParticles = lastIndex - firstIndex;
        const totalMass = numParticles * this.GetParticleMass();
        const velocityDelta = Vec2.Scale(1 / totalMass, impulse, new Vec2());
        for (let i = firstIndex; i < lastIndex; i++) {
            vel_data[i].Add(velocityDelta);
        }
    }

    public static IsSignificantForce(force: XY): boolean {
        return force.x !== 0 || force.y !== 0;
    }

    /**
     * Apply a force to the center of a particle.
     *
     * @param index The particle that will be modified.
     * @param force The world force vector, usually in Newtons (N).
     */
    public ParticleApplyForce(index: number, force: XY): void {
        if (ParticleSystem.IsSignificantForce(force) && this.ForceCanBeApplied(this.m_flagsBuffer.data[index])) {
            this.PrepareForceBuffer();
            this.m_forceBuffer[index].Add(force);
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
    public ApplyForce(firstIndex: number, lastIndex: number, force: XY): void {
        // Ensure we're not trying to apply force to particles that can't move,
        // such as wall particles.
        // DEBUG: let flags = 0;
        // DEBUG: for (let i = firstIndex; i < lastIndex; i++) {
        // DEBUG:   flags |= this.m_flagsBuffer.data[i];
        // DEBUG: }
        // DEBUG: Assert(this.ForceCanBeApplied(flags));

        // Early out if force does nothing (optimization).
        const distributedForce = Vec2.Scale(1 / (lastIndex - firstIndex), force, new Vec2());
        if (ParticleSystem.IsSignificantForce(distributedForce)) {
            this.PrepareForceBuffer();

            // Distribute the force over all the particles.
            for (let i = firstIndex; i < lastIndex; i++) {
                this.m_forceBuffer[i].Add(distributedForce);
            }
        }
    }

    /**
     * Get the next particle-system in the world's particle-system
     * list.
     */
    public GetNext(): ParticleSystem | null {
        return this.m_next;
    }

    /**
     * Query the particle system for all particles that potentially
     * overlap the provided AABB.
     *
     * @param callback A user implemented callback class.
     * @param aabb The query box.
     */
    public QueryAABB(aabb: AABB, callback: ParticleQueryCallback): void {
        if (this.m_proxyBuffer.count === 0) {
            return;
        }
        const beginProxy = 0;
        const endProxy = this.m_proxyBuffer.count;
        const firstProxy = std_lower_bound(
            this.m_proxyBuffer.data,
            beginProxy,
            endProxy,
            ParticleSystem.computeTag(
                this.m_inverseDiameter * aabb.lowerBound.x,
                this.m_inverseDiameter * aabb.lowerBound.y,
            ),
            ParticleSystem_Proxy.CompareProxyTag,
        );
        const lastProxy = std_upper_bound(
            this.m_proxyBuffer.data,
            firstProxy,
            endProxy,
            ParticleSystem.computeTag(
                this.m_inverseDiameter * aabb.upperBound.x,
                this.m_inverseDiameter * aabb.upperBound.y,
            ),
            ParticleSystem_Proxy.CompareTagProxy,
        );
        const pos_data = this.m_positionBuffer.data;
        for (let k = firstProxy; k < lastProxy; ++k) {
            const proxy = this.m_proxyBuffer.data[k];
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
    public QueryShapeAABB(shape: Shape, xf: Transform, childIndex: number, callback: ParticleQueryCallback): void {
        const s_aabb = ParticleSystem.QueryShapeAABB_s_aabb;
        const aabb = s_aabb;
        shape.ComputeAABB(aabb, xf, childIndex);
        this.QueryAABB(aabb, callback);
    }

    public static readonly QueryShapeAABB_s_aabb = new AABB();

    public QueryPointAABB(point: XY, slop: number, callback: ParticleQueryCallback): void {
        const s_aabb = ParticleSystem.QueryPointAABB_s_aabb;
        const aabb = s_aabb;
        aabb.lowerBound.Set(point.x - slop, point.y - slop);
        aabb.upperBound.Set(point.x + slop, point.y + slop);
        this.QueryAABB(aabb, callback);
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
    public RayCast(point1: XY, point2: XY, callback: ParticleRayCastCallback): void {
        const s_aabb = ParticleSystem.RayCast_s_aabb;
        const s_p = ParticleSystem.RayCast_s_p;
        const s_v = ParticleSystem.RayCast_s_v;
        const s_n = ParticleSystem.RayCast_s_n;
        const s_point = ParticleSystem.RayCast_s_point;
        if (this.m_proxyBuffer.count === 0) {
            return;
        }
        const pos_data = this.m_positionBuffer.data;
        const aabb = s_aabb;
        Vec2.Min(point1, point2, aabb.lowerBound);
        Vec2.Max(point1, point2, aabb.upperBound);
        let fraction = 1;
        // solving the following equation:
        // ((1-t)*point1+t*point2-position)^2=diameter^2
        // where t is a potential fraction
        const v = Vec2.Subtract(point2, point1, s_v);
        const v2 = Vec2.Dot(v, v);
        const enumerator = this.GetInsideBoundsEnumerator(aabb);

        let i: number;
        // eslint-disable-next-line no-cond-assign
        while ((i = enumerator.GetNext()) >= 0) {
            const p = Vec2.Subtract(point1, pos_data[i], s_p);
            const pv = Vec2.Dot(p, v);
            const p2 = Vec2.Dot(p, p);
            const determinant = pv * pv - v2 * (p2 - this.m_squaredDiameter);
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

                const n = Vec2.AddScaled(p, t, v, s_n);
                n.Normalize();
                const f = callback(i, Vec2.AddScaled(point1, t, v, s_point), n, t);
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
    public ComputeAABB(aabb: AABB): void {
        const particleCount = this.GetParticleCount();
        // DEBUG: Assert(aabb !== null);
        aabb.lowerBound.x = +MAX_FLOAT;
        aabb.lowerBound.y = +MAX_FLOAT;
        aabb.upperBound.x = -MAX_FLOAT;
        aabb.upperBound.y = -MAX_FLOAT;

        const pos_data = this.m_positionBuffer.data;
        for (let i = 0; i < particleCount; i++) {
            const p = pos_data[i];
            Vec2.Min(aabb.lowerBound, p, aabb.lowerBound);
            Vec2.Max(aabb.upperBound, p, aabb.upperBound);
        }
        aabb.lowerBound.x -= this.m_particleDiameter;
        aabb.lowerBound.y -= this.m_particleDiameter;
        aabb.upperBound.x += this.m_particleDiameter;
        aabb.upperBound.y += this.m_particleDiameter;
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

    public FreeBuffer<T>(b: T[] | null): void {
        if (b === null) {
            return;
        }
        b.length = 0;
    }

    public FreeUserOverridableBuffer<T>(b: ParticleSystem_UserOverridableBuffer<T>): void {
        if (b.userSuppliedCapacity === 0) {
            this.FreeBuffer(b.data);
        }
    }

    /**
     * Reallocate a buffer
     */
    public ReallocateBuffer3<T>(oldBuffer: T[] | null, oldCapacity: number, newCapacity: number): T[] {
        Assert(newCapacity > oldCapacity);
        const newBuffer = oldBuffer ? oldBuffer.slice() : [];
        newBuffer.length = newCapacity;
        return newBuffer;
    }

    /**
     * Reallocate a buffer
     */
    public ReallocateBuffer5<T>(
        buffer: T[] | null,
        userSuppliedCapacity: number,
        oldCapacity: number,
        newCapacity: number,
        deferred: boolean,
    ): T[] {
        Assert(newCapacity > oldCapacity);
        // A 'deferred' buffer is reallocated only if it is not NULL.
        // If 'userSuppliedCapacity' is not zero, buffer is user supplied and must
        // be kept.
        Assert(!userSuppliedCapacity || newCapacity <= userSuppliedCapacity);
        if ((!deferred || buffer) && !userSuppliedCapacity) {
            buffer = this.ReallocateBuffer3(buffer, oldCapacity, newCapacity);
        }
        return buffer as T[]; // TODO: fix this
    }

    /**
     * Reallocate a buffer
     */
    public ReallocateBuffer4<T>(
        buffer: ParticleSystem_UserOverridableBuffer<any>,
        oldCapacity: number,
        newCapacity: number,
        deferred: boolean,
    ): T[] {
        // DEBUG: Assert(newCapacity > oldCapacity);
        return this.ReallocateBuffer5(buffer.data, buffer.userSuppliedCapacity, oldCapacity, newCapacity, deferred);
    }

    public RequestBuffer<T>(buffer: T[] | null): T[] {
        if (!buffer) {
            if (this.m_internalAllocatedCapacity === 0) {
                this.ReallocateInternalAllocatedBuffers(b2_minParticleSystemBufferCapacity);
            }

            buffer = [];
            buffer.length = this.m_internalAllocatedCapacity;
        }
        return buffer;
    }

    /**
     * Reallocate the handle / index map and schedule the allocation
     * of a new pool for handle allocation.
     */
    public ReallocateHandleBuffers(newCapacity: number): void {
        // DEBUG: Assert(newCapacity > this.m_internalAllocatedCapacity);
        // Reallocate a new handle / index map buffer, copying old handle pointers
        // is fine since they're kept around.
        this.m_handleIndexBuffer.data = this.ReallocateBuffer4(
            this.m_handleIndexBuffer,
            this.m_internalAllocatedCapacity,
            newCapacity,
            true,
        );
        // Set the size of the next handle allocation.
        // this.m_handleAllocator.SetItemsPerSlab(newCapacity - this.m_internalAllocatedCapacity);
    }

    public ReallocateInternalAllocatedBuffers(capacity: number): void {
        function LimitCapacity(maxCount: number): number {
            return maxCount && capacity > maxCount ? maxCount : capacity;
        }

        // Don't increase capacity beyond the smallest user-supplied buffer size.
        capacity = LimitCapacity(this.m_def.maxCount);
        capacity = LimitCapacity(this.m_flagsBuffer.userSuppliedCapacity);
        capacity = LimitCapacity(this.m_positionBuffer.userSuppliedCapacity);
        capacity = LimitCapacity(this.m_velocityBuffer.userSuppliedCapacity);
        capacity = LimitCapacity(this.m_colorBuffer.userSuppliedCapacity);
        capacity = LimitCapacity(this.m_userDataBuffer.userSuppliedCapacity);
        if (this.m_internalAllocatedCapacity < capacity) {
            this.ReallocateHandleBuffers(capacity);
            this.m_flagsBuffer.data = this.ReallocateBuffer4(
                this.m_flagsBuffer,
                this.m_internalAllocatedCapacity,
                capacity,
                false,
            );

            // Conditionally defer these as they are optional if the feature is
            // not enabled.
            const stuck = this.m_stuckThreshold > 0;
            this.m_lastBodyContactStepBuffer.data = this.ReallocateBuffer4(
                this.m_lastBodyContactStepBuffer,
                this.m_internalAllocatedCapacity,
                capacity,
                stuck,
            );
            this.m_bodyContactCountBuffer.data = this.ReallocateBuffer4(
                this.m_bodyContactCountBuffer,
                this.m_internalAllocatedCapacity,
                capacity,
                stuck,
            );
            this.m_consecutiveContactStepsBuffer.data = this.ReallocateBuffer4(
                this.m_consecutiveContactStepsBuffer,
                this.m_internalAllocatedCapacity,
                capacity,
                stuck,
            );
            this.m_positionBuffer.data = this.ReallocateBuffer4(
                this.m_positionBuffer,
                this.m_internalAllocatedCapacity,
                capacity,
                false,
            );
            this.m_velocityBuffer.data = this.ReallocateBuffer4(
                this.m_velocityBuffer,
                this.m_internalAllocatedCapacity,
                capacity,
                false,
            );
            this.m_forceBuffer = this.ReallocateBuffer5(
                this.m_forceBuffer,
                0,
                this.m_internalAllocatedCapacity,
                capacity,
                false,
            );
            this.m_weightBuffer = this.ReallocateBuffer5(
                this.m_weightBuffer,
                0,
                this.m_internalAllocatedCapacity,
                capacity,
                false,
            );
            this.m_staticPressureBuffer = this.ReallocateBuffer5(
                this.m_staticPressureBuffer,
                0,
                this.m_internalAllocatedCapacity,
                capacity,
                true,
            );
            this.m_accumulationBuffer = this.ReallocateBuffer5(
                this.m_accumulationBuffer,
                0,
                this.m_internalAllocatedCapacity,
                capacity,
                false,
            );
            this.m_accumulation2Buffer = this.ReallocateBuffer5(
                this.m_accumulation2Buffer,
                0,
                this.m_internalAllocatedCapacity,
                capacity,
                true,
            );
            this.m_depthBuffer = this.ReallocateBuffer5(
                this.m_depthBuffer,
                0,
                this.m_internalAllocatedCapacity,
                capacity,
                true,
            );
            this.m_colorBuffer.data = this.ReallocateBuffer4(
                this.m_colorBuffer,
                this.m_internalAllocatedCapacity,
                capacity,
                true,
            );
            this.m_groupBuffer = this.ReallocateBuffer5(
                this.m_groupBuffer,
                0,
                this.m_internalAllocatedCapacity,
                capacity,
                false,
            );
            this.m_userDataBuffer.data = this.ReallocateBuffer4(
                this.m_userDataBuffer,
                this.m_internalAllocatedCapacity,
                capacity,
                true,
            );
            this.m_expirationTimeBuffer.data = this.ReallocateBuffer4(
                this.m_expirationTimeBuffer,
                this.m_internalAllocatedCapacity,
                capacity,
                true,
            );
            this.m_indexByExpirationTimeBuffer.data = this.ReallocateBuffer4(
                this.m_indexByExpirationTimeBuffer,
                this.m_internalAllocatedCapacity,
                capacity,
                false,
            );
            this.m_internalAllocatedCapacity = capacity;
        }
    }

    public CreateParticleForGroup(groupDef: IParticleGroupDef, xf: Transform, p: XY): void {
        const particleDef = new ParticleDef();
        particleDef.flags = groupDef.flags ?? 0;
        Transform.MultiplyVec2(xf, p, particleDef.position);
        Vec2.Add(
            groupDef.linearVelocity ?? Vec2.ZERO,
            Vec2.CrossScalarVec2(
                groupDef.angularVelocity ?? 0,
                Vec2.Subtract(particleDef.position, groupDef.position ?? Vec2.ZERO, Vec2.s_t0),
                Vec2.s_t0,
            ),
            particleDef.velocity,
        );
        particleDef.color.Copy(groupDef.color ?? Color.ZERO);
        particleDef.lifetime = groupDef.lifetime ?? 0;
        particleDef.userData = groupDef.userData;
        this.CreateParticle(particleDef);
    }

    public CreateParticlesStrokeShapeForGroup(shape: Shape, groupDef: IParticleGroupDef, xf: Transform): void {
        const s_edge = ParticleSystem.CreateParticlesStrokeShapeForGroup_s_edge;
        const s_d = ParticleSystem.CreateParticlesStrokeShapeForGroup_s_d;
        const s_p = ParticleSystem.CreateParticlesStrokeShapeForGroup_s_p;
        let stride = groupDef.stride ?? 0;
        if (stride === 0) {
            stride = this.GetParticleStride();
        }
        let positionOnEdge = 0;
        const childCount = shape.GetChildCount();
        for (let childIndex = 0; childIndex < childCount; childIndex++) {
            let edge: EdgeShape | null = null;
            if (shape.GetType() === ShapeType.Edge) {
                edge = shape as EdgeShape;
            } else {
                // DEBUG: Assert(shape.GetType() === ShapeType.Chain);
                edge = s_edge;
                (shape as ChainShape).GetChildEdge(edge, childIndex);
            }
            const d = Vec2.Subtract(edge.m_vertex2, edge.m_vertex1, s_d);
            const edgeLength = d.Length();

            while (positionOnEdge < edgeLength) {
                const p = Vec2.AddScaled(edge.m_vertex1, positionOnEdge / edgeLength, d, s_p);
                this.CreateParticleForGroup(groupDef, xf, p);
                positionOnEdge += stride;
            }
            positionOnEdge -= edgeLength;
        }
    }

    public static readonly CreateParticlesStrokeShapeForGroup_s_edge = new EdgeShape();

    public static readonly CreateParticlesStrokeShapeForGroup_s_d = new Vec2();

    public static readonly CreateParticlesStrokeShapeForGroup_s_p = new Vec2();

    public CreateParticlesFillShapeForGroup(shape: Shape, groupDef: IParticleGroupDef, xf: Transform): void {
        const s_aabb = ParticleSystem.CreateParticlesFillShapeForGroup_s_aabb;
        const s_p = ParticleSystem.CreateParticlesFillShapeForGroup_s_p;
        let stride = groupDef.stride ?? 0;
        if (stride === 0) {
            stride = this.GetParticleStride();
        }
        const identity = Transform.IDENTITY;
        const aabb = s_aabb;
        // DEBUG: Assert(shape.GetChildCount() === 1);
        shape.ComputeAABB(aabb, identity, 0);
        for (let y = Math.floor(aabb.lowerBound.y / stride) * stride; y < aabb.upperBound.y; y += stride) {
            for (let x = Math.floor(aabb.lowerBound.x / stride) * stride; x < aabb.upperBound.x; x += stride) {
                const p = s_p.Set(x, y);
                if (shape.TestPoint(identity, p)) {
                    this.CreateParticleForGroup(groupDef, xf, p);
                }
            }
        }
    }

    public static readonly CreateParticlesFillShapeForGroup_s_aabb = new AABB();

    public static readonly CreateParticlesFillShapeForGroup_s_p = new Vec2();

    public CreateParticlesWithShapeForGroup(shape: Shape, groupDef: IParticleGroupDef, xf: Transform): void {
        switch (shape.GetType()) {
            case ShapeType.Edge:
            case ShapeType.Chain:
                this.CreateParticlesStrokeShapeForGroup(shape, groupDef, xf);
                break;
            case ShapeType.Polygon:
            case ShapeType.Circle:
                this.CreateParticlesFillShapeForGroup(shape, groupDef, xf);
                break;
            default:
                // DEBUG: Assert(false);
                break;
        }
    }

    public CreateParticlesWithShapesForGroup(
        shapes: Shape[],
        shapeCount: number,
        groupDef: IParticleGroupDef,
        xf: Transform,
    ): void {
        const compositeShape = new ParticleSystem_CompositeShape(shapes, shapeCount);
        this.CreateParticlesFillShapeForGroup(compositeShape, groupDef, xf);
    }

    public CloneParticle(oldIndex: number, group: ParticleGroup): number {
        const def = new ParticleDef();
        def.flags = this.m_flagsBuffer.data[oldIndex];
        def.position.Copy(this.m_positionBuffer.data[oldIndex]);
        def.velocity.Copy(this.m_velocityBuffer.data[oldIndex]);
        if (this.m_colorBuffer.data) {
            def.color.Copy(this.m_colorBuffer.data[oldIndex]);
        }
        if (this.m_userDataBuffer.data) {
            def.userData = this.m_userDataBuffer.data[oldIndex];
        }
        def.group = group;
        const newIndex = this.CreateParticle(def);
        if (this.m_handleIndexBuffer.data) {
            const handle = this.m_handleIndexBuffer.data[oldIndex];
            if (handle) {
                handle.SetIndex(newIndex);
            }
            this.m_handleIndexBuffer.data[newIndex] = handle;
            this.m_handleIndexBuffer.data[oldIndex] = null;
        }
        if (this.m_lastBodyContactStepBuffer.data) {
            this.m_lastBodyContactStepBuffer.data[newIndex] = this.m_lastBodyContactStepBuffer.data[oldIndex];
        }
        if (this.m_bodyContactCountBuffer.data) {
            this.m_bodyContactCountBuffer.data[newIndex] = this.m_bodyContactCountBuffer.data[oldIndex];
        }
        if (this.m_consecutiveContactStepsBuffer.data) {
            this.m_consecutiveContactStepsBuffer.data[newIndex] = this.m_consecutiveContactStepsBuffer.data[oldIndex];
        }
        if (this.m_hasForce) {
            this.m_forceBuffer[newIndex].Copy(this.m_forceBuffer[oldIndex]);
        }
        if (this.m_staticPressureBuffer) {
            this.m_staticPressureBuffer[newIndex] = this.m_staticPressureBuffer[oldIndex];
        }
        if (this.m_depthBuffer) {
            this.m_depthBuffer[newIndex] = this.m_depthBuffer[oldIndex];
        }
        if (this.m_expirationTimeBuffer.data) {
            this.m_expirationTimeBuffer.data[newIndex] = this.m_expirationTimeBuffer.data[oldIndex];
        }
        return newIndex;
    }

    public DestroyParticlesInGroup(group: ParticleGroup, callDestructionListener = false): void {
        for (let i = group.m_firstIndex; i < group.m_lastIndex; i++) {
            this.DestroyParticle(i, callDestructionListener);
        }
    }

    public DestroyParticleGroup(group: ParticleGroup): void {
        // DEBUG: Assert(this.m_groupCount > 0);
        // DEBUG: Assert(group !== null);

        const destructionListener = this.m_world.GetDestructionListener();
        destructionListener?.SayGoodbyeParticleGroup(group);

        this.SetGroupFlags(group, 0);
        for (let i = group.m_firstIndex; i < group.m_lastIndex; i++) {
            this.m_groupBuffer[i] = null;
        }

        if (group.m_prev) {
            group.m_prev.m_next = group.m_next;
        }
        if (group.m_next) {
            group.m_next.m_prev = group.m_prev;
        }
        if (group === this.m_groupList) {
            this.m_groupList = group.m_next;
        }

        --this.m_groupCount;
    }

    public static ParticleCanBeConnected(flags: ParticleFlag, group: ParticleGroup | null): boolean {
        return (
            (flags & (ParticleFlag.Wall | ParticleFlag.Spring | ParticleFlag.Elastic)) !== 0 ||
            (group !== null && (group.GetGroupFlags() & ParticleGroupFlag.Rigid) !== 0)
        );
    }

    public UpdatePairsAndTriads(firstIndex: number, lastIndex: number, filter: ParticleSystem_ConnectionFilter): void {
        const s_dab = ParticleSystem.UpdatePairsAndTriads_s_dab;
        const s_dbc = ParticleSystem.UpdatePairsAndTriads_s_dbc;
        const s_dca = ParticleSystem.UpdatePairsAndTriads_s_dca;
        const pos_data = this.m_positionBuffer.data;
        // Create pairs or triads.
        // All particles in each pair/triad should satisfy the following:
        // * firstIndex <= index < lastIndex
        // * don't have Zombie
        // * ParticleCanBeConnected returns true
        // * ShouldCreatePair/ShouldCreateTriad returns true
        // Any particles in each pair/triad should satisfy the following:
        // * filter.IsNeeded returns true
        // * have one of k_pairFlags/k_triadsFlags
        // DEBUG: Assert(firstIndex <= lastIndex);
        let particleFlags = 0;
        for (let i = firstIndex; i < lastIndex; i++) {
            particleFlags |= this.m_flagsBuffer.data[i];
        }
        if (particleFlags & ParticleSystem.k_pairFlags) {
            for (let k = 0; k < this.m_contactBuffer.count; k++) {
                const contact = this.m_contactBuffer.data[k];
                const a = contact.indexA;
                const b = contact.indexB;
                const af = this.m_flagsBuffer.data[a];
                const bf = this.m_flagsBuffer.data[b];
                const groupA = this.m_groupBuffer[a];
                const groupB = this.m_groupBuffer[b];
                if (
                    a >= firstIndex &&
                    a < lastIndex &&
                    b >= firstIndex &&
                    b < lastIndex &&
                    !((af | bf) & ParticleFlag.Zombie) &&
                    (af | bf) & ParticleSystem.k_pairFlags &&
                    (filter.IsNecessary(a) || filter.IsNecessary(b)) &&
                    ParticleSystem.ParticleCanBeConnected(af, groupA) &&
                    ParticleSystem.ParticleCanBeConnected(bf, groupB) &&
                    filter.ShouldCreatePair(a, b)
                ) {
                    const pair = this.m_pairBuffer.data[this.m_pairBuffer.Append()];
                    pair.indexA = a;
                    pair.indexB = b;
                    pair.flags = contact.flags;
                    pair.strength = Math.min(groupA ? groupA.m_strength : 1, groupB ? groupB.m_strength : 1);
                    pair.distance = Vec2.Distance(pos_data[a], pos_data[b]);
                }
                std_stable_sort(this.m_pairBuffer.data, 0, this.m_pairBuffer.count, ParticleSystem.ComparePairIndices);
                this.m_pairBuffer.Unique(ParticleSystem.MatchPairIndices);
            }
        }
        if (particleFlags & ParticleSystem.k_triadFlags) {
            const diagram = new VoronoiDiagram(lastIndex - firstIndex);
            // let necessary_count = 0;
            for (let i = firstIndex; i < lastIndex; i++) {
                const flags = this.m_flagsBuffer.data[i];
                const group = this.m_groupBuffer[i];
                if (!(flags & ParticleFlag.Zombie) && ParticleSystem.ParticleCanBeConnected(flags, group)) {
                    // if (filter.IsNecessary(i)) {
                    //     ++necessary_count;
                    // }
                    diagram.AddGenerator(pos_data[i], i, filter.IsNecessary(i));
                }
            }
            // if (necessary_count === 0) {
            //     debugger;
            //     for (let i = firstIndex; i < lastIndex; i++) {
            //         filter.IsNecessary(i);
            //     }
            // }
            const stride = this.GetParticleStride();
            diagram.Generate(stride / 2, stride * 2);
            diagram.GetNodes((a, b, c) => {
                const af = this.m_flagsBuffer.data[a];
                const bf = this.m_flagsBuffer.data[b];
                const cf = this.m_flagsBuffer.data[c];
                if ((af | bf | cf) & ParticleSystem.k_triadFlags && filter.ShouldCreateTriad(a, b, c)) {
                    const pa = pos_data[a];
                    const pb = pos_data[b];
                    const pc = pos_data[c];
                    const dab = Vec2.Subtract(pa, pb, s_dab);
                    const dbc = Vec2.Subtract(pb, pc, s_dbc);
                    const dca = Vec2.Subtract(pc, pa, s_dca);
                    const maxDistanceSquared = b2_maxTriadDistanceSquared * this.m_squaredDiameter;
                    if (
                        Vec2.Dot(dab, dab) > maxDistanceSquared ||
                        Vec2.Dot(dbc, dbc) > maxDistanceSquared ||
                        Vec2.Dot(dca, dca) > maxDistanceSquared
                    ) {
                        return;
                    }
                    const groupA = this.m_groupBuffer[a];
                    const groupB = this.m_groupBuffer[b];
                    const groupC = this.m_groupBuffer[c];
                    const triad = this.m_triadBuffer.data[this.m_triadBuffer.Append()];
                    triad.indexA = a;
                    triad.indexB = b;
                    triad.indexC = c;
                    triad.flags = af | bf | cf;
                    triad.strength = Math.min(
                        Math.min(groupA ? groupA.m_strength : 1, groupB ? groupB.m_strength : 1),
                        groupC ? groupC.m_strength : 1,
                    );
                    const midPoint_x = (pa.x + pb.x + pc.x) / 3;
                    const midPoint_y = (pa.y + pb.y + pc.y) / 3;
                    triad.pa.x = pa.x - midPoint_x;
                    triad.pa.y = pa.y - midPoint_y;
                    triad.pb.x = pb.x - midPoint_x;
                    triad.pb.y = pb.y - midPoint_y;
                    triad.pc.x = pc.x - midPoint_x;
                    triad.pc.y = pc.y - midPoint_y;
                    triad.ka = -Vec2.Dot(dca, dab);
                    triad.kb = -Vec2.Dot(dab, dbc);
                    triad.kc = -Vec2.Dot(dbc, dca);
                    triad.s = Vec2.Cross(pa, pb) + Vec2.Cross(pb, pc) + Vec2.Cross(pc, pa);
                }
            });
            std_stable_sort(this.m_triadBuffer.data, 0, this.m_triadBuffer.count, ParticleSystem.CompareTriadIndices);
            this.m_triadBuffer.Unique(ParticleSystem.MatchTriadIndices);
        }
    }

    private static UpdatePairsAndTriads_s_dab = new Vec2();

    private static UpdatePairsAndTriads_s_dbc = new Vec2();

    private static UpdatePairsAndTriads_s_dca = new Vec2();

    public UpdatePairsAndTriadsWithReactiveParticles(): void {
        const filter = new ParticleSystem_ReactiveFilter(this.m_flagsBuffer);
        this.UpdatePairsAndTriads(0, this.m_count, filter);

        for (let i = 0; i < this.m_count; i++) {
            this.m_flagsBuffer.data[i] &= ~ParticleFlag.Reactive;
        }
        this.m_allParticleFlags &= ~ParticleFlag.Reactive;
    }

    public static ComparePairIndices(a: ParticlePair, b: ParticlePair): boolean {
        const diffA = a.indexA - b.indexA;
        if (diffA !== 0) {
            return diffA < 0;
        }
        return a.indexB < b.indexB;
    }

    public static MatchPairIndices(a: ParticlePair, b: ParticlePair): boolean {
        return a.indexA === b.indexA && a.indexB === b.indexB;
    }

    public static CompareTriadIndices(a: ParticleTriad, b: ParticleTriad): boolean {
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

    public static MatchTriadIndices(a: ParticleTriad, b: ParticleTriad): boolean {
        return a.indexA === b.indexA && a.indexB === b.indexB && a.indexC === b.indexC;
    }

    public static InitializeParticleLists(group: ParticleGroup, nodeBuffer: ParticleSystem_ParticleListNode[]): void {
        const bufferIndex = group.GetBufferIndex();
        const particleCount = group.GetParticleCount();
        for (let i = 0; i < particleCount; i++) {
            const node = nodeBuffer[i];
            node.list = node;
            node.next = null;
            node.count = 1;
            node.index = i + bufferIndex;
        }
    }

    public MergeParticleListsInContact(group: ParticleGroup, nodeBuffer: ParticleSystem_ParticleListNode[]): void {
        const bufferIndex = group.GetBufferIndex();
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            /* const ParticleContact& */
            const contact = this.m_contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            if (!group.ContainsParticle(a) || !group.ContainsParticle(b)) {
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
            // DEBUG: Assert(listA.count >= listB.count);
            ParticleSystem.MergeParticleLists(listA, listB);
        }
    }

    public static MergeParticleLists(
        listA: ParticleSystem_ParticleListNode,
        listB: ParticleSystem_ParticleListNode,
    ): void {
        // Insert listB between index 0 and 1 of listA
        // Example:
        //     listA => a1 => a2 => a3 => null
        //     listB => b1 => b2 => null
        // to
        //     listA => listB => b1 => b2 => a1 => a2 => a3 => null
        // DEBUG: Assert(listA !== listB);
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

    public static FindLongestParticleList(
        group: ParticleGroup,
        nodeBuffer: ParticleSystem_ParticleListNode[],
    ): ParticleSystem_ParticleListNode {
        const particleCount = group.GetParticleCount();
        let result = nodeBuffer[0];
        for (let i = 0; i < particleCount; i++) {
            const node = nodeBuffer[i];
            if (result.count < node.count) {
                result = node;
            }
        }
        return result;
    }

    public MergeZombieParticleListNodes(
        group: ParticleGroup,
        nodeBuffer: ParticleSystem_ParticleListNode[],
        survivingList: ParticleSystem_ParticleListNode,
    ): void {
        const particleCount = group.GetParticleCount();
        for (let i = 0; i < particleCount; i++) {
            const node = nodeBuffer[i];
            if (node !== survivingList && this.m_flagsBuffer.data[node.index] & ParticleFlag.Zombie) {
                ParticleSystem.MergeParticleListAndNode(survivingList, node);
            }
        }
    }

    public static MergeParticleListAndNode(
        list: ParticleSystem_ParticleListNode,
        node: ParticleSystem_ParticleListNode,
    ): void {
        // Insert node between index 0 and 1 of list
        // Example:
        //     list => a1 => a2 => a3 => null
        //     node => null
        // to
        //     list => node => a1 => a2 => a3 => null
        // DEBUG: Assert(node !== list);
        // DEBUG: Assert(node.list === node);
        // DEBUG: Assert(node.count === 1);
        node.list = list;
        node.next = list.next;
        list.next = node;
        list.count++;
        node.count = 0;
    }

    public CreateParticleGroupsFromParticleList(
        group: ParticleGroup,
        nodeBuffer: ParticleSystem_ParticleListNode[],
        survivingList: ParticleSystem_ParticleListNode,
    ): void {
        const particleCount = group.GetParticleCount();
        const def = new ParticleGroupDef();
        def.groupFlags = group.GetGroupFlags();
        def.userData = group.GetUserData();
        for (let i = 0; i < particleCount; i++) {
            const list = nodeBuffer[i];
            if (!list.count || list === survivingList) {
                continue;
            }
            // DEBUG: Assert(list.list === list);
            const newGroup = this.CreateParticleGroup(def);
            for (let node: ParticleSystem_ParticleListNode | null = list; node; node = node.next) {
                const oldIndex = node.index;
                // DEBUG: const flags = this.m_flagsBuffer.data[oldIndex];
                // DEBUG: Assert(!(flags & ParticleFlag.Zombie));
                const newIndex = this.CloneParticle(oldIndex, newGroup);
                this.m_flagsBuffer.data[oldIndex] |= ParticleFlag.Zombie;
                node.index = newIndex;
            }
        }
    }

    public UpdatePairsAndTriadsWithParticleList(
        group: ParticleGroup,
        nodeBuffer: ParticleSystem_ParticleListNode[],
    ): void {
        const bufferIndex = group.GetBufferIndex();
        // Update indices in pairs and triads. If an index belongs to the group,
        // replace it with the corresponding value in nodeBuffer.
        // Note that nodeBuffer is allocated only for the group and the index should
        // be shifted by bufferIndex.
        for (let k = 0; k < this.m_pairBuffer.count; k++) {
            const pair = this.m_pairBuffer.data[k];
            const a = pair.indexA;
            const b = pair.indexB;
            if (group.ContainsParticle(a)) {
                pair.indexA = nodeBuffer[a - bufferIndex].index;
            }
            if (group.ContainsParticle(b)) {
                pair.indexB = nodeBuffer[b - bufferIndex].index;
            }
        }
        for (let k = 0; k < this.m_triadBuffer.count; k++) {
            const triad = this.m_triadBuffer.data[k];
            const a = triad.indexA;
            const b = triad.indexB;
            const c = triad.indexC;
            if (group.ContainsParticle(a)) {
                triad.indexA = nodeBuffer[a - bufferIndex].index;
            }
            if (group.ContainsParticle(b)) {
                triad.indexB = nodeBuffer[b - bufferIndex].index;
            }
            if (group.ContainsParticle(c)) {
                triad.indexC = nodeBuffer[c - bufferIndex].index;
            }
        }
    }

    public ComputeDepth(): void {
        const contactGroups: ParticleContact[] = []; // TODO: static
        let contactGroupsCount = 0;
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            const groupA = this.m_groupBuffer[a];
            const groupB = this.m_groupBuffer[b];
            if (groupA && groupA === groupB && groupA.m_groupFlags & ParticleGroupFlag.NeedsUpdateDepth) {
                contactGroups[contactGroupsCount++] = contact;
            }
        }
        const groupsToUpdate: ParticleGroup[] = []; // TODO: static
        let groupsToUpdateCount = 0;
        for (let group = this.m_groupList; group; group = group.GetNext()) {
            if (group.m_groupFlags & ParticleGroupFlag.NeedsUpdateDepth) {
                groupsToUpdate[groupsToUpdateCount++] = group;
                this.SetGroupFlags(group, group.m_groupFlags & ~ParticleGroupFlag.NeedsUpdateDepth);
                for (let i = group.m_firstIndex; i < group.m_lastIndex; i++) {
                    this.m_accumulationBuffer[i] = 0;
                }
            }
        }
        // Compute sum of weight of contacts except between different groups.
        for (let k = 0; k < contactGroupsCount; k++) {
            const contact = contactGroups[k];
            const a = contact.indexA;
            const b = contact.indexB;
            const w = contact.weight;
            this.m_accumulationBuffer[a] += w;
            this.m_accumulationBuffer[b] += w;
        }

        // DEBUG: Assert(this.m_depthBuffer !== null);
        for (let i = 0; i < groupsToUpdateCount; i++) {
            const group = groupsToUpdate[i];
            for (let j = group.m_firstIndex; j < group.m_lastIndex; j++) {
                const w = this.m_accumulationBuffer[j];
                this.m_depthBuffer[j] = w < 0.8 ? 0 : MAX_FLOAT;
            }
        }
        // The number of iterations is equal to particle number from the deepest
        // particle to the nearest surface particle, and in general it is smaller
        // than sqrt of total particle number.
        const iterationCount = Math.sqrt(this.m_count) >> 0;
        for (let t = 0; t < iterationCount; t++) {
            let updated = false;
            for (let k = 0; k < contactGroupsCount; k++) {
                const contact = contactGroups[k];
                const a = contact.indexA;
                const b = contact.indexB;
                const r = 1 - contact.weight;
                const ap0 = this.m_depthBuffer[a];
                const bp0 = this.m_depthBuffer[b];
                const ap1 = bp0 + r;
                const bp1 = ap0 + r;
                if (ap0 > ap1) {
                    this.m_depthBuffer[a] = ap1;
                    updated = true;
                }
                if (bp0 > bp1) {
                    this.m_depthBuffer[b] = bp1;
                    updated = true;
                }
            }
            if (!updated) {
                break;
            }
        }
        for (let i = 0; i < groupsToUpdateCount; i++) {
            const group = groupsToUpdate[i];
            for (let j = group.m_firstIndex; j < group.m_lastIndex; j++) {
                if (this.m_depthBuffer[j] < MAX_FLOAT) {
                    this.m_depthBuffer[j] *= this.m_particleDiameter;
                } else {
                    this.m_depthBuffer[j] = 0;
                }
            }
        }
    }

    public GetInsideBoundsEnumerator(aabb: Readonly<AABB>): ParticleSystem_InsideBoundsEnumerator {
        const lowerTag = ParticleSystem.computeTag(
            this.m_inverseDiameter * aabb.lowerBound.x - 1,
            this.m_inverseDiameter * aabb.lowerBound.y - 1,
        );
        const upperTag = ParticleSystem.computeTag(
            this.m_inverseDiameter * aabb.upperBound.x + 1,
            this.m_inverseDiameter * aabb.upperBound.y + 1,
        );
        const beginProxy = 0;
        const endProxy = this.m_proxyBuffer.count;
        const firstProxy = std_lower_bound(
            this.m_proxyBuffer.data,
            beginProxy,
            endProxy,
            lowerTag,
            ParticleSystem_Proxy.CompareProxyTag,
        );
        const lastProxy = std_upper_bound(
            this.m_proxyBuffer.data,
            beginProxy,
            endProxy,
            upperTag,
            ParticleSystem_Proxy.CompareTagProxy,
        );

        // DEBUG: Assert(beginProxy <= firstProxy);
        // DEBUG: Assert(firstProxy <= lastProxy);
        // DEBUG: Assert(lastProxy <= endProxy);

        return new ParticleSystem_InsideBoundsEnumerator(this, lowerTag, upperTag, firstProxy, lastProxy);
    }

    public UpdateAllParticleFlags(): void {
        this.m_allParticleFlags = 0;
        for (let i = 0; i < this.m_count; i++) {
            this.m_allParticleFlags |= this.m_flagsBuffer.data[i];
        }
        this.m_needsUpdateAllParticleFlags = false;
    }

    public UpdateAllGroupFlags(): void {
        this.m_allGroupFlags = 0;
        for (let group = this.m_groupList; group; group = group.GetNext()) {
            this.m_allGroupFlags |= group.m_groupFlags;
        }
        this.m_needsUpdateAllGroupFlags = false;
    }

    public AddContact(a: number, b: number): void {
        // DEBUG: Assert(contacts === this.m_contactBuffer);
        const flags_data = this.m_flagsBuffer.data;
        const pos_data = this.m_positionBuffer.data;
        const d = Vec2.Subtract(pos_data[b], pos_data[a], ParticleSystem.AddContact_s_d);
        const distBtParticlesSq = Vec2.Dot(d, d);
        if (distBtParticlesSq > 0 && distBtParticlesSq < this.m_squaredDiameter) {
            const invD = 1 / Math.sqrt(distBtParticlesSq);
            const contact = this.m_contactBuffer.data[this.m_contactBuffer.Append()];
            contact.indexA = a;
            contact.indexB = b;
            contact.flags = flags_data[a] | flags_data[b];
            contact.weight = 1 - distBtParticlesSq * invD * this.m_inverseDiameter;
            contact.normal.x = invD * d.x;
            contact.normal.y = invD * d.y;
        }
    }

    public static readonly AddContact_s_d = new Vec2();

    public FindContacts_Reference(): void {
        // DEBUG: Assert(contacts === this.m_contactBuffer);
        const beginProxy = 0;
        const endProxy = this.m_proxyBuffer.count;

        this.m_contactBuffer.count = 0;
        for (let a = beginProxy, c = beginProxy; a < endProxy; a++) {
            const rightTag = ParticleSystem.computeRelativeTag(this.m_proxyBuffer.data[a].tag, 1, 0);
            for (let b = a + 1; b < endProxy; b++) {
                if (rightTag < this.m_proxyBuffer.data[b].tag) {
                    break;
                }
                this.AddContact(this.m_proxyBuffer.data[a].index, this.m_proxyBuffer.data[b].index);
            }
            const bottomLeftTag = ParticleSystem.computeRelativeTag(this.m_proxyBuffer.data[a].tag, -1, 1);
            for (; c < endProxy; c++) {
                if (bottomLeftTag <= this.m_proxyBuffer.data[c].tag) {
                    break;
                }
            }
            const bottomRightTag = ParticleSystem.computeRelativeTag(this.m_proxyBuffer.data[a].tag, 1, 1);
            for (let b = c; b < endProxy; b++) {
                if (bottomRightTag < this.m_proxyBuffer.data[b].tag) {
                    break;
                }
                this.AddContact(this.m_proxyBuffer.data[a].index, this.m_proxyBuffer.data[b].index);
            }
        }
    }

    public FindContacts(): void {
        this.FindContacts_Reference();
    }

    public UpdateProxies_Reference(): void {
        // DEBUG: Assert(proxies === this.m_proxyBuffer);
        const pos_data = this.m_positionBuffer.data;
        const inv_diam = this.m_inverseDiameter;
        for (let k = 0; k < this.m_proxyBuffer.count; ++k) {
            const proxy = this.m_proxyBuffer.data[k];
            const i = proxy.index;
            const p = pos_data[i];
            proxy.tag = ParticleSystem.computeTag(inv_diam * p.x, inv_diam * p.y);
        }
    }

    public UpdateProxies(): void {
        this.UpdateProxies_Reference();
    }

    public SortProxies(): void {
        // DEBUG: Assert(proxies === this.m_proxyBuffer);

        std_sort(this.m_proxyBuffer.data, 0, this.m_proxyBuffer.count, ParticleSystem_Proxy.CompareProxyProxy);
    }

    public FilterContacts(): void {
        // Optionally filter the contact.
        const contactFilter = this.GetParticleContactFilter();
        if (contactFilter === null) {
            return;
        }

        // DEBUG: Assert(contacts === this.m_contactBuffer);
        this.m_contactBuffer.RemoveIf(
            (contact) =>
                (contact.flags & ParticleFlag.ParticleContactFilter) !== 0 &&
                !contactFilter.ShouldCollideParticleParticle(this, contact.indexA, contact.indexB),
        );
    }

    public NotifyContactListenerPreContact(particlePairs: ParticlePairSet): void {
        const contactListener = this.GetParticleContactListener();
        if (contactListener === null) {
            return;
        }

        particlePairs.Initialize(this.m_contactBuffer, this.m_flagsBuffer);

        throw new Error(); // TODO: notify
    }

    public NotifyContactListenerPostContact(particlePairs: ParticlePairSet): void {
        const contactListener = this.GetParticleContactListener();
        if (contactListener === null) {
            return;
        }

        // Loop through all new contacts, reporting any new ones, and
        // "invalidating" the ones that still exist.
        for (let k = 0; k < this.m_contactBuffer.count; ++k) {
            const contact = this.m_contactBuffer.data[k];
            // ParticlePair pair;
            // pair.first = contact.GetIndexA();
            // pair.second = contact.GetIndexB();
            // const int32 itemIndex = particlePairs.Find(pair);
            const itemIndex = -1; // TODO
            if (itemIndex >= 0) {
                // Already touching, ignore this contact.
                particlePairs.Invalidate(itemIndex);
            } else {
                // Just started touching, inform the listener.
                contactListener.BeginContactParticleParticle(this, contact);
            }
        }

        // Report particles that are no longer touching.
        // That is, any pairs that were not invalidated above.
        // const int32 pairCount = particlePairs.GetCount();
        // const ParticlePair* const pairs = particlePairs.GetBuffer();
        // const int8* const valid = particlePairs.GetValidBuffer();
        // for (int32 i = 0; i < pairCount; ++i)
        // {
        //  if (valid[i])
        //  {
        //    contactListener.EndContactParticleParticle(this, pairs[i].first, pairs[i].second);
        //  }
        // }

        throw new Error(); // TODO: notify
    }

    public static ParticleContactIsZombie(contact: ParticleContact): boolean {
        return (contact.flags & ParticleFlag.Zombie) === ParticleFlag.Zombie;
    }

    public UpdateContacts(exceptZombie: boolean): void {
        this.UpdateProxies();
        this.SortProxies();

        const particlePairs = new ParticlePairSet(); // TODO: static
        this.NotifyContactListenerPreContact(particlePairs);

        this.FindContacts();
        this.FilterContacts();

        this.NotifyContactListenerPostContact(particlePairs);

        if (exceptZombie) {
            this.m_contactBuffer.RemoveIf(ParticleSystem.ParticleContactIsZombie);
        }
    }

    public NotifyBodyContactListenerPreContact(fixtureSet: ParticleSystem_FixtureParticleSet): void {
        const contactListener = this.GetFixtureContactListener();
        if (contactListener === null) {
            return;
        }

        fixtureSet.Initialize(this.m_bodyContactBuffer, this.m_flagsBuffer);

        throw new Error(); // TODO: notify
    }

    public NotifyBodyContactListenerPostContact(fixtureSet: ParticleSystem_FixtureParticleSet): void {
        const contactListener = this.GetFixtureContactListener();
        if (contactListener === null) {
            return;
        }

        // Loop through all new contacts, reporting any new ones, and
        // "invalidating" the ones that still exist.
        for (let k = 0; k < this.m_bodyContactBuffer.count; k++) {
            const contact = this.m_bodyContactBuffer.data[k];
            // DEBUG: Assert(contact !== null);
            // FixtureParticle fixtureParticleToFind;
            // fixtureParticleToFind.first = contact.fixture;
            // fixtureParticleToFind.second = contact.index;
            // const int32 index = fixtureSet.Find(fixtureParticleToFind);
            const index = -1; // TODO
            if (index >= 0) {
                // Already touching remove this from the set.
                fixtureSet.Invalidate(index);
            } else {
                // Just started touching, report it!
                contactListener.BeginContactFixtureParticle(this, contact);
            }
        }

        // If the contact listener is enabled, report all fixtures that are no
        // longer in contact with particles.
        // const FixtureParticle* const fixtureParticles = fixtureSet.GetBuffer();
        // const int8* const fixtureParticlesValid = fixtureSet.GetValidBuffer();
        // const int32 fixtureParticleCount = fixtureSet.GetCount();
        // for (int32 i = 0; i < fixtureParticleCount; ++i)
        // {
        //  if (fixtureParticlesValid[i])
        //  {
        //    const FixtureParticle* const fixtureParticle = &fixtureParticles[i];
        //    contactListener.EndContactFixtureParticle(fixtureParticle.first, this, fixtureParticle.second);
        //  }
        // }

        throw new Error(); // TODO: notify
    }

    public UpdateBodyContacts(): void {
        const s_aabb = ParticleSystem.UpdateBodyContacts_s_aabb;

        // If the particle contact listener is enabled, generate a set of
        // fixture / particle contacts.
        const fixtureSet = new ParticleSystem_FixtureParticleSet(); // TODO: static
        this.NotifyBodyContactListenerPreContact(fixtureSet);

        if (this.m_stuckThreshold > 0) {
            const particleCount = this.GetParticleCount();
            for (let i = 0; i < particleCount; i++) {
                // Detect stuck particles, see comment in
                // ParticleSystem::DetectStuckParticle()
                this.m_bodyContactCountBuffer.data[i] = 0;
                if (this.m_timestamp > this.m_lastBodyContactStepBuffer.data[i] + 1) {
                    this.m_consecutiveContactStepsBuffer.data[i] = 0;
                }
            }
        }
        this.m_bodyContactBuffer.SetCount(0);
        this.m_stuckParticleBuffer.SetCount(0);

        const aabb = s_aabb;
        this.ComputeAABB(aabb);

        const callback = this.UpdateBodyContacts_callback;
        callback.m_contactFilter = this.GetFixtureContactFilter();
        this.m_world.QueryAABB(aabb, (fixture) => callback.ReportFixture(fixture));

        if (this.m_def.strictContactCheck) {
            this.RemoveSpuriousBodyContacts();
        }

        this.NotifyBodyContactListenerPostContact(fixtureSet);
    }

    public static readonly UpdateBodyContacts_s_aabb = new AABB();

    private readonly UpdateBodyContacts_callback = new ParticleSystem_UpdateBodyContactsCallback(this);

    public Solve(step: TimeStep): void {
        const s_subStep = ParticleSystem.Solve_s_subStep;
        if (this.m_count === 0) {
            return;
        }
        // If particle lifetimes are enabled, destroy particles that are too old.
        if (this.m_expirationTimeBuffer.data) {
            this.SolveLifetimes(step);
        }
        if (this.m_allParticleFlags & ParticleFlag.Zombie) {
            this.SolveZombie();
        }
        if (this.m_needsUpdateAllParticleFlags) {
            this.UpdateAllParticleFlags();
        }
        if (this.m_needsUpdateAllGroupFlags) {
            this.UpdateAllGroupFlags();
        }
        if (this.m_paused) {
            return;
        }
        const { particleIterations } = step.config;

        for (this.m_iterationIndex = 0; this.m_iterationIndex < particleIterations; this.m_iterationIndex++) {
            ++this.m_timestamp;
            const subStep = s_subStep.Copy(step);
            subStep.dt /= particleIterations;
            subStep.inv_dt *= particleIterations;
            this.UpdateContacts(false);
            this.UpdateBodyContacts();
            this.ComputeWeight();
            if (this.m_allGroupFlags & ParticleGroupFlag.NeedsUpdateDepth) {
                this.ComputeDepth();
            }
            if (this.m_allParticleFlags & ParticleFlag.Reactive) {
                this.UpdatePairsAndTriadsWithReactiveParticles();
            }
            if (this.m_hasForce) {
                this.SolveForce(subStep);
            }
            if (this.m_allParticleFlags & ParticleFlag.Viscous) {
                this.SolveViscous();
            }
            if (this.m_allParticleFlags & ParticleFlag.Repulsive) {
                this.SolveRepulsive(subStep);
            }
            if (this.m_allParticleFlags & ParticleFlag.Powder) {
                this.SolvePowder(subStep);
            }
            if (this.m_allParticleFlags & ParticleFlag.Tensile) {
                this.SolveTensile(subStep);
            }
            if (this.m_allGroupFlags & ParticleGroupFlag.Solid) {
                this.SolveSolid(subStep);
            }
            if (this.m_allParticleFlags & ParticleFlag.ColorMixing) {
                this.SolveColorMixing();
            }
            this.SolveGravity(subStep);
            if (this.m_allParticleFlags & ParticleFlag.StaticPressure) {
                this.SolveStaticPressure(subStep);
            }
            this.SolvePressure(subStep);
            this.SolveDamping(subStep);
            if (this.m_allParticleFlags & ParticleSystem.k_extraDampingFlags) {
                this.SolveExtraDamping();
            }
            // SolveElastic and SolveSpring refer the current velocities for
            // numerical stability, they should be called as late as possible.
            if (this.m_allParticleFlags & ParticleFlag.Elastic) {
                this.SolveElastic(subStep);
            }
            if (this.m_allParticleFlags & ParticleFlag.Spring) {
                this.SolveSpring(subStep);
            }
            this.LimitVelocity(subStep);
            if (this.m_allGroupFlags & ParticleGroupFlag.Rigid) {
                this.SolveRigidDamping();
            }
            if (this.m_allParticleFlags & ParticleFlag.Barrier) {
                this.SolveBarrier(subStep);
            }
            // SolveCollision, SolveRigid and SolveWall should be called after
            // other force functions because they may require particles to have
            // specific velocities.
            this.SolveCollision(subStep);
            if (this.m_allGroupFlags & ParticleGroupFlag.Rigid) {
                this.SolveRigid(subStep);
            }
            if (this.m_allParticleFlags & ParticleFlag.Wall) {
                this.SolveWall();
            }
            // The particle positions can be updated only at the end of substep.
            for (let i = 0; i < this.m_count; i++) {
                this.m_positionBuffer.data[i].AddScaled(subStep.dt, this.m_velocityBuffer.data[i]);
            }
        }
    }

    public static readonly Solve_s_subStep = TimeStep.Create();

    public SolveCollision(step: TimeStep): void {
        const s_aabb = ParticleSystem.SolveCollision_s_aabb;
        const pos_data = this.m_positionBuffer.data;
        const vel_data = this.m_velocityBuffer.data;

        // This function detects particles which are crossing boundary of bodies
        // and modifies velocities of them so that they will move just in front of
        // boundary. This function function also applies the reaction force to
        // bodies as precisely as the numerical stability is kept.
        const aabb = s_aabb;
        aabb.lowerBound.x = +MAX_FLOAT;
        aabb.lowerBound.y = +MAX_FLOAT;
        aabb.upperBound.x = -MAX_FLOAT;
        aabb.upperBound.y = -MAX_FLOAT;
        for (let i = 0; i < this.m_count; i++) {
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
        callback.m_step = step;
        this.m_world.QueryAABB(aabb, (fixture) => callback.ReportFixture(fixture));
    }

    public static readonly SolveCollision_s_aabb = new AABB();

    private readonly SolveCollision_callback = new ParticleSystem_SolveCollisionCallback(this);

    public LimitVelocity(step: TimeStep): void {
        const vel_data = this.m_velocityBuffer.data;
        const criticalVelocitySquared = this.GetCriticalVelocitySquared(step);
        for (let i = 0; i < this.m_count; i++) {
            const v = vel_data[i];
            const v2 = Vec2.Dot(v, v);
            if (v2 > criticalVelocitySquared) {
                v.Scale(Math.sqrt(criticalVelocitySquared / v2));
            }
        }
    }

    public SolveGravity(step: TimeStep): void {
        const s_gravity = ParticleSystem.SolveGravity_s_gravity;
        const vel_data = this.m_velocityBuffer.data;
        const gravity = Vec2.Scale(step.dt * this.m_def.gravityScale, this.m_world.GetGravity(), s_gravity);
        for (let i = 0; i < this.m_count; i++) {
            vel_data[i].Add(gravity);
        }
    }

    public static readonly SolveGravity_s_gravity = new Vec2();

    public SolveBarrier(step: TimeStep): void {
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
        const pos_data = this.m_positionBuffer.data;
        const vel_data = this.m_velocityBuffer.data;
        // If a particle is passing between paired barrier particles,
        // its velocity will be decelerated to avoid passing.
        for (let i = 0; i < this.m_count; i++) {
            const flags = this.m_flagsBuffer.data[i];
            if ((flags & ParticleSystem.k_barrierWallFlags) !== 0) {
                vel_data[i].SetZero();
            }
        }
        const tmax = b2_barrierCollisionTime * step.dt;
        const mass = this.GetParticleMass();
        for (let k = 0; k < this.m_pairBuffer.count; k++) {
            const pair = this.m_pairBuffer.data[k];
            if (pair.flags & ParticleFlag.Barrier) {
                const a = pair.indexA;
                const b = pair.indexB;
                const pa = pos_data[a];
                const pb = pos_data[b];
                const aabb = s_aabb;
                Vec2.Min(pa, pb, aabb.lowerBound);
                Vec2.Max(pa, pb, aabb.upperBound);
                const aGroup = this.m_groupBuffer[a];
                const bGroup = this.m_groupBuffer[b];
                const va = this.GetLinearVelocity(aGroup, a, pa, s_va);
                const vb = this.GetLinearVelocity(bGroup, b, pb, s_vb);
                const pba = Vec2.Subtract(pb, pa, s_pba);
                const vba = Vec2.Subtract(vb, va, s_vba);
                const enumerator = this.GetInsideBoundsEnumerator(aabb);
                let c: number;
                // eslint-disable-next-line no-cond-assign
                while ((c = enumerator.GetNext()) >= 0) {
                    const pc = pos_data[c];
                    const cGroup = this.m_groupBuffer[c];
                    if (aGroup !== cGroup && bGroup !== cGroup) {
                        const vc = this.GetLinearVelocity(cGroup, c, pc, s_vc);
                        // Solve the equation below:
                        //   (1-s)*(pa+t*va)+s*(pb+t*vb) = pc+t*vc
                        // which expresses that the particle c will pass a line
                        // connecting the particles a and b at the time of t.
                        // if s is between 0 and 1, c will pass between a and b.
                        const pca = Vec2.Subtract(pc, pa, s_pca);
                        const vca = Vec2.Subtract(vc, va, s_vca);
                        const e2 = Vec2.Cross(vba, vca);
                        const e1 = Vec2.Cross(pba, vca) - Vec2.Cross(pca, vba);
                        const e0 = Vec2.Cross(pba, pca);
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
                            Vec2.AddScaled(pba, t, vba, qba);
                            Vec2.AddScaled(pca, t, vca, qca);
                            s = Vec2.Dot(qba, qca) / Vec2.Dot(qba, qba);
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
                            Vec2.AddScaled(pba, t, vba, qba);
                            Vec2.AddScaled(pca, t, vca, qca);
                            s = Vec2.Dot(qba, qca) / Vec2.Dot(qba, qba);
                            if (!(t >= 0 && t < tmax && s >= 0 && s <= 1)) {
                                t = t2;
                                if (!(t >= 0 && t < tmax)) {
                                    continue;
                                }

                                Vec2.AddScaled(pba, t, vba, qba);
                                Vec2.AddScaled(pca, t, vca, qca);
                                s = Vec2.Dot(qba, qca) / Vec2.Dot(qba, qba);
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
                        const f = Vec2.Scale(mass, dv, s_f);
                        if (cGroup && this.IsRigidGroup(cGroup)) {
                            // If c belongs to a rigid group, the force will be
                            // distributed in the group.
                            const groupMass = cGroup.GetMass();
                            const inertia = cGroup.GetInertia();
                            if (groupMass > 0) {
                                cGroup.m_linearVelocity.AddScaled(1 / groupMass, f);
                            }
                            if (inertia > 0) {
                                cGroup.m_angularVelocity +=
                                    Vec2.Cross(Vec2.Subtract(pc, cGroup.GetCenter(), Vec2.s_t0), f) / inertia;
                            }
                        } else {
                            vel_data[c].Add(dv);
                        }
                        // Apply a reversed force to particle c after particle
                        // movement so that momentum will be preserved.
                        this.ParticleApplyForce(c, f.Scale(-step.inv_dt));
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

    public SolveStaticPressure(step: TimeStep): void {
        this.m_staticPressureBuffer = this.RequestBuffer(this.m_staticPressureBuffer);
        const criticalPressure = this.GetCriticalPressure(step);
        const pressurePerWeight = this.m_def.staticPressureStrength * criticalPressure;
        const maxPressure = b2_maxParticlePressure * criticalPressure;
        const relaxation = this.m_def.staticPressureRelaxation;
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
        for (let t = 0; t < this.m_def.staticPressureIterations; t++) {
            for (let i = 0; i < this.m_count; i++) {
                this.m_accumulationBuffer[i] = 0;
            }
            for (let k = 0; k < this.m_contactBuffer.count; k++) {
                const contact = this.m_contactBuffer.data[k];
                if (contact.flags & ParticleFlag.StaticPressure) {
                    const a = contact.indexA;
                    const b = contact.indexB;
                    const w = contact.weight;
                    this.m_accumulationBuffer[a] += w * this.m_staticPressureBuffer[b]; // a <- b
                    this.m_accumulationBuffer[b] += w * this.m_staticPressureBuffer[a]; // b <- a
                }
            }
            for (let i = 0; i < this.m_count; i++) {
                const w = this.m_weightBuffer[i];
                if (this.m_flagsBuffer.data[i] & ParticleFlag.StaticPressure) {
                    const wh = this.m_accumulationBuffer[i];
                    const h = (wh + pressurePerWeight * (w - b2_minParticleWeight)) / (w + relaxation);
                    this.m_staticPressureBuffer[i] = Clamp(h, 0, maxPressure);
                } else {
                    this.m_staticPressureBuffer[i] = 0;
                }
            }
        }
    }

    public ComputeWeight(): void {
        // calculates the sum of contact-weights for each particle
        // that means dimensionless density
        for (let k = 0; k < this.m_count; k++) {
            this.m_weightBuffer[k] = 0;
        }
        for (let k = 0; k < this.m_bodyContactBuffer.count; k++) {
            const contact = this.m_bodyContactBuffer.data[k];
            const a = contact.index;
            const w = contact.weight;
            this.m_weightBuffer[a] += w;
        }
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            const w = contact.weight;
            this.m_weightBuffer[a] += w;
            this.m_weightBuffer[b] += w;
        }
    }

    public SolvePressure(step: TimeStep): void {
        const s_f = ParticleSystem.SolvePressure_s_f;
        const pos_data = this.m_positionBuffer.data;
        const vel_data = this.m_velocityBuffer.data;
        // calculates pressure as a linear function of density
        const criticalPressure = this.GetCriticalPressure(step);
        const pressurePerWeight = this.m_def.pressureStrength * criticalPressure;
        const maxPressure = b2_maxParticlePressure * criticalPressure;
        for (let i = 0; i < this.m_count; i++) {
            const w = this.m_weightBuffer[i];
            const h = pressurePerWeight * Math.max(0, w - b2_minParticleWeight);
            this.m_accumulationBuffer[i] = Math.min(h, maxPressure);
        }
        // ignores particles which have their own repulsive force
        if (this.m_allParticleFlags & ParticleSystem.k_noPressureFlags) {
            for (let i = 0; i < this.m_count; i++) {
                if (this.m_flagsBuffer.data[i] & ParticleSystem.k_noPressureFlags) {
                    this.m_accumulationBuffer[i] = 0;
                }
            }
        }
        // static pressure
        if (this.m_allParticleFlags & ParticleFlag.StaticPressure) {
            // DEBUG: Assert(this.m_staticPressureBuffer !== null);
            for (let i = 0; i < this.m_count; i++) {
                if (this.m_flagsBuffer.data[i] & ParticleFlag.StaticPressure) {
                    this.m_accumulationBuffer[i] += this.m_staticPressureBuffer[i];
                }
            }
        }
        // applies pressure between each particles in contact
        const velocityPerPressure = step.dt / (this.m_def.density * this.m_particleDiameter);
        const inv_mass = this.GetParticleInvMass();
        for (let k = 0; k < this.m_bodyContactBuffer.count; k++) {
            const contact = this.m_bodyContactBuffer.data[k];
            const a = contact.index;
            const b = contact.body;
            const w = contact.weight;
            const m = contact.mass;
            const n = contact.normal;
            const p = pos_data[a];
            const h = this.m_accumulationBuffer[a] + pressurePerWeight * w;
            const f = Vec2.Scale(velocityPerPressure * w * m * h, n, s_f);
            vel_data[a].SubtractScaled(inv_mass, f);
            b.ApplyLinearImpulse(f, p, true);
        }
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            const w = contact.weight;
            const n = contact.normal;
            const h = this.m_accumulationBuffer[a] + this.m_accumulationBuffer[b];
            const f = Vec2.Scale(velocityPerPressure * w * h, n, s_f);
            vel_data[a].Subtract(f);
            vel_data[b].Add(f);
        }
    }

    public static readonly SolvePressure_s_f = new Vec2();

    public SolveDamping(step: TimeStep): void {
        const s_v = ParticleSystem.SolveDamping_s_v;
        const s_f = ParticleSystem.SolveDamping_s_f;
        const pos_data = this.m_positionBuffer.data;
        const vel_data = this.m_velocityBuffer.data;
        // reduces normal velocity of each contact
        const linearDamping = this.m_def.dampingStrength;
        const quadraticDamping = 1 / this.GetCriticalVelocity(step);
        const inv_mass = this.GetParticleInvMass();
        for (let k = 0; k < this.m_bodyContactBuffer.count; k++) {
            const contact = this.m_bodyContactBuffer.data[k];
            const a = contact.index;
            const b = contact.body;
            const w = contact.weight;
            const m = contact.mass;
            const n = contact.normal;
            const p = pos_data[a];
            const v = Vec2.Subtract(b.GetLinearVelocityFromWorldPoint(p, Vec2.s_t0), vel_data[a], s_v);
            const vn = Vec2.Dot(v, n);
            if (vn < 0) {
                const damping = Math.max(linearDamping * w, Math.min(-quadraticDamping * vn, 0.5));
                const f = Vec2.Scale(damping * m * vn, n, s_f);
                vel_data[a].AddScaled(inv_mass, f);
                b.ApplyLinearImpulse(f.Negate(), p, true);
            }
        }
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            const w = contact.weight;
            const n = contact.normal;
            const v = Vec2.Subtract(vel_data[b], vel_data[a], s_v);
            const vn = Vec2.Dot(v, n);
            if (vn < 0) {
                const damping = Math.max(linearDamping * w, Math.min(-quadraticDamping * vn, 0.5));
                const f = Vec2.Scale(damping * vn, n, s_f);
                vel_data[a].Add(f);
                vel_data[b].Subtract(f);
            }
        }
    }

    public static readonly SolveDamping_s_v = new Vec2();

    public static readonly SolveDamping_s_f = new Vec2();

    public SolveRigidDamping(): void {
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
        const pos_data = this.m_positionBuffer.data;
        const damping = this.m_def.dampingStrength;
        for (let k = 0; k < this.m_bodyContactBuffer.count; k++) {
            const contact = this.m_bodyContactBuffer.data[k];
            const a = contact.index;
            const aGroup = this.m_groupBuffer[a];
            if (aGroup && this.IsRigidGroup(aGroup)) {
                const b = contact.body;
                const n = contact.normal;
                const w = contact.weight;
                const p = pos_data[a];
                const v = Vec2.Subtract(
                    b.GetLinearVelocityFromWorldPoint(p, s_t0),
                    aGroup.GetLinearVelocityFromWorldPoint(p, s_t1),
                    s_v,
                );
                const vn = Vec2.Dot(v, n);
                if (vn < 0) {
                    // The group's average velocity at particle position 'p' is pushing
                    // the particle into the body.
                    this.InitDampingParameterWithRigidGroupOrParticle(
                        invMassA,
                        invInertiaA,
                        tangentDistanceA,
                        true,
                        aGroup,
                        a,
                        p,
                        n,
                    );
                    // Calculate b.m_I from public functions of Body.
                    this.InitDampingParameter(
                        invMassB,
                        invInertiaB,
                        tangentDistanceB,
                        b.GetMass(),
                        b.GetInertia() - b.GetMass() * b.GetLocalCenter().LengthSquared(),
                        b.GetWorldCenter(),
                        p,
                        n,
                    );
                    const f =
                        damping *
                        Math.min(w, 1) *
                        this.ComputeDampingImpulse(
                            invMassA[0],
                            invInertiaA[0],
                            tangentDistanceA[0],
                            invMassB[0],
                            invInertiaB[0],
                            tangentDistanceB[0],
                            vn,
                        );
                    this.ApplyDamping(invMassA[0], invInertiaA[0], tangentDistanceA[0], true, aGroup, a, f, n);
                    b.ApplyLinearImpulse(Vec2.Scale(-f, n, Vec2.s_t0), p, true);
                }
            }
        }
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            const n = contact.normal;
            const w = contact.weight;
            const aGroup = this.m_groupBuffer[a];
            const bGroup = this.m_groupBuffer[b];
            const aRigid = this.IsRigidGroup(aGroup);
            const bRigid = this.IsRigidGroup(bGroup);
            if (aGroup !== bGroup && (aRigid || bRigid)) {
                const p = Vec2.Mid(pos_data[a], pos_data[b], s_p);
                const v = Vec2.Subtract(
                    this.GetLinearVelocity(bGroup, b, p, s_t0),
                    this.GetLinearVelocity(aGroup, a, p, s_t1),
                    s_v,
                );
                const vn = Vec2.Dot(v, n);
                if (vn < 0) {
                    this.InitDampingParameterWithRigidGroupOrParticle(
                        invMassA,
                        invInertiaA,
                        tangentDistanceA,
                        aRigid,
                        aGroup,
                        a,
                        p,
                        n,
                    );
                    this.InitDampingParameterWithRigidGroupOrParticle(
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
                        this.ComputeDampingImpulse(
                            invMassA[0],
                            invInertiaA[0],
                            tangentDistanceA[0],
                            invMassB[0],
                            invInertiaB[0],
                            tangentDistanceB[0],
                            vn,
                        );
                    this.ApplyDamping(invMassA[0], invInertiaA[0], tangentDistanceA[0], aRigid, aGroup, a, f, n);
                    this.ApplyDamping(invMassB[0], invInertiaB[0], tangentDistanceB[0], bRigid, bGroup, b, -f, n);
                }
            }
        }
    }

    public static readonly SolveRigidDamping_s_t0 = new Vec2();

    public static readonly SolveRigidDamping_s_t1 = new Vec2();

    public static readonly SolveRigidDamping_s_p = new Vec2();

    public static readonly SolveRigidDamping_s_v = new Vec2();

    public SolveExtraDamping(): void {
        const s_v = ParticleSystem.SolveExtraDamping_s_v;
        const s_f = ParticleSystem.SolveExtraDamping_s_f;
        const vel_data = this.m_velocityBuffer.data;
        // Applies additional damping force between bodies and particles which can
        // produce strong repulsive force. Applying damping force multiple times
        // is effective in suppressing vibration.
        const pos_data = this.m_positionBuffer.data;
        const inv_mass = this.GetParticleInvMass();
        for (let k = 0; k < this.m_bodyContactBuffer.count; k++) {
            const contact = this.m_bodyContactBuffer.data[k];
            const a = contact.index;
            if (this.m_flagsBuffer.data[a] & ParticleSystem.k_extraDampingFlags) {
                const b = contact.body;
                const m = contact.mass;
                const n = contact.normal;
                const p = pos_data[a];
                const v = Vec2.Subtract(b.GetLinearVelocityFromWorldPoint(p, Vec2.s_t0), vel_data[a], s_v);
                const vn = Vec2.Dot(v, n);
                if (vn < 0) {
                    const f = Vec2.Scale(0.5 * m * vn, n, s_f);
                    vel_data[a].AddScaled(inv_mass, f);
                    b.ApplyLinearImpulse(f.Negate(), p, true);
                }
            }
        }
    }

    public static readonly SolveExtraDamping_s_v = new Vec2();

    public static readonly SolveExtraDamping_s_f = new Vec2();

    public SolveWall(): void {
        const vel_data = this.m_velocityBuffer.data;
        for (let i = 0; i < this.m_count; i++) {
            if (this.m_flagsBuffer.data[i] & ParticleFlag.Wall) {
                vel_data[i].SetZero();
            }
        }
    }

    public SolveRigid(step: TimeStep): void {
        const s_position = ParticleSystem.SolveRigid_s_position;
        const s_rotation = ParticleSystem.SolveRigid_s_rotation;
        const s_transform = ParticleSystem.SolveRigid_s_transform;
        const s_velocityTransform = ParticleSystem.SolveRigid_s_velocityTransform;
        const pos_data = this.m_positionBuffer.data;
        const vel_data = this.m_velocityBuffer.data;
        for (let group = this.m_groupList; group; group = group.GetNext()) {
            if (group.m_groupFlags & ParticleGroupFlag.Rigid) {
                group.UpdateStatistics();
                const rotation = s_rotation;
                rotation.Set(step.dt * group.m_angularVelocity);
                const position = Vec2.Add(
                    group.m_center,
                    Vec2.Subtract(
                        Vec2.Scale(step.dt, group.m_linearVelocity, Vec2.s_t0),
                        Rot.MultiplyVec2(rotation, group.m_center, Vec2.s_t1),
                        Vec2.s_t0,
                    ),
                    s_position,
                );
                const transform = s_transform;
                transform.SetPositionRotation(position, rotation);
                Transform.Multiply(transform, group.m_transform, group.m_transform);
                const velocityTransform = s_velocityTransform;
                velocityTransform.p.x = step.inv_dt * transform.p.x;
                velocityTransform.p.y = step.inv_dt * transform.p.y;
                velocityTransform.q.s = step.inv_dt * transform.q.s;
                velocityTransform.q.c = step.inv_dt * (transform.q.c - 1);
                for (let i = group.m_firstIndex; i < group.m_lastIndex; i++) {
                    Transform.MultiplyVec2(velocityTransform, pos_data[i], vel_data[i]);
                }
            }
        }
    }

    public static readonly SolveRigid_s_position = new Vec2();

    public static readonly SolveRigid_s_rotation = new Rot();

    public static readonly SolveRigid_s_transform = new Transform();

    public static readonly SolveRigid_s_velocityTransform = new Transform();

    public SolveElastic(step: TimeStep): void {
        const s_pa = ParticleSystem.SolveElastic_s_pa;
        const s_pb = ParticleSystem.SolveElastic_s_pb;
        const s_pc = ParticleSystem.SolveElastic_s_pc;
        const s_r = ParticleSystem.SolveElastic_s_r;
        const s_t0 = ParticleSystem.SolveElastic_s_t0;
        const pos_data = this.m_positionBuffer.data;
        const vel_data = this.m_velocityBuffer.data;
        const elasticStrength = step.inv_dt * this.m_def.elasticStrength;
        for (let k = 0; k < this.m_triadBuffer.count; k++) {
            const triad = this.m_triadBuffer.data[k];
            if (triad.flags & ParticleFlag.Elastic) {
                const a = triad.indexA;
                const b = triad.indexB;
                const c = triad.indexC;
                const oa = triad.pa;
                const ob = triad.pb;
                const oc = triad.pc;
                const pa = s_pa.Copy(pos_data[a]);
                const pb = s_pb.Copy(pos_data[b]);
                const pc = s_pc.Copy(pos_data[c]);
                const va = vel_data[a];
                const vb = vel_data[b];
                const vc = vel_data[c];
                pa.AddScaled(step.dt, va);
                pb.AddScaled(step.dt, vb);
                pc.AddScaled(step.dt, vc);
                const midPoint_x = (pa.x + pb.x + pc.x) / 3;
                const midPoint_y = (pa.y + pb.y + pc.y) / 3;
                pa.x -= midPoint_x;
                pa.y -= midPoint_y;
                pb.x -= midPoint_x;
                pb.y -= midPoint_y;
                pc.x -= midPoint_x;
                pc.y -= midPoint_y;
                const r = s_r;
                r.s = Vec2.Cross(oa, pa) + Vec2.Cross(ob, pb) + Vec2.Cross(oc, pc);
                r.c = Vec2.Dot(oa, pa) + Vec2.Dot(ob, pb) + Vec2.Dot(oc, pc);
                const r2 = r.s * r.s + r.c * r.c;
                let invR = 1 / Math.sqrt(r2);
                if (!Number.isFinite(invR)) {
                    invR = 1.98177537e19;
                }
                r.s *= invR;
                r.c *= invR;
                const strength = elasticStrength * triad.strength;
                Rot.MultiplyVec2(r, oa, s_t0);
                Vec2.Subtract(s_t0, pa, s_t0);
                Vec2.Scale(strength, s_t0, s_t0);
                va.Add(s_t0);
                Rot.MultiplyVec2(r, ob, s_t0);
                Vec2.Subtract(s_t0, pb, s_t0);
                Vec2.Scale(strength, s_t0, s_t0);
                vb.Add(s_t0);
                Rot.MultiplyVec2(r, oc, s_t0);
                Vec2.Subtract(s_t0, pc, s_t0);
                Vec2.Scale(strength, s_t0, s_t0);
                vc.Add(s_t0);
            }
        }
    }

    public static readonly SolveElastic_s_pa = new Vec2();

    public static readonly SolveElastic_s_pb = new Vec2();

    public static readonly SolveElastic_s_pc = new Vec2();

    public static readonly SolveElastic_s_r = new Rot();

    public static readonly SolveElastic_s_t0 = new Vec2();

    public SolveSpring(step: TimeStep): void {
        const s_pa = ParticleSystem.SolveSpring_s_pa;
        const s_pb = ParticleSystem.SolveSpring_s_pb;
        const s_d = ParticleSystem.SolveSpring_s_d;
        const s_f = ParticleSystem.SolveSpring_s_f;
        const pos_data = this.m_positionBuffer.data;
        const vel_data = this.m_velocityBuffer.data;
        const springStrength = step.inv_dt * this.m_def.springStrength;
        for (let k = 0; k < this.m_pairBuffer.count; k++) {
            const pair = this.m_pairBuffer.data[k];
            if (pair.flags & ParticleFlag.Spring) {
                const a = pair.indexA;
                const b = pair.indexB;
                const pa = s_pa.Copy(pos_data[a]);
                const pb = s_pb.Copy(pos_data[b]);
                const va = vel_data[a];
                const vb = vel_data[b];
                pa.AddScaled(step.dt, va);
                pb.AddScaled(step.dt, vb);
                const d = Vec2.Subtract(pb, pa, s_d);
                const r0 = pair.distance;
                const r1 = d.Length();
                const strength = springStrength * pair.strength;
                const f = Vec2.Scale((strength * (r0 - r1)) / r1, d, s_f);
                va.Subtract(f);
                vb.Add(f);
            }
        }
    }

    public static readonly SolveSpring_s_pa = new Vec2();

    public static readonly SolveSpring_s_pb = new Vec2();

    public static readonly SolveSpring_s_d = new Vec2();

    public static readonly SolveSpring_s_f = new Vec2();

    public SolveTensile(step: TimeStep): void {
        const s_weightedNormal = ParticleSystem.SolveTensile_s_weightedNormal;
        const s_s = ParticleSystem.SolveTensile_s_s;
        const s_f = ParticleSystem.SolveTensile_s_f;
        const vel_data = this.m_velocityBuffer.data;
        // DEBUG: Assert(this.m_accumulation2Buffer !== null);
        for (let i = 0; i < this.m_count; i++) {
            this.m_accumulation2Buffer[i] = new Vec2();
            this.m_accumulation2Buffer[i].SetZero();
        }
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            if (contact.flags & ParticleFlag.Tensile) {
                const a = contact.indexA;
                const b = contact.indexB;
                const w = contact.weight;
                const n = contact.normal;
                const weightedNormal = Vec2.Scale((1 - w) * w, n, s_weightedNormal);
                this.m_accumulation2Buffer[a].Subtract(weightedNormal);
                this.m_accumulation2Buffer[b].Add(weightedNormal);
            }
        }
        const criticalVelocity = this.GetCriticalVelocity(step);
        const pressureStrength = this.m_def.surfaceTensionPressureStrength * criticalVelocity;
        const normalStrength = this.m_def.surfaceTensionNormalStrength * criticalVelocity;
        const maxVelocityVariation = b2_maxParticleForce * criticalVelocity;
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            if (contact.flags & ParticleFlag.Tensile) {
                const a = contact.indexA;
                const b = contact.indexB;
                const w = contact.weight;
                const n = contact.normal;
                const h = this.m_weightBuffer[a] + this.m_weightBuffer[b];
                const s = Vec2.Subtract(this.m_accumulation2Buffer[b], this.m_accumulation2Buffer[a], s_s);
                const fn =
                    Math.min(pressureStrength * (h - 2) + normalStrength * Vec2.Dot(s, n), maxVelocityVariation) * w;
                const f = Vec2.Scale(fn, n, s_f);
                vel_data[a].Subtract(f);
                vel_data[b].Add(f);
            }
        }
    }

    public static readonly SolveTensile_s_weightedNormal = new Vec2();

    public static readonly SolveTensile_s_s = new Vec2();

    public static readonly SolveTensile_s_f = new Vec2();

    public SolveViscous(): void {
        const s_v = ParticleSystem.SolveViscous_s_v;
        const s_f = ParticleSystem.SolveViscous_s_f;
        const pos_data = this.m_positionBuffer.data;
        const vel_data = this.m_velocityBuffer.data;
        const { viscousStrength } = this.m_def;
        const inv_mass = this.GetParticleInvMass();
        for (let k = 0; k < this.m_bodyContactBuffer.count; k++) {
            const contact = this.m_bodyContactBuffer.data[k];
            const a = contact.index;
            if (this.m_flagsBuffer.data[a] & ParticleFlag.Viscous) {
                const b = contact.body;
                const w = contact.weight;
                const m = contact.mass;
                const p = pos_data[a];
                const v = Vec2.Subtract(b.GetLinearVelocityFromWorldPoint(p, Vec2.s_t0), vel_data[a], s_v);
                const f = Vec2.Scale(viscousStrength * m * w, v, s_f);
                vel_data[a].AddScaled(inv_mass, f);
                b.ApplyLinearImpulse(f.Negate(), p, true);
            }
        }
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            if (contact.flags & ParticleFlag.Viscous) {
                const a = contact.indexA;
                const b = contact.indexB;
                const w = contact.weight;
                const v = Vec2.Subtract(vel_data[b], vel_data[a], s_v);
                const f = Vec2.Scale(viscousStrength * w, v, s_f);
                vel_data[a].Add(f);
                vel_data[b].Subtract(f);
            }
        }
    }

    public static readonly SolveViscous_s_v = new Vec2();

    public static readonly SolveViscous_s_f = new Vec2();

    public SolveRepulsive(step: TimeStep): void {
        const s_f = ParticleSystem.SolveRepulsive_s_f;
        const vel_data = this.m_velocityBuffer.data;
        const repulsiveStrength = this.m_def.repulsiveStrength * this.GetCriticalVelocity(step);
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            if (contact.flags & ParticleFlag.Repulsive) {
                const a = contact.indexA;
                const b = contact.indexB;
                if (this.m_groupBuffer[a] !== this.m_groupBuffer[b]) {
                    const w = contact.weight;
                    const n = contact.normal;
                    const f = Vec2.Scale(repulsiveStrength * w, n, s_f);
                    vel_data[a].Subtract(f);
                    vel_data[b].Add(f);
                }
            }
        }
    }

    public static readonly SolveRepulsive_s_f = new Vec2();

    public SolvePowder(step: TimeStep): void {
        const s_f = ParticleSystem.SolvePowder_s_f;
        const pos_data = this.m_positionBuffer.data;
        const vel_data = this.m_velocityBuffer.data;
        const powderStrength = this.m_def.powderStrength * this.GetCriticalVelocity(step);
        const minWeight = 1 - b2_particleStride;
        const inv_mass = this.GetParticleInvMass();
        for (let k = 0; k < this.m_bodyContactBuffer.count; k++) {
            const contact = this.m_bodyContactBuffer.data[k];
            const a = contact.index;
            if (this.m_flagsBuffer.data[a] & ParticleFlag.Powder) {
                const w = contact.weight;
                if (w > minWeight) {
                    const b = contact.body;
                    const m = contact.mass;
                    const p = pos_data[a];
                    const n = contact.normal;
                    const f = Vec2.Scale(powderStrength * m * (w - minWeight), n, s_f);
                    vel_data[a].SubtractScaled(inv_mass, f);
                    b.ApplyLinearImpulse(f, p, true);
                }
            }
        }
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            if (contact.flags & ParticleFlag.Powder) {
                const w = contact.weight;
                if (w > minWeight) {
                    const a = contact.indexA;
                    const b = contact.indexB;
                    const n = contact.normal;
                    const f = Vec2.Scale(powderStrength * (w - minWeight), n, s_f);
                    vel_data[a].Subtract(f);
                    vel_data[b].Add(f);
                }
            }
        }
    }

    public static readonly SolvePowder_s_f = new Vec2();

    public SolveSolid(step: TimeStep): void {
        const s_f = ParticleSystem.SolveSolid_s_f;
        const vel_data = this.m_velocityBuffer.data;
        // applies extra repulsive force from solid particle groups
        this.m_depthBuffer = this.RequestBuffer(this.m_depthBuffer);
        const ejectionStrength = step.inv_dt * this.m_def.ejectionStrength;
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            if (this.m_groupBuffer[a] !== this.m_groupBuffer[b]) {
                const w = contact.weight;
                const n = contact.normal;
                const h = this.m_depthBuffer[a] + this.m_depthBuffer[b];
                const f = Vec2.Scale(ejectionStrength * h * w, n, s_f);
                vel_data[a].Subtract(f);
                vel_data[b].Add(f);
            }
        }
    }

    public static readonly SolveSolid_s_f = new Vec2();

    public SolveForce(step: TimeStep): void {
        const vel_data = this.m_velocityBuffer.data;
        const velocityPerForce = step.dt * this.GetParticleInvMass();
        for (let i = 0; i < this.m_count; i++) {
            vel_data[i].AddScaled(velocityPerForce, this.m_forceBuffer[i]);
        }
        this.m_hasForce = false;
    }

    public SolveColorMixing(): void {
        // mixes color between contacting particles
        const colorMixing = 0.5 * this.m_def.colorMixingStrength;
        if (colorMixing) {
            for (let k = 0; k < this.m_contactBuffer.count; k++) {
                const contact = this.m_contactBuffer.data[k];
                const a = contact.indexA;
                const b = contact.indexB;
                if (this.m_flagsBuffer.data[a] & this.m_flagsBuffer.data[b] & ParticleFlag.ColorMixing) {
                    const colorA = this.m_colorBuffer.data[a];
                    const colorB = this.m_colorBuffer.data[b];
                    // Use the static method to ensure certain compilers inline
                    // this correctly.
                    Color.MixColors(colorA, colorB, colorMixing);
                }
            }
        }
    }

    public SolveZombie(): void {
        // removes particles with zombie flag
        let newCount = 0;
        const newIndices: number[] = []; // TODO: static
        for (let i = 0; i < this.m_count; i++) {
            newIndices[i] = b2_invalidParticleIndex;
        }
        // DEBUG: Assert(newIndices.length === this.m_count);
        let allParticleFlags = 0;
        const destructionListener = this.m_world.GetDestructionListener();
        for (let i = 0; i < this.m_count; i++) {
            const flags = this.m_flagsBuffer.data[i];
            if (flags & ParticleFlag.Zombie) {
                if (flags & ParticleFlag.DestructionListener) {
                    destructionListener?.SayGoodbyeParticle(this, i);
                }
                // Destroy particle handle.
                if (this.m_handleIndexBuffer.data) {
                    const handle = this.m_handleIndexBuffer.data[i];
                    if (handle) {
                        handle.SetIndex(b2_invalidParticleIndex);
                        this.m_handleIndexBuffer.data[i] = null;
                    }
                }
                newIndices[i] = b2_invalidParticleIndex;
            } else {
                newIndices[i] = newCount;
                if (i !== newCount) {
                    // Update handle to reference new particle index.
                    if (this.m_handleIndexBuffer.data) {
                        const handle = this.m_handleIndexBuffer.data[i];
                        if (handle) {
                            handle.SetIndex(newCount);
                        }
                        this.m_handleIndexBuffer.data[newCount] = handle;
                    }
                    this.m_flagsBuffer.data[newCount] = this.m_flagsBuffer.data[i];
                    if (this.m_lastBodyContactStepBuffer.data) {
                        this.m_lastBodyContactStepBuffer.data[newCount] = this.m_lastBodyContactStepBuffer.data[i];
                    }
                    if (this.m_bodyContactCountBuffer.data) {
                        this.m_bodyContactCountBuffer.data[newCount] = this.m_bodyContactCountBuffer.data[i];
                    }
                    if (this.m_consecutiveContactStepsBuffer.data) {
                        this.m_consecutiveContactStepsBuffer.data[newCount] = this.m_consecutiveContactStepsBuffer.data[
                            i
                        ];
                    }
                    this.m_positionBuffer.data[newCount].Copy(this.m_positionBuffer.data[i]);
                    this.m_velocityBuffer.data[newCount].Copy(this.m_velocityBuffer.data[i]);
                    this.m_groupBuffer[newCount] = this.m_groupBuffer[i];
                    if (this.m_hasForce) {
                        this.m_forceBuffer[newCount].Copy(this.m_forceBuffer[i]);
                    }
                    if (this.m_staticPressureBuffer) {
                        this.m_staticPressureBuffer[newCount] = this.m_staticPressureBuffer[i];
                    }
                    if (this.m_depthBuffer) {
                        this.m_depthBuffer[newCount] = this.m_depthBuffer[i];
                    }
                    if (this.m_colorBuffer.data) {
                        this.m_colorBuffer.data[newCount].Copy(this.m_colorBuffer.data[i]);
                    }
                    if (this.m_userDataBuffer.data) {
                        this.m_userDataBuffer.data[newCount] = this.m_userDataBuffer.data[i];
                    }
                    if (this.m_expirationTimeBuffer.data) {
                        this.m_expirationTimeBuffer.data[newCount] = this.m_expirationTimeBuffer.data[i];
                    }
                }
                newCount++;
                allParticleFlags |= flags;
            }
        }

        // predicate functions
        const Test = {
            IsProxyInvalid: (proxy: ParticleSystem_Proxy) => {
                return proxy.index < 0;
            },
            IsContactInvalid: (contact: ParticleContact) => {
                return contact.indexA < 0 || contact.indexB < 0;
            },
            IsBodyContactInvalid: (contact: ParticleBodyContact) => {
                return contact.index < 0;
            },
            IsPairInvalid: (pair: ParticlePair) => {
                return pair.indexA < 0 || pair.indexB < 0;
            },
            IsTriadInvalid: (triad: ParticleTriad) => {
                return triad.indexA < 0 || triad.indexB < 0 || triad.indexC < 0;
            },
        };

        // update proxies
        for (let k = 0; k < this.m_proxyBuffer.count; k++) {
            const proxy = this.m_proxyBuffer.data[k];
            proxy.index = newIndices[proxy.index];
        }
        this.m_proxyBuffer.RemoveIf(Test.IsProxyInvalid);

        // update contacts
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            contact.indexA = newIndices[contact.indexA];
            contact.indexB = newIndices[contact.indexB];
        }
        this.m_contactBuffer.RemoveIf(Test.IsContactInvalid);

        // update particle-body contacts
        for (let k = 0; k < this.m_bodyContactBuffer.count; k++) {
            const contact = this.m_bodyContactBuffer.data[k];
            contact.index = newIndices[contact.index];
        }
        this.m_bodyContactBuffer.RemoveIf(Test.IsBodyContactInvalid);

        // update pairs
        for (let k = 0; k < this.m_pairBuffer.count; k++) {
            const pair = this.m_pairBuffer.data[k];
            pair.indexA = newIndices[pair.indexA];
            pair.indexB = newIndices[pair.indexB];
        }
        this.m_pairBuffer.RemoveIf(Test.IsPairInvalid);

        // update triads
        for (let k = 0; k < this.m_triadBuffer.count; k++) {
            const triad = this.m_triadBuffer.data[k];
            triad.indexA = newIndices[triad.indexA];
            triad.indexB = newIndices[triad.indexB];
            triad.indexC = newIndices[triad.indexC];
        }
        this.m_triadBuffer.RemoveIf(Test.IsTriadInvalid);

        // Update lifetime indices.
        if (this.m_indexByExpirationTimeBuffer.data) {
            let writeOffset = 0;
            for (let readOffset = 0; readOffset < this.m_count; readOffset++) {
                const newIndex = newIndices[this.m_indexByExpirationTimeBuffer.data[readOffset]];
                if (newIndex !== b2_invalidParticleIndex) {
                    this.m_indexByExpirationTimeBuffer.data[writeOffset++] = newIndex;
                }
            }
        }

        // update groups
        for (let group = this.m_groupList; group; group = group.GetNext()) {
            let firstIndex = newCount;
            let lastIndex = 0;
            let modified = false;
            for (let i = group.m_firstIndex; i < group.m_lastIndex; i++) {
                const j = newIndices[i];
                if (j >= 0) {
                    firstIndex = Math.min(firstIndex, j);
                    lastIndex = Math.max(lastIndex, j + 1);
                } else {
                    modified = true;
                }
            }
            if (firstIndex < lastIndex) {
                group.m_firstIndex = firstIndex;
                group.m_lastIndex = lastIndex;
                if (modified) {
                    if (group.m_groupFlags & ParticleGroupFlag.Solid) {
                        this.SetGroupFlags(group, group.m_groupFlags | ParticleGroupFlag.NeedsUpdateDepth);
                    }
                }
            } else {
                group.m_firstIndex = 0;
                group.m_lastIndex = 0;
                if (!(group.m_groupFlags & ParticleGroupFlag.CanBeEmpty)) {
                    this.SetGroupFlags(group, group.m_groupFlags | ParticleGroupFlag.WillBeDestroyed);
                }
            }
        }

        // update particle count
        this.m_count = newCount;
        this.m_allParticleFlags = allParticleFlags;
        this.m_needsUpdateAllParticleFlags = false;

        // destroy bodies with no particles
        for (let group = this.m_groupList; group; ) {
            const next = group.GetNext();
            if (group.m_groupFlags & ParticleGroupFlag.WillBeDestroyed) {
                this.DestroyParticleGroup(group);
            }
            group = next;
        }
    }

    /**
     * Destroy all particles which have outlived their lifetimes set
     * by SetParticleLifetime().
     */
    public SolveLifetimes(step: TimeStep): void {
        // Update the time elapsed.
        this.m_timeElapsed = this.LifetimeToExpirationTime(step.dt);
        // Get the floor (non-fractional component) of the elapsed time.
        const quantizedTimeElapsed = this.GetQuantizedTimeElapsed();

        const expirationTimes = this.m_expirationTimeBuffer.data;
        const expirationTimeIndices = this.m_indexByExpirationTimeBuffer.data;
        const particleCount = this.GetParticleCount();
        // Sort the lifetime buffer if it's required.
        if (this.m_expirationTimeBufferRequiresSorting) {
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

            this.m_expirationTimeBufferRequiresSorting = false;
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
            this.DestroyParticle(particleIndex);
        }
    }

    public RotateBuffer(start: number, mid: number, end: number): void {
        // move the particles assigned to the given group toward the end of array
        if (start === mid || mid === end) {
            return;
        }
        // DEBUG: Assert(mid >= start && mid <= end);

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

        std_rotate(this.m_flagsBuffer.data, start, mid, end);
        if (this.m_lastBodyContactStepBuffer.data) {
            std_rotate(this.m_lastBodyContactStepBuffer.data, start, mid, end);
        }
        if (this.m_bodyContactCountBuffer.data) {
            std_rotate(this.m_bodyContactCountBuffer.data, start, mid, end);
        }
        if (this.m_consecutiveContactStepsBuffer.data) {
            std_rotate(this.m_consecutiveContactStepsBuffer.data, start, mid, end);
        }
        std_rotate(this.m_positionBuffer.data, start, mid, end);
        std_rotate(this.m_velocityBuffer.data, start, mid, end);
        std_rotate(this.m_groupBuffer, start, mid, end);
        if (this.m_hasForce) {
            std_rotate(this.m_forceBuffer, start, mid, end);
        }
        if (this.m_staticPressureBuffer) {
            std_rotate(this.m_staticPressureBuffer, start, mid, end);
        }
        if (this.m_depthBuffer) {
            std_rotate(this.m_depthBuffer, start, mid, end);
        }
        if (this.m_colorBuffer.data) {
            std_rotate(this.m_colorBuffer.data, start, mid, end);
        }
        if (this.m_userDataBuffer.data) {
            std_rotate(this.m_userDataBuffer.data, start, mid, end);
        }

        // Update handle indices.
        if (this.m_handleIndexBuffer.data) {
            std_rotate(this.m_handleIndexBuffer.data, start, mid, end);
            for (let i = start; i < end; ++i) {
                const handle = this.m_handleIndexBuffer.data[i];
                if (handle) {
                    handle.SetIndex(newIndices(handle.GetIndex()));
                }
            }
        }

        if (this.m_expirationTimeBuffer.data) {
            std_rotate(this.m_expirationTimeBuffer.data, start, mid, end);
            // Update expiration time buffer indices.
            const particleCount = this.GetParticleCount();
            const indexByExpirationTime = this.m_indexByExpirationTimeBuffer.data;
            for (let i = 0; i < particleCount; ++i) {
                indexByExpirationTime[i] = newIndices(indexByExpirationTime[i]);
            }
        }

        // update proxies
        for (let k = 0; k < this.m_proxyBuffer.count; k++) {
            const proxy = this.m_proxyBuffer.data[k];
            proxy.index = newIndices(proxy.index);
        }

        // update contacts
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            contact.indexA = newIndices(contact.indexA);
            contact.indexB = newIndices(contact.indexB);
        }

        // update particle-body contacts
        for (let k = 0; k < this.m_bodyContactBuffer.count; k++) {
            const contact = this.m_bodyContactBuffer.data[k];
            contact.index = newIndices(contact.index);
        }

        // update pairs
        for (let k = 0; k < this.m_pairBuffer.count; k++) {
            const pair = this.m_pairBuffer.data[k];
            pair.indexA = newIndices(pair.indexA);
            pair.indexB = newIndices(pair.indexB);
        }

        // update triads
        for (let k = 0; k < this.m_triadBuffer.count; k++) {
            const triad = this.m_triadBuffer.data[k];
            triad.indexA = newIndices(triad.indexA);
            triad.indexB = newIndices(triad.indexB);
            triad.indexC = newIndices(triad.indexC);
        }

        // update groups
        for (let group = this.m_groupList; group; group = group.GetNext()) {
            group.m_firstIndex = newIndices(group.m_firstIndex);
            group.m_lastIndex = newIndices(group.m_lastIndex - 1) + 1;
        }
    }

    public GetCriticalVelocity(step: TimeStep): number {
        return this.m_particleDiameter * step.inv_dt;
    }

    public GetCriticalVelocitySquared(step: TimeStep): number {
        const velocity = this.GetCriticalVelocity(step);
        return velocity * velocity;
    }

    public GetCriticalPressure(step: TimeStep): number {
        return this.m_def.density * this.GetCriticalVelocitySquared(step);
    }

    public GetParticleStride(): number {
        return b2_particleStride * this.m_particleDiameter;
    }

    public GetParticleMass(): number {
        const stride = this.GetParticleStride();
        return this.m_def.density * stride * stride;
    }

    public GetParticleInvMass(): number {
        // mass = density * stride^2, so we take the inverse of this.
        const inverseStride = this.m_inverseDiameter * (1 / b2_particleStride);
        return this.m_inverseDensity * inverseStride * inverseStride;
    }

    /**
     * Get the world's contact filter if any particles with the
     * ContactFilter flag are present in the system.
     */
    public GetFixtureContactFilter(): ContactFilter | null {
        return this.m_allParticleFlags & ParticleFlag.FixtureContactFilter
            ? this.m_world.GetContactManager().m_contactFilter
            : null;
    }

    /**
     * Get the world's contact filter if any particles with the
     * ParticleContactFilter flag are present in the
     * system.
     */
    public GetParticleContactFilter(): ContactFilter | null {
        return this.m_allParticleFlags & ParticleFlag.ParticleContactFilter
            ? this.m_world.GetContactManager().m_contactFilter
            : null;
    }

    /**
     * Get the world's contact listener if any particles with the
     * FixtureContactListener flag are present in the
     * system.
     */
    public GetFixtureContactListener(): ContactListener | null {
        return this.m_allParticleFlags & ParticleFlag.FixtureContactListener
            ? this.m_world.GetContactManager().m_contactListener
            : null;
    }

    /**
     * Get the world's contact listener if any particles with the
     * ParticleContactListener flag are present in the
     * system.
     */
    public GetParticleContactListener(): ContactListener | null {
        return this.m_allParticleFlags & ParticleFlag.ParticleContactListener
            ? this.m_world.GetContactManager().m_contactListener
            : null;
    }

    public SetUserOverridableBuffer<T>(buffer: ParticleSystem_UserOverridableBuffer<T>, data: T[]): void {
        buffer.data = data;
        buffer.userSuppliedCapacity = data.length;
    }

    public SetGroupFlags(group: ParticleGroup, newFlags: ParticleGroupFlag): void {
        const oldFlags = group.m_groupFlags;
        if ((oldFlags ^ newFlags) & ParticleGroupFlag.Solid) {
            // If the Solid flag changed schedule depth update.
            newFlags |= ParticleGroupFlag.NeedsUpdateDepth;
        }
        if (oldFlags & ~newFlags) {
            // If any flags might be removed
            this.m_needsUpdateAllGroupFlags = true;
        }
        if (~this.m_allGroupFlags & newFlags) {
            // If any flags were added
            if (newFlags & ParticleGroupFlag.Solid) {
                this.m_depthBuffer = this.RequestBuffer(this.m_depthBuffer);
            }
            this.m_allGroupFlags |= newFlags;
        }
        group.m_groupFlags = newFlags;
    }

    public static BodyContactCompare(lhs: ParticleBodyContact, rhs: ParticleBodyContact): boolean {
        if (lhs.index === rhs.index) {
            // Subsort by weight, decreasing.
            return lhs.weight > rhs.weight;
        }
        return lhs.index < rhs.index;
    }

    public RemoveSpuriousBodyContacts(): void {
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
        std_sort(this.m_bodyContactBuffer.data, 0, this.m_bodyContactBuffer.count, ParticleSystem.BodyContactCompare);

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
        const ParticleBodyContactRemovePredicate = (contact: ParticleBodyContact): boolean => {
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
            const n = s_n.Copy(contact.normal);
            // weight is 1-(inv(diameter) * distance)
            n.Scale(this.m_particleDiameter * (1 - contact.weight));
            const pos = Vec2.Add(this.m_positionBuffer.data[contact.index], n, s_pos);

            // pos is now a point projected back along the contact normal to the
            // contact distance. If the surface makes sense for a contact, pos will
            // now lie on or in the fixture generating
            if (!contact.fixture.TestPoint(pos)) {
                const childCount = contact.fixture.GetShape().GetChildCount();
                for (let childIndex = 0; childIndex < childCount; childIndex++) {
                    const normal = s_normal;

                    const distance = computeDistance(
                        contact.fixture.GetShape(),
                        contact.fixture.GetBody().GetTransform(),
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
        this.m_bodyContactBuffer.count = std_remove_if(
            this.m_bodyContactBuffer.data,
            ParticleBodyContactRemovePredicate,
            this.m_bodyContactBuffer.count,
        );
    }

    private static RemoveSpuriousBodyContacts_s_n = new Vec2();

    private static RemoveSpuriousBodyContacts_s_pos = new Vec2();

    private static RemoveSpuriousBodyContacts_s_normal = new Vec2();

    public DetectStuckParticle(particle: number): void {
        // Detect stuck particles
        //
        // The basic algorithm is to allow the user to specify an optional
        // threshold where we detect whenever a particle is contacting
        // more than one fixture for more than threshold consecutive
        // steps. This is considered to be "stuck", and these are put
        // in a list the user can query per step, if enabled, to deal with
        // such particles.

        if (this.m_stuckThreshold <= 0) {
            return;
        }

        // Get the state variables for this particle.
        // int32 * const consecutiveCount = &m_consecutiveContactStepsBuffer.data[particle];
        // int32 * const lastStep = &m_lastBodyContactStepBuffer.data[particle];
        // int32 * const bodyCount = &m_bodyContactCountBuffer.data[particle];

        // This is only called when there is a body contact for this particle.
        // ++(*bodyCount);
        ++this.m_bodyContactCountBuffer.data[particle];

        // We want to only trigger detection once per step, the first time we
        // contact more than one fixture in a step for a given particle.
        // if (*bodyCount === 2)
        if (this.m_bodyContactCountBuffer.data[particle] === 2) {
            // ++(*consecutiveCount);
            ++this.m_consecutiveContactStepsBuffer.data[particle];
            // if (*consecutiveCount > m_stuckThreshold)
            if (this.m_consecutiveContactStepsBuffer.data[particle] > this.m_stuckThreshold) {
                // int32& newStuckParticle = m_stuckParticleBuffer.Append();
                // newStuckParticle = particle;
                this.m_stuckParticleBuffer.data[this.m_stuckParticleBuffer.Append()] = particle;
            }
        }
        // *lastStep = m_timestamp;
        this.m_lastBodyContactStepBuffer.data[particle] = this.m_timestamp;
    }

    /**
     * Determine whether a particle index is valid.
     */
    public ValidateParticleIndex(index: number): boolean {
        return index >= 0 && index < this.GetParticleCount() && index !== b2_invalidParticleIndex;
    }

    /**
     * Get the time elapsed in
     * ParticleSystemDef::lifetimeGranularity.
     */
    public GetQuantizedTimeElapsed(): number {
        // return (int32)(m_timeElapsed >> 32);
        return Math.floor(this.m_timeElapsed / 0x100000000);
    }

    /**
     * Convert a lifetime in seconds to an expiration time.
     */
    public LifetimeToExpirationTime(lifetime: number): number {
        return this.m_timeElapsed + Math.floor((lifetime / this.m_def.lifetimeGranularity) * 0x100000000);
    }

    public ForceCanBeApplied(flags: ParticleFlag): boolean {
        return !(flags & ParticleFlag.Wall);
    }

    public PrepareForceBuffer(): void {
        if (!this.m_hasForce) {
            for (let i = 0; i < this.m_count; i++) {
                this.m_forceBuffer[i].SetZero();
            }
            this.m_hasForce = true;
        }
    }

    public IsRigidGroup(group: ParticleGroup | null): boolean {
        return group !== null && (group.m_groupFlags & ParticleGroupFlag.Rigid) !== 0;
    }

    public GetLinearVelocity(group: ParticleGroup | null, particleIndex: number, point: Vec2, out: Vec2): Vec2 {
        if (group && this.IsRigidGroup(group)) {
            return group.GetLinearVelocityFromWorldPoint(point, out);
        }
        return out.Copy(this.m_velocityBuffer.data[particleIndex]);
    }

    public InitDampingParameter(
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
        tangentDistance[0] = Vec2.Cross(Vec2.Subtract(point, center, Vec2.s_t0), normal);
    }

    public InitDampingParameterWithRigidGroupOrParticle(
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
            this.InitDampingParameter(
                invMass,
                invInertia,
                tangentDistance,
                group.GetMass(),
                group.GetInertia(),
                group.GetCenter(),
                point,
                normal,
            );
        } else {
            const flags = this.m_flagsBuffer.data[particleIndex];
            this.InitDampingParameter(
                invMass,
                invInertia,
                tangentDistance,
                flags & ParticleFlag.Wall ? 0 : this.GetParticleMass(),
                0,
                point,
                point,
                normal,
            );
        }
    }

    public ComputeDampingImpulse(
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

    public ApplyDamping(
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
            group.m_linearVelocity.AddScaled(impulse * invMass, normal);
            group.m_angularVelocity += impulse * tangentDistance * invInertia;
        } else {
            this.m_velocityBuffer.data[particleIndex].AddScaled(impulse * invMass, normal);
        }
    }
}

export class ParticleSystem_Proxy {
    public index = b2_invalidParticleIndex;

    public tag = 0;

    public static CompareProxyProxy(a: ParticleSystem_Proxy, b: ParticleSystem_Proxy): boolean {
        return a.tag < b.tag;
    }

    public static CompareTagProxy(a: number, b: ParticleSystem_Proxy): boolean {
        return a < b.tag;
    }

    public static CompareProxyTag(a: ParticleSystem_Proxy, b: number): boolean {
        return a.tag < b;
    }
}

export class ParticleSystem_InsideBoundsEnumerator {
    public m_system: ParticleSystem;

    public m_xLower: number;

    public m_xUpper: number;

    public m_yLower: number;

    public m_yUpper: number;

    public m_first: number;

    public m_last: number;

    /**
     * InsideBoundsEnumerator enumerates all particles inside the
     * given bounds.
     *
     * Construct an enumerator with bounds of tags and a range of
     * proxies.
     */
    public constructor(system: ParticleSystem, lower: number, upper: number, first: number, last: number) {
        this.m_system = system;
        this.m_xLower = (lower & ParticleSystem.xMask) >>> 0;
        this.m_xUpper = (upper & ParticleSystem.xMask) >>> 0;
        this.m_yLower = (lower & ParticleSystem.yMask) >>> 0;
        this.m_yUpper = (upper & ParticleSystem.yMask) >>> 0;
        this.m_first = first;
        this.m_last = last;
        // DEBUG: Assert(this.m_first <= this.m_last);
    }

    /**
     * Get index of the next particle. Returns
     * b2_invalidParticleIndex if there are no more particles.
     */
    public GetNext(): number {
        while (this.m_first < this.m_last) {
            const xTag = (this.m_system.m_proxyBuffer.data[this.m_first].tag & ParticleSystem.xMask) >>> 0;
            // #if B2_ASSERT_ENABLED
            // DEBUG: const yTag = (this.m_system.m_proxyBuffer.data[this.m_first].tag & ParticleSystem_yMask) >>> 0;
            // DEBUG: Assert(yTag >= this.m_yLower);
            // DEBUG: Assert(yTag <= this.m_yUpper);
            // #endif
            if (xTag >= this.m_xLower && xTag <= this.m_xUpper) {
                return this.m_system.m_proxyBuffer.data[this.m_first++].index;
            }
            this.m_first++;
        }
        return b2_invalidParticleIndex;
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
    public Allocate(_itemSize: number, count: number): number {
        // TODO
        return count;
    }

    public Clear(): void {
        // TODO
    }

    public GetCount(): number {
        // TODO
        return 0;
    }

    public Invalidate(_itemIndex: number): void {
        // TODO
    }

    public GetValidBuffer(): boolean[] {
        // TODO
        return [];
    }

    public GetBuffer(): T[] {
        // TODO
        return [];
    }

    public SetCount(_count: number): void {
        // TODO
    }
}

export class ParticleSystem_FixtureParticle {
    public first: Fixture;

    public second = b2_invalidParticleIndex;

    public constructor(fixture: Fixture, particle: number) {
        this.first = fixture;
        this.second = particle;
    }
}

export class ParticleSystem_FixtureParticleSet extends ParticleSystem_FixedSetAllocator<
    ParticleSystem_FixtureParticle
> {
    public Initialize(
        _bodyContactBuffer: GrowableBuffer<ParticleBodyContact>,
        _flagsBuffer: ParticleSystem_UserOverridableBuffer<ParticleFlag>,
    ): void {
        // TODO
    }

    public Find(_pair: ParticleSystem_FixtureParticle): number {
        // TODO
        return b2_invalidParticleIndex;
    }
}

export class ParticleSystem_ParticlePair {
    public first = b2_invalidParticleIndex;

    public second = b2_invalidParticleIndex;

    public constructor(particleA: number, particleB: number) {
        this.first = particleA;
        this.second = particleB;
    }
}

export class ParticlePairSet extends ParticleSystem_FixedSetAllocator<ParticleSystem_ParticlePair> {
    public Initialize(
        _contactBuffer: GrowableBuffer<ParticleContact>,
        _flagsBuffer: ParticleSystem_UserOverridableBuffer<ParticleFlag>,
    ): void {
        // TODO
    }

    public Find(_pair: ParticleSystem_ParticlePair): number {
        // TODO
        return b2_invalidParticleIndex;
    }
}

export class ParticleSystem_ConnectionFilter {
    /**
     * Is the particle necessary for connection?
     * A pair or a triad should contain at least one 'necessary'
     * particle.
     */
    public IsNecessary(_index: number): boolean {
        return true;
    }

    /**
     * An additional condition for creating a pair.
     */
    public ShouldCreatePair(_a: number, _b: number): boolean {
        return true;
    }

    /**
     * An additional condition for creating a triad.
     */
    public ShouldCreateTriad(_a: number, _b: number, _c: number): boolean {
        return true;
    }
}

export class ParticleSystem_JoinParticleGroupsFilter extends ParticleSystem_ConnectionFilter {
    public m_threshold = 0;

    public constructor(threshold: number) {
        super();
        this.m_threshold = threshold;
    }

    /**
     * An additional condition for creating a pair.
     */
    public ShouldCreatePair(a: number, b: number): boolean {
        return (a < this.m_threshold && this.m_threshold <= b) || (b < this.m_threshold && this.m_threshold <= a);
    }

    /**
     * An additional condition for creating a triad.
     */
    public ShouldCreateTriad(a: number, b: number, c: number): boolean {
        return (
            (a < this.m_threshold || b < this.m_threshold || c < this.m_threshold) &&
            (this.m_threshold <= a || this.m_threshold <= b || this.m_threshold <= c)
        );
    }
}

export class ParticleSystem_CompositeShape extends Shape {
    public constructor(shapes: Shape[], shapeCount = shapes.length) {
        super(ShapeType.Unknown, 0);
        this.m_shapes = shapes;
        this.m_shapeCount = shapeCount;
    }

    public m_shapes: Shape[];

    public m_shapeCount = 0;

    public Clone(): Shape {
        throw new Error();
    }

    public GetChildCount(): number {
        return 1;
    }

    /**
     * @see Shape::TestPoint
     */
    public TestPoint(xf: Transform, p: XY): boolean {
        for (let i = 0; i < this.m_shapeCount; i++) {
            if (this.m_shapes[i].TestPoint(xf, p)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Implement Shape.
     */
    public RayCast(_output: RayCastOutput, _input: RayCastInput, _xf: Transform, _childIndex: number): boolean {
        // DEBUG: Assert(false);
        return false;
    }

    /**
     * @see Shape::ComputeAABB
     */
    public ComputeAABB(aabb: AABB, xf: Transform, _childIndex: number): void {
        const s_subaabb = new AABB();
        aabb.lowerBound.x = +MAX_FLOAT;
        aabb.lowerBound.y = +MAX_FLOAT;
        aabb.upperBound.x = -MAX_FLOAT;
        aabb.upperBound.y = -MAX_FLOAT;
        // DEBUG: Assert(childIndex === 0);
        for (let i = 0; i < this.m_shapeCount; i++) {
            const childCount = this.m_shapes[i].GetChildCount();
            for (let j = 0; j < childCount; j++) {
                const subaabb = s_subaabb;
                this.m_shapes[i].ComputeAABB(subaabb, xf, j);
                aabb.Combine1(subaabb);
            }
        }
    }

    /**
     * @see Shape::ComputeMass
     */
    public ComputeMass(_massData: MassData, _density: number): void {
        // DEBUG: Assert(false);
    }

    public SetupDistanceProxy(_proxy: DistanceProxy, _index: number): void {
        // DEBUG: Assert(false);
    }

    public Draw() {}
}

export class ParticleSystem_ReactiveFilter extends ParticleSystem_ConnectionFilter {
    public m_flagsBuffer: ParticleSystem_UserOverridableBuffer<ParticleFlag>;

    public constructor(flagsBuffer: ParticleSystem_UserOverridableBuffer<ParticleFlag>) {
        super();
        this.m_flagsBuffer = flagsBuffer;
    }

    public IsNecessary(index: number): boolean {
        return (this.m_flagsBuffer.data[index] & ParticleFlag.Reactive) !== 0;
    }
}
