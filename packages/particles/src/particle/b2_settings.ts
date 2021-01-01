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

/** A symbolic constant that stands for particle allocation error. */
export const INVALID_PARTICLE_INDEX = -1;

export const MAX_PARTICLE_INDEX = 0x7fffffff;

/** The default distance between particles, multiplied by the particle diameter. */
export const PARTICLE_STRIDE = 0.75;

/** The minimum particle weight that produces pressure. */
export const MIN_PARTICLE_WEIGHT = 1;

/** The upper limit for particle pressure. */
export const MAX_PARTICLE_PRESSURE = 0.25;

/** The upper limit for force between particles. */
export const MAX_PARTICLE_FORCE = 0.5;

/** The maximum distance between particles in a triad, multiplied by the particle diameter. */
export const MAX_TRIAD_DISTANCE = 2;
export const MAX_TRIAD_DISTANCE_SQUARED = MAX_TRIAD_DISTANCE * MAX_TRIAD_DISTANCE;

/** The initial size of particle data buffers. */
export const MIN_PARTICLE_SYSTEM_BUFFER_CAPACITY = 256;

/** The time into the future that collisions against barrier particles will be detected. */
export const BARRIER_COLLISION_TIME = 2.5;
