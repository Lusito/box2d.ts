// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

import { b2_lengthUnitsPerMeter } from "./b2_settings";

export function b2Assert(condition: boolean, message?: string): asserts condition {
    if (!condition) throw new Error(message);
}

export function b2Verify<T>(value: T | null): T {
    if (value === null) throw new Error();
    return value;
}

export const b2_maxFloat = 1e37; // FLT_MAX instead of Number.MAX_VALUE;
export const b2_epsilon = 1e-5; // FLT_EPSILON instead of Number.MIN_VALUE;
export const b2_epsilon_sq = b2_epsilon * b2_epsilon;

// Global tuning constants based on meters-kilograms-seconds (MKS) units.

// Collision

/**
 * The maximum number of contact points between two convex shapes. Do
 * not change this value.
 */
export const b2_maxManifoldPoints = 2;

/**
 * This is used to fatten AABBs in the dynamic tree. This allows proxies
 * to move by a small amount without triggering a tree adjustment.
 * This is in meters.
 */
export const b2_aabbExtension = 0.1 * b2_lengthUnitsPerMeter;

/**
 * This is used to fatten AABBs in the dynamic tree. This is used to predict
 * the future position based on the current displacement.
 * This is a dimensionless multiplier.
 */
export const b2_aabbMultiplier = 4;

/**
 * A small length used as a collision and constraint tolerance. Usually it is
 * chosen to be numerically significant, but visually insignificant. In meters.
 */
export const b2_linearSlop = 0.005 * b2_lengthUnitsPerMeter;

/**
 * A small angle used as a collision and constraint tolerance. Usually it is
 * chosen to be numerically significant, but visually insignificant.
 */
export const b2_angularSlop = (2 / 180) * Math.PI;

/**
 * The radius of the polygon/edge shape skin. This should not be modified. Making
 * this smaller means polygons will have an insufficient buffer for continuous collision.
 * Making it larger may create artifacts for vertex collision.
 */
export const b2_polygonRadius = 2 * b2_linearSlop;

/** Maximum number of sub-steps per contact in continuous physics simulation. */
export const b2_maxSubSteps = 8;

// Dynamics

/** Maximum number of contacts to be handled to solve a TOI impact. */
export const b2_maxTOIContacts = 32;

/**
 * The maximum linear position correction used when solving constraints. This helps to
 * prevent overshoot. Meters.
 */
export const b2_maxLinearCorrection = 0.2 * b2_lengthUnitsPerMeter;

/**
 * The maximum angular position correction used when solving constraints. This helps to
 * prevent overshoot.
 */
export const b2_maxAngularCorrection = (8 / 180) * Math.PI;

/**
 * The maximum linear translation of a body per step. This limit is very large and is used
 * to prevent numerical problems. You shouldn't need to adjust this. Meters.
 */
export const b2_maxTranslation = 2 * b2_lengthUnitsPerMeter;
export const b2_maxTranslationSquared = b2_maxTranslation * b2_maxTranslation;

/**
 * The maximum angular velocity of a body. This limit is very large and is used
 * to prevent numerical problems. You shouldn't need to adjust this.
 */
export const b2_maxRotation = 0.5 * Math.PI;
export const b2_maxRotationSquared = b2_maxRotation * b2_maxRotation;

/**
 * This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
 * that overlap is removed in one time step. However using values close to 1 often lead
 * to overshoot.
 */
export const b2_baumgarte = 0.2;
export const b2_toiBaumgarte = 0.75;

// Sleep

/** The time that a body must be still before it will go to sleep. */
export const b2_timeToSleep = 0.5;

/** A body cannot sleep if its linear velocity is above this tolerance. */
export const b2_linearSleepTolerance = 0.01 * b2_lengthUnitsPerMeter;

/** A body cannot sleep if its angular velocity is above this tolerance. */
export const b2_angularSleepTolerance = (2 / 180) * Math.PI;

/**
 * Current version.
 * @see http://en.wikipedia.org/wiki/Software_versioning
 */
export const b2_version = {
    major: 2,
    minor: 4,
    patch: 1,
};

export function b2MakeNumberArray(length: number, init = 0): number[] {
    const result = new Array<number>(length);
    for (let i = 0; i < length; i++) result[i] = init;
    return result;
}

export function b2MakeBooleanArray(length: number, init = false): boolean[] {
    const result = new Array<boolean>(length);
    for (let i = 0; i < length; i++) result[i] = init;
    return result;
}

export interface NoArgsConstructor<T> {
    new (): T;
}

export function b2MakeArray<T>(length: number, Class: NoArgsConstructor<T>): T[] {
    const result = new Array<T>(length);
    for (let i = 0; i < length; i++) result[i] = new Class();
    return result;
}
