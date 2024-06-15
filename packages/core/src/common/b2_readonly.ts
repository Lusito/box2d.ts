// MIT License

import type { b2Mat22, b2Mat33, b2Rot, b2Sweep, b2Transform, b2Vec2, b2Vec3 } from "./b2_math";

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

/**
 * A map, so b2Readonly can determine the readonly properties of a Type
 *
 * The key should be the name of the type to be readonly-fied. The value is a tuple with 3 parts:
 * - The type that can be made readonly
 * - The properties that should be picked from that type
 * - A manual set of properties to be added to the readonly type.
 *   Use this to specify properties which need to use b2Readonly.
 *   Set it to `unknown` if there is no need for manual properties
 */
export interface b2ReadonlyTypes {
    b2Vec2: [
        b2Vec2,
        "x" | "y" | "Clone" | "Dot" | "Cross" | "Length" | "LengthSquared" | "IsValid" | "GetAbs",
        unknown,
    ];
    b2Vec3: [b2Vec3, "x" | "y" | "z" | "Clone", unknown];
    b2Mat22: [
        b2Mat22,
        "Clone" | "GetAngle" | "GetInverse" | "GetAbs",
        { ex: b2Readonly<b2Vec2>; ey: b2Readonly<b2Vec2> },
    ];
    b2Mat33: [
        b2Mat33,
        "Clone" | "Solve33" | "Solve22" | "GetInverse22" | "GetSymInverse33",
        { ex: b2Readonly<b2Vec3>; ey: b2Readonly<b2Vec3>; ez: b2Readonly<b2Vec3> },
    ];
    b2Rot: [b2Rot, "s" | "c" | "Clone" | "GetAngle" | "GetXAxis" | "GetYAxis", unknown];
    b2Transform: [
        b2Transform,
        "GetPosition" | "GetRotation" | "GetAngle",
        { p: b2Readonly<b2Vec2>; q: b2Readonly<b2Rot> },
    ];
    b2Sweep: [b2Sweep, "a0" | "a" | "alpha" | "Clone" | "GetTransform", { localCenter: b2Vec2; c0: b2Vec2; c: b2Vec2 }];
}

type GetFromTypes<T> = {
    [TKey in keyof b2ReadonlyTypes as b2ReadonlyTypes[TKey] extends [T, any, any]
        ? "result"
        : never]: b2ReadonlyTypes[TKey] extends [T, infer TPicks, infer TManual] ? [TPicks, TManual] : never;
};

/**
 * This type can be used to make a type like ReadonlyArray without introducing new types to learn.
 * So you can simply write `b2Readonly<b2Vec2>` instead of `b2ReadonlyVec2`.
 *
 * By default it only works with math types from @box2d/core, but you can extend it via declaration merging of `b2ReadonlyTypes`.
 */
export type b2Readonly<T> = T extends (...args: any[]) => any
    ? T
    : GetFromTypes<T> extends { result: [infer TProperties extends keyof T, infer TManual] }
      ? Readonly<TManual & { [TKey in TProperties]: b2Readonly<T[TKey]> }>
      : Readonly<T>;
