// MIT License

import type { b2Mat22, b2Mat33, b2Rot, b2Transform, b2Vec2, b2Vec3 } from "./b2_math";

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

type ReadonlyfiableTypes = b2Vec2 | b2Vec3 | b2Mat22 | b2Mat33 | b2Rot | b2Transform;

type MapEntry<T, TKey extends keyof T, TManual extends Record<keyof T, any>> = [T, TKey, TManual];

interface ReadonlyMap extends Record<string, MapEntry<ReadonlyfiableTypes, any, any>> {
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
}

type GetFromMap<T extends ReadonlyfiableTypes> = {
    [TKey in keyof ReadonlyMap as ReadonlyMap[TKey] extends [T, any, any]
        ? "result"
        : never]: ReadonlyMap[TKey] extends [T, infer TPicks, infer TManual] ? [TPicks, TManual] : never;
};
type PickReadonly<T, K extends keyof T, TManual> =
    T extends Record<string, any> ? Readonly<TManual & { [TKey in K]: b2Readonly<T[TKey]> }> : T;

export type b2Readonly<T extends ReadonlyfiableTypes> = T extends (...args: any[]) => any
    ? T
    : GetFromMap<T> extends { result: [infer T2 extends keyof T, infer TManual] }
      ? PickReadonly<T, T2, TManual>
      : Readonly<T>;
