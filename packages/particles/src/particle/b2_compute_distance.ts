/*
 * Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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
    b2EdgeShape,
    b2Shape,
    b2Transform,
    b2Vec2,
    b2ChainShape,
    b2ShapeType,
    b2PolygonShape,
    b2_maxFloat,
    b2Rot,
    b2CircleShape,
    b2Readonly,
} from "@box2d/core";

type ComputeDistanceFn<T extends b2Shape> = (
    shape: T,
    xf: b2Readonly<b2Transform>,
    p: b2Readonly<b2Vec2>,
    normal: b2Vec2,
    childIndex: number,
) => number;

const tempEdgeShape = new b2EdgeShape();
const tempLocal = new b2Vec2();
const tempNormalForMaxDistance = new b2Vec2();
const tempMinDistance = new b2Vec2();
const tempDistance = new b2Vec2();
const tempV1 = new b2Vec2();
const tempV2 = new b2Vec2();
const tempD = new b2Vec2();
const tempS = new b2Vec2();
const tempCenter = new b2Vec2();

const implementations: Array<ComputeDistanceFn<any>> = [
    (shape: b2CircleShape, xf, p, normal) => {
        const center = b2Transform.MultiplyVec2(xf, shape.m_p, tempCenter);
        b2Vec2.Subtract(p, center, normal);
        return normal.Normalize() - shape.m_radius;
    },
    (shape: b2EdgeShape, xf, p, normal) => {
        const v1 = b2Transform.MultiplyVec2(xf, shape.m_vertex1, tempV1);
        const v2 = b2Transform.MultiplyVec2(xf, shape.m_vertex2, tempV2);

        const d = b2Vec2.Subtract(p, v1, tempD);
        const s = b2Vec2.Subtract(v2, v1, tempS);
        const ds = b2Vec2.Dot(d, s);
        if (ds > 0) {
            const s2 = b2Vec2.Dot(s, s);
            if (ds > s2) {
                b2Vec2.Subtract(p, v2, d);
            } else {
                d.SubtractScaled(ds / s2, s);
            }
        }
        normal.Copy(d);
        return normal.Normalize();
    },
    (shape: b2PolygonShape, xf, p, normal) => {
        const pLocal = b2Transform.TransposeMultiplyVec2(xf, p, tempLocal);
        let maxDistance = -b2_maxFloat;
        const normalForMaxDistance = tempNormalForMaxDistance.Copy(pLocal);

        for (let i = 0; i < shape.m_count; ++i) {
            const dot = b2Vec2.Dot(shape.m_normals[i], b2Vec2.Subtract(pLocal, shape.m_vertices[i], b2Vec2.s_t0));
            if (dot > maxDistance) {
                maxDistance = dot;
                normalForMaxDistance.Copy(shape.m_normals[i]);
            }
        }

        if (maxDistance > 0) {
            const minDistance = tempMinDistance.Copy(normalForMaxDistance);
            let minDistance2 = maxDistance * maxDistance;
            for (let i = 0; i < shape.m_count; ++i) {
                const distance = b2Vec2.Subtract(pLocal, shape.m_vertices[i], tempDistance);
                const distance2 = distance.LengthSquared();
                if (minDistance2 > distance2) {
                    minDistance.Copy(distance);
                    minDistance2 = distance2;
                }
            }

            b2Rot.MultiplyVec2(xf.q, minDistance, normal);
            normal.Normalize();
            return Math.sqrt(minDistance2);
        }
        b2Rot.MultiplyVec2(xf.q, normalForMaxDistance, normal);
        return maxDistance;
    },
    (shape: b2ChainShape, xf, p, normal, childIndex) => {
        shape.GetChildEdge(tempEdgeShape, childIndex);
        return implementations[b2ShapeType.e_edge](tempEdgeShape, xf, p, normal, 0);
    },
];

export const computeDistance = (shape: b2Shape, xf: b2Readonly<b2Transform>, p: b2Readonly<b2Vec2>, normal: b2Vec2, childIndex: number) => {
    const fn = implementations[shape.GetType()];
    return fn ? fn(shape, xf, p, normal, childIndex) : 0;
};
