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
    EdgeShape,
    Shape,
    Transform,
    Vec2,
    ChainShape,
    ShapeType,
    PolygonShape,
    MAX_FLOAT,
    Rot,
    CircleShape,
} from "@box2d/core";

type ComputeDistanceFn<T extends Shape> = (
    shape: T,
    xf: Transform,
    p: Vec2,
    normal: Vec2,
    childIndex: number,
) => number;

const tempEdgeShape = new EdgeShape();
const tempLocal = new Vec2();
const tempNormalForMaxDistance = new Vec2();
const tempMinDistance = new Vec2();
const tempDistance = new Vec2();
const tempV1 = new Vec2();
const tempV2 = new Vec2();
const tempD = new Vec2();
const tempS = new Vec2();
const tempCenter = new Vec2();

const implementations: Array<ComputeDistanceFn<any>> = [
    (shape: CircleShape, xf, p, normal) => {
        const center = Transform.MultiplyVec2(xf, shape.m_p, tempCenter);
        Vec2.Subtract(p, center, normal);
        return normal.Normalize() - shape.m_radius;
    },
    (shape: EdgeShape, xf, p, normal) => {
        const v1 = Transform.MultiplyVec2(xf, shape.m_vertex1, tempV1);
        const v2 = Transform.MultiplyVec2(xf, shape.m_vertex2, tempV2);

        const d = Vec2.Subtract(p, v1, tempD);
        const s = Vec2.Subtract(v2, v1, tempS);
        const ds = Vec2.Dot(d, s);
        if (ds > 0) {
            const s2 = Vec2.Dot(s, s);
            if (ds > s2) {
                Vec2.Subtract(p, v2, d);
            } else {
                d.SubtractScaled(ds / s2, s);
            }
        }
        normal.Copy(d);
        return normal.Normalize();
    },
    (shape: PolygonShape, xf, p, normal) => {
        const pLocal = Transform.TransposeMultiplyVec2(xf, p, tempLocal);
        let maxDistance = -MAX_FLOAT;
        const normalForMaxDistance = tempNormalForMaxDistance.Copy(pLocal);

        for (let i = 0; i < shape.m_count; ++i) {
            const dot = Vec2.Dot(shape.m_normals[i], Vec2.Subtract(pLocal, shape.m_vertices[i], Vec2.s_t0));
            if (dot > maxDistance) {
                maxDistance = dot;
                normalForMaxDistance.Copy(shape.m_normals[i]);
            }
        }

        if (maxDistance > 0) {
            const minDistance = tempMinDistance.Copy(normalForMaxDistance);
            let minDistance2 = maxDistance * maxDistance;
            for (let i = 0; i < shape.m_count; ++i) {
                const distance = Vec2.Subtract(pLocal, shape.m_vertices[i], tempDistance);
                const distance2 = distance.LengthSquared();
                if (minDistance2 > distance2) {
                    minDistance.Copy(distance);
                    minDistance2 = distance2;
                }
            }

            Rot.MultiplyVec2(xf.q, minDistance, normal);
            normal.Normalize();
            return Math.sqrt(minDistance2);
        }
        Rot.MultiplyVec2(xf.q, normalForMaxDistance, normal);
        return maxDistance;
    },
    (shape: ChainShape, xf, p, normal, childIndex) => {
        shape.GetChildEdge(tempEdgeShape, childIndex);
        return implementations[ShapeType.Edge](tempEdgeShape, xf, p, normal, 0);
    },
];

export const computeDistance = (shape: Shape, xf: Transform, p: Vec2, normal: Vec2, childIndex: number) => {
    const fn = implementations[shape.GetType()];
    return fn ? fn(shape, xf, p, normal, childIndex) : 0;
};
