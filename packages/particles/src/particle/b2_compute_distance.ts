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
        const center = Transform.multiplyVec2(xf, shape.p, tempCenter);
        Vec2.subtract(p, center, normal);
        return normal.normalize() - shape.radius;
    },
    (shape: EdgeShape, xf, p, normal) => {
        const v1 = Transform.multiplyVec2(xf, shape.vertex1, tempV1);
        const v2 = Transform.multiplyVec2(xf, shape.vertex2, tempV2);

        const d = Vec2.subtract(p, v1, tempD);
        const s = Vec2.subtract(v2, v1, tempS);
        const ds = Vec2.dot(d, s);
        if (ds > 0) {
            const s2 = Vec2.dot(s, s);
            if (ds > s2) {
                Vec2.subtract(p, v2, d);
            } else {
                d.subtractScaled(ds / s2, s);
            }
        }
        normal.copy(d);
        return normal.normalize();
    },
    (shape: PolygonShape, xf, p, normal) => {
        const pLocal = Transform.transposeMultiplyVec2(xf, p, tempLocal);
        let maxDistance = -MAX_FLOAT;
        const normalForMaxDistance = tempNormalForMaxDistance.copy(pLocal);

        for (let i = 0; i < shape.count; ++i) {
            const dot = Vec2.dot(shape.normals[i], Vec2.subtract(pLocal, shape.vertices[i], Vec2.s_t0));
            if (dot > maxDistance) {
                maxDistance = dot;
                normalForMaxDistance.copy(shape.normals[i]);
            }
        }

        if (maxDistance > 0) {
            const minDistance = tempMinDistance.copy(normalForMaxDistance);
            let minDistance2 = maxDistance * maxDistance;
            for (let i = 0; i < shape.count; ++i) {
                const distance = Vec2.subtract(pLocal, shape.vertices[i], tempDistance);
                const distance2 = distance.lengthSquared();
                if (minDistance2 > distance2) {
                    minDistance.copy(distance);
                    minDistance2 = distance2;
                }
            }

            Rot.multiplyVec2(xf.q, minDistance, normal);
            normal.normalize();
            return Math.sqrt(minDistance2);
        }
        Rot.multiplyVec2(xf.q, normalForMaxDistance, normal);
        return maxDistance;
    },
    (shape: ChainShape, xf, p, normal, childIndex) => {
        shape.getChildEdge(tempEdgeShape, childIndex);
        return implementations[ShapeType.Edge](tempEdgeShape, xf, p, normal, 0);
    },
];

export const computeDistance = (shape: Shape, xf: Transform, p: Vec2, normal: Vec2, childIndex: number) => {
    const fn = implementations[shape.getType()];
    return fn ? fn(shape, xf, p, normal, childIndex) : 0;
};
