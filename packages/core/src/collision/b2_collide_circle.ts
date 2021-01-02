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

import { MAX_FLOAT, EPSILON } from "../common/b2_common";
import { Vec2, Transform } from "../common/b2_math";
import { Manifold, ManifoldType } from "./b2_collision";
import { CircleShape } from "./b2_circle_shape";
import { PolygonShape } from "./b2_polygon_shape";

const CollideCircles_s_pA = new Vec2();
const CollideCircles_s_pB = new Vec2();
export function collideCircles(
    manifold: Manifold,
    circleA: CircleShape,
    xfA: Transform,
    circleB: CircleShape,
    xfB: Transform,
): void {
    manifold.pointCount = 0;

    const pA = Transform.multiplyVec2(xfA, circleA.p, CollideCircles_s_pA);
    const pB = Transform.multiplyVec2(xfB, circleB.p, CollideCircles_s_pB);

    const distSqr = Vec2.distanceSquared(pA, pB);
    const radius = circleA.radius + circleB.radius;
    if (distSqr > radius * radius) {
        return;
    }

    manifold.type = ManifoldType.Circles;
    manifold.localPoint.copy(circleA.p);
    manifold.localNormal.setZero();
    manifold.pointCount = 1;

    manifold.points[0].localPoint.copy(circleB.p);
    manifold.points[0].id.cf.reset();
}

const CollidePolygonAndCircle_s_c = new Vec2();
const CollidePolygonAndCircle_s_cLocal = new Vec2();
const CollidePolygonAndCircle_s_faceCenter = new Vec2();
export function collidePolygonAndCircle(
    manifold: Manifold,
    polygonA: PolygonShape,
    xfA: Transform,
    circleB: CircleShape,
    xfB: Transform,
): void {
    manifold.pointCount = 0;

    // Compute circle position in the frame of the polygon.
    const c = Transform.multiplyVec2(xfB, circleB.p, CollidePolygonAndCircle_s_c);
    const cLocal = Transform.transposeMultiplyVec2(xfA, c, CollidePolygonAndCircle_s_cLocal);

    // Find the min separating edge.
    let normalIndex = 0;
    let separation = -MAX_FLOAT;
    const radius = polygonA.radius + circleB.radius;
    const vertexCount = polygonA.count;
    const { vertices, normals } = polygonA;

    for (let i = 0; i < vertexCount; ++i) {
        const s = Vec2.dot(normals[i], Vec2.subtract(cLocal, vertices[i], Vec2.s_t0));

        if (s > radius) {
            // Early out.
            return;
        }

        if (s > separation) {
            separation = s;
            normalIndex = i;
        }
    }

    // Vertices that subtend the incident face.
    const vertIndex1 = normalIndex;
    const vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
    const v1 = vertices[vertIndex1];
    const v2 = vertices[vertIndex2];

    // If the center is inside the polygon ...
    if (separation < EPSILON) {
        manifold.pointCount = 1;
        manifold.type = ManifoldType.FaceA;
        manifold.localNormal.copy(normals[normalIndex]);
        Vec2.mid(v1, v2, manifold.localPoint);
        manifold.points[0].localPoint.copy(circleB.p);
        manifold.points[0].id.cf.reset();
        return;
    }

    // Compute barycentric coordinates
    const u1 = Vec2.dot(Vec2.subtract(cLocal, v1, Vec2.s_t0), Vec2.subtract(v2, v1, Vec2.s_t1));
    const u2 = Vec2.dot(Vec2.subtract(cLocal, v2, Vec2.s_t0), Vec2.subtract(v1, v2, Vec2.s_t1));
    if (u1 <= 0) {
        if (Vec2.distanceSquared(cLocal, v1) > radius * radius) {
            return;
        }

        manifold.pointCount = 1;
        manifold.type = ManifoldType.FaceA;
        Vec2.subtract(cLocal, v1, manifold.localNormal).normalize();
        manifold.localPoint.copy(v1);
        manifold.points[0].localPoint.copy(circleB.p);
        manifold.points[0].id.cf.reset();
    } else if (u2 <= 0) {
        if (Vec2.distanceSquared(cLocal, v2) > radius * radius) {
            return;
        }

        manifold.pointCount = 1;
        manifold.type = ManifoldType.FaceA;
        Vec2.subtract(cLocal, v2, manifold.localNormal).normalize();
        manifold.localPoint.copy(v2);
        manifold.points[0].localPoint.copy(circleB.p);
        manifold.points[0].id.cf.reset();
    } else {
        const faceCenter = Vec2.mid(v1, v2, CollidePolygonAndCircle_s_faceCenter);
        const separation2 = Vec2.dot(Vec2.subtract(cLocal, faceCenter, Vec2.s_t1), normals[vertIndex1]);
        if (separation2 > radius) {
            return;
        }

        manifold.pointCount = 1;
        manifold.type = ManifoldType.FaceA;
        manifold.localNormal.copy(normals[vertIndex1]);
        manifold.localPoint.copy(faceCenter);
        manifold.points[0].localPoint.copy(circleB.p);
        manifold.points[0].id.cf.reset();
    }
}
