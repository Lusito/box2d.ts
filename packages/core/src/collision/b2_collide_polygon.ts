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

import { LINEAR_SLOP, MAX_FLOAT, MAX_MANIFOLD_POINTS } from "../common/b2_common";
import { Vec2, Transform, Rot } from "../common/b2_math";
import { ContactFeatureType, Manifold, ManifoldType, ClipVertex, ClipSegmentToLine } from "./b2_collision";
import { PolygonShape } from "./b2_polygon_shape";

const FindMaxSeparation_s_xf = new Transform();
const FindMaxSeparation_s_n = new Vec2();
const FindMaxSeparation_s_v1 = new Vec2();

/** Find the max separation between poly1 and poly2 using edge normals from poly1. */
function FindMaxSeparation(
    edgeIndex: [number],
    poly1: PolygonShape,
    xf1: Transform,
    poly2: PolygonShape,
    xf2: Transform,
) {
    const count1 = poly1.m_count;
    const count2 = poly2.m_count;
    const n1s = poly1.m_normals;
    const v1s = poly1.m_vertices;
    const v2s = poly2.m_vertices;
    const xf = Transform.TransposeMultiply(xf2, xf1, FindMaxSeparation_s_xf);

    let bestIndex = 0;
    let maxSeparation = -MAX_FLOAT;

    for (let i = 0; i < count1; ++i) {
        // Get poly1 normal in frame2.
        const n = Rot.MultiplyVec2(xf.q, n1s[i], FindMaxSeparation_s_n);
        const v1 = Transform.MultiplyVec2(xf, v1s[i], FindMaxSeparation_s_v1);

        // Find deepest point for normal i.
        let si = MAX_FLOAT;
        for (let j = 0; j < count2; ++j) {
            const sij = Vec2.Dot(n, Vec2.Subtract(v2s[j], v1, Vec2.s_t0));
            if (sij < si) {
                si = sij;
            }
        }

        if (si > maxSeparation) {
            maxSeparation = si;
            bestIndex = i;
        }
    }

    edgeIndex[0] = bestIndex;

    return maxSeparation;
}

const FindIncidentEdge_s_normal1 = new Vec2();
function FindIncidentEdge(
    c: readonly [ClipVertex, ClipVertex],
    poly1: PolygonShape,
    xf1: Transform,
    edge1: number,
    poly2: PolygonShape,
    xf2: Transform,
): void {
    const normals1 = poly1.m_normals;

    const count2 = poly2.m_count;
    const vertices2 = poly2.m_vertices;
    const normals2 = poly2.m_normals;

    // DEBUG: Assert(0 <= edge1 && edge1 < poly1.m_count);

    // Get the normal of the reference edge in poly2's frame.
    const normal1 = Rot.TransposeMultiplyVec2(
        xf2.q,
        Rot.MultiplyVec2(xf1.q, normals1[edge1], Vec2.s_t0),
        FindIncidentEdge_s_normal1,
    );

    // Find the incident edge on poly2.
    let index = 0;
    let minDot = MAX_FLOAT;
    for (let i = 0; i < count2; ++i) {
        const dot = Vec2.Dot(normal1, normals2[i]);
        if (dot < minDot) {
            minDot = dot;
            index = i;
        }
    }

    // Build the clip vertices for the incident edge.
    const i1 = index;
    const i2 = i1 + 1 < count2 ? i1 + 1 : 0;

    const c0 = c[0];
    Transform.MultiplyVec2(xf2, vertices2[i1], c0.v);
    const cf0 = c0.id.cf;
    cf0.indexA = edge1;
    cf0.indexB = i1;
    cf0.typeA = ContactFeatureType.Face;
    cf0.typeB = ContactFeatureType.Vertex;

    const c1 = c[1];
    Transform.MultiplyVec2(xf2, vertices2[i2], c1.v);
    const cf1 = c1.id.cf;
    cf1.indexA = edge1;
    cf1.indexB = i2;
    cf1.typeA = ContactFeatureType.Face;
    cf1.typeB = ContactFeatureType.Vertex;
}

const CollidePolygons_s_incidentEdge = [new ClipVertex(), new ClipVertex()] as const;
const CollidePolygons_s_clipPoints1 = [new ClipVertex(), new ClipVertex()] as const;
const CollidePolygons_s_clipPoints2 = [new ClipVertex(), new ClipVertex()] as const;
const CollidePolygons_s_edgeA: [number] = [0];
const CollidePolygons_s_edgeB: [number] = [0];
const CollidePolygons_s_localTangent = new Vec2();
const CollidePolygons_s_localNormal = new Vec2();
const CollidePolygons_s_planePoint = new Vec2();
const CollidePolygons_s_normal = new Vec2();
const CollidePolygons_s_tangent = new Vec2();
const CollidePolygons_s_ntangent = new Vec2();
const CollidePolygons_s_v11 = new Vec2();
const CollidePolygons_s_v12 = new Vec2();

/**
 * Find edge normal of max separation on A - return if separating axis is found
 * Find edge normal of max separation on B - return if separation axis is found
 * Choose reference edge as min(minA, minB)
 * Find incident edge
 * Clip

 * The normal points from 1 to 2
 */
export function CollidePolygons(
    manifold: Manifold,
    polyA: PolygonShape,
    xfA: Transform,
    polyB: PolygonShape,
    xfB: Transform,
): void {
    manifold.pointCount = 0;
    const totalRadius = polyA.m_radius + polyB.m_radius;

    const edgeIndexA = CollidePolygons_s_edgeA;
    const separationA = FindMaxSeparation(edgeIndexA, polyA, xfA, polyB, xfB);
    if (separationA > totalRadius) {
        return;
    }

    const edgeIndexB = CollidePolygons_s_edgeB;
    const separationB = FindMaxSeparation(edgeIndexB, polyB, xfB, polyA, xfA);
    if (separationB > totalRadius) {
        return;
    }

    let poly1: PolygonShape; // reference polygon
    let poly2: PolygonShape; // incident polygon
    let xf1: Transform;
    let xf2: Transform;
    let edge1: number; // reference edge
    let flip: number;
    const k_tol = 0.1 * LINEAR_SLOP;

    if (separationB > separationA + k_tol) {
        poly1 = polyB;
        poly2 = polyA;
        xf1 = xfB;
        xf2 = xfA;
        // eslint-disable-next-line prefer-destructuring
        edge1 = edgeIndexB[0];
        manifold.type = ManifoldType.FaceB;
        flip = 1;
    } else {
        poly1 = polyA;
        poly2 = polyB;
        xf1 = xfA;
        xf2 = xfB;
        // eslint-disable-next-line prefer-destructuring
        edge1 = edgeIndexA[0];
        manifold.type = ManifoldType.FaceA;
        flip = 0;
    }

    const incidentEdge = CollidePolygons_s_incidentEdge;
    FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

    const count1 = poly1.m_count;
    const vertices1 = poly1.m_vertices;

    const iv1 = edge1;
    const iv2 = edge1 + 1 < count1 ? edge1 + 1 : 0;

    let v11 = vertices1[iv1];
    let v12 = vertices1[iv2];

    const localTangent = Vec2.Subtract(v12, v11, CollidePolygons_s_localTangent);
    localTangent.Normalize();

    const localNormal = Vec2.CrossVec2One(localTangent, CollidePolygons_s_localNormal);
    const planePoint = Vec2.Mid(v11, v12, CollidePolygons_s_planePoint);

    const tangent = Rot.MultiplyVec2(xf1.q, localTangent, CollidePolygons_s_tangent);
    const normal = Vec2.CrossVec2One(tangent, CollidePolygons_s_normal);

    v11 = Transform.MultiplyVec2(xf1, v11, CollidePolygons_s_v11);
    v12 = Transform.MultiplyVec2(xf1, v12, CollidePolygons_s_v12);

    // Face offset.
    const frontOffset = Vec2.Dot(normal, v11);

    // Side offsets, extended by polytope skin thickness.
    const sideOffset1 = -Vec2.Dot(tangent, v11) + totalRadius;
    const sideOffset2 = Vec2.Dot(tangent, v12) + totalRadius;

    // Clip incident edge against extruded edge1 side edges.
    const clipPoints1 = CollidePolygons_s_clipPoints1;
    const clipPoints2 = CollidePolygons_s_clipPoints2;

    // Clip to box side 1
    const ntangent = Vec2.Negate(tangent, CollidePolygons_s_ntangent);
    let np = ClipSegmentToLine(clipPoints1, incidentEdge, ntangent, sideOffset1, iv1);

    if (np < 2) {
        return;
    }

    // Clip to negative box side 1
    np = ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2, iv2);

    if (np < 2) {
        return;
    }

    // Now clipPoints2 contains the clipped points.
    manifold.localNormal.Copy(localNormal);
    manifold.localPoint.Copy(planePoint);

    let pointCount = 0;
    for (let i = 0; i < MAX_MANIFOLD_POINTS; ++i) {
        const cv = clipPoints2[i];
        const separation = Vec2.Dot(normal, cv.v) - frontOffset;

        if (separation <= totalRadius) {
            const cp = manifold.points[pointCount];
            Transform.TransposeMultiplyVec2(xf2, cv.v, cp.localPoint);
            cp.id.Copy(cv.id);
            if (flip) {
                // Swap features
                const { cf } = cp.id;
                cf.indexA = cf.indexB;
                cf.indexB = cf.indexA;
                cf.typeA = cf.typeB;
                cf.typeB = cf.typeA;
            }
            ++pointCount;
        }
    }

    manifold.pointCount = pointCount;
}
