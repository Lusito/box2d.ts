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

// DEBUG: import { b2Assert } from "../common/b2_common";
import { b2MakeArray, b2_maxFloat, b2_maxManifoldPoints } from "../common/b2_common";
import { b2Vec2, b2Rot, b2Transform } from "../common/b2_math";
import {
    b2ContactFeatureType,
    b2ContactID,
    b2Manifold,
    b2ManifoldType,
    b2ClipVertex,
    b2ClipSegmentToLine,
} from "./b2_collision";
import { b2CircleShape } from "./b2_circle_shape";
import { b2PolygonShape } from "./b2_polygon_shape";
import { b2EdgeShape } from "./b2_edge_shape";
import { b2_maxPolygonVertices } from "../common/b2_settings";

const b2CollideEdgeAndCircle_s_Q = new b2Vec2();
const b2CollideEdgeAndCircle_s_e = new b2Vec2();
const b2CollideEdgeAndCircle_s_d = new b2Vec2();
const b2CollideEdgeAndCircle_s_e1 = new b2Vec2();
const b2CollideEdgeAndCircle_s_e2 = new b2Vec2();
const b2CollideEdgeAndCircle_s_P = new b2Vec2();
const b2CollideEdgeAndCircle_s_n = new b2Vec2();
const b2CollideEdgeAndCircle_s_id = new b2ContactID();

/**
 * Compute contact points for edge versus circle.
 * This accounts for edge connectivity.
 */
export function b2CollideEdgeAndCircle(
    manifold: b2Manifold,
    edgeA: b2EdgeShape,
    xfA: b2Transform,
    circleB: b2CircleShape,
    xfB: b2Transform,
): void {
    manifold.pointCount = 0;

    // Compute circle in frame of edge
    const Q = b2Transform.TransposeMultiplyVec2(
        xfA,
        b2Transform.MultiplyVec2(xfB, circleB.m_p, b2Vec2.s_t0),
        b2CollideEdgeAndCircle_s_Q,
    );

    const A = edgeA.m_vertex1;
    const B = edgeA.m_vertex2;
    const e = b2Vec2.Subtract(B, A, b2CollideEdgeAndCircle_s_e);

    // Normal points to the right for a CCW winding
    const n = b2CollideEdgeAndCircle_s_n.Set(e.y, -e.x);
    const offset = b2Vec2.Dot(n, b2Vec2.Subtract(Q, A, b2Vec2.s_t0));

    const oneSided = edgeA.m_oneSided;
    if (oneSided && offset < 0) {
        return;
    }

    // Barycentric coordinates
    const u = b2Vec2.Dot(e, b2Vec2.Subtract(B, Q, b2Vec2.s_t0));
    const v = b2Vec2.Dot(e, b2Vec2.Subtract(Q, A, b2Vec2.s_t0));

    const radius = edgeA.m_radius + circleB.m_radius;

    const id = b2CollideEdgeAndCircle_s_id;
    id.cf.indexB = 0;
    id.cf.typeB = b2ContactFeatureType.e_vertex;

    // Region A
    if (v <= 0) {
        const P = A;
        const d = b2Vec2.Subtract(Q, P, b2CollideEdgeAndCircle_s_d);
        const dd = b2Vec2.Dot(d, d);
        if (dd > radius * radius) {
            return;
        }

        // Is there an edge connected to A?
        if (edgeA.m_oneSided) {
            const A1 = edgeA.m_vertex0;
            const B1 = A;
            const e1 = b2Vec2.Subtract(B1, A1, b2CollideEdgeAndCircle_s_e1);
            const u1 = b2Vec2.Dot(e1, b2Vec2.Subtract(B1, Q, b2Vec2.s_t0));

            // Is the circle in Region AB of the previous edge?
            if (u1 > 0) {
                return;
            }
        }

        id.cf.indexA = 0;
        id.cf.typeA = b2ContactFeatureType.e_vertex;
        manifold.pointCount = 1;
        manifold.type = b2ManifoldType.e_circles;
        manifold.localNormal.SetZero();
        manifold.localPoint.Copy(P);
        manifold.points[0].id.Copy(id);
        manifold.points[0].localPoint.Copy(circleB.m_p);
        return;
    }

    // Region B
    if (u <= 0) {
        const P = B;
        const d = b2Vec2.Subtract(Q, P, b2CollideEdgeAndCircle_s_d);
        const dd = b2Vec2.Dot(d, d);
        if (dd > radius * radius) {
            return;
        }

        // Is there an edge connected to B?
        if (edgeA.m_oneSided) {
            const B2 = edgeA.m_vertex3;
            const A2 = B;
            const e2 = b2Vec2.Subtract(B2, A2, b2CollideEdgeAndCircle_s_e2);
            const v2 = b2Vec2.Dot(e2, b2Vec2.Subtract(Q, A2, b2Vec2.s_t0));

            // Is the circle in Region AB of the next edge?
            if (v2 > 0) {
                return;
            }
        }

        id.cf.indexA = 1;
        id.cf.typeA = b2ContactFeatureType.e_vertex;
        manifold.pointCount = 1;
        manifold.type = b2ManifoldType.e_circles;
        manifold.localNormal.SetZero();
        manifold.localPoint.Copy(P);
        manifold.points[0].id.Copy(id);
        manifold.points[0].localPoint.Copy(circleB.m_p);
        return;
    }

    // Region AB
    const den = b2Vec2.Dot(e, e);
    // DEBUG: b2Assert(den > 0);
    const P = b2CollideEdgeAndCircle_s_P;
    P.x = (1 / den) * (u * A.x + v * B.x);
    P.y = (1 / den) * (u * A.y + v * B.y);
    const d = b2Vec2.Subtract(Q, P, b2CollideEdgeAndCircle_s_d);
    const dd = b2Vec2.Dot(d, d);
    if (dd > radius * radius) {
        return;
    }

    if (offset < 0) {
        n.Set(-n.x, -n.y);
    }
    n.Normalize();

    id.cf.indexA = 0;
    id.cf.typeA = b2ContactFeatureType.e_face;
    manifold.pointCount = 1;
    manifold.type = b2ManifoldType.e_faceA;
    manifold.localNormal.Copy(n);
    manifold.localPoint.Copy(A);
    manifold.points[0].id.Copy(id);
    manifold.points[0].localPoint.Copy(circleB.m_p);
}

enum b2EPAxisType {
    e_unknown,
    e_edgeA,
    e_edgeB,
}

/** This structure is used to keep track of the best separating axis. */
class b2EPAxis {
    public normal = new b2Vec2();

    public type = b2EPAxisType.e_unknown;

    public index = 0;

    public separation = 0;
}

/** This holds polygon B expressed in frame A. */
class b2TempPolygon {
    public vertices: b2Vec2[] = b2MakeArray(b2_maxPolygonVertices, b2Vec2);

    public normals: b2Vec2[] = b2MakeArray(b2_maxPolygonVertices, b2Vec2);

    public count = 0;
}

/** Reference face used for clipping */
class b2ReferenceFace {
    public i1 = 0;

    public i2 = 0;

    public readonly v1 = new b2Vec2();

    public readonly v2 = new b2Vec2();

    public readonly normal = new b2Vec2();

    public readonly sideNormal1 = new b2Vec2();

    public sideOffset1 = 0;

    public readonly sideNormal2 = new b2Vec2();

    public sideOffset2 = 0;
}

const b2ComputeEdgeSeparation_s_axis = new b2EPAxis();
const b2ComputeEdgeSeparation_s_axes = [new b2Vec2(), new b2Vec2()] as const;
function b2ComputeEdgeSeparation(
    polygonB: Readonly<b2TempPolygon>,
    v1: Readonly<b2Vec2>,
    normal1: Readonly<b2Vec2>,
): b2EPAxis {
    const axis = b2ComputeEdgeSeparation_s_axis;
    axis.type = b2EPAxisType.e_edgeA;
    axis.index = -1;
    axis.separation = -b2_maxFloat;
    axis.normal.SetZero();

    const axes = b2ComputeEdgeSeparation_s_axes;
    axes[0].Copy(normal1);
    b2Vec2.Negate(normal1, axes[1]);

    // Find axis with least overlap (min-max problem)
    for (let j = 0; j < 2; ++j) {
        let sj = b2_maxFloat;

        // Find deepest polygon vertex along axis j
        for (let i = 0; i < polygonB.count; ++i) {
            const si = b2Vec2.Dot(axes[j], b2Vec2.Subtract(polygonB.vertices[i], v1, b2Vec2.s_t0));
            if (si < sj) {
                sj = si;
            }
        }

        if (sj > axis.separation) {
            axis.index = j;
            axis.separation = sj;
            axis.normal.Copy(axes[j]);
        }
    }

    return axis;
}

const b2ComputePolygonSeparation_s_axis = new b2EPAxis();
const b2ComputePolygonSeparation_s_n = new b2Vec2();
function b2ComputePolygonSeparation(
    polygonB: Readonly<b2TempPolygon>,
    v1: Readonly<b2Vec2>,
    v2: Readonly<b2Vec2>,
): b2EPAxis {
    const axis = b2ComputePolygonSeparation_s_axis;
    axis.type = b2EPAxisType.e_unknown;
    axis.index = -1;
    axis.separation = -b2_maxFloat;
    axis.normal.SetZero();

    for (let i = 0; i < polygonB.count; ++i) {
        const n = b2Vec2.Negate(polygonB.normals[i], b2ComputePolygonSeparation_s_n);

        const s1 = b2Vec2.Dot(n, b2Vec2.Subtract(polygonB.vertices[i], v1, b2Vec2.s_t0));
        const s2 = b2Vec2.Dot(n, b2Vec2.Subtract(polygonB.vertices[i], v2, b2Vec2.s_t0));
        const s = Math.min(s1, s2);

        if (s > axis.separation) {
            axis.type = b2EPAxisType.e_edgeB;
            axis.index = i;
            axis.separation = s;
            axis.normal.Copy(n);
        }
    }

    return axis;
}

const b2CollideEdgeAndPolygon_s_xf = new b2Transform();
const b2CollideEdgeAndPolygon_s_centroidB = new b2Vec2();
const b2CollideEdgeAndPolygon_s_edge1 = new b2Vec2();
const b2CollideEdgeAndPolygon_s_normal1 = new b2Vec2();
const b2CollideEdgeAndPolygon_s_edge0 = new b2Vec2();
const b2CollideEdgeAndPolygon_s_normal0 = new b2Vec2();
const b2CollideEdgeAndPolygon_s_edge2 = new b2Vec2();
const b2CollideEdgeAndPolygon_s_normal2 = new b2Vec2();
const b2CollideEdgeAndPolygon_s_tempPolygonB = new b2TempPolygon();
const b2CollideEdgeAndPolygon_s_ref = new b2ReferenceFace();
const b2CollideEdgeAndPolygon_s_clipPoints = [new b2ClipVertex(), new b2ClipVertex()] as const;
const b2CollideEdgeAndPolygon_s_clipPoints1 = [new b2ClipVertex(), new b2ClipVertex()] as const;
const b2CollideEdgeAndPolygon_s_clipPoints2 = [new b2ClipVertex(), new b2ClipVertex()] as const;
export function b2CollideEdgeAndPolygon(
    manifold: b2Manifold,
    edgeA: b2EdgeShape,
    xfA: b2Transform,
    polygonB: b2PolygonShape,
    xfB: b2Transform,
): void {
    manifold.pointCount = 0;

    const xf = b2Transform.TransposeMultiply(xfA, xfB, b2CollideEdgeAndPolygon_s_xf);

    const centroidB = b2Transform.MultiplyVec2(xf, polygonB.m_centroid, b2CollideEdgeAndPolygon_s_centroidB);

    const v1 = edgeA.m_vertex1;
    const v2 = edgeA.m_vertex2;

    const edge1 = b2Vec2.Subtract(v2, v1, b2CollideEdgeAndPolygon_s_edge1);
    edge1.Normalize();

    // Normal points to the right for a CCW winding
    const normal1 = b2CollideEdgeAndPolygon_s_normal1.Set(edge1.y, -edge1.x);
    const offset1 = b2Vec2.Dot(normal1, b2Vec2.Subtract(centroidB, v1, b2Vec2.s_t0));

    const oneSided = edgeA.m_oneSided;
    if (oneSided && offset1 < 0) {
        return;
    }

    // Get polygonB in frameA
    const tempPolygonB = b2CollideEdgeAndPolygon_s_tempPolygonB;
    tempPolygonB.count = polygonB.m_count;
    for (let i = 0; i < polygonB.m_count; ++i) {
        b2Transform.MultiplyVec2(xf, polygonB.m_vertices[i], tempPolygonB.vertices[i]);
        b2Rot.MultiplyVec2(xf.q, polygonB.m_normals[i], tempPolygonB.normals[i]);
    }

    const radius = polygonB.m_radius + edgeA.m_radius;

    const edgeAxis = b2ComputeEdgeSeparation(tempPolygonB, v1, normal1);
    if (edgeAxis.separation > radius) {
        return;
    }

    const polygonAxis = b2ComputePolygonSeparation(tempPolygonB, v1, v2);
    if (polygonAxis.separation > radius) {
        return;
    }

    // Use hysteresis for jitter reduction.
    const k_relativeTol = 0.98;
    const k_absoluteTol = 0.001;

    // b2EPAxis primaryAxis;
    let primaryAxis: b2EPAxis;
    if (polygonAxis.separation - radius > k_relativeTol * (edgeAxis.separation - radius) + k_absoluteTol) {
        primaryAxis = polygonAxis;
    } else {
        primaryAxis = edgeAxis;
    }

    if (oneSided) {
        // Smooth collision
        // See https://box2d.org/posts/2020/06/ghost-collisions/

        const edge0 = b2Vec2.Subtract(v1, edgeA.m_vertex0, b2CollideEdgeAndPolygon_s_edge0);
        edge0.Normalize();
        const normal0 = b2CollideEdgeAndPolygon_s_normal0.Set(edge0.y, -edge0.x);
        const convex1 = b2Vec2.Cross(edge0, edge1) >= 0;

        const edge2 = b2Vec2.Subtract(edgeA.m_vertex3, v2, b2CollideEdgeAndPolygon_s_edge2);
        edge2.Normalize();
        const normal2 = b2CollideEdgeAndPolygon_s_normal2.Set(edge2.y, -edge2.x);
        const convex2 = b2Vec2.Cross(edge1, edge2) >= 0;

        const sinTol = 0.1;
        const side1 = b2Vec2.Dot(primaryAxis.normal, edge1) <= 0;

        // Check Gauss Map
        if (side1) {
            if (convex1) {
                if (b2Vec2.Cross(primaryAxis.normal, normal0) > sinTol) {
                    // Skip region
                    return;
                }

                // Admit region
            } else {
                // Snap region
                primaryAxis = edgeAxis;
            }
        } else if (convex2) {
            if (b2Vec2.Cross(normal2, primaryAxis.normal) > sinTol) {
                // Skip region
                return;
            }

            // Admit region
        } else {
            // Snap region
            primaryAxis = edgeAxis;
        }
    }

    const clipPoints = b2CollideEdgeAndPolygon_s_clipPoints;
    const ref = b2CollideEdgeAndPolygon_s_ref;
    if (primaryAxis.type === b2EPAxisType.e_edgeA) {
        manifold.type = b2ManifoldType.e_faceA;

        // Search for the polygon normal that is most anti-parallel to the edge normal.
        let bestIndex = 0;
        let bestValue = b2Vec2.Dot(primaryAxis.normal, tempPolygonB.normals[0]);
        for (let i = 1; i < tempPolygonB.count; ++i) {
            const value = b2Vec2.Dot(primaryAxis.normal, tempPolygonB.normals[i]);
            if (value < bestValue) {
                bestValue = value;
                bestIndex = i;
            }
        }

        const i1 = bestIndex;
        const i2 = i1 + 1 < tempPolygonB.count ? i1 + 1 : 0;

        clipPoints[0].v.Copy(tempPolygonB.vertices[i1]);
        clipPoints[0].id.cf.indexA = 0;
        clipPoints[0].id.cf.indexB = i1;
        clipPoints[0].id.cf.typeA = b2ContactFeatureType.e_face;
        clipPoints[0].id.cf.typeB = b2ContactFeatureType.e_vertex;

        clipPoints[1].v.Copy(tempPolygonB.vertices[i2]);
        clipPoints[1].id.cf.indexA = 0;
        clipPoints[1].id.cf.indexB = i2;
        clipPoints[1].id.cf.typeA = b2ContactFeatureType.e_face;
        clipPoints[1].id.cf.typeB = b2ContactFeatureType.e_vertex;

        ref.i1 = 0;
        ref.i2 = 1;
        ref.v1.Copy(v1);
        ref.v2.Copy(v2);
        ref.normal.Copy(primaryAxis.normal);
        b2Vec2.Negate(edge1, ref.sideNormal1);
        ref.sideNormal2.Copy(edge1);
    } else {
        manifold.type = b2ManifoldType.e_faceB;

        clipPoints[0].v.Copy(v2);
        clipPoints[0].id.cf.indexA = 1;
        clipPoints[0].id.cf.indexB = primaryAxis.index;
        clipPoints[0].id.cf.typeA = b2ContactFeatureType.e_vertex;
        clipPoints[0].id.cf.typeB = b2ContactFeatureType.e_face;

        clipPoints[1].v.Copy(v1);
        clipPoints[1].id.cf.indexA = 0;
        clipPoints[1].id.cf.indexB = primaryAxis.index;
        clipPoints[1].id.cf.typeA = b2ContactFeatureType.e_vertex;
        clipPoints[1].id.cf.typeB = b2ContactFeatureType.e_face;

        ref.i1 = primaryAxis.index;
        ref.i2 = ref.i1 + 1 < tempPolygonB.count ? ref.i1 + 1 : 0;
        ref.v1.Copy(tempPolygonB.vertices[ref.i1]);
        ref.v2.Copy(tempPolygonB.vertices[ref.i2]);
        ref.normal.Copy(tempPolygonB.normals[ref.i1]);

        // CCW winding
        ref.sideNormal1.Set(ref.normal.y, -ref.normal.x);
        b2Vec2.Negate(ref.sideNormal1, ref.sideNormal2);
    }

    ref.sideOffset1 = b2Vec2.Dot(ref.sideNormal1, ref.v1);
    ref.sideOffset2 = b2Vec2.Dot(ref.sideNormal2, ref.v2);

    // Clip incident edge against reference face side planes
    const clipPoints1 = b2CollideEdgeAndPolygon_s_clipPoints1;
    const clipPoints2 = b2CollideEdgeAndPolygon_s_clipPoints2;
    let np: number;

    // Clip to side 1
    np = b2ClipSegmentToLine(clipPoints1, clipPoints, ref.sideNormal1, ref.sideOffset1, ref.i1);

    if (np < b2_maxManifoldPoints) {
        return;
    }

    // Clip to side 2
    np = b2ClipSegmentToLine(clipPoints2, clipPoints1, ref.sideNormal2, ref.sideOffset2, ref.i2);

    if (np < b2_maxManifoldPoints) {
        return;
    }

    // Now clipPoints2 contains the clipped points.
    if (primaryAxis.type === b2EPAxisType.e_edgeA) {
        manifold.localNormal.Copy(ref.normal);
        manifold.localPoint.Copy(ref.v1);
    } else {
        manifold.localNormal.Copy(polygonB.m_normals[ref.i1]);
        manifold.localPoint.Copy(polygonB.m_vertices[ref.i1]);
    }

    let pointCount = 0;
    for (let i = 0; i < b2_maxManifoldPoints; ++i) {
        const separation = b2Vec2.Dot(ref.normal, b2Vec2.Subtract(clipPoints2[i].v, ref.v1, b2Vec2.s_t0));

        if (separation <= radius) {
            const cp = manifold.points[pointCount];

            if (primaryAxis.type === b2EPAxisType.e_edgeA) {
                b2Transform.TransposeMultiplyVec2(xf, clipPoints2[i].v, cp.localPoint);
                cp.id.Copy(clipPoints2[i].id);
            } else {
                cp.localPoint.Copy(clipPoints2[i].v);
                cp.id.cf.typeA = clipPoints2[i].id.cf.typeB;
                cp.id.cf.typeB = clipPoints2[i].id.cf.typeA;
                cp.id.cf.indexA = clipPoints2[i].id.cf.indexB;
                cp.id.cf.indexB = clipPoints2[i].id.cf.indexA;
            }

            ++pointCount;
        }
    }

    manifold.pointCount = pointCount;
}
