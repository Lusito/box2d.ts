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

// DEBUG: import { assert } from "../common/b2_common";
import { makeArray, MAX_FLOAT, MAX_MANIFOLD_POINTS } from "../common/b2_common";
import { Vec2, Rot, Transform } from "../common/b2_math";
import { ContactFeatureType, ContactID, Manifold, ManifoldType, ClipVertex, clipSegmentToLine } from "./b2_collision";
import { CircleShape } from "./b2_circle_shape";
import { PolygonShape } from "./b2_polygon_shape";
import { EdgeShape } from "./b2_edge_shape";
import { MAX_POLYGON_VERTICES } from "../common/b2_settings";

const CollideEdgeAndCircle_s_Q = new Vec2();
const CollideEdgeAndCircle_s_e = new Vec2();
const CollideEdgeAndCircle_s_d = new Vec2();
const CollideEdgeAndCircle_s_e1 = new Vec2();
const CollideEdgeAndCircle_s_e2 = new Vec2();
const CollideEdgeAndCircle_s_P = new Vec2();
const CollideEdgeAndCircle_s_n = new Vec2();
const CollideEdgeAndCircle_s_id = new ContactID();
export function collideEdgeAndCircle(
    manifold: Manifold,
    edgeA: EdgeShape,
    xfA: Transform,
    circleB: CircleShape,
    xfB: Transform,
): void {
    manifold.pointCount = 0;

    // Compute circle in frame of edge
    const Q = Transform.transposeMultiplyVec2(
        xfA,
        Transform.multiplyVec2(xfB, circleB.m_p, Vec2.s_t0),
        CollideEdgeAndCircle_s_Q,
    );

    const A = edgeA.m_vertex1;
    const B = edgeA.m_vertex2;
    const e = Vec2.subtract(B, A, CollideEdgeAndCircle_s_e);

    // Normal points to the right for a CCW winding
    const n = CollideEdgeAndCircle_s_n.set(e.y, -e.x);
    const offset = Vec2.dot(n, Vec2.subtract(Q, A, Vec2.s_t0));

    const oneSided = edgeA.m_oneSided;
    if (oneSided && offset < 0) {
        return;
    }

    // Barycentric coordinates
    const u = Vec2.dot(e, Vec2.subtract(B, Q, Vec2.s_t0));
    const v = Vec2.dot(e, Vec2.subtract(Q, A, Vec2.s_t0));

    const radius = edgeA.m_radius + circleB.m_radius;

    const id = CollideEdgeAndCircle_s_id;
    id.cf.indexB = 0;
    id.cf.typeB = ContactFeatureType.Vertex;

    // Region A
    if (v <= 0) {
        const P = A;
        const d = Vec2.subtract(Q, P, CollideEdgeAndCircle_s_d);
        const dd = Vec2.dot(d, d);
        if (dd > radius * radius) {
            return;
        }

        // Is there an edge connected to A?
        if (edgeA.m_oneSided) {
            const A1 = edgeA.m_vertex0;
            const B1 = A;
            const e1 = Vec2.subtract(B1, A1, CollideEdgeAndCircle_s_e1);
            const u1 = Vec2.dot(e1, Vec2.subtract(B1, Q, Vec2.s_t0));

            // Is the circle in Region AB of the previous edge?
            if (u1 > 0) {
                return;
            }
        }

        id.cf.indexA = 0;
        id.cf.typeA = ContactFeatureType.Vertex;
        manifold.pointCount = 1;
        manifold.type = ManifoldType.Circles;
        manifold.localNormal.setZero();
        manifold.localPoint.copy(P);
        manifold.points[0].id.copy(id);
        manifold.points[0].localPoint.copy(circleB.m_p);
        return;
    }

    // Region B
    if (u <= 0) {
        const P = B;
        const d = Vec2.subtract(Q, P, CollideEdgeAndCircle_s_d);
        const dd = Vec2.dot(d, d);
        if (dd > radius * radius) {
            return;
        }

        // Is there an edge connected to B?
        if (edgeA.m_oneSided) {
            const B2 = edgeA.m_vertex3;
            const A2 = B;
            const e2 = Vec2.subtract(B2, A2, CollideEdgeAndCircle_s_e2);
            const v2 = Vec2.dot(e2, Vec2.subtract(Q, A2, Vec2.s_t0));

            // Is the circle in Region AB of the next edge?
            if (v2 > 0) {
                return;
            }
        }

        id.cf.indexA = 1;
        id.cf.typeA = ContactFeatureType.Vertex;
        manifold.pointCount = 1;
        manifold.type = ManifoldType.Circles;
        manifold.localNormal.setZero();
        manifold.localPoint.copy(P);
        manifold.points[0].id.copy(id);
        manifold.points[0].localPoint.copy(circleB.m_p);
        return;
    }

    // Region AB
    const den = Vec2.dot(e, e);
    // DEBUG: assert(den > 0);
    const P = CollideEdgeAndCircle_s_P;
    P.x = (1 / den) * (u * A.x + v * B.x);
    P.y = (1 / den) * (u * A.y + v * B.y);
    const d = Vec2.subtract(Q, P, CollideEdgeAndCircle_s_d);
    const dd = Vec2.dot(d, d);
    if (dd > radius * radius) {
        return;
    }

    if (offset < 0) {
        n.set(-n.x, -n.y);
    }
    n.normalize();

    id.cf.indexA = 0;
    id.cf.typeA = ContactFeatureType.Face;
    manifold.pointCount = 1;
    manifold.type = ManifoldType.FaceA;
    manifold.localNormal.copy(n);
    manifold.localPoint.copy(A);
    manifold.points[0].id.copy(id);
    manifold.points[0].localPoint.copy(circleB.m_p);
}

enum EPAxisType {
    Unknown,
    EdgeA,
    EdgeB,
}

/** This structure is used to keep track of the best separating axis. */
class EPAxis {
    public normal = new Vec2();

    public type = EPAxisType.Unknown;

    public index = 0;

    public separation = 0;
}

/** This holds polygon B expressed in frame A. */
class TempPolygon {
    public vertices: Vec2[] = makeArray(MAX_POLYGON_VERTICES, Vec2);

    public normals: Vec2[] = makeArray(MAX_POLYGON_VERTICES, Vec2);

    public count = 0;
}

/** Reference face used for clipping */
class ReferenceFace {
    public i1 = 0;

    public i2 = 0;

    public readonly v1 = new Vec2();

    public readonly v2 = new Vec2();

    public readonly normal = new Vec2();

    public readonly sideNormal1 = new Vec2();

    public sideOffset1 = 0;

    public readonly sideNormal2 = new Vec2();

    public sideOffset2 = 0;
}

const ComputeEdgeSeparation_s_axis = new EPAxis();
const ComputeEdgeSeparation_s_axes = [new Vec2(), new Vec2()] as const;
function computeEdgeSeparation(polygonB: Readonly<TempPolygon>, v1: Readonly<Vec2>, normal1: Readonly<Vec2>): EPAxis {
    const axis = ComputeEdgeSeparation_s_axis;
    axis.type = EPAxisType.EdgeA;
    axis.index = -1;
    axis.separation = -MAX_FLOAT;
    axis.normal.setZero();

    const axes = ComputeEdgeSeparation_s_axes;
    axes[0].copy(normal1);
    Vec2.negate(normal1, axes[1]);

    // Find axis with least overlap (min-max problem)
    for (let j = 0; j < 2; ++j) {
        let sj = MAX_FLOAT;

        // Find deepest polygon vertex along axis j
        for (let i = 0; i < polygonB.count; ++i) {
            const si = Vec2.dot(axes[j], Vec2.subtract(polygonB.vertices[i], v1, Vec2.s_t0));
            if (si < sj) {
                sj = si;
            }
        }

        if (sj > axis.separation) {
            axis.index = j;
            axis.separation = sj;
            axis.normal.copy(axes[j]);
        }
    }

    return axis;
}

const ComputePolygonSeparation_s_axis = new EPAxis();
const ComputePolygonSeparation_s_n = new Vec2();
function computePolygonSeparation(polygonB: Readonly<TempPolygon>, v1: Readonly<Vec2>, v2: Readonly<Vec2>): EPAxis {
    const axis = ComputePolygonSeparation_s_axis;
    axis.type = EPAxisType.Unknown;
    axis.index = -1;
    axis.separation = -MAX_FLOAT;
    axis.normal.setZero();

    for (let i = 0; i < polygonB.count; ++i) {
        const n = Vec2.negate(polygonB.normals[i], ComputePolygonSeparation_s_n);

        const s1 = Vec2.dot(n, Vec2.subtract(polygonB.vertices[i], v1, Vec2.s_t0));
        const s2 = Vec2.dot(n, Vec2.subtract(polygonB.vertices[i], v2, Vec2.s_t0));
        const s = Math.min(s1, s2);

        if (s > axis.separation) {
            axis.type = EPAxisType.EdgeB;
            axis.index = i;
            axis.separation = s;
            axis.normal.copy(n);
        }
    }

    return axis;
}

const CollideEdgeAndPolygon_s_xf = new Transform();
const CollideEdgeAndPolygon_s_centroidB = new Vec2();
const CollideEdgeAndPolygon_s_edge1 = new Vec2();
const CollideEdgeAndPolygon_s_normal1 = new Vec2();
const CollideEdgeAndPolygon_s_edge0 = new Vec2();
const CollideEdgeAndPolygon_s_normal0 = new Vec2();
const CollideEdgeAndPolygon_s_edge2 = new Vec2();
const CollideEdgeAndPolygon_s_normal2 = new Vec2();
const CollideEdgeAndPolygon_s_tempPolygonB = new TempPolygon();
const CollideEdgeAndPolygon_s_ref = new ReferenceFace();
const CollideEdgeAndPolygon_s_clipPoints = [new ClipVertex(), new ClipVertex()] as const;
const CollideEdgeAndPolygon_s_clipPoints1 = [new ClipVertex(), new ClipVertex()] as const;
const CollideEdgeAndPolygon_s_clipPoints2 = [new ClipVertex(), new ClipVertex()] as const;
export function collideEdgeAndPolygon(
    manifold: Manifold,
    edgeA: EdgeShape,
    xfA: Transform,
    polygonB: PolygonShape,
    xfB: Transform,
): void {
    manifold.pointCount = 0;

    const xf = Transform.transposeMultiply(xfA, xfB, CollideEdgeAndPolygon_s_xf);

    const centroidB = Transform.multiplyVec2(xf, polygonB.m_centroid, CollideEdgeAndPolygon_s_centroidB);

    const v1 = edgeA.m_vertex1;
    const v2 = edgeA.m_vertex2;

    const edge1 = Vec2.subtract(v2, v1, CollideEdgeAndPolygon_s_edge1);
    edge1.normalize();

    // Normal points to the right for a CCW winding
    const normal1 = CollideEdgeAndPolygon_s_normal1.set(edge1.y, -edge1.x);
    const offset1 = Vec2.dot(normal1, Vec2.subtract(centroidB, v1, Vec2.s_t0));

    const oneSided = edgeA.m_oneSided;
    if (oneSided && offset1 < 0) {
        return;
    }

    // Get polygonB in frameA
    const tempPolygonB = CollideEdgeAndPolygon_s_tempPolygonB;
    tempPolygonB.count = polygonB.m_count;
    for (let i = 0; i < polygonB.m_count; ++i) {
        Transform.multiplyVec2(xf, polygonB.m_vertices[i], tempPolygonB.vertices[i]);
        Rot.multiplyVec2(xf.q, polygonB.m_normals[i], tempPolygonB.normals[i]);
    }

    const radius = polygonB.m_radius + edgeA.m_radius;

    const edgeAxis = computeEdgeSeparation(tempPolygonB, v1, normal1);
    if (edgeAxis.separation > radius) {
        return;
    }

    const polygonAxis = computePolygonSeparation(tempPolygonB, v1, v2);
    if (polygonAxis.separation > radius) {
        return;
    }

    // Use hysteresis for jitter reduction.
    const k_relativeTol = 0.98;
    const k_absoluteTol = 0.001;

    // EPAxis primaryAxis;
    let primaryAxis: EPAxis;
    if (polygonAxis.separation - radius > k_relativeTol * (edgeAxis.separation - radius) + k_absoluteTol) {
        primaryAxis = polygonAxis;
    } else {
        primaryAxis = edgeAxis;
    }

    if (oneSided) {
        // Smooth collision
        // See https://box2d.org/posts/2020/06/ghost-collisions/

        const edge0 = Vec2.subtract(v1, edgeA.m_vertex0, CollideEdgeAndPolygon_s_edge0);
        edge0.normalize();
        const normal0 = CollideEdgeAndPolygon_s_normal0.set(edge0.y, -edge0.x);
        const convex1 = Vec2.cross(edge0, edge1) >= 0;

        const edge2 = Vec2.subtract(edgeA.m_vertex3, v2, CollideEdgeAndPolygon_s_edge2);
        edge2.normalize();
        const normal2 = CollideEdgeAndPolygon_s_normal2.set(edge2.y, -edge2.x);
        const convex2 = Vec2.cross(edge1, edge2) >= 0;

        const sinTol = 0.1;
        const side1 = Vec2.dot(primaryAxis.normal, edge1) <= 0;

        // Check Gauss Map
        if (side1) {
            if (convex1) {
                if (Vec2.cross(primaryAxis.normal, normal0) > sinTol) {
                    // Skip region
                    return;
                }

                // Admit region
            } else {
                // Snap region
                primaryAxis = edgeAxis;
            }
        } else if (convex2) {
            if (Vec2.cross(normal2, primaryAxis.normal) > sinTol) {
                // Skip region
                return;
            }

            // Admit region
        } else {
            // Snap region
            primaryAxis = edgeAxis;
        }
    }

    const clipPoints = CollideEdgeAndPolygon_s_clipPoints;
    const ref = CollideEdgeAndPolygon_s_ref;
    if (primaryAxis.type === EPAxisType.EdgeA) {
        manifold.type = ManifoldType.FaceA;

        // Search for the polygon normal that is most anti-parallel to the edge normal.
        let bestIndex = 0;
        let bestValue = Vec2.dot(primaryAxis.normal, tempPolygonB.normals[0]);
        for (let i = 1; i < tempPolygonB.count; ++i) {
            const value = Vec2.dot(primaryAxis.normal, tempPolygonB.normals[i]);
            if (value < bestValue) {
                bestValue = value;
                bestIndex = i;
            }
        }

        const i1 = bestIndex;
        const i2 = i1 + 1 < tempPolygonB.count ? i1 + 1 : 0;

        clipPoints[0].v.copy(tempPolygonB.vertices[i1]);
        clipPoints[0].id.cf.indexA = 0;
        clipPoints[0].id.cf.indexB = i1;
        clipPoints[0].id.cf.typeA = ContactFeatureType.Face;
        clipPoints[0].id.cf.typeB = ContactFeatureType.Vertex;

        clipPoints[1].v.copy(tempPolygonB.vertices[i2]);
        clipPoints[1].id.cf.indexA = 0;
        clipPoints[1].id.cf.indexB = i2;
        clipPoints[1].id.cf.typeA = ContactFeatureType.Face;
        clipPoints[1].id.cf.typeB = ContactFeatureType.Vertex;

        ref.i1 = 0;
        ref.i2 = 1;
        ref.v1.copy(v1);
        ref.v2.copy(v2);
        ref.normal.copy(primaryAxis.normal);
        Vec2.negate(edge1, ref.sideNormal1);
        ref.sideNormal2.copy(edge1);
    } else {
        manifold.type = ManifoldType.FaceB;

        clipPoints[0].v.copy(v2);
        clipPoints[0].id.cf.indexA = 1;
        clipPoints[0].id.cf.indexB = primaryAxis.index;
        clipPoints[0].id.cf.typeA = ContactFeatureType.Vertex;
        clipPoints[0].id.cf.typeB = ContactFeatureType.Face;

        clipPoints[1].v.copy(v1);
        clipPoints[1].id.cf.indexA = 0;
        clipPoints[1].id.cf.indexB = primaryAxis.index;
        clipPoints[1].id.cf.typeA = ContactFeatureType.Vertex;
        clipPoints[1].id.cf.typeB = ContactFeatureType.Face;

        ref.i1 = primaryAxis.index;
        ref.i2 = ref.i1 + 1 < tempPolygonB.count ? ref.i1 + 1 : 0;
        ref.v1.copy(tempPolygonB.vertices[ref.i1]);
        ref.v2.copy(tempPolygonB.vertices[ref.i2]);
        ref.normal.copy(tempPolygonB.normals[ref.i1]);

        // CCW winding
        ref.sideNormal1.set(ref.normal.y, -ref.normal.x);
        Vec2.negate(ref.sideNormal1, ref.sideNormal2);
    }

    ref.sideOffset1 = Vec2.dot(ref.sideNormal1, ref.v1);
    ref.sideOffset2 = Vec2.dot(ref.sideNormal2, ref.v2);

    // Clip incident edge against reference face side planes
    const clipPoints1 = CollideEdgeAndPolygon_s_clipPoints1;
    const clipPoints2 = CollideEdgeAndPolygon_s_clipPoints2;
    let np: number;

    // Clip to side 1
    np = clipSegmentToLine(clipPoints1, clipPoints, ref.sideNormal1, ref.sideOffset1, ref.i1);

    if (np < MAX_MANIFOLD_POINTS) {
        return;
    }

    // Clip to side 2
    np = clipSegmentToLine(clipPoints2, clipPoints1, ref.sideNormal2, ref.sideOffset2, ref.i2);

    if (np < MAX_MANIFOLD_POINTS) {
        return;
    }

    // Now clipPoints2 contains the clipped points.
    if (primaryAxis.type === EPAxisType.EdgeA) {
        manifold.localNormal.copy(ref.normal);
        manifold.localPoint.copy(ref.v1);
    } else {
        manifold.localNormal.copy(polygonB.m_normals[ref.i1]);
        manifold.localPoint.copy(polygonB.m_vertices[ref.i1]);
    }

    let pointCount = 0;
    for (let i = 0; i < MAX_MANIFOLD_POINTS; ++i) {
        const separation = Vec2.dot(ref.normal, Vec2.subtract(clipPoints2[i].v, ref.v1, Vec2.s_t0));

        if (separation <= radius) {
            const cp = manifold.points[pointCount];

            if (primaryAxis.type === EPAxisType.EdgeA) {
                Transform.transposeMultiplyVec2(xf, clipPoints2[i].v, cp.localPoint);
                cp.id.copy(clipPoints2[i].id);
            } else {
                cp.localPoint.copy(clipPoints2[i].v);
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
