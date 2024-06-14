import {
    b2Vec2,
    b2Transform,
    b2Rot,
    b2PolygonShape,
    b2_epsilon,
    b2MassData,
    b2ShapeType,
    b2Shape,
    b2CircleShape,
    b2Readonly,
} from "@box2d/core";

const ComputeSubmergedArea_s_normalL = new b2Vec2();
const ComputeSubmergedArea_s_md = new b2MassData();
const ComputeSubmergedArea_s_intoVec = new b2Vec2();
const ComputeSubmergedArea_s_outoVec = new b2Vec2();
const ComputeSubmergedArea_s_center = new b2Vec2();

function submergedAreaForPolygon(shape: b2Shape, normal: b2Vec2, offset: number, xf: b2Readonly<b2Transform>, c: b2Vec2): number {
    const polygon = shape as b2PolygonShape;

    // Transform plane into shape co-ordinates
    const normalL = b2Rot.TransposeMultiplyVec2(xf.q, normal, ComputeSubmergedArea_s_normalL);
    const offsetL = offset - b2Vec2.Dot(normal, xf.p);

    const depths: number[] = [];
    let diveCount = 0;
    let intoIndex = -1;
    let outoIndex = -1;

    let lastSubmerged = false;
    for (let i = 0; i < polygon.m_count; ++i) {
        depths[i] = b2Vec2.Dot(normalL, polygon.m_vertices[i]) - offsetL;
        const isSubmerged = depths[i] < -b2_epsilon;
        if (i > 0) {
            if (isSubmerged) {
                if (!lastSubmerged) {
                    intoIndex = i - 1;
                    diveCount++;
                }
            } else if (lastSubmerged) {
                outoIndex = i - 1;
                diveCount++;
            }
        }
        lastSubmerged = isSubmerged;
    }
    switch (diveCount) {
        case 0:
            if (lastSubmerged) {
                // Completely submerged
                const md = ComputeSubmergedArea_s_md;
                polygon.ComputeMass(md, 1);
                b2Transform.MultiplyVec2(xf, md.center, c);
                return md.mass;
            }
            // Completely dry
            return 0;

        case 1:
            if (intoIndex === -1) {
                intoIndex = polygon.m_count - 1;
            } else {
                outoIndex = polygon.m_count - 1;
            }
            break;
    }
    const intoIndex2 = (intoIndex + 1) % polygon.m_count;
    const outoIndex2 = (outoIndex + 1) % polygon.m_count;
    const intoLamdda = (0 - depths[intoIndex]) / (depths[intoIndex2] - depths[intoIndex]);
    const outoLamdda = (0 - depths[outoIndex]) / (depths[outoIndex2] - depths[outoIndex]);

    const intoVec = ComputeSubmergedArea_s_intoVec.Set(
        polygon.m_vertices[intoIndex].x * (1 - intoLamdda) + polygon.m_vertices[intoIndex2].x * intoLamdda,
        polygon.m_vertices[intoIndex].y * (1 - intoLamdda) + polygon.m_vertices[intoIndex2].y * intoLamdda,
    );
    const outoVec = ComputeSubmergedArea_s_outoVec.Set(
        polygon.m_vertices[outoIndex].x * (1 - outoLamdda) + polygon.m_vertices[outoIndex2].x * outoLamdda,
        polygon.m_vertices[outoIndex].y * (1 - outoLamdda) + polygon.m_vertices[outoIndex2].y * outoLamdda,
    );

    // Initialize accumulator
    let area = 0;
    const center = ComputeSubmergedArea_s_center.SetZero();
    let p2 = polygon.m_vertices[intoIndex2];
    let p3: b2Vec2;

    // An awkward loop from intoIndex2+1 to outIndex2
    let i = intoIndex2;
    while (i !== outoIndex2) {
        i = (i + 1) % polygon.m_count;
        if (i === outoIndex2) {
            p3 = outoVec;
        } else {
            p3 = polygon.m_vertices[i];
        }

        const triangleArea = 0.5 * ((p2.x - intoVec.x) * (p3.y - intoVec.y) - (p2.y - intoVec.y) * (p3.x - intoVec.x));
        area += triangleArea;
        // Area weighted centroid
        center.x += (triangleArea * (intoVec.x + p2.x + p3.x)) / 3;
        center.y += (triangleArea * (intoVec.y + p2.y + p3.y)) / 3;

        p2 = p3;
    }

    // Normalize and transform centroid
    center.Scale(1 / area);
    b2Transform.MultiplyVec2(xf, center, c);

    return area;
}

function submergedAreaForEdge(shape: b2Shape, _normal: b2Vec2, _offset: number, _xf: b2Readonly<b2Transform>, c: b2Vec2): number {
    c.SetZero();
    return 0;
}

function submergedAreaForChain(shape: b2Shape, _normal: b2Vec2, _offset: number, _xf: b2Readonly<b2Transform>, c: b2Vec2): number {
    c.SetZero();
    return 0;
}

function submergedAreaForCircle(shape: b2Shape, normal: b2Vec2, offset: number, xf: b2Readonly<b2Transform>, c: b2Vec2): number {
    const circle = shape as b2CircleShape;
    const p = b2Transform.MultiplyVec2(xf, circle.m_p, new b2Vec2());
    const l = -(b2Vec2.Dot(normal, p) - offset);

    if (l < -circle.m_radius + b2_epsilon) {
        // Completely dry
        return 0;
    }
    if (l > circle.m_radius) {
        // Completely wet
        c.Copy(p);
        return Math.PI * circle.m_radius * circle.m_radius;
    }

    // Magic
    const r2 = circle.m_radius * circle.m_radius;
    const l2 = l * l;
    const area = r2 * (Math.asin(l / circle.m_radius) + Math.PI / 2) + l * Math.sqrt(r2 - l2);
    const com = ((-2 / 3) * (r2 - l2) ** 1.5) / area;

    c.x = p.x + normal.x * com;
    c.y = p.y + normal.y * com;

    return area;
}
type submergedAreaFn = (shape: b2Shape, _normal: b2Vec2, _offset: number, _xf: b2Readonly<b2Transform>, c: b2Vec2) => number;

export const b2SubmergedAreaByShape: submergedAreaFn[] = [];

b2SubmergedAreaByShape[b2ShapeType.e_circle] = submergedAreaForCircle;
b2SubmergedAreaByShape[b2ShapeType.e_edge] = submergedAreaForEdge;
b2SubmergedAreaByShape[b2ShapeType.e_polygon] = submergedAreaForPolygon;
b2SubmergedAreaByShape[b2ShapeType.e_chain] = submergedAreaForChain;

export function b2SubmergedAreaForShape(shape: b2Shape, normal: b2Vec2, offset: number, xf: b2Readonly<b2Transform>, c: b2Vec2) {
    const fn = b2SubmergedAreaByShape[shape.GetType()];
    return fn ? fn(shape, normal, offset, xf, c) : 0;
}
