import { Vec2, Transform, Rot, PolygonShape, EPSILON, MassData, ShapeType, Shape, CircleShape } from "@box2d/core";

const ComputeSubmergedArea_s_normalL = new Vec2();
const ComputeSubmergedArea_s_md = new MassData();
const ComputeSubmergedArea_s_intoVec = new Vec2();
const ComputeSubmergedArea_s_outoVec = new Vec2();
const ComputeSubmergedArea_s_center = new Vec2();

function submergedAreaForPolygon(shape: Shape, normal: Vec2, offset: number, xf: Transform, c: Vec2): number {
    const polygon = shape as PolygonShape;

    // Transform plane into shape co-ordinates
    const normalL = Rot.transposeMultiplyVec2(xf.q, normal, ComputeSubmergedArea_s_normalL);
    const offsetL = offset - Vec2.dot(normal, xf.p);

    const depths: number[] = [];
    let diveCount = 0;
    let intoIndex = -1;
    let outoIndex = -1;

    let lastSubmerged = false;
    for (let i = 0; i < polygon.count; ++i) {
        depths[i] = Vec2.dot(normalL, polygon.vertices[i]) - offsetL;
        const isSubmerged = depths[i] < -EPSILON;
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
                polygon.computeMass(md, 1);
                Transform.multiplyVec2(xf, md.center, c);
                return md.mass;
            }
            // Completely dry
            return 0;

        case 1:
            if (intoIndex === -1) {
                intoIndex = polygon.count - 1;
            } else {
                outoIndex = polygon.count - 1;
            }
            break;
    }
    const intoIndex2 = (intoIndex + 1) % polygon.count;
    const outoIndex2 = (outoIndex + 1) % polygon.count;
    const intoLamdda = (0 - depths[intoIndex]) / (depths[intoIndex2] - depths[intoIndex]);
    const outoLamdda = (0 - depths[outoIndex]) / (depths[outoIndex2] - depths[outoIndex]);

    const intoVec = ComputeSubmergedArea_s_intoVec.set(
        polygon.vertices[intoIndex].x * (1 - intoLamdda) + polygon.vertices[intoIndex2].x * intoLamdda,
        polygon.vertices[intoIndex].y * (1 - intoLamdda) + polygon.vertices[intoIndex2].y * intoLamdda,
    );
    const outoVec = ComputeSubmergedArea_s_outoVec.set(
        polygon.vertices[outoIndex].x * (1 - outoLamdda) + polygon.vertices[outoIndex2].x * outoLamdda,
        polygon.vertices[outoIndex].y * (1 - outoLamdda) + polygon.vertices[outoIndex2].y * outoLamdda,
    );

    // Initialize accumulator
    let area = 0;
    const center = ComputeSubmergedArea_s_center.setZero();
    let p2 = polygon.vertices[intoIndex2];
    let p3: Vec2;

    // An awkward loop from intoIndex2+1 to outIndex2
    let i = intoIndex2;
    while (i !== outoIndex2) {
        i = (i + 1) % polygon.count;
        if (i === outoIndex2) {
            p3 = outoVec;
        } else {
            p3 = polygon.vertices[i];
        }

        const triangleArea = 0.5 * ((p2.x - intoVec.x) * (p3.y - intoVec.y) - (p2.y - intoVec.y) * (p3.x - intoVec.x));
        area += triangleArea;
        // Area weighted centroid
        center.x += (triangleArea * (intoVec.x + p2.x + p3.x)) / 3;
        center.y += (triangleArea * (intoVec.y + p2.y + p3.y)) / 3;

        p2 = p3;
    }

    // Normalize and transform centroid
    center.scale(1 / area);
    Transform.multiplyVec2(xf, center, c);

    return area;
}

function submergedAreaForEdge(shape: Shape, _normal: Vec2, _offset: number, _xf: Transform, c: Vec2): number {
    c.setZero();
    return 0;
}

function submergedAreaForChain(shape: Shape, _normal: Vec2, _offset: number, _xf: Transform, c: Vec2): number {
    c.setZero();
    return 0;
}

function submergedAreaForCircle(shape: Shape, normal: Vec2, offset: number, xf: Transform, c: Vec2): number {
    const circle = shape as CircleShape;
    const p = Transform.multiplyVec2(xf, circle.p, new Vec2());
    const l = -(Vec2.dot(normal, p) - offset);

    if (l < -circle.radius + EPSILON) {
        // Completely dry
        return 0;
    }
    if (l > circle.radius) {
        // Completely wet
        c.copy(p);
        return Math.PI * circle.radius * circle.radius;
    }

    // Magic
    const r2 = circle.radius * circle.radius;
    const l2 = l * l;
    const area = r2 * (Math.asin(l / circle.radius) + Math.PI / 2) + l * Math.sqrt(r2 - l2);
    const com = ((-2 / 3) * (r2 - l2) ** 1.5) / area;

    c.x = p.x + normal.x * com;
    c.y = p.y + normal.y * com;

    return area;
}
type submergedAreaFn = (shape: Shape, _normal: Vec2, _offset: number, _xf: Transform, c: Vec2) => number;

export const submergedAreaByShape: submergedAreaFn[] = [];

submergedAreaByShape[ShapeType.Circle] = submergedAreaForCircle;
submergedAreaByShape[ShapeType.Edge] = submergedAreaForEdge;
submergedAreaByShape[ShapeType.Polygon] = submergedAreaForPolygon;
submergedAreaByShape[ShapeType.Chain] = submergedAreaForChain;

export function submergedAreaForShape(shape: Shape, normal: Vec2, offset: number, xf: Transform, c: Vec2) {
    const fn = submergedAreaByShape[shape.getType()];
    return fn ? fn(shape, normal, offset, xf, c) : 0;
}
