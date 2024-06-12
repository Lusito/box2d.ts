import { DEG_TO_RAD } from "@box2d/lights";

export function setRect(out: Float32Array, left: number, top: number, width: number, height: number) {
    const right = left + width;
    const bottom = top + height;
    // prettier-ignore
    out.set([
        left, bottom,
        left, top,
        right, bottom,
        right, bottom,
        left, top,
        right, top,
    ]);
    return out;
}

export function setBounds(out: Float32Array, top: number, right: number, bottom: number, left: number) {
    // prettier-ignore
    out.set([
        left, bottom,
        left, top,
        right, bottom,
        right, bottom,
        left, top,
        right, top,
    ]);
    return out;
}

export function setRotatedRect(
    out: Float32Array,
    x: number,
    y: number,
    width: number,
    height: number,
    rotation: number,
    centerX: number,
    centerY: number,
    scale = 1,
) {
    const absCenterX = x + centerX;
    const absCenterY = y + centerY;

    const left = -centerX * scale;
    const top = -centerY * scale;
    const right = (width - centerX) * scale;
    const bottom = (height - centerY) * scale;

    const rotationRad = rotation * DEG_TO_RAD;
    const cos = Math.cos(rotationRad);
    const sin = Math.sin(rotationRad);

    const bottomLeftX = cos * left - sin * bottom + absCenterX;
    const bottomLeftY = sin * left + cos * bottom + absCenterY;

    const topLeftX = cos * left - sin * top + absCenterX;
    const topLeftY = sin * left + cos * top + absCenterY;

    const bottomRightX = cos * right - sin * bottom + absCenterX;
    const bottomRightY = sin * right + cos * bottom + absCenterY;

    const topRightX = topLeftX + (bottomRightX - bottomLeftX);
    const topRightY = bottomRightY - (bottomLeftY - topLeftY);

    // prettier-ignore
    out.set([
        bottomLeftX, bottomLeftY,
        topLeftX, topLeftY,
        bottomRightX, bottomRightY,
        bottomRightX, bottomRightY,
        topLeftX, topLeftY,
        topRightX, topRightY,
    ]);
    return out;
}
