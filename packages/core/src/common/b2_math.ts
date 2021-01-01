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

// DEBUG: import { assert } from "./b2_common";
import { EPSILON, EPSILON_SQUARED } from "./b2_common";

export const b2_pi_over_180 = Math.PI / 180;
export const b2_180_over_pi = 180 / Math.PI;
export const b2_two_pi = 2 * Math.PI;

export function clamp(a: number, low: number, high: number) {
    if (a < low) return low;
    return a > high ? high : a;
}

export function degToRad(degrees: number) {
    return degrees * b2_pi_over_180;
}

export function radToDeg(radians: number) {
    return radians * b2_180_over_pi;
}

/**
 * "Next Largest Power of 2
 * Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
 * that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
 * the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
 * largest power of 2. For a 32-bit value:"
 */
export function nextPowerOfTwo(x: number) {
    x |= x >> 1;
    x |= x >> 2;
    x |= x >> 4;
    x |= x >> 8;
    x |= x >> 16;
    return x + 1;
}

export function isPowerOfTwo(x: number) {
    return x > 0 && (x & (x - 1)) === 0;
}

export function random(): number {
    return Math.random() * 2 - 1;
}

export function randomFloat(lo: number, hi: number) {
    return (hi - lo) * Math.random() + lo;
}

export function randomInt(lo: number, hi: number) {
    return Math.round((hi - lo) * Math.random() + lo);
}

export interface XY {
    x: number;
    y: number;
}

/**
 * A 2D column vector.
 */
export class Vec2 implements XY {
    public static readonly ZERO: Readonly<XY> = new Vec2();

    public static readonly UNITX: Readonly<XY> = new Vec2(1, 0);

    public static readonly UNITY: Readonly<XY> = new Vec2(0, 1);

    public static readonly s_t0 = new Vec2();

    public static readonly s_t1 = new Vec2();

    public static readonly s_t2 = new Vec2();

    public static readonly s_t3 = new Vec2();

    public x: number;

    public y: number;

    public constructor(x = 0, y = 0) {
        this.x = x;
        this.y = y;
    }

    public clone() {
        return new Vec2(this.x, this.y);
    }

    /**
     * Set this vector to all zeros.
     */
    public setZero() {
        this.x = 0;
        this.y = 0;
        return this;
    }

    /**
     * Set this vector to some specified coordinates.
     */
    public set(x: number, y: number) {
        this.x = x;
        this.y = y;
        return this;
    }

    public copy(other: XY) {
        this.x = other.x;
        this.y = other.y;
        return this;
    }

    /**
     * Add a vector to this vector.
     */
    public add(v: XY) {
        this.x += v.x;
        this.y += v.y;
        return this;
    }

    /**
     * Add a vector to this vector.
     */
    public addXY(x: number, y: number) {
        this.x += x;
        this.y += y;
        return this;
    }

    /**
     * Subtract a vector from this vector.
     */
    public subtract(v: XY) {
        this.x -= v.x;
        this.y -= v.y;
        return this;
    }

    /**
     * Subtract a vector from this vector.
     */
    public subtractXY(x: number, y: number) {
        this.x -= x;
        this.y -= y;
        return this;
    }

    /**
     * Multiply this vector by a scalar.
     */
    public scale(s: number) {
        this.x *= s;
        this.y *= s;
        return this;
    }

    public addScaled(s: number, v: XY) {
        this.x += s * v.x;
        this.y += s * v.y;
        return this;
    }

    public subtractScaled(s: number, v: XY) {
        this.x -= s * v.x;
        this.y -= s * v.y;
        return this;
    }

    /**
     * Perform the dot product on two vectors.
     */
    public dot(v: XY) {
        return this.x * v.x + this.y * v.y;
    }

    /**
     * Perform the cross product on two vectors. In 2D this produces a scalar.
     */
    public cross(v: XY) {
        return this.x * v.y - this.y * v.x;
    }

    /**
     * Get the length of this vector (the norm).
     */
    public length() {
        const { x, y } = this;
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Get the length squared. For performance, use this instead of
     * Vec2::Length (if possible).
     */
    public lengthSquared() {
        const { x, y } = this;
        return x * x + y * y;
    }

    /**
     * Convert this vector into a unit vector. Returns the length.
     */
    public normalize() {
        const length = this.length();
        if (length < EPSILON) {
            return 0;
        }
        const inv_length = 1 / length;
        this.x *= inv_length;
        this.y *= inv_length;
        return length;
    }

    public rotate(radians: number) {
        const c = Math.cos(radians);
        const s = Math.sin(radians);
        const { x } = this;
        this.x = c * x - s * this.y;
        this.y = s * x + c * this.y;
        return this;
    }

    public rotateCosSin(c: number, s: number) {
        const { x } = this;
        this.x = c * x - s * this.y;
        this.y = s * x + c * this.y;
        return this;
    }

    /**
     * Does this vector contain finite coordinates?
     */
    public isValid() {
        return Number.isFinite(this.x) && Number.isFinite(this.y);
    }

    public abs() {
        this.x = Math.abs(this.x);
        this.y = Math.abs(this.y);
        return this;
    }

    public getAbs<T extends XY>(out: T) {
        out.x = Math.abs(this.x);
        out.y = Math.abs(this.y);
        return out;
    }

    /**
     * Negate this vector.
     */
    public negate() {
        this.x = -this.x;
        this.y = -this.y;
        return this;
    }

    /**
     * Skew this vector such that dot(skew_vec, other) == cross(vec, other)
     */
    public skew() {
        const { x } = this;
        this.x = -this.y;
        this.y = x;
        return this;
    }

    public static min<T extends XY>(a: XY, b: XY, out: T) {
        out.x = Math.min(a.x, b.x);
        out.y = Math.min(a.y, b.y);
        return out;
    }

    public static max<T extends XY>(a: XY, b: XY, out: T) {
        out.x = Math.max(a.x, b.x);
        out.y = Math.max(a.y, b.y);
        return out;
    }

    public static clamp<T extends XY>(v: XY, lo: XY, hi: XY, out: T) {
        out.x = clamp(v.x, lo.x, hi.x);
        out.y = clamp(v.y, lo.y, hi.y);
        return out;
    }

    public static rotate<T extends XY>(v: XY, radians: number, out: T) {
        const v_x = v.x;
        const v_y = v.y;
        const c = Math.cos(radians);
        const s = Math.sin(radians);
        out.x = c * v_x - s * v_y;
        out.y = s * v_x + c * v_y;
        return out;
    }

    public static dot(a: XY, b: XY) {
        return a.x * b.x + a.y * b.y;
    }

    public static cross(a: XY, b: XY) {
        return a.x * b.y - a.y * b.x;
    }

    /**
     * Perform the cross product on a vector and a scalar. In 2D this produces
     * a vector.
     */
    public static crossVec2Scalar<T extends XY>(v: XY, s: number, out: T) {
        const v_x = v.x;
        out.x = s * v.y;
        out.y = -s * v_x;
        return out;
    }

    public static crossVec2One<T extends XY>(v: XY, out: T) {
        const v_x = v.x;
        out.x = v.y;
        out.y = -v_x;
        return out;
    }

    /**
     * Perform the cross product on a scalar and a vector. In 2D this produces
     * a vector.
     */
    public static crossScalarVec2<T extends XY>(s: number, v: XY, out: T) {
        const v_x = v.x;
        out.x = -s * v.y;
        out.y = s * v_x;
        return out;
    }

    public static crossOneVec2<T extends XY>(v: XY, out: T) {
        const v_x = v.x;
        out.x = -v.y;
        out.y = v_x;
        return out;
    }

    /**
     * Add two vectors component-wise.
     */
    public static add<T extends XY>(a: XY, b: XY, out: T) {
        out.x = a.x + b.x;
        out.y = a.y + b.y;
        return out;
    }

    /**
     * Subtract two vectors component-wise.
     */
    public static subtract<T extends XY>(a: XY, b: XY, out: T) {
        out.x = a.x - b.x;
        out.y = a.y - b.y;
        return out;
    }

    public static scale<T extends XY>(s: number, v: XY, out: T) {
        out.x = v.x * s;
        out.y = v.y * s;
        return out;
    }

    public static addScaled<T extends XY>(a: XY, s: number, b: XY, out: T) {
        out.x = a.x + s * b.x;
        out.y = a.y + s * b.y;
        return out;
    }

    public static subtractScaled<T extends XY>(a: XY, s: number, b: XY, out: T) {
        out.x = a.x - s * b.x;
        out.y = a.y - s * b.y;
        return out;
    }

    public static addCrossScalarVec2<T extends XY>(a: XY, s: number, v: XY, out: T) {
        const v_x = v.x;
        out.x = a.x - s * v.y;
        out.y = a.y + s * v_x;
        return out;
    }

    public static mid<T extends XY>(a: XY, b: XY, out: T) {
        out.x = (a.x + b.x) * 0.5;
        out.y = (a.y + b.y) * 0.5;
        return out;
    }

    public static extents<T extends XY>(a: XY, b: XY, out: T) {
        out.x = (b.x - a.x) * 0.5;
        out.y = (b.y - a.y) * 0.5;
        return out;
    }

    public static equals(a: XY, b: XY) {
        return a.x === b.x && a.y === b.y;
    }

    public static distance(a: XY, b: XY) {
        return Math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2);
    }

    public static distanceSquared(a: XY, b: XY) {
        return (a.x - b.x) ** 2 + (a.y - b.y) ** 2;
    }

    /**
     * Negate a vector.
     */
    public static negate<T extends XY>(v: XY, out: T) {
        out.x = -v.x;
        out.y = -v.y;
        return out;
    }

    public static normalize<T extends XY>(v: XY, out: T) {
        const length_sq = v.x ** 2 + v.y ** 2;
        if (length_sq >= EPSILON_SQUARED) {
            const inv_length = 1 / Math.sqrt(length_sq);
            out.x = inv_length * v.x;
            out.y = inv_length * v.y;
        } else {
            out.x = 0;
            out.y = 0;
        }
        return out;
    }

    /**
     * Skew a vector such that dot(skew_vec, other) == cross(vec, other)
     */
    public static skew<T extends XY>(v: XY, out: T) {
        const { x } = v;
        out.x = -v.y;
        out.y = x;
        return out;
    }
}

export interface XYZ extends XY {
    z: number;
}

/**
 * A 2D column vector with 3 elements.
 */
export class Vec3 implements XYZ {
    public static readonly ZERO: Readonly<XYZ> = new Vec3(0, 0, 0);

    public static readonly s_t0 = new Vec3();

    public x: number;

    public y: number;

    public z: number;

    public constructor(x = 0, y = 0, z = 0) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public clone() {
        return new Vec3(this.x, this.y, this.z);
    }

    /**
     * Set this vector to all zeros.
     */
    public setZero() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
        return this;
    }

    /**
     * Set this vector to some specified coordinates.
     */
    public set(x: number, y: number, z: number) {
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }

    public copy(other: XYZ) {
        this.x = other.x;
        this.y = other.y;
        this.z = other.z;
        return this;
    }

    /**
     * Negate this vector.
     */
    public negate() {
        this.x = -this.x;
        this.y = -this.y;
        this.z = -this.z;
        return this;
    }

    /**
     * Add a vector to this vector.
     */
    public add(v: XYZ) {
        this.x += v.x;
        this.y += v.y;
        this.z += v.z;
        return this;
    }

    /**
     * Add a vector to this vector.
     */
    public addXYZ(x: number, y: number, z: number) {
        this.x += x;
        this.y += y;
        this.z += z;
        return this;
    }

    /**
     * Subtract a vector from this vector.
     */
    public subtract(v: XYZ) {
        this.x -= v.x;
        this.y -= v.y;
        this.z -= v.z;
        return this;
    }

    /**
     * Subtract a vector from this vector.
     */
    public subtractXYZ(x: number, y: number, z: number) {
        this.x -= x;
        this.y -= y;
        this.z -= z;
        return this;
    }

    /**
     * Multiply this vector by a scalar.
     */
    public scale(s: number) {
        this.x *= s;
        this.y *= s;
        this.z *= s;
        return this;
    }

    /**
     * Perform the dot product on two vectors.
     */
    public static dot(a: XYZ, b: XYZ): number {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    /**
     * Perform the cross product on two vectors.
     */
    public static cross<T extends XYZ>(a: XYZ, b: XYZ, out: T) {
        const a_x = a.x;
        const a_y = a.y;
        const a_z = a.z;
        const b_x = b.x;
        const b_y = b.y;
        const b_z = b.z;
        out.x = a_y * b_z - a_z * b_y;
        out.y = a_z * b_x - a_x * b_z;
        out.z = a_x * b_y - a_y * b_x;
        return out;
    }
}

/**
 * A 2-by-2 matrix. Stored in column-major order.
 */
export class Mat22 {
    public static readonly IDENTITY: Readonly<Mat22> = new Mat22();

    public readonly ex = new Vec2(1, 0);

    public readonly ey = new Vec2(0, 1);

    public clone() {
        return new Mat22().copy(this);
    }

    /**
     * Construct a matrix using columns.
     */
    public static fromColumns(c1: XY, c2: XY) {
        return new Mat22().setColumns(c1, c2);
    }

    /**
     * Construct a matrix using scalars.
     */
    public static fromScalars(r1c1: number, r1c2: number, r2c1: number, r2c2: number) {
        return new Mat22().setScalars(r1c1, r1c2, r2c1, r2c2);
    }

    public static fromAngle(radians: number) {
        return new Mat22().setAngle(radians);
    }

    /**
     * Set this matrix using scalars.
     */
    public setScalars(r1c1: number, r1c2: number, r2c1: number, r2c2: number) {
        this.ex.set(r1c1, r2c1);
        this.ey.set(r1c2, r2c2);
        return this;
    }

    /**
     * Initialize this matrix using columns.
     */
    public setColumns(c1: XY, c2: XY) {
        this.ex.copy(c1);
        this.ey.copy(c2);
        return this;
    }

    public setAngle(radians: number) {
        const c = Math.cos(radians);
        const s = Math.sin(radians);
        this.ex.set(c, s);
        this.ey.set(-s, c);
        return this;
    }

    public copy(other: Mat22) {
        this.ex.copy(other.ex);
        this.ey.copy(other.ey);
        return this;
    }

    /**
     * Set this to the identity matrix.
     */
    public setIdentity() {
        this.ex.set(1, 0);
        this.ey.set(0, 1);
        return this;
    }

    /**
     * Set this matrix to all zeros.
     */
    public setZero() {
        this.ex.setZero();
        this.ey.setZero();
        return this;
    }

    public getAngle() {
        return Math.atan2(this.ex.y, this.ex.x);
    }

    /**
     * Solve A * x = b, where b is a column vector. This is more efficient
     * than computing the inverse in one-shot cases.
     */
    public solve<T extends XY>(b_x: number, b_y: number, out: T) {
        const a11 = this.ex.x;
        const a12 = this.ey.x;
        const a21 = this.ex.y;
        const a22 = this.ey.y;
        let det = a11 * a22 - a12 * a21;
        if (det !== 0) {
            det = 1 / det;
        }
        out.x = det * (a22 * b_x - a12 * b_y);
        out.y = det * (a11 * b_y - a21 * b_x);
        return out;
    }

    public abs() {
        this.ex.abs();
        this.ey.abs();
        return this;
    }

    public inverse() {
        this.getInverse(this);
        return this;
    }

    public add(M: Mat22) {
        this.ex.add(M.ex);
        this.ey.add(M.ey);
        return this;
    }

    public subtract(M: Mat22) {
        this.ex.subtract(M.ex);
        this.ey.subtract(M.ey);
        return this;
    }

    public getInverse(out: Mat22) {
        const a = this.ex.x;
        const b = this.ey.x;
        const c = this.ex.y;
        const d = this.ey.y;
        let det = a * d - b * c;
        if (det !== 0) {
            det = 1 / det;
        }
        out.ex.x = det * d;
        out.ey.x = -det * b;
        out.ex.y = -det * c;
        out.ey.y = det * a;
        return out;
    }

    public getAbs(out: Mat22) {
        out.ex.x = Math.abs(this.ex.x);
        out.ex.y = Math.abs(this.ex.y);
        out.ey.x = Math.abs(this.ey.x);
        out.ey.y = Math.abs(this.ey.y);
        return out;
    }

    /**
     * Multiply a matrix times a vector. If a rotation matrix is provided,
     * then this transforms the vector from one frame to another.
     */
    public static multiplyVec2<T extends XY>(M: Mat22, v: XY, out: T) {
        const v_x = v.x;
        const v_y = v.y;
        out.x = M.ex.x * v_x + M.ey.x * v_y;
        out.y = M.ex.y * v_x + M.ey.y * v_y;
        return out;
    }

    /**
     * Multiply a matrix transpose times a vector. If a rotation matrix is provided,
     * then this transforms the vector from one frame to another (inverse transform).
     */
    public static transposeMultiplyVec2<T extends XY>(M: Mat22, v: XY, out: T) {
        const v_x = v.x;
        const v_y = v.y;
        out.x = M.ex.x * v_x + M.ex.y * v_y;
        out.y = M.ey.x * v_x + M.ey.y * v_y;
        return out;
    }

    public static add(A: Mat22, B: Mat22, out: Mat22) {
        out.ex.x = A.ex.x + B.ex.x;
        out.ex.y = A.ex.y + B.ex.y;
        out.ey.x = A.ey.x + B.ey.x;
        out.ey.y = A.ey.y + B.ey.y;
        return out;
    }

    /** A * B */
    public static multiply(A: Mat22, B: Mat22, out: Mat22) {
        const A_ex_x = A.ex.x;
        const A_ex_y = A.ex.y;
        const A_ey_x = A.ey.x;
        const A_ey_y = A.ey.y;
        const B_ex_x = B.ex.x;
        const B_ex_y = B.ex.y;
        const B_ey_x = B.ey.x;
        const B_ey_y = B.ey.y;
        out.ex.x = A_ex_x * B_ex_x + A_ey_x * B_ex_y;
        out.ex.y = A_ex_y * B_ex_x + A_ey_y * B_ex_y;
        out.ey.x = A_ex_x * B_ey_x + A_ey_x * B_ey_y;
        out.ey.y = A_ex_y * B_ey_x + A_ey_y * B_ey_y;
        return out;
    }

    /** A^T * B */
    public static transposeMultiply(A: Mat22, B: Mat22, out: Mat22) {
        const A_ex_x = A.ex.x;
        const A_ex_y = A.ex.y;
        const A_ey_x = A.ey.x;
        const A_ey_y = A.ey.y;
        const B_ex_x = B.ex.x;
        const B_ex_y = B.ex.y;
        const B_ey_x = B.ey.x;
        const B_ey_y = B.ey.y;
        out.ex.x = A_ex_x * B_ex_x + A_ex_y * B_ex_y;
        out.ex.y = A_ey_x * B_ex_x + A_ey_y * B_ex_y;
        out.ey.x = A_ex_x * B_ey_x + A_ex_y * B_ey_y;
        out.ey.y = A_ey_x * B_ey_x + A_ey_y * B_ey_y;
        return out;
    }
}

/**
 * A 3-by-3 matrix. Stored in column-major order.
 */
export class Mat33 {
    public static readonly IDENTITY: Readonly<Mat33> = new Mat33();

    public readonly ex = new Vec3(1, 0, 0);

    public readonly ey = new Vec3(0, 1, 0);

    public readonly ez = new Vec3(0, 0, 1);

    public clone(): Mat33 {
        return new Mat33().copy(this);
    }

    /**
     * Set this matrix using columns.
     */
    public setColumns(c1: XYZ, c2: XYZ, c3: XYZ) {
        this.ex.copy(c1);
        this.ey.copy(c2);
        this.ez.copy(c3);
        return this;
    }

    public copy(other: Mat33) {
        this.ex.copy(other.ex);
        this.ey.copy(other.ey);
        this.ez.copy(other.ez);
        return this;
    }

    public setIdentity() {
        this.ex.set(1, 0, 0);
        this.ey.set(0, 1, 0);
        this.ez.set(0, 0, 1);
        return this;
    }

    /**
     * Set this matrix to all zeros.
     */
    public setZero() {
        this.ex.setZero();
        this.ey.setZero();
        this.ez.setZero();
        return this;
    }

    public add(M: Mat33) {
        this.ex.add(M.ex);
        this.ey.add(M.ey);
        this.ez.add(M.ez);
        return this;
    }

    /**
     * Solve A * x = b, where b is a column vector. This is more efficient
     * than computing the inverse in one-shot cases.
     */
    public solve33<T extends XYZ>(b_x: number, b_y: number, b_z: number, out: T) {
        const a11 = this.ex.x;
        const a21 = this.ex.y;
        const a31 = this.ex.z;
        const a12 = this.ey.x;
        const a22 = this.ey.y;
        const a32 = this.ey.z;
        const a13 = this.ez.x;
        const a23 = this.ez.y;
        const a33 = this.ez.z;
        let det = a11 * (a22 * a33 - a32 * a23) + a21 * (a32 * a13 - a12 * a33) + a31 * (a12 * a23 - a22 * a13);
        if (det !== 0) {
            det = 1 / det;
        }
        out.x = det * (b_x * (a22 * a33 - a32 * a23) + b_y * (a32 * a13 - a12 * a33) + b_z * (a12 * a23 - a22 * a13));
        out.y = det * (a11 * (b_y * a33 - b_z * a23) + a21 * (b_z * a13 - b_x * a33) + a31 * (b_x * a23 - b_y * a13));
        out.z = det * (a11 * (a22 * b_z - a32 * b_y) + a21 * (a32 * b_x - a12 * b_z) + a31 * (a12 * b_y - a22 * b_x));
        return out;
    }

    /**
     * Solve A * x = b, where b is a column vector. This is more efficient
     * than computing the inverse in one-shot cases. Solve only the upper
     * 2-by-2 matrix equation.
     */
    public solve22<T extends XY>(b_x: number, b_y: number, out: T) {
        const a11 = this.ex.x;
        const a12 = this.ey.x;
        const a21 = this.ex.y;
        const a22 = this.ey.y;
        let det = a11 * a22 - a12 * a21;
        if (det !== 0) {
            det = 1 / det;
        }
        out.x = det * (a22 * b_x - a12 * b_y);
        out.y = det * (a11 * b_y - a21 * b_x);
        return out;
    }

    /**
     * Get the inverse of this matrix as a 2-by-2.
     * Returns the zero matrix if singular.
     */
    public getInverse22(M: Mat33) {
        const a = this.ex.x;
        const b = this.ey.x;
        const c = this.ex.y;
        const d = this.ey.y;
        let det = a * d - b * c;
        if (det !== 0) {
            det = 1 / det;
        }

        M.ex.x = det * d;
        M.ey.x = -det * b;
        M.ex.z = 0;
        M.ex.y = -det * c;
        M.ey.y = det * a;
        M.ey.z = 0;
        M.ez.x = 0;
        M.ez.y = 0;
        M.ez.z = 0;
    }

    /**
     * Get the symmetric inverse of this matrix as a 3-by-3.
     * Returns the zero matrix if singular.
     */
    public getSymInverse33(M: Mat33) {
        let det = Vec3.dot(this.ex, Vec3.cross(this.ey, this.ez, Vec3.s_t0));
        if (det !== 0) {
            det = 1 / det;
        }

        const a11 = this.ex.x;
        const a12 = this.ey.x;
        const a13 = this.ez.x;
        const a22 = this.ey.y;
        const a23 = this.ez.y;
        const a33 = this.ez.z;

        M.ex.x = det * (a22 * a33 - a23 * a23);
        M.ex.y = det * (a13 * a23 - a12 * a33);
        M.ex.z = det * (a12 * a23 - a13 * a22);

        M.ey.x = M.ex.y;
        M.ey.y = det * (a11 * a33 - a13 * a13);
        M.ey.z = det * (a13 * a12 - a11 * a23);

        M.ez.x = M.ex.z;
        M.ez.y = M.ey.z;
        M.ez.z = det * (a11 * a22 - a12 * a12);
    }

    /**
     * Multiply a matrix times a vector.
     */
    public static multiplyVec3<T extends XYZ>(A: Mat33, v: XYZ, out: T) {
        const { x, y, z } = v;
        out.x = A.ex.x * x + A.ey.x * y + A.ez.x * z;
        out.y = A.ex.y * x + A.ey.y * y + A.ez.y * z;
        out.z = A.ex.z * x + A.ey.z * y + A.ez.z * z;
        return out;
    }

    /**
     * Multiply a matrix times a vector.
     */
    public static multiplyVec2<T extends XY>(A: Mat33, v: XY, out: T) {
        const { x, y } = v;
        out.x = A.ex.x * x + A.ey.x * y;
        out.y = A.ex.y * x + A.ey.y * y;
        return out;
    }
}

/**
 * Rotation
 */
export class Rot {
    public static readonly IDENTITY: Readonly<Rot> = new Rot();

    /** Sine */
    public s = 0;

    /** Cosine */
    public c = 1;

    /**
     * Initialize from an angle in radians
     */
    public constructor(angle = 0) {
        if (angle) {
            this.s = Math.sin(angle);
            this.c = Math.cos(angle);
        }
    }

    public clone() {
        return new Rot().copy(this);
    }

    public copy(other: Rot) {
        this.s = other.s;
        this.c = other.c;
        return this;
    }

    /**
     * Set using an angle in radians.
     */
    public set(angle: number) {
        this.s = Math.sin(angle);
        this.c = Math.cos(angle);
        return this;
    }

    /**
     * Set to the identity rotation
     */
    public setIdentity() {
        this.s = 0;
        this.c = 1;
        return this;
    }

    /**
     * Get the angle in radians
     */
    public getAngle() {
        return Math.atan2(this.s, this.c);
    }

    /**
     * Get the x-axis
     */
    public getXAxis<T extends XY>(out: T) {
        out.x = this.c;
        out.y = this.s;
        return out;
    }

    /**
     * Get the u-axis
     */
    public getYAxis<T extends XY>(out: T) {
        out.x = -this.s;
        out.y = this.c;
        return out;
    }

    /**
     * Multiply two rotations: q * r
     */
    public static multiply(q: Rot, r: Rot, out: Rot) {
        // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
        // [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
        // s = qs * rc + qc * rs
        // c = qc * rc - qs * rs
        const s = q.s * r.c + q.c * r.s;
        const c = q.c * r.c - q.s * r.s;
        out.s = s;
        out.c = c;
        return out;
    }

    /**
     * Transpose multiply two rotations: qT * r
     */
    public static transposeMultiply(q: Rot, r: Rot, out: Rot) {
        // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
        // [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
        // s = qc * rs - qs * rc
        // c = qc * rc + qs * rs
        const s = q.c * r.s - q.s * r.c;
        const c = q.c * r.c + q.s * r.s;
        out.s = s;
        out.c = c;
        return out;
    }

    /**
     * Rotate a vector
     */
    public static multiplyVec2<T extends XY>(q: Rot, v: XY, out: T) {
        const v_x = v.x;
        const v_y = v.y;
        out.x = q.c * v_x - q.s * v_y;
        out.y = q.s * v_x + q.c * v_y;
        return out;
    }

    /**
     * Inverse rotate a vector
     */
    public static transposeMultiplyVec2<T extends XY>(q: Rot, v: XY, out: T) {
        const v_x = v.x;
        const v_y = v.y;
        out.x = q.c * v_x + q.s * v_y;
        out.y = -q.s * v_x + q.c * v_y;
        return out;
    }
}

/**
 * A transform contains translation and rotation. It is used to represent
 * the position and orientation of rigid frames.
 */
export class Transform {
    public static readonly IDENTITY: Readonly<Transform> = new Transform();

    public readonly p = new Vec2();

    public readonly q = new Rot();

    public clone() {
        return new Transform().copy(this);
    }

    public copy(other: Transform) {
        this.p.copy(other.p);
        this.q.copy(other.q);
        return this;
    }

    /**
     * Set this to the identity transform.
     */
    public setIdentity() {
        this.p.setZero();
        this.q.setIdentity();
        return this;
    }

    /**
     * Set this based on the position and rotation.
     */
    public setPositionRotation(position: XY, q: Readonly<Rot>) {
        this.p.copy(position);
        this.q.copy(q);
        return this;
    }

    /**
     * Set this based on the position and angle.
     */
    public setPositionAngle(pos: XY, a: number) {
        this.p.copy(pos);
        this.q.set(a);
        return this;
    }

    public setPosition(position: XY) {
        this.p.copy(position);
        return this;
    }

    public setPositionXY(x: number, y: number) {
        this.p.set(x, y);
        return this;
    }

    public setRotation(rotation: Readonly<Rot>) {
        this.q.copy(rotation);
        return this;
    }

    public setRotationAngle(radians: number) {
        this.q.set(radians);
        return this;
    }

    public getPosition(): Readonly<Vec2> {
        return this.p;
    }

    public getRotation(): Readonly<Rot> {
        return this.q;
    }

    public getAngle() {
        return this.q.getAngle();
    }

    public static multiplyVec2<T extends XY>(T: Transform, v: Readonly<XY>, out: T) {
        const v_x = v.x;
        const v_y = v.y;
        out.x = T.q.c * v_x - T.q.s * v_y + T.p.x;
        out.y = T.q.s * v_x + T.q.c * v_y + T.p.y;
        return out;
    }

    public static transposeMultiplyVec2<T extends XY>(T: Transform, v: Readonly<XY>, out: T) {
        const px = v.x - T.p.x;
        const py = v.y - T.p.y;
        out.x = T.q.c * px + T.q.s * py;
        out.y = -T.q.s * px + T.q.c * py;
        return out;
    }

    /**
     * v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
     *    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
     */
    public static multiply(A: Transform, B: Transform, out: Transform) {
        Rot.multiply(A.q, B.q, out.q);
        Rot.multiplyVec2(A.q, B.p, out.p).add(A.p);
        return out;
    }

    /**
     * v2 = A.q' * (B.q * v1 + B.p - A.p)
     *    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
     */
    public static transposeMultiply(A: Transform, B: Transform, out: Transform) {
        Rot.transposeMultiply(A.q, B.q, out.q);
        Rot.transposeMultiplyVec2(A.q, Vec2.subtract(B.p, A.p, out.p), out.p);
        return out;
    }
}

/**
 * This describes the motion of a body/shape for TOI computation.
 * Shapes are defined with respect to the body origin, which may
 * no coincide with the center of mass. However, to support dynamics
 * we must interpolate the center of mass position.
 */
export class Sweep {
    /** Local center of mass position */
    public readonly localCenter = new Vec2();

    /** Center world position at time 0 */
    public readonly c0 = new Vec2();

    /** Center world position at time 1 */
    public readonly c = new Vec2();

    /** World angle at time 0 */
    public a0 = 0;

    /** World angle at time 1 */
    public a = 0;

    /**
     * Fraction of the current time step in the range [0,1]
     * c0 and a0 are the positions at alpha0.
     */
    public alpha0 = 0;

    public clone(): Sweep {
        return new Sweep().copy(this);
    }

    public copy(other: Sweep) {
        this.localCenter.copy(other.localCenter);
        this.c0.copy(other.c0);
        this.c.copy(other.c);
        this.a0 = other.a0;
        this.a = other.a;
        this.alpha0 = other.alpha0;
        return this;
    }

    /**
     * Get the interpolated transform at a specific time.
     *
     * @param transform The output transform
     * @param beta Is a factor in [0,1], where 0 indicates alpha0.
     * @see https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
     */
    public getTransform(xf: Transform, beta: number) {
        const oneMinusBeta = 1 - beta;
        xf.p.x = oneMinusBeta * this.c0.x + beta * this.c.x;
        xf.p.y = oneMinusBeta * this.c0.y + beta * this.c.y;
        const angle = oneMinusBeta * this.a0 + beta * this.a;
        xf.q.set(angle);

        // Shift to origin
        xf.p.subtract(Rot.multiplyVec2(xf.q, this.localCenter, Vec2.s_t0));
        return xf;
    }

    /**
     * Advance the sweep forward, yielding a new initial state.
     *
     * @param alpha The new initial time.
     */
    public advance(alpha: number) {
        // DEBUG: assert(this.alpha0 < 1);
        const beta = (alpha - this.alpha0) / (1 - this.alpha0);
        this.c0.x += beta * (this.c.x - this.c0.x);
        this.c0.y += beta * (this.c.y - this.c0.y);
        this.a0 += beta * (this.a - this.a0);
        this.alpha0 = alpha;
    }

    /**
     * Normalize an angle in radians to be between -pi and pi
     */
    public normalize() {
        const d = b2_two_pi * Math.floor(this.a0 / b2_two_pi);
        this.a0 -= d;
        this.a -= d;
    }
}
