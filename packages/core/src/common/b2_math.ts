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

// DEBUG: import { b2Assert } from "./b2_common";
import type { b2Readonly } from "./b2_readonly";
import { b2_epsilon, b2_epsilon_sq } from "./b2_common";

export const b2_pi_over_180 = Math.PI / 180;
export const b2_180_over_pi = 180 / Math.PI;
export const b2_two_pi = 2 * Math.PI;

export function b2Clamp(a: number, low: number, high: number) {
    if (a < low) return low;
    return a > high ? high : a;
}

export function b2DegToRad(degrees: number) {
    return degrees * b2_pi_over_180;
}

export function b2RadToDeg(radians: number) {
    return radians * b2_180_over_pi;
}

/**
 * "Next Largest Power of 2
 * Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
 * that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
 * the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
 * largest power of 2. For a 32-bit value:"
 */
export function b2NextPowerOfTwo(x: number) {
    x |= x >> 1;
    x |= x >> 2;
    x |= x >> 4;
    x |= x >> 8;
    x |= x >> 16;
    return x + 1;
}

export function b2IsPowerOfTwo(x: number) {
    return x > 0 && (x & (x - 1)) === 0;
}

export function b2Random(): number {
    return Math.random() * 2 - 1;
}

export function b2RandomFloat(lo: number, hi: number) {
    return (hi - lo) * Math.random() + lo;
}

export function b2RandomInt(lo: number, hi: number) {
    return Math.round((hi - lo) * Math.random() + lo);
}

export interface XY {
    x: number;
    y: number;
}

/**
 * A 2D column vector.
 */
export class b2Vec2 implements XY {
    public static readonly ZERO: b2Readonly<b2Vec2> = new b2Vec2();

    public static readonly UNITX: b2Readonly<b2Vec2> = new b2Vec2(1, 0);

    public static readonly UNITY: b2Readonly<b2Vec2> = new b2Vec2(0, 1);

    public static readonly s_t0 = new b2Vec2();

    public static readonly s_t1 = new b2Vec2();

    public static readonly s_t2 = new b2Vec2();

    public static readonly s_t3 = new b2Vec2();

    public x: number;

    public y: number;

    public constructor(x = 0, y = 0) {
        this.x = x;
        this.y = y;
    }

    public Clone() {
        return new b2Vec2(this.x, this.y);
    }

    /**
     * Set this vector to all zeros.
     */
    public SetZero() {
        this.x = 0;
        this.y = 0;
        return this;
    }

    /**
     * Set this vector to some specified coordinates.
     */
    public Set(x: number, y: number) {
        this.x = x;
        this.y = y;
        return this;
    }

    public Copy(other: Readonly<XY>) {
        this.x = other.x;
        this.y = other.y;
        return this;
    }

    /**
     * Add a vector to this vector.
     */
    public Add(v: Readonly<XY>) {
        this.x += v.x;
        this.y += v.y;
        return this;
    }

    /**
     * Add a vector to this vector.
     */
    public AddXY(x: number, y: number) {
        this.x += x;
        this.y += y;
        return this;
    }

    /**
     * Subtract a vector from this vector.
     */
    public Subtract(v: Readonly<XY>) {
        this.x -= v.x;
        this.y -= v.y;
        return this;
    }

    /**
     * Subtract a vector from this vector.
     */
    public SubtractXY(x: number, y: number) {
        this.x -= x;
        this.y -= y;
        return this;
    }

    /**
     * Multiply this vector by a scalar.
     */
    public Scale(s: number) {
        this.x *= s;
        this.y *= s;
        return this;
    }

    public AddScaled(s: number, v: Readonly<XY>) {
        this.x += s * v.x;
        this.y += s * v.y;
        return this;
    }

    public SubtractScaled(s: number, v: Readonly<XY>) {
        this.x -= s * v.x;
        this.y -= s * v.y;
        return this;
    }

    /**
     * Perform the dot product on two vectors.
     */
    public Dot(v: Readonly<XY>) {
        return this.x * v.x + this.y * v.y;
    }

    /**
     * Perform the cross product on two vectors. In 2D this produces a scalar.
     */
    public Cross(v: Readonly<XY>) {
        return this.x * v.y - this.y * v.x;
    }

    /**
     * Get the length of this vector (the norm).
     */
    public Length() {
        const { x, y } = this;
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Get the length squared. For performance, use this instead of
     * b2Vec2::Length (if possible).
     */
    public LengthSquared() {
        const { x, y } = this;
        return x * x + y * y;
    }

    /**
     * Convert this vector into a unit vector. Returns the length.
     */
    public Normalize() {
        const length = this.Length();
        if (length < b2_epsilon) {
            return 0;
        }
        const inv_length = 1 / length;
        this.x *= inv_length;
        this.y *= inv_length;
        return length;
    }

    public Rotate(radians: number) {
        const c = Math.cos(radians);
        const s = Math.sin(radians);
        const { x } = this;
        this.x = c * x - s * this.y;
        this.y = s * x + c * this.y;
        return this;
    }

    public RotateCosSin(c: number, s: number) {
        const { x } = this;
        this.x = c * x - s * this.y;
        this.y = s * x + c * this.y;
        return this;
    }

    /**
     * Does this vector contain finite coordinates?
     */
    public IsValid() {
        return Number.isFinite(this.x) && Number.isFinite(this.y);
    }

    public Abs() {
        this.x = Math.abs(this.x);
        this.y = Math.abs(this.y);
        return this;
    }

    public GetAbs<T extends XY>(out: T) {
        out.x = Math.abs(this.x);
        out.y = Math.abs(this.y);
        return out;
    }

    /**
     * Negate this vector.
     */
    public Negate() {
        this.x = -this.x;
        this.y = -this.y;
        return this;
    }

    /**
     * Skew this vector such that dot(skew_vec, other) == cross(vec, other)
     */
    public Skew() {
        const { x } = this;
        this.x = -this.y;
        this.y = x;
        return this;
    }

    public static Min<T extends XY>(a: Readonly<XY>, b: Readonly<XY>, out: T) {
        out.x = Math.min(a.x, b.x);
        out.y = Math.min(a.y, b.y);
        return out;
    }

    public static Max<T extends XY>(a: Readonly<XY>, b: Readonly<XY>, out: T) {
        out.x = Math.max(a.x, b.x);
        out.y = Math.max(a.y, b.y);
        return out;
    }

    public static Clamp<T extends XY>(v: Readonly<XY>, lo: Readonly<XY>, hi: Readonly<XY>, out: T) {
        out.x = b2Clamp(v.x, lo.x, hi.x);
        out.y = b2Clamp(v.y, lo.y, hi.y);
        return out;
    }

    public static Rotate<T extends XY>(v: Readonly<XY>, radians: number, out: T) {
        const v_x = v.x;
        const v_y = v.y;
        const c = Math.cos(radians);
        const s = Math.sin(radians);
        out.x = c * v_x - s * v_y;
        out.y = s * v_x + c * v_y;
        return out;
    }

    /** Perform the dot product on two vectors. */
    public static Dot(a: Readonly<XY>, b: Readonly<XY>) {
        return a.x * b.x + a.y * b.y;
    }

    /** Perform the cross product on two vectors. In 2D this produces a scalar. */
    public static Cross(a: Readonly<XY>, b: Readonly<XY>) {
        return a.x * b.y - a.y * b.x;
    }

    /**
     * Perform the cross product on a vector and a scalar. In 2D this produces
     * a vector.
     */
    public static CrossVec2Scalar<T extends XY>(v: Readonly<XY>, s: number, out: T) {
        const v_x = v.x;
        out.x = s * v.y;
        out.y = -s * v_x;
        return out;
    }

    public static CrossVec2One<T extends XY>(v: Readonly<XY>, out: T) {
        const v_x = v.x;
        out.x = v.y;
        out.y = -v_x;
        return out;
    }

    /**
     * Perform the cross product on a scalar and a vector. In 2D this produces
     * a vector.
     */
    public static CrossScalarVec2<T extends XY>(s: number, v: Readonly<XY>, out: T) {
        const v_x = v.x;
        out.x = -s * v.y;
        out.y = s * v_x;
        return out;
    }

    public static CrossOneVec2<T extends XY>(v: Readonly<XY>, out: T) {
        const v_x = v.x;
        out.x = -v.y;
        out.y = v_x;
        return out;
    }

    /**
     * Add two vectors component-wise.
     */
    public static Add<T extends XY>(a: Readonly<XY>, b: Readonly<XY>, out: T) {
        out.x = a.x + b.x;
        out.y = a.y + b.y;
        return out;
    }

    /**
     * Subtract two vectors component-wise.
     */
    public static Subtract<T extends XY>(a: Readonly<XY>, b: Readonly<XY>, out: T) {
        out.x = a.x - b.x;
        out.y = a.y - b.y;
        return out;
    }

    public static Scale<T extends XY>(s: number, v: Readonly<XY>, out: T) {
        out.x = v.x * s;
        out.y = v.y * s;
        return out;
    }

    public static AddScaled<T extends XY>(a: Readonly<XY>, s: number, b: Readonly<XY>, out: T) {
        out.x = a.x + s * b.x;
        out.y = a.y + s * b.y;
        return out;
    }

    public static SubtractScaled<T extends XY>(a: Readonly<XY>, s: number, b: Readonly<XY>, out: T) {
        out.x = a.x - s * b.x;
        out.y = a.y - s * b.y;
        return out;
    }

    public static AddCrossScalarVec2<T extends XY>(a: Readonly<XY>, s: number, v: Readonly<XY>, out: T) {
        const v_x = v.x;
        out.x = a.x - s * v.y;
        out.y = a.y + s * v_x;
        return out;
    }

    public static Mid<T extends XY>(a: Readonly<XY>, b: Readonly<XY>, out: T) {
        out.x = (a.x + b.x) * 0.5;
        out.y = (a.y + b.y) * 0.5;
        return out;
    }

    public static Extents<T extends XY>(a: Readonly<XY>, b: Readonly<XY>, out: T) {
        out.x = (b.x - a.x) * 0.5;
        out.y = (b.y - a.y) * 0.5;
        return out;
    }

    public static Equals(a: Readonly<XY>, b: Readonly<XY>) {
        return a.x === b.x && a.y === b.y;
    }

    public static Distance(a: Readonly<XY>, b: Readonly<XY>) {
        return Math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2);
    }

    public static DistanceSquared(a: Readonly<XY>, b: Readonly<XY>) {
        return (a.x - b.x) ** 2 + (a.y - b.y) ** 2;
    }

    /**
     * Negate a vector.
     */
    public static Negate<T extends XY>(v: Readonly<XY>, out: T) {
        out.x = -v.x;
        out.y = -v.y;
        return out;
    }

    public static Normalize<T extends XY>(v: Readonly<XY>, out: T) {
        const length_sq = v.x ** 2 + v.y ** 2;
        if (length_sq >= b2_epsilon_sq) {
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
    public static Skew<T extends XY>(v: Readonly<XY>, out: T) {
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
export class b2Vec3 implements XYZ {
    public static readonly ZERO: b2Readonly<b2Vec3> = new b2Vec3(0, 0, 0);

    public static readonly s_t0 = new b2Vec3();

    public x: number;

    public y: number;

    public z: number;

    public constructor(x = 0, y = 0, z = 0) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Clone() {
        return new b2Vec3(this.x, this.y, this.z);
    }

    /**
     * Set this vector to all zeros.
     */
    public SetZero() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
        return this;
    }

    /**
     * Set this vector to some specified coordinates.
     */
    public Set(x: number, y: number, z: number) {
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }

    public Copy(other: Readonly<XYZ>) {
        this.x = other.x;
        this.y = other.y;
        this.z = other.z;
        return this;
    }

    /**
     * Negate this vector.
     */
    public Negate() {
        this.x = -this.x;
        this.y = -this.y;
        this.z = -this.z;
        return this;
    }

    /**
     * Add a vector to this vector.
     */
    public Add(v: Readonly<XYZ>) {
        this.x += v.x;
        this.y += v.y;
        this.z += v.z;
        return this;
    }

    /**
     * Add a vector to this vector.
     */
    public AddXYZ(x: number, y: number, z: number) {
        this.x += x;
        this.y += y;
        this.z += z;
        return this;
    }

    /**
     * Subtract a vector from this vector.
     */
    public Subtract(v: Readonly<XYZ>) {
        this.x -= v.x;
        this.y -= v.y;
        this.z -= v.z;
        return this;
    }

    /**
     * Subtract a vector from this vector.
     */
    public SubtractXYZ(x: number, y: number, z: number) {
        this.x -= x;
        this.y -= y;
        this.z -= z;
        return this;
    }

    /**
     * Multiply this vector by a scalar.
     */
    public Scale(s: number) {
        this.x *= s;
        this.y *= s;
        this.z *= s;
        return this;
    }

    /**
     * Perform the dot product on two vectors.
     */
    public static Dot(a: Readonly<XYZ>, b: Readonly<XYZ>): number {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    /**
     * Perform the cross product on two vectors.
     */
    public static Cross<T extends XYZ>(a: Readonly<XYZ>, b: Readonly<XYZ>, out: T) {
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
export class b2Mat22 {
    public static readonly IDENTITY: b2Readonly<b2Mat22> = new b2Mat22();

    public readonly ex = new b2Vec2(1, 0);

    public readonly ey = new b2Vec2(0, 1);

    public Clone() {
        return new b2Mat22().Copy(this);
    }

    /**
     * Construct a matrix using columns.
     */
    public static FromColumns(c1: Readonly<XY>, c2: Readonly<XY>) {
        return new b2Mat22().SetColumns(c1, c2);
    }

    /**
     * Construct a matrix using scalars.
     */
    public static FromScalars(r1c1: number, r1c2: number, r2c1: number, r2c2: number) {
        return new b2Mat22().SetScalars(r1c1, r1c2, r2c1, r2c2);
    }

    public static FromAngle(radians: number) {
        return new b2Mat22().SetAngle(radians);
    }

    /**
     * Set this matrix using scalars.
     */
    public SetScalars(r1c1: number, r1c2: number, r2c1: number, r2c2: number) {
        this.ex.Set(r1c1, r2c1);
        this.ey.Set(r1c2, r2c2);
        return this;
    }

    /**
     * Initialize this matrix using columns.
     */
    public SetColumns(c1: Readonly<XY>, c2: Readonly<XY>) {
        this.ex.Copy(c1);
        this.ey.Copy(c2);
        return this;
    }

    public SetAngle(radians: number) {
        const c = Math.cos(radians);
        const s = Math.sin(radians);
        this.ex.Set(c, s);
        this.ey.Set(-s, c);
        return this;
    }

    public Copy(other: b2Readonly<b2Mat22>) {
        this.ex.Copy(other.ex);
        this.ey.Copy(other.ey);
        return this;
    }

    /**
     * Set this to the identity matrix.
     */
    public SetIdentity() {
        this.ex.Set(1, 0);
        this.ey.Set(0, 1);
        return this;
    }

    /**
     * Set this matrix to all zeros.
     */
    public SetZero() {
        this.ex.SetZero();
        this.ey.SetZero();
        return this;
    }

    public GetAngle() {
        return Math.atan2(this.ex.y, this.ex.x);
    }

    /**
     * Solve A * x = b, where b is a column vector. This is more efficient
     * than computing the inverse in one-shot cases.
     */
    public Solve<T extends XY>(b_x: number, b_y: number, out: T) {
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

    public Abs() {
        this.ex.Abs();
        this.ey.Abs();
        return this;
    }

    public Inverse() {
        this.GetInverse(this);
        return this;
    }

    public Add(M: b2Readonly<b2Mat22>) {
        this.ex.Add(M.ex);
        this.ey.Add(M.ey);
        return this;
    }

    public Subtract(M: b2Readonly<b2Mat22>) {
        this.ex.Subtract(M.ex);
        this.ey.Subtract(M.ey);
        return this;
    }

    public GetInverse(out: b2Mat22) {
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

    public GetAbs(out: b2Mat22) {
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
    public static MultiplyVec2<T extends XY>(M: b2Readonly<b2Mat22>, v: Readonly<XY>, out: T) {
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
    public static TransposeMultiplyVec2<T extends XY>(M: b2Readonly<b2Mat22>, v: Readonly<XY>, out: T) {
        const v_x = v.x;
        const v_y = v.y;
        out.x = M.ex.x * v_x + M.ex.y * v_y;
        out.y = M.ey.x * v_x + M.ey.y * v_y;
        return out;
    }

    public static Add(A: b2Readonly<b2Mat22>, B: b2Readonly<b2Mat22>, out: b2Mat22) {
        out.ex.x = A.ex.x + B.ex.x;
        out.ex.y = A.ex.y + B.ex.y;
        out.ey.x = A.ey.x + B.ey.x;
        out.ey.y = A.ey.y + B.ey.y;
        return out;
    }

    /** A * B */
    public static Multiply(A: b2Readonly<b2Mat22>, B: b2Readonly<b2Mat22>, out: b2Mat22) {
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
    public static TransposeMultiply(A: b2Readonly<b2Mat22>, B: b2Readonly<b2Mat22>, out: b2Mat22) {
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
export class b2Mat33 {
    public static readonly IDENTITY: b2Readonly<b2Mat33> = new b2Mat33();

    public readonly ex = new b2Vec3(1, 0, 0);

    public readonly ey = new b2Vec3(0, 1, 0);

    public readonly ez = new b2Vec3(0, 0, 1);

    public Clone(): b2Mat33 {
        return new b2Mat33().Copy(this);
    }

    /**
     * Set this matrix using columns.
     */
    public SetColumns(c1: Readonly<XYZ>, c2: Readonly<XYZ>, c3: Readonly<XYZ>) {
        this.ex.Copy(c1);
        this.ey.Copy(c2);
        this.ez.Copy(c3);
        return this;
    }

    public Copy(other: b2Readonly<b2Mat33>) {
        this.ex.Copy(other.ex);
        this.ey.Copy(other.ey);
        this.ez.Copy(other.ez);
        return this;
    }

    public SetIdentity() {
        this.ex.Set(1, 0, 0);
        this.ey.Set(0, 1, 0);
        this.ez.Set(0, 0, 1);
        return this;
    }

    /**
     * Set this matrix to all zeros.
     */
    public SetZero() {
        this.ex.SetZero();
        this.ey.SetZero();
        this.ez.SetZero();
        return this;
    }

    public Add(M: b2Readonly<b2Mat33>) {
        this.ex.Add(M.ex);
        this.ey.Add(M.ey);
        this.ez.Add(M.ez);
        return this;
    }

    /**
     * Solve A * x = b, where b is a column vector. This is more efficient
     * than computing the inverse in one-shot cases.
     */
    public Solve33<T extends XYZ>(b_x: number, b_y: number, b_z: number, out: T) {
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
    public Solve22<T extends XY>(b_x: number, b_y: number, out: T) {
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
    public GetInverse22(M: b2Mat33) {
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
    public GetSymInverse33(M: b2Mat33) {
        let det = b2Vec3.Dot(this.ex, b2Vec3.Cross(this.ey, this.ez, b2Vec3.s_t0));
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
    public static MultiplyVec3<T extends XYZ>(A: b2Readonly<b2Mat33>, v: Readonly<XYZ>, out: T) {
        const { x, y, z } = v;
        out.x = A.ex.x * x + A.ey.x * y + A.ez.x * z;
        out.y = A.ex.y * x + A.ey.y * y + A.ez.y * z;
        out.z = A.ex.z * x + A.ey.z * y + A.ez.z * z;
        return out;
    }

    /**
     * Multiply a matrix times a vector.
     */
    public static MultiplyVec2<T extends XY>(A: b2Readonly<b2Mat33>, v: Readonly<XY>, out: T) {
        const { x, y } = v;
        out.x = A.ex.x * x + A.ey.x * y;
        out.y = A.ex.y * x + A.ey.y * y;
        return out;
    }
}

/**
 * Rotation
 */
export class b2Rot {
    public static readonly IDENTITY: b2Readonly<b2Rot> = new b2Rot();

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

    public Clone() {
        return new b2Rot().Copy(this);
    }

    public Copy(other: b2Readonly<b2Rot>) {
        this.s = other.s;
        this.c = other.c;
        return this;
    }

    /**
     * Set using an angle in radians.
     */
    public Set(angle: number) {
        this.s = Math.sin(angle);
        this.c = Math.cos(angle);
        return this;
    }

    /**
     * Set to the identity rotation
     */
    public SetIdentity() {
        this.s = 0;
        this.c = 1;
        return this;
    }

    /**
     * Get the angle in radians
     */
    public GetAngle() {
        return Math.atan2(this.s, this.c);
    }

    /**
     * Get the x-axis
     */
    public GetXAxis<T extends XY>(out: T) {
        out.x = this.c;
        out.y = this.s;
        return out;
    }

    /**
     * Get the u-axis
     */
    public GetYAxis<T extends XY>(out: T) {
        out.x = -this.s;
        out.y = this.c;
        return out;
    }

    /**
     * Multiply two rotations: q * r
     */
    public static Multiply(q: b2Readonly<b2Rot>, r: b2Readonly<b2Rot>, out: b2Rot) {
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
    public static TransposeMultiply(q: b2Readonly<b2Rot>, r: b2Readonly<b2Rot>, out: b2Rot) {
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
    public static MultiplyVec2<T extends XY>(q: b2Readonly<b2Rot>, v: Readonly<XY>, out: T) {
        const v_x = v.x;
        const v_y = v.y;
        out.x = q.c * v_x - q.s * v_y;
        out.y = q.s * v_x + q.c * v_y;
        return out;
    }

    /**
     * Inverse rotate a vector
     */
    public static TransposeMultiplyVec2<T extends XY>(q: b2Readonly<b2Rot>, v: Readonly<XY>, out: T) {
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
export class b2Transform {
    public static readonly IDENTITY: b2Readonly<b2Transform> = new b2Transform();

    public readonly p = new b2Vec2();

    public readonly q = new b2Rot();

    public Clone() {
        return new b2Transform().Copy(this);
    }

    public Copy(other: b2Readonly<b2Transform>) {
        this.p.Copy(other.p);
        this.q.Copy(other.q);
        return this;
    }

    /**
     * Set this to the identity transform.
     */
    public SetIdentity() {
        this.p.SetZero();
        this.q.SetIdentity();
        return this;
    }

    /**
     * Set this based on the position and rotation.
     */
    public SetPositionRotation(position: Readonly<XY>, q: b2Readonly<b2Rot>) {
        this.p.Copy(position);
        this.q.Copy(q);
        return this;
    }

    /**
     * Set this based on the position and angle.
     */
    public SetPositionAngle(pos: Readonly<XY>, a: number) {
        this.p.Copy(pos);
        this.q.Set(a);
        return this;
    }

    public SetPosition(position: Readonly<XY>) {
        this.p.Copy(position);
        return this;
    }

    public SetPositionXY(x: number, y: number) {
        this.p.Set(x, y);
        return this;
    }

    public SetRotation(rotation: b2Readonly<b2Rot>) {
        this.q.Copy(rotation);
        return this;
    }

    public SetRotationAngle(radians: number) {
        this.q.Set(radians);
        return this;
    }

    public GetPosition(): b2Readonly<b2Vec2> {
        return this.p;
    }

    public GetRotation(): b2Readonly<b2Rot> {
        return this.q;
    }

    public GetAngle() {
        return this.q.GetAngle();
    }

    public static MultiplyVec2<T extends XY>(T: b2Readonly<b2Transform>, v: Readonly<XY>, out: T) {
        const v_x = v.x;
        const v_y = v.y;
        out.x = T.q.c * v_x - T.q.s * v_y + T.p.x;
        out.y = T.q.s * v_x + T.q.c * v_y + T.p.y;
        return out;
    }

    public static TransposeMultiplyVec2<T extends XY>(T: b2Readonly<b2Transform>, v: Readonly<XY>, out: T) {
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
    public static Multiply(A: b2Readonly<b2Transform>, B: b2Readonly<b2Transform>, out: b2Transform) {
        b2Rot.Multiply(A.q, B.q, out.q);
        b2Rot.MultiplyVec2(A.q, B.p, out.p).Add(A.p);
        return out;
    }

    /**
     * v2 = A.q' * (B.q * v1 + B.p - A.p)
     *    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
     */
    public static TransposeMultiply(A: b2Readonly<b2Transform>, B: b2Readonly<b2Transform>, out: b2Transform) {
        b2Rot.TransposeMultiply(A.q, B.q, out.q);
        b2Rot.TransposeMultiplyVec2(A.q, b2Vec2.Subtract(B.p, A.p, out.p), out.p);
        return out;
    }
}

/**
 * This describes the motion of a body/shape for TOI computation.
 * Shapes are defined with respect to the body origin, which may
 * no coincide with the center of mass. However, to support dynamics
 * we must interpolate the center of mass position.
 */
export class b2Sweep {
    /** Local center of mass position */
    public readonly localCenter = new b2Vec2();

    /** Center world position at time 0 */
    public readonly c0 = new b2Vec2();

    /** Center world position at time 1 */
    public readonly c = new b2Vec2();

    /** World angle at time 0 */
    public a0 = 0;

    /** World angle at time 1 */
    public a = 0;

    /**
     * Fraction of the current time step in the range [0,1]
     * c0 and a0 are the positions at alpha0.
     */
    public alpha0 = 0;

    public Clone(): b2Sweep {
        return new b2Sweep().Copy(this);
    }

    public Copy(other: b2Sweep) {
        this.localCenter.Copy(other.localCenter);
        this.c0.Copy(other.c0);
        this.c.Copy(other.c);
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
    public GetTransform(xf: b2Transform, beta: number) {
        const oneMinusBeta = 1 - beta;
        xf.p.x = oneMinusBeta * this.c0.x + beta * this.c.x;
        xf.p.y = oneMinusBeta * this.c0.y + beta * this.c.y;
        const angle = oneMinusBeta * this.a0 + beta * this.a;
        xf.q.Set(angle);

        // Shift to origin
        xf.p.Subtract(b2Rot.MultiplyVec2(xf.q, this.localCenter, b2Vec2.s_t0));
        return xf;
    }

    /**
     * Advance the sweep forward, yielding a new initial state.
     *
     * @param alpha The new initial time.
     */
    public Advance(alpha: number) {
        // DEBUG: b2Assert(this.alpha0 < 1);
        const beta = (alpha - this.alpha0) / (1 - this.alpha0);
        this.c0.x += beta * (this.c.x - this.c0.x);
        this.c0.y += beta * (this.c.y - this.c0.y);
        this.a0 += beta * (this.a - this.a0);
        this.alpha0 = alpha;
    }

    /**
     * Normalize an angle in radians to be between -pi and pi
     */
    public Normalize() {
        const d = b2_two_pi * Math.floor(this.a0 / b2_two_pi);
        this.a0 -= d;
        this.a -= d;
    }
}
