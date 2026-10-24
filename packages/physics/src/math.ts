// Small fixed-size vector, quaternion and matrix helpers on Float64Array, written with out
// parameters so the solver's inner loops allocate nothing. Quaternions are [x, y, z, w].
// Only +, -, *, / and Math.sqrt (correctly rounded by IEEE 754) touch simulation state, so
// results are bit-identical across platforms; see datan2 for the one angle we need.

export type Vec3 = Float64Array;
export type Quat = Float64Array;
/** 3x3 matrix, row-major. */
export type Mat3 = Float64Array;

export function vec3(x = 0, y = 0, z = 0): Vec3 {
  const v = new Float64Array(3);
  v[0] = x;
  v[1] = y;
  v[2] = z;
  return v;
}

export function quat(x = 0, y = 0, z = 0, w = 1): Quat {
  const q = new Float64Array(4);
  q[0] = x;
  q[1] = y;
  q[2] = z;
  q[3] = w;
  return q;
}

export function set3(out: Vec3, x: number, y: number, z: number): Vec3 {
  out[0] = x;
  out[1] = y;
  out[2] = z;
  return out;
}

export function copy3(out: Vec3, a: ArrayLike<number>): Vec3 {
  out[0] = a[0]!;
  out[1] = a[1]!;
  out[2] = a[2]!;
  return out;
}

export function copy4(out: Quat, a: ArrayLike<number>): Quat {
  out[0] = a[0]!;
  out[1] = a[1]!;
  out[2] = a[2]!;
  out[3] = a[3]!;
  return out;
}

export function sub3(out: Vec3, a: Vec3, b: Vec3): Vec3 {
  out[0] = a[0]! - b[0]!;
  out[1] = a[1]! - b[1]!;
  out[2] = a[2]! - b[2]!;
  return out;
}

export function dot3(a: Vec3, b: Vec3): number {
  return a[0]! * b[0]! + a[1]! * b[1]! + a[2]! * b[2]!;
}

export function cross3(out: Vec3, a: Vec3, b: Vec3): Vec3 {
  const x = a[1]! * b[2]! - a[2]! * b[1]!;
  const y = a[2]! * b[0]! - a[0]! * b[2]!;
  const z = a[0]! * b[1]! - a[1]! * b[0]!;
  out[0] = x;
  out[1] = y;
  out[2] = z;
  return out;
}

export function length3(a: Vec3): number {
  return Math.sqrt(dot3(a, a));
}

export function normalize3(out: Vec3, a: Vec3): Vec3 {
  const l = length3(a);
  const s = l > 0 ? 1 / l : 0;
  out[0] = a[0]! * s;
  out[1] = a[1]! * s;
  out[2] = a[2]! * s;
  return out;
}

/** out = q · v · q* */
export function rotate(out: Vec3, q: Quat, v: Vec3): Vec3 {
  const qx = q[0]!, qy = q[1]!, qz = q[2]!, qw = q[3]!;
  const vx = v[0]!, vy = v[1]!, vz = v[2]!;
  // t = 2 (u × v); v' = v + w t + u × t
  const tx = 2 * (qy * vz - qz * vy);
  const ty = 2 * (qz * vx - qx * vz);
  const tz = 2 * (qx * vy - qy * vx);
  out[0] = vx + qw * tx + (qy * tz - qz * ty);
  out[1] = vy + qw * ty + (qz * tx - qx * tz);
  out[2] = vz + qw * tz + (qx * ty - qy * tx);
  return out;
}

/** out = q* · v · q, the inverse rotation. */
export function rotateInv(out: Vec3, q: Quat, v: Vec3): Vec3 {
  const qx = -q[0]!, qy = -q[1]!, qz = -q[2]!, qw = q[3]!;
  const vx = v[0]!, vy = v[1]!, vz = v[2]!;
  const tx = 2 * (qy * vz - qz * vy);
  const ty = 2 * (qz * vx - qx * vz);
  const tz = 2 * (qx * vy - qy * vx);
  out[0] = vx + qw * tx + (qy * tz - qz * ty);
  out[1] = vy + qw * ty + (qz * tx - qx * tz);
  out[2] = vz + qw * tz + (qx * ty - qy * tx);
  return out;
}

export function quatMul(out: Quat, a: Quat, b: Quat): Quat {
  const ax = a[0]!, ay = a[1]!, az = a[2]!, aw = a[3]!;
  const bx = b[0]!, by = b[1]!, bz = b[2]!, bw = b[3]!;
  out[0] = aw * bx + ax * bw + ay * bz - az * by;
  out[1] = aw * by - ax * bz + ay * bw + az * bx;
  out[2] = aw * bz + ax * by - ay * bx + az * bw;
  out[3] = aw * bw - ax * bx - ay * by - az * bz;
  return out;
}

export function quatConj(out: Quat, q: Quat): Quat {
  out[0] = -q[0]!;
  out[1] = -q[1]!;
  out[2] = -q[2]!;
  out[3] = q[3]!;
  return out;
}

/** Scales q to unit length in place. A zero-length q comes back as all zeros, so callers that can see one must check first. */
export function quatNormalize(q: Quat): Quat {
  const l = Math.sqrt(q[0]! * q[0]! + q[1]! * q[1]! + q[2]! * q[2]! + q[3]! * q[3]!);
  const s = l > 0 ? 1 / l : 0;
  q[0] = q[0]! * s;
  q[1] = q[1]! * s;
  q[2] = q[2]! * s;
  q[3] = q[3]! * s;
  return q;
}

/**
 * q ← normalize(q + ½ (s·w, 0) ⊗ q): apply a small world-frame rotation vector s·w.
 * This is the AVBD reference's `quat + float3` operator, used for both integration and updates.
 */
export function quatAddRotation(out: Quat, q: Quat, w: ArrayLike<number>, s: number): Quat {
  const wx = 0.5 * s * w[0]!, wy = 0.5 * s * w[1]!, wz = 0.5 * s * w[2]!;
  const qx = q[0]!, qy = q[1]!, qz = q[2]!, qw = q[3]!;
  out[0] = qx + (wy * qz - wz * qy + wx * qw);
  out[1] = qy + (wz * qx - wx * qz + wy * qw);
  out[2] = qz + (wx * qy - wy * qx + wz * qw);
  out[3] = qw - (wx * qx + wy * qy + wz * qz);
  // never zero-length: (s·w, 0) ⊗ q is orthogonal to a unit q, so |out|² = 1 + |½ s·w|²
  return quatNormalize(out);
}

/**
 * World-frame rotation vector taking b to a, small-angle form: 2 · vec(a ⊗ b*), on the short arc.
 * Matches the AVBD reference's `quat - quat` operator.
 */
export function quatDiff(out: Vec3, a: Quat, b: Quat): Vec3 {
  const ax = a[0]!, ay = a[1]!, az = a[2]!, aw = a[3]!;
  const bx = -b[0]!, by = -b[1]!, bz = -b[2]!, bw = b[3]!;
  const x = aw * bx + ax * bw + ay * bz - az * by;
  const y = aw * by - ax * bz + ay * bw + az * bx;
  const z = aw * bz + ax * by - ay * bx + az * bw;
  const w = aw * bw - ax * bx - ay * by - az * bz;
  const s = w < 0 ? -2 : 2;
  out[0] = x * s;
  out[1] = y * s;
  out[2] = z * s;
  return out;
}

/** Rotation matrix of q, row-major. */
export function quatToMat3(out: Mat3, q: Quat): Mat3 {
  const x = q[0]!, y = q[1]!, z = q[2]!, w = q[3]!;
  const xx = x * x, yy = y * y, zz = z * z;
  const xy = x * y, xz = x * z, yz = y * z;
  const wx = w * x, wy = w * y, wz = w * z;
  out[0] = 1 - 2 * (yy + zz);
  out[1] = 2 * (xy - wz);
  out[2] = 2 * (xz + wy);
  out[3] = 2 * (xy + wz);
  out[4] = 1 - 2 * (xx + zz);
  out[5] = 2 * (yz - wx);
  out[6] = 2 * (xz - wy);
  out[7] = 2 * (yz + wx);
  out[8] = 1 - 2 * (xx + yy);
  return out;
}

/** out = R · S · Rᵀ for a rotation R and symmetric S (both row-major 3x3). */
export function rotateTensor(out: Mat3, r: Mat3, s: Mat3): Mat3 {
  const t = TMP_M;
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      t[i * 3 + j] = r[i * 3]! * s[j]! + r[i * 3 + 1]! * s[3 + j]! + r[i * 3 + 2]! * s[6 + j]!;
    }
  }
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      out[i * 3 + j] = t[i * 3]! * r[j * 3]! + t[i * 3 + 1]! * r[j * 3 + 1]! + t[i * 3 + 2]! * r[j * 3 + 2]!;
    }
  }
  return out;
}
const TMP_M = new Float64Array(9);

/** Two unit vectors completing n to a right-handed orthonormal basis (n, t1, t2). */
export function tangentBasis(n: Vec3, t1: Vec3, t2: Vec3): void {
  // same branch as the AVBD reference's orthonormal()
  if (Math.abs(n[0]!) > Math.abs(n[2]!)) set3(t1, -n[1]!, n[0]!, 0);
  else set3(t1, 0, -n[2]!, n[1]!);
  normalize3(t1, t1);
  cross3(t2, n, t1);
}

/**
 * atan2 from +, *, / and sqrt only, so hinge angles are bit-identical on every platform
 * (Math.atan2 is implementation-approximated in ECMAScript). Two half-angle reductions,
 * atan(x) = 2·atan(x / (1 + √(1 + x²))), bring |x| under tan(π/16) ≈ 0.2, where the Taylor
 * series to x¹⁹ is accurate to about 1e-15.
 */
export function datan2(y: number, x: number): number {
  if (x === 0 && y === 0) return 0;
  const ax = Math.abs(x), ay = Math.abs(y);
  // atan of the smaller ratio, then reflect
  const swap = ay > ax;
  let t = swap ? ax / ay : ay / ax;
  t = t / (1 + Math.sqrt(1 + t * t));
  t = t / (1 + Math.sqrt(1 + t * t));
  const t2 = t * t;
  let s = 0;
  for (let k = 19; k >= 1; k -= 2) s = 1 / k - t2 * s;
  // s now holds 1 - t²/3 + t⁴/5 - ... ; the loop builds it inside-out
  let a = 4 * t * s;
  if (swap) a = HALF_PI - a;
  if (x < 0) a = PI - a;
  return y < 0 ? -a : a;
}
const PI = 3.141592653589793;
const HALF_PI = 1.5707963267948966;
