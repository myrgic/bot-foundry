import type { Mat4 } from 'manifold-3d';
import type { Plane, Quat, Transform, V3 } from '@botfoundry/model';

// Small fixed-size linear algebra. Matrices are 4x4 column-major (the layout manifold-3d expects):
// element (row r, column c) lives at index c * 4 + r.

export type M4 = number[];

export const add3 = (a: V3, b: V3): V3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub3 = (a: V3, b: V3): V3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const scale3 = (a: V3, s: number): V3 => [a[0] * s, a[1] * s, a[2] * s];
export const dot3 = (a: V3, b: V3): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross3 = (a: V3, b: V3): V3 => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
export const len3 = (a: V3): number => Math.hypot(a[0], a[1], a[2]);

export function norm3(a: V3): V3 {
  const l = len3(a);
  if (l < 1e-12) throw new Error('cannot normalise a zero-length vector');
  return [a[0] / l, a[1] / l, a[2] / l];
}

/** View a matrix in the tuple type manifold-3d expects. */
export const asMat4 = (m: readonly number[]): Mat4 => {
  if (m.length !== 16) throw new Error('expected a 4x4 matrix');
  return m as unknown as Mat4;
};

export const M4_IDENTITY: readonly number[] = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];

/** a · b (apply b first, then a). */
export function mul4(a: readonly number[], b: readonly number[]): M4 {
  const out = new Array<number>(16).fill(0);
  for (let c = 0; c < 4; c++) {
    for (let r = 0; r < 4; r++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += (a[k * 4 + r] ?? 0) * (b[c * 4 + k] ?? 0);
      out[c * 4 + r] = s;
    }
  }
  return out;
}

export function translation(t: V3): M4 {
  return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, t[0], t[1], t[2], 1];
}

/** Rotation matrix columns from a unit quaternion [x, y, z, w]; the input is normalised. */
export function rotationFromQuat(q: Quat): M4 {
  const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1;
  const x = q[0] / n, y = q[1] / n, z = q[2] / n, w = q[3] / n;
  const xx = x * x, yy = y * y, zz = z * z;
  const xy = x * y, xz = x * z, yz = y * z, wx = w * x, wy = w * y, wz = w * z;
  return [
    1 - 2 * (yy + zz), 2 * (xy + wz), 2 * (xz - wy), 0,
    2 * (xy - wz), 1 - 2 * (xx + zz), 2 * (yz + wx), 0,
    2 * (xz + wy), 2 * (yz - wx), 1 - 2 * (xx + yy), 0,
    0, 0, 0, 1,
  ];
}

export function fromTransform(t: Transform): M4 {
  const m = rotationFromQuat(t.rotation);
  m[12] = t.position[0];
  m[13] = t.position[1];
  m[14] = t.position[2];
  return m;
}

/** Matrix whose columns are the given basis vectors and origin. */
export function fromBasis(x: V3, y: V3, z: V3, origin: V3): M4 {
  return [x[0], x[1], x[2], 0, y[0], y[1], y[2], 0, z[0], z[1], z[2], 0, origin[0], origin[1], origin[2], 1];
}

export function applyPoint(m: readonly number[], p: V3): V3 {
  return [
    (m[0] ?? 0) * p[0] + (m[4] ?? 0) * p[1] + (m[8] ?? 0) * p[2] + (m[12] ?? 0),
    (m[1] ?? 0) * p[0] + (m[5] ?? 0) * p[1] + (m[9] ?? 0) * p[2] + (m[13] ?? 0),
    (m[2] ?? 0) * p[0] + (m[6] ?? 0) * p[1] + (m[10] ?? 0) * p[2] + (m[14] ?? 0),
  ];
}

export function applyDir(m: readonly number[], v: V3): V3 {
  return [
    (m[0] ?? 0) * v[0] + (m[4] ?? 0) * v[1] + (m[8] ?? 0) * v[2],
    (m[1] ?? 0) * v[0] + (m[5] ?? 0) * v[1] + (m[9] ?? 0) * v[2],
    (m[2] ?? 0) * v[0] + (m[6] ?? 0) * v[1] + (m[10] ?? 0) * v[2],
  ];
}

/** Inverse of a rigid transform (orthonormal rotation plus translation). */
export function invertRigid(m: readonly number[]): M4 {
  const g = (i: number) => m[i] ?? 0;
  const out: M4 = [
    g(0), g(4), g(8), 0,
    g(1), g(5), g(9), 0,
    g(2), g(6), g(10), 0,
    0, 0, 0, 1,
  ];
  const t = applyDir(out, [g(12), g(13), g(14)]);
  out[12] = -t[0];
  out[13] = -t[1];
  out[14] = -t[2];
  return out;
}

export function powM4(m: readonly number[], k: number): M4 {
  let out: M4 = [...M4_IDENTITY];
  for (let i = 0; i < k; i++) out = mul4(m, out);
  return out;
}

export interface PlaneFrame {
  origin: V3;
  x: V3;
  /** v axis = normal × x */
  y: V3;
  normal: V3;
  /** sketch space to design frame */
  matrix: M4;
}

/** Orthonormal frame of a datum plane: (x, y, normal) is right-handed with y = normal × x. */
export function planeFrame(plane: Plane): PlaneFrame {
  const normal = norm3(plane.normal);
  const xRaw = sub3(plane.xAxis, scale3(normal, dot3(plane.xAxis, normal)));
  if (len3(xRaw) < 1e-9) throw new Error('plane xAxis is parallel to its normal');
  const x = norm3(xRaw);
  const y = cross3(normal, x);
  return { origin: plane.origin, x, y, normal, matrix: fromBasis(x, y, normal, plane.origin) };
}

/** Sketch coordinates (u, v) to a point in the design frame: origin + u * x + v * y. */
export function planeToWorld(plane: Plane, p: readonly [number, number]): V3 {
  const f = planeFrame(plane);
  return [
    f.origin[0] + p[0] * f.x[0] + p[1] * f.y[0],
    f.origin[1] + p[0] * f.x[1] + p[1] * f.y[1],
    f.origin[2] + p[0] * f.x[2] + p[1] * f.y[2],
  ];
}

/** Project a design-frame point onto the plane's (u, v) coordinates. */
export function worldToPlane(plane: Plane, w: V3): [number, number] {
  const f = planeFrame(plane);
  const d = sub3(w, f.origin);
  return [dot3(d, f.x), dot3(d, f.y)];
}
