import { planeFrame, planeToWorld, worldToPlane } from '@botfoundry/cad';
import type { Plane, V3 } from '@botfoundry/model';

// The plane basis and the sketch <-> world mapping come from the kernel (planeFrame, planeToWorld,
// worldToPlane), so the sketcher uses exactly the frame the solid builder extrudes along. What
// lives here is the editing helpers the kernel has no use for.

const dot = (a: V3, b: V3): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const unit = (a: V3): V3 => {
  const l = Math.hypot(a[0], a[1], a[2]) || 1;
  return [a[0] / l, a[1] / l, a[2] / l];
};

export interface PlaneBasis {
  u: V3;
  v: V3;
  n: V3;
}

export function planeBasis(plane: Plane): PlaneBasis {
  const f = planeFrame(plane);
  return { u: f.x, v: f.y, n: f.normal };
}

export { planeToWorld, worldToPlane };

/** The plane moved along its normal. */
export function offsetPlane(plane: Plane, distance: number): Plane {
  const n = unit(plane.normal);
  return {
    ...plane,
    origin: [plane.origin[0] + n[0] * distance, plane.origin[1] + n[1] * distance, plane.origin[2] + n[2] * distance],
  };
}

/** The same plane seen from the other side: normal flipped, u kept, so v flips too. */
export function flipPlane(plane: Plane): Plane {
  return { ...plane, normal: [-plane.normal[0], -plane.normal[1], -plane.normal[2]] };
}

/** Distance along the ray from `from` along `dir` to the plane, or null when parallel or behind. */
export function rayPlane(plane: Plane, from: V3, dir: V3): number | null {
  const n = unit(plane.normal);
  const denom = dot(dir, n);
  if (Math.abs(denom) < 1e-9) return null;
  const t = dot([plane.origin[0] - from[0], plane.origin[1] - from[1], plane.origin[2] - from[2]], n) / denom;
  return t >= 0 ? t : null;
}

/** Name of the signed world axis a unit vector points along, or null when it is not axis aligned. */
export function axisName(n: V3): string | null {
  const a = unit(n);
  const names = ['X', 'Y', 'Z'];
  for (let i = 0; i < 3; i++) {
    const c = a[i] as number;
    if (Math.abs(c) > 0.9995) return `${c > 0 ? '+' : '-'}${names[i]}`;
  }
  return null;
}
