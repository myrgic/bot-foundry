import type { Loop, Profile, V2 } from '@botfoundry/model';
import { arcGeometry, sampleArc } from './arc.ts';

/** Maximum distance between an arc and its chord approximation, m. */
export const CHORD_TOLERANCE = 0.0002;

/** A loop whose end point is further than this from its start is reported as open, m. */
const CLOSURE_TOLERANCE = 1e-7;
const MIN_AREA = 1e-10;

/**
 * Number of chords needed to keep the sagitta of an arc of radius r and sweep `sweep` below `tol`.
 * Sagitta of a chord spanning angle φ is r (1 - cos(φ/2)), so φ = 2 acos(1 - tol / r).
 */
export function arcSegmentCount(radius: number, sweep: number, tol = CHORD_TOLERANCE): number {
  if (radius <= tol) return Math.max(1, Math.ceil(Math.abs(sweep) / Math.PI));
  const maxStep = 2 * Math.acos(1 - tol / radius);
  return Math.max(1, Math.ceil(Math.abs(sweep) / maxStep));
}

/**
 * Polyline of a loop with arcs tessellated. The returned ring is open (the closing point equals
 * the first one and is dropped). Bulge is the DXF convention, tan(sweep / 4), positive CCW.
 */
export function tessellateLoop(loop: Loop, tol = CHORD_TOLERANCE): V2[] {
  const pts: V2[] = [loop.start];
  let p0: V2 = loop.start;
  for (const seg of loop.segments) {
    const p1 = seg.to;
    const bulge = seg.bulge ?? 0;
    const arc = arcGeometry(p0, p1, bulge);
    if (arc) pts.push(...sampleArc(arc, arcSegmentCount(arc.radius, arc.sweep, tol)));
    pts.push(p1);
    p0 = p1;
  }
  // drop the closing point and consecutive duplicates
  const ring: V2[] = [];
  for (let i = 0; i < pts.length; i++) {
    const p = pts[i] as V2;
    const prev = ring[ring.length - 1];
    if (prev && Math.hypot(p[0] - prev[0], p[1] - prev[1]) < 1e-12) continue;
    ring.push(p);
  }
  const first = ring[0];
  const last = ring[ring.length - 1];
  if (first && last && ring.length > 1 && Math.hypot(first[0] - last[0], first[1] - last[1]) < 1e-9) ring.pop();
  return ring;
}

export function signedArea(ring: readonly V2[]): number {
  let a = 0;
  for (let i = 0; i < ring.length; i++) {
    const p = ring[i] as V2;
    const q = ring[(i + 1) % ring.length] as V2;
    a += p[0] * q[1] - q[0] * p[1];
  }
  return a / 2;
}

function orient(a: V2, b: V2, c: V2): number {
  return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}

function onSegment(a: V2, b: V2, p: V2): boolean {
  return (
    Math.min(a[0], b[0]) - 1e-12 <= p[0] && p[0] <= Math.max(a[0], b[0]) + 1e-12 &&
    Math.min(a[1], b[1]) - 1e-12 <= p[1] && p[1] <= Math.max(a[1], b[1]) + 1e-12
  );
}

/** True when closed segments ab and cd share any point. */
function segmentsTouch(a: V2, b: V2, c: V2, d: V2): boolean {
  const eps = 1e-14;
  const o1 = orient(a, b, c), o2 = orient(a, b, d), o3 = orient(c, d, a), o4 = orient(c, d, b);
  if (((o1 > eps && o2 < -eps) || (o1 < -eps && o2 > eps)) && ((o3 > eps && o4 < -eps) || (o3 < -eps && o4 > eps))) return true;
  if (Math.abs(o1) <= eps && onSegment(a, b, c)) return true;
  if (Math.abs(o2) <= eps && onSegment(a, b, d)) return true;
  if (Math.abs(o3) <= eps && onSegment(c, d, a)) return true;
  if (Math.abs(o4) <= eps && onSegment(c, d, b)) return true;
  return false;
}

function ringSelfIntersects(ring: readonly V2[]): boolean {
  const n = ring.length;
  for (let i = 0; i < n; i++) {
    const a = ring[i] as V2, b = ring[(i + 1) % n] as V2;
    for (let j = i + 1; j < n; j++) {
      // edges sharing a vertex are adjacent and allowed to touch there
      if (j === i + 1 || (i === 0 && j === n - 1)) continue;
      if (segmentsTouch(a, b, ring[j] as V2, ring[(j + 1) % n] as V2)) return true;
    }
  }
  return false;
}

function ringsCross(r1: readonly V2[], r2: readonly V2[]): boolean {
  for (let i = 0; i < r1.length; i++) {
    const a = r1[i] as V2, b = r1[(i + 1) % r1.length] as V2;
    for (let j = 0; j < r2.length; j++) {
      if (segmentsTouch(a, b, r2[j] as V2, r2[(j + 1) % r2.length] as V2)) return true;
    }
  }
  return false;
}

/** Even-odd point-in-polygon. */
function inside(ring: readonly V2[], p: V2): boolean {
  let c = false;
  for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
    const a = ring[i] as V2, b = ring[j] as V2;
    if (a[1] > p[1] !== b[1] > p[1] && p[0] < ((b[0] - a[0]) * (p[1] - a[1])) / (b[1] - a[1]) + a[0]) c = !c;
  }
  return c;
}

export interface TessellatedProfile {
  /** outer ring first, then holes; open rings, no orientation guarantee */
  rings: V2[][];
  errors: string[];
}

function loopClosed(loop: Loop): boolean {
  const last = loop.segments[loop.segments.length - 1];
  if (!last) return false;
  return Math.hypot(last.to[0] - loop.start[0], last.to[1] - loop.start[1]) <= CLOSURE_TOLERANCE;
}

/** Tessellate and validate: loops closed, no self-intersection, holes inside the outer boundary and apart. */
export function tessellateProfile(profile: Profile, tol = CHORD_TOLERANCE): TessellatedProfile {
  const errors: string[] = [];
  const loops = [profile.outer, ...profile.holes];
  const rings: V2[][] = [];
  loops.forEach((loop, i) => {
    const name = i === 0 ? 'outer loop' : `hole ${i - 1}`;
    if (!loopClosed(loop)) {
      errors.push(`${name} is not closed (its last segment must end at its start)`);
      rings.push([]);
      return;
    }
    const ring = tessellateLoop(loop, tol);
    if (ring.length < 3) errors.push(`${name} encloses no area`);
    else if (ringSelfIntersects(ring)) errors.push(`${name} intersects itself`);
    else if (Math.abs(signedArea(ring)) < MIN_AREA) errors.push(`${name} encloses no area`);
    rings.push(ring);
  });
  if (errors.length > 0) return { rings, errors };

  const outer = rings[0] as V2[];
  for (let i = 1; i < rings.length; i++) {
    const hole = rings[i] as V2[];
    if (ringsCross(outer, hole)) errors.push(`hole ${i - 1} crosses the outer loop`);
    else if (!inside(outer, hole[0] as V2)) errors.push(`hole ${i - 1} lies outside the outer loop`);
    for (let j = 1; j < i; j++) {
      const other = rings[j] as V2[];
      if (ringsCross(other, hole)) errors.push(`holes ${j - 1} and ${i - 1} cross`);
      else if (inside(other, hole[0] as V2) || inside(hole, other[0] as V2)) errors.push(`holes ${j - 1} and ${i - 1} are nested`);
    }
  }
  return { rings, errors };
}
