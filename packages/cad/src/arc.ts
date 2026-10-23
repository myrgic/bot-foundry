import type { V2 } from '@botfoundry/model';

// Arcs follow the DXF bulge convention used by the model: bulge = tan(sweep / 4), positive for
// counter-clockwise travel from the segment start to its end. The kernel tessellates with these
// helpers and the workshop sketcher draws and edits with the same ones, so the two cannot disagree
// about where an arc goes.

const TAU = Math.PI * 2;
const wrap = (a: number): number => ((a % TAU) + TAU) % TAU;

export interface ArcGeometry {
  center: V2;
  radius: number;
  /** angle of the start point, rad */
  start: number;
  /** signed sweep, rad (positive counter-clockwise) */
  sweep: number;
}

/** Centre, radius and angles of the arc from p0 to p1 with the given bulge, or null for a straight segment. */
export function arcGeometry(p0: V2, p1: V2, bulge: number): ArcGeometry | null {
  const chord = Math.hypot(p1[0] - p0[0], p1[1] - p0[1]);
  if (chord <= 1e-12 || Math.abs(bulge) <= 1e-12) return null;
  const sweep = 4 * Math.atan(bulge);
  // centre sits on the left of the chord for CCW arcs: mid + leftNormal * (c / 2) / tan(sweep / 2)
  const offset = chord / 2 / Math.tan(sweep / 2);
  const mx = (p0[0] + p1[0]) / 2, my = (p0[1] + p1[1]) / 2;
  const cx = mx - ((p1[1] - p0[1]) / chord) * offset;
  const cy = my + ((p1[0] - p0[0]) / chord) * offset;
  return {
    center: [cx, cy],
    radius: Math.hypot(p0[0] - cx, p0[1] - cy),
    start: Math.atan2(p0[1] - cy, p0[0] - cx),
    sweep,
  };
}

/** The `n - 1` points that split the arc into `n` equal steps, excluding both end points. */
export function sampleArc(arc: ArcGeometry, n: number): V2[] {
  const out: V2[] = [];
  for (let i = 1; i < n; i++) {
    const a = arc.start + (arc.sweep * i) / n;
    out.push([arc.center[0] + arc.radius * Math.cos(a), arc.center[1] + arc.radius * Math.sin(a)]);
  }
  return out;
}

/**
 * Bulge of the circular arc that runs from p0 to p1 through m. Returns 0 when the three points are
 * (nearly) collinear, which is a straight line.
 */
export function bulgeThrough(p0: V2, p1: V2, m: V2): number {
  const ax = p0[0], ay = p0[1], bx = m[0], by = m[1], cx = p1[0], cy = p1[1];
  const d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
  const scale = Math.hypot(cx - ax, cy - ay) * Math.hypot(bx - ax, by - ay);
  if (Math.abs(d) < 1e-9 * Math.max(scale, 1e-12)) return 0;
  const a2 = ax * ax + ay * ay, b2 = bx * bx + by * by, c2 = cx * cx + cy * cy;
  const ox = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d;
  const oy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d;
  const a0 = Math.atan2(ay - oy, ax - ox);
  const am = Math.atan2(by - oy, bx - ox);
  const a1 = Math.atan2(cy - oy, cx - ox);
  const ccw = wrap(a1 - a0);
  // m lies on the counter-clockwise arc when it is reached before p1 going counter-clockwise
  const sweep = wrap(am - a0) < ccw ? ccw : ccw - TAU;
  return Math.tan(sweep / 4);
}
