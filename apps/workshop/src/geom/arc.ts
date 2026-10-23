import { arcGeometry, sampleArc } from '@botfoundry/cad';
import type { Loop, Segment, V2 } from '@botfoundry/model';

// Arc geometry (arcGeometry, bulgeThrough) comes from the kernel so the sketcher and the solid
// builder cannot disagree. What lives here is only how the sketcher draws a segment on screen.

/** Points along one segment after its start, ending exactly at `seg.to`. Arcs are sampled at most `step` rad apart. */
export function sampleSegment(from: V2, seg: Segment, step = (4 * Math.PI) / 180): V2[] {
  const arc = arcGeometry(from, seg.to, seg.bulge ?? 0);
  if (!arc) return [seg.to];
  const n = Math.max(2, Math.ceil(Math.abs(arc.sweep) / step));
  return [...sampleArc(arc, n), seg.to];
}

/** Polyline of a loop: the start point, then every sampled segment. A closed loop ends where it began. */
export function sampleLoop(loop: Loop, step?: number): V2[] {
  const pts: V2[] = [loop.start];
  let from = loop.start;
  for (const seg of loop.segments) {
    pts.push(...sampleSegment(from, seg, step));
    from = seg.to;
  }
  return pts;
}
