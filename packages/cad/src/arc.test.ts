import { describe, expect, it } from 'vitest';
import { arcGeometry, bulgeThrough, sampleArc } from './arc.ts';
import { tessellateLoop } from './profile.ts';
import { circleLoop } from './sketch.ts';

describe('arc helpers', () => {
  it('finds a half circle through three points (bulge 1, counter-clockwise)', () => {
    expect(bulgeThrough([-1, 0], [1, 0], [0, -1])).toBeCloseTo(1, 12);
    expect(bulgeThrough([-1, 0], [1, 0], [0, 1])).toBeCloseTo(-1, 12);
  });

  it('finds a quarter circle and a major arc', () => {
    const s = Math.SQRT1_2;
    expect(bulgeThrough([1, 0], [0, 1], [s, s])).toBeCloseTo(Math.tan(Math.PI / 8), 12);
    // three-quarter turn the long way round
    expect(bulgeThrough([1, 0], [0, 1], [-s, -s])).toBeCloseTo(-Math.tan((3 * Math.PI) / 8), 12);
  });

  it('treats collinear points as a line', () => {
    expect(bulgeThrough([0, 0], [2, 0], [1, 0])).toBe(0);
  });

  it('recovers centre and radius from a bulge', () => {
    const g = arcGeometry([-1, 0], [1, 0], 1);
    expect(g?.center[0]).toBeCloseTo(0, 12);
    expect(g?.center[1]).toBeCloseTo(0, 12);
    expect(g?.radius).toBeCloseTo(1, 12);
    expect(g?.sweep).toBeCloseTo(Math.PI, 12);
  });

  it('returns null for a straight or degenerate segment', () => {
    expect(arcGeometry([0, 0], [1, 0], 0)).toBeNull();
    expect(arcGeometry([1, 1], [1, 1], 1)).toBeNull();
  });

  it('samples interior points only, on the arc', () => {
    const g = arcGeometry([-1, 0], [1, 0], 1);
    if (!g) throw new Error('expected an arc');
    const pts = sampleArc(g, 4);
    expect(pts).toHaveLength(3);
    for (const p of pts) expect(Math.hypot(p[0], p[1])).toBeCloseTo(1, 12);
  });

  it('is what the kernel tessellates with', () => {
    const loop = circleLoop(0.1, 0.2, 0.05);
    const ring = tessellateLoop(loop);
    const first = loop.segments[0];
    const g = first ? arcGeometry(loop.start, first.to, first.bulge ?? 0) : null;
    if (!g) throw new Error('expected an arc');
    // the second ring point is the first interior sample of the first arc
    const n = (ring.length) / 2;
    expect(ring[1]).toEqual(sampleArc(g, n)[0]);
  });
});
