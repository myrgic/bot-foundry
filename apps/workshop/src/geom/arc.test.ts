import { describe, expect, it } from 'vitest';
import { sampleLoop, sampleSegment } from './arc.ts';

// Arc geometry itself (bulge, centre, radius) is tested with the kernel helpers in packages/cad.
describe('arc sampling', () => {
  it('samples a circle loop on its radius and closes it', () => {
    const r = 0.02;
    const pts = sampleLoop({ start: [-r, 0], segments: [{ to: [r, 0], bulge: 1 }, { to: [-r, 0], bulge: 1 }] });
    expect(pts[0]).toEqual(pts[pts.length - 1]);
    for (const p of pts) expect(Math.hypot(p[0], p[1])).toBeCloseTo(r, 9);
    expect(pts.length).toBeGreaterThan(40);
  });

  it('ends a segment exactly at its target, and a straight one is just that point', () => {
    expect(sampleSegment([0, 0], { to: [1, 0] })).toEqual([[1, 0]]);
    const pts = sampleSegment([-1, 0], { to: [1, 0], bulge: 1 });
    expect(pts[pts.length - 1]).toEqual([1, 0]);
  });
});
