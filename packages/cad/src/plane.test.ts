import { describe, expect, it } from 'vitest';
import { PLANE_XY, PLANE_XZ, PLANE_YZ } from '@botfoundry/model';
import { planeFrame, planeToWorld, worldToPlane } from './vec.ts';

describe('plane frame', () => {
  it('maps the standard planes the way the contract says (y = normal x xAxis)', () => {
    expect(planeFrame(PLANE_XY).y).toEqual([0, 1, 0]);
    expect(planeFrame(PLANE_XZ).y).toEqual([0, 0, -1]);
    expect(planeFrame(PLANE_YZ).y).toEqual([0, 0, 1]);
  });

  it('round-trips plane and world coordinates on a tilted plane', () => {
    const plane = { origin: [0.2, 0.03, 0] as const, normal: [Math.sin(0.2), Math.cos(0.2), 0] as const, xAxis: [0, 0, 1] as const };
    const w = planeToWorld(plane, [0.05, -0.02]);
    const back = worldToPlane(plane, w);
    expect(back[0]).toBeCloseTo(0.05, 12);
    expect(back[1]).toBeCloseTo(-0.02, 12);
  });

  it('agrees with the frame matrix the solid builder uses', () => {
    const plane = { origin: [0.1, 0.2, 0.3] as const, normal: [0, 1, 0] as const, xAxis: [1, 0, 0] as const };
    const f = planeFrame(plane);
    const w = planeToWorld(plane, [0.4, 0.5]);
    expect(w[0]).toBeCloseTo((f.matrix[0] as number) * 0.4 + (f.matrix[4] as number) * 0.5 + (f.matrix[12] as number), 12);
    expect(w[1]).toBeCloseTo((f.matrix[1] as number) * 0.4 + (f.matrix[5] as number) * 0.5 + (f.matrix[13] as number), 12);
    expect(w[2]).toBeCloseTo((f.matrix[2] as number) * 0.4 + (f.matrix[6] as number) * 0.5 + (f.matrix[14] as number), 12);
  });
});
