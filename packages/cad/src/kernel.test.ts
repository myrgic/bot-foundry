import { afterAll, beforeAll, describe, expect, it } from 'vitest';
import { MATERIALS, PLANE_XY, PLANE_XZ, type ComponentDef, type Plane } from '@botfoundry/model';
import { circleSegments } from './primitives.ts';
import { circleProfile, polygonLoop, rectProfile } from './sketch.ts';
import { body, bounds, makeDesign, sketch } from './testing.ts';
import { createKernel, type CadKernel } from './kernel.ts';

let kernel: CadKernel;
beforeAll(async () => {
  kernel = await createKernel();
});
afterAll(() => kernel.dispose());

/** Relative error. */
const rel = (actual: number, expected: number) => Math.abs(actual - expected) / Math.abs(expected);

describe('extrude', () => {
  it('rectangle volume and 6061 mass', () => {
    const d = makeDesign({
      sketches: [sketch('s', PLANE_XY, rectProfile(0, 0, 0.1, 0.05))],
      bodies: [body('b', 'al6061', { kind: 'billet' }, { kind: 'extrude', id: 'e', sketch: 's', distance: 0.01, op: 'new' })],
    });
    const b = kernel.evaluate(d).bodies[0]!;
    expect(b.errors).toEqual([]);
    expect(b.mass.volume).toBeCloseTo(0.1 * 0.05 * 0.01, 10);
    // mesh coordinates are float32, so compare to ~1e-6 relative
    expect(rel(b.mass.mass, 0.1 * 0.05 * 0.01 * MATERIALS.al6061.density)).toBeLessThan(1e-6);
    expect(rel(b.mass.mass, 0.135)).toBeLessThan(1e-6);
    expect(b.mass.com[0]).toBeCloseTo(0.05, 6);
    expect(b.mass.com[1]).toBeCloseTo(0.025, 6);
    expect(b.mass.com[2]).toBeCloseTo(0.005, 6);
  });

  it('maps sketch coordinates through the plane: v = normal × xAxis', () => {
    // XZ plane with normal +y and xAxis +x gives v = y × x = -z
    const d = makeDesign({
      sketches: [sketch('s', { ...PLANE_XZ, origin: [0, 0.5, 0] }, rectProfile(0, 0, 0.2, 0.1))],
      bodies: [body('b', 'a36', { kind: 'billet' }, { kind: 'extrude', id: 'e', sketch: 's', distance: 0.03, op: 'new' })],
    });
    const bb = bounds(kernel.evaluate(d).bodies[0]!.mesh.positions);
    expect(bb.min[0]).toBeCloseTo(0, 6);
    expect(bb.max[0]).toBeCloseTo(0.2, 6);
    expect(bb.min[1]).toBeCloseTo(0.5, 6);
    expect(bb.max[1]).toBeCloseTo(0.53, 6);
    expect(bb.min[2]).toBeCloseTo(-0.1, 6);
    expect(bb.max[2]).toBeCloseTo(0, 6);
  });

  it('extrudes symmetrically and with a negative distance', () => {
    const mk = (extra: object) => makeDesign({
      sketches: [sketch('s', PLANE_XY, rectProfile(0, 0, 0.1, 0.1))],
      bodies: [body('b', 'a36', { kind: 'billet' }, { kind: 'extrude', id: 'e', sketch: 's', distance: 0.04, op: 'new', ...extra })],
    });
    const sym = bounds(kernel.evaluate(mk({ symmetric: true })).bodies[0]!.mesh.positions);
    expect(sym.min[2]).toBeCloseTo(-0.02, 6);
    expect(sym.max[2]).toBeCloseTo(0.02, 6);
    const neg = kernel.evaluate(mk({ distance: -0.04 })).bodies[0]!;
    const nb = bounds(neg.mesh.positions);
    expect(nb.min[2]).toBeCloseTo(-0.04, 6);
    expect(nb.max[2]).toBeCloseTo(0, 6);
    expect(neg.mass.volume).toBeCloseTo(0.1 * 0.1 * 0.04, 10);
  });

  it('applies a draft angle: positive narrows the far face', () => {
    const w = 0.1, h = 0.06, len = 0.04, taper = 0.1;
    const d = makeDesign({
      sketches: [sketch('s', PLANE_XY, rectProfile(0, 0, w, h))],
      bodies: [body('b', 'a36', { kind: 'billet' }, { kind: 'extrude', id: 'e', sketch: 's', distance: len, taper, op: 'new' })],
    });
    const b = kernel.evaluate(d).bodies[0]!;
    expect(b.errors).toEqual([]);
    const inset = len * Math.tan(taper);
    const a1 = w * h;
    const a2 = (w - 2 * inset) * (h - 2 * inset);
    // the ends are not similar rectangles, so use the prismatoid rule (exact: area is quadratic in height)
    const mid = (w - inset) * (h - inset);
    const prismatoid = (len / 6) * (a1 + 4 * mid + a2);
    expect(rel(b.mass.volume, prismatoid)).toBeLessThan(1e-6);
    // the far face is smaller by the draft on every side
    const top = [];
    const pos = b.mesh.positions;
    for (let i = 0; i < pos.length; i += 3) if (Math.abs((pos[i + 2] as number) - len) < 1e-6) top.push(pos[i] as number);
    expect(Math.min(...top)).toBeCloseTo(inset, 6);
    expect(Math.max(...top)).toBeCloseTo(w - inset, 6);
  });

  it('reports a taper that closes the profile', () => {
    const d = makeDesign({
      sketches: [sketch('s', PLANE_XY, rectProfile(0, 0, 0.02, 0.02))],
      bodies: [body('b', 'a36', { kind: 'billet' }, { kind: 'extrude', id: 'e', sketch: 's', distance: 0.1, taper: 0.5, op: 'new' })],
    });
    expect(kernel.evaluate(d).bodies[0]!.errors.join()).toMatch(/closes the profile/);
  });

  it('unions several profiles of a sketch and honours a profile selection', () => {
    const d = makeDesign({
      sketches: [sketch('s', PLANE_XY, rectProfile(0, 0, 0.1, 0.1), rectProfile(0.2, 0, 0.25, 0.1))],
      bodies: [
        body('all', 'a36', { kind: 'billet' }, { kind: 'extrude', id: 'e', sketch: 's', distance: 0.01, op: 'new' }),
        body('one', 'a36', { kind: 'billet' }, { kind: 'extrude', id: 'e', sketch: 's', profiles: [1], distance: 0.01, op: 'new' }),
      ],
    });
    const [all, one] = kernel.evaluate(d).bodies;
    expect(all!.mass.volume).toBeCloseTo(0.015 * 0.01, 9);
    expect(one!.mass.volume).toBeCloseTo(0.005 * 0.01, 10);
  });
});

describe('mass properties', () => {
  it('box inertia matches the analytic tensor within 1 %, away from the origin', () => {
    const [a, b, c] = [0.2, 0.1, 0.05];
    const d = makeDesign({
      sketches: [sketch('s', { origin: [0.3, -0.2, 0.1], normal: [0, 0, 1], xAxis: [1, 0, 0] }, rectProfile(0, 0, a, b))],
      bodies: [body('b', 'a36', { kind: 'billet' }, { kind: 'extrude', id: 'e', sketch: 's', distance: c, op: 'new' })],
    });
    const m = kernel.evaluate(d).bodies[0]!.mass;
    const mass = a * b * c * MATERIALS.a36.density;
    expect(rel(m.mass, mass)).toBeLessThan(1e-6);
    expect(m.com[0]).toBeCloseTo(0.3 + a / 2, 6);
    expect(m.com[1]).toBeCloseTo(-0.2 + b / 2, 6);
    expect(m.com[2]).toBeCloseTo(0.1 + c / 2, 6);
    expect(rel(m.inertia[0], (mass * (b * b + c * c)) / 12)).toBeLessThan(0.01);
    expect(rel(m.inertia[1], (mass * (a * a + c * c)) / 12)).toBeLessThan(0.01);
    expect(rel(m.inertia[2], (mass * (a * a + b * b)) / 12)).toBeLessThan(0.01);
    for (const k of [3, 4, 5]) expect(Math.abs(m.inertia[k] as number)).toBeLessThan(1e-4 * mass * a * a);
  });
});

describe('revolve', () => {
  const rect = (x0: number, y0: number, x1: number, y1: number) => rectProfile(x0, y0, x1, y1);

  it('full revolution of a rectangle is a cylinder: π r² h', () => {
    const r = 0.03, h = 0.1;
    const d = makeDesign({
      sketches: [sketch('s', PLANE_XY, rect(0, 0, r, h))],
      bodies: [body('b', 'al7075', { kind: 'billet' }, {
        kind: 'revolve', id: 'r', sketch: 's', axis: { origin: [0, 0], direction: [0, 1] }, angle: 2 * Math.PI, op: 'new',
      })],
    });
    const b = kernel.evaluate(d).bodies[0]!;
    expect(b.errors).toEqual([]);
    expect(rel(b.mass.volume, Math.PI * r * r * h)).toBeLessThan(0.01);
    expect(b.mass.com[1]).toBeCloseTo(h / 2, 6);
    // inertia about the axis of a cylinder: m r² / 2 (within the polygonal approximation)
    expect(rel(b.mass.inertia[1], 0.5 * b.mass.mass * r * r)).toBeLessThan(0.02);
  });

  it('a partial revolution sweeps right-handed about the axis direction', () => {
    const r = 0.03, h = 0.1;
    const mk = (x0: number, x1: number) => makeDesign({
      sketches: [sketch('s', PLANE_XY, rect(x0, 0, x1, h))],
      bodies: [body('b', 'a36', { kind: 'billet' }, {
        kind: 'revolve', id: 'r', sketch: 's', axis: { origin: [0, 0], direction: [0, 1] }, angle: Math.PI / 2, op: 'new',
      })],
    });
    const right = kernel.evaluate(mk(0.01, r)).bodies[0]!;
    const rb = bounds(right.mass.volume > 0 ? right.mesh.positions : []);
    // +x turning about +y goes toward -z
    expect(rb.max[0]).toBeCloseTo(r, 6);
    expect(rb.min[2]).toBeCloseTo(-r, 3);
    expect(rb.max[2]).toBeCloseTo(0, 6);
    expect(rel(right.mass.volume, (Math.PI / 4) * (r * r - 0.01 * 0.01) * h)).toBeLessThan(0.012);

    // a profile on the left of the axis turns the same way (-x toward +z)
    const left = kernel.evaluate(mk(-r, -0.01)).bodies[0]!;
    const lb = bounds(left.mesh.positions);
    expect(lb.min[0]).toBeCloseTo(-r, 6);
    expect(lb.max[0]).toBeCloseTo(0, 6);
    expect(lb.min[2]).toBeCloseTo(0, 6);
    expect(lb.max[2]).toBeCloseTo(r, 3);
    expect(rel(left.mass.volume, right.mass.volume)).toBeLessThan(1e-5);
  });

  it('revolves about an off-origin, tilted axis', () => {
    // a tube (annulus profile beside the x axis, axis along x through y = 0.2)
    const d = makeDesign({
      sketches: [sketch('s', PLANE_XY, rect(0, 0.21, 0.1, 0.23))],
      bodies: [body('b', 'a36', { kind: 'billet' }, {
        kind: 'revolve', id: 'r', sketch: 's', axis: { origin: [0, 0.2], direction: [1, 0] }, angle: 2 * Math.PI, op: 'new',
      })],
    });
    const b = kernel.evaluate(d).bodies[0]!;
    expect(b.errors).toEqual([]);
    const ideal = Math.PI * (0.03 ** 2 - 0.01 ** 2) * 0.1;
    expect(rel(b.mass.volume, ideal)).toBeLessThan(0.01);
    expect(b.mass.com[1]).toBeCloseTo(0.2, 4);
    expect(b.mass.com[2]).toBeCloseTo(0, 4);
  });

  it('rejects a profile that crosses the axis', () => {
    const d = makeDesign({
      sketches: [sketch('s', PLANE_XY, rect(-0.01, 0, 0.02, 0.1))],
      bodies: [body('b', 'a36', { kind: 'billet' }, {
        kind: 'revolve', id: 'r', sketch: 's', axis: { origin: [0, 0], direction: [0, 1] }, angle: 2 * Math.PI, op: 'new',
      })],
    });
    expect(kernel.evaluate(d).bodies[0]!.errors.join()).toMatch(/crosses the revolve axis/);
  });
});

describe('booleans', () => {
  const plate = (extra: Parameters<typeof body>[3][] = []) => makeDesign({
    sketches: [
      sketch('plate', PLANE_XY, rectProfile(0, 0, 0.1, 0.1)),
      sketch('hole', PLANE_XY, circleProfile(0.05, 0.05, 0.01)),
    ],
    bodies: [body('b', 'al6061', { kind: 'billet' },
      { kind: 'extrude', id: 'e', sketch: 'plate', distance: 0.01, op: 'new' }, ...extra)],
  });

  it('subtract removes a drilled hole', () => {
    const d = plate([{ kind: 'extrude', id: 'h', sketch: 'hole', distance: 0.03, symmetric: true, op: 'subtract' }]);
    const v = kernel.evaluate(d).bodies[0]!.mass.volume;
    const ideal = 0.1 * 0.1 * 0.01 - Math.PI * 0.01 ** 2 * 0.01;
    expect(rel(v, ideal)).toBeLessThan(0.005);
    expect(v).toBeLessThan(1e-4 - 2.5e-6);
  });

  it('a hole in the profile gives the same part as a subtract', () => {
    const viaProfile = makeDesign({
      sketches: [sketch('p', PLANE_XY, rectProfile(0, 0, 0.1, 0.1, [{ start: [0.04, 0.05], segments: [{ to: [0.06, 0.05], bulge: 1 }, { to: [0.04, 0.05], bulge: 1 }] }]))],
      bodies: [body('b', 'al6061', { kind: 'billet' }, { kind: 'extrude', id: 'e', sketch: 'p', distance: 0.01, op: 'new' })],
    });
    const a = kernel.evaluate(viaProfile).bodies[0]!.mass.volume;
    const b = kernel.evaluate(plate([{ kind: 'extrude', id: 'h', sketch: 'hole', distance: 0.03, symmetric: true, op: 'subtract' }])).bodies[0]!.mass.volume;
    expect(rel(a, b)).toBeLessThan(1e-9);
  });

  it('add and intersect', () => {
    const d = makeDesign({
      sketches: [sketch('a', PLANE_XY, rectProfile(0, 0, 0.1, 0.1)), sketch('b', PLANE_XY, rectProfile(0.05, 0.05, 0.15, 0.15))],
      bodies: [
        body('add', 'a36', { kind: 'billet' },
          { kind: 'extrude', id: 'e1', sketch: 'a', distance: 0.01, op: 'new' },
          { kind: 'extrude', id: 'e2', sketch: 'b', distance: 0.01, op: 'add' }),
        body('and', 'a36', { kind: 'billet' },
          { kind: 'extrude', id: 'e1', sketch: 'a', distance: 0.01, op: 'new' },
          { kind: 'extrude', id: 'e2', sketch: 'b', distance: 0.01, op: 'intersect' }),
      ],
    });
    const [add, and] = kernel.evaluate(d).bodies;
    expect(add!.mass.volume).toBeCloseTo((0.01 + 0.01 - 0.0025) * 0.01, 9);
    expect(and!.mass.volume).toBeCloseTo(0.0025 * 0.01, 9);
  });

  it('mirror doubles a body across a plane', () => {
    const d = makeDesign({
      sketches: [sketch('s', PLANE_XY, rectProfile(0.1, 0, 0.2, 0.1))],
      bodies: [body('b', 'a36', { kind: 'billet' },
        { kind: 'extrude', id: 'e', sketch: 's', distance: 0.01, op: 'new' },
        { kind: 'mirror', id: 'm', plane: { origin: [0, 0, 0], normal: [1, 0, 0], xAxis: [0, 1, 0] } })],
    });
    const b = kernel.evaluate(d).bodies[0]!;
    expect(b.mass.volume).toBeCloseTo(2 * 0.1 * 0.1 * 0.01, 10);
    expect(b.mass.com[0]).toBeCloseTo(0, 6);
    const bb = bounds(b.mesh.positions);
    expect(bb.min[0]).toBeCloseTo(-0.2, 6);
    expect(bb.max[0]).toBeCloseTo(0.2, 6);
  });

  it('pattern repeats a feature with the step applied cumulatively', () => {
    const d = makeDesign({
      sketches: [sketch('s', PLANE_XY, rectProfile(0, 0, 0.02, 0.02))],
      bodies: [body('b', 'a36', { kind: 'billet' },
        { kind: 'extrude', id: 'e', sketch: 's', distance: 0.01, op: 'new' },
        { kind: 'pattern', id: 'p', feature: 'e', count: 4, step: { position: [0.03, 0, 0], rotation: [0, 0, 0, 1] } })],
    });
    const b = kernel.evaluate(d).bodies[0]!;
    expect(b.errors).toEqual([]);
    expect(b.mass.volume).toBeCloseTo(4 * 0.02 * 0.02 * 0.01, 10);
    expect(bounds(b.mesh.positions).max[0]).toBeCloseTo(0.09 + 0.02, 6);
  });

  it('reports bad feature trees without throwing', () => {
    const d = makeDesign({
      sketches: [sketch('s', PLANE_XY, rectProfile(0, 0, 0.1, 0.1))],
      bodies: [
        body('no-material', 'a36', { kind: 'billet' }, { kind: 'extrude', id: 'e', sketch: 's', distance: 0.01, op: 'subtract' }),
        body('bad-sketch', 'a36', { kind: 'billet' }, { kind: 'extrude', id: 'e', sketch: 'nope', distance: 0.01, op: 'new' }),
        body('bad-pattern', 'a36', { kind: 'billet' },
          { kind: 'extrude', id: 'e', sketch: 's', distance: 0.01, op: 'new' },
          { kind: 'pattern', id: 'p', feature: 'missing', count: 2, step: { position: [0, 0, 0], rotation: [0, 0, 0, 1] } }),
      ],
    });
    const [a, b, c] = kernel.evaluate(d).bodies;
    expect(a!.errors.length).toBeGreaterThan(0);
    expect(a!.mesh.indices).toHaveLength(0);
    expect(a!.mass.mass).toBe(0);
    expect(b!.errors.join()).toMatch(/sketch "nope" does not exist/);
    expect(c!.errors.join()).toMatch(/missing/);
    expect(c!.mass.volume).toBeGreaterThan(0); // the rest of the tree still builds
  });

  it('reports invalid sketches on the bodies that use them', () => {
    const d = makeDesign({
      sketches: [sketch('bowtie', PLANE_XY, { outer: polygonLoop([[0, 0], [1, 1], [1, 0], [0, 1]]), holes: [] })],
      bodies: [body('b', 'a36', { kind: 'billet' }, { kind: 'extrude', id: 'e', sketch: 'bowtie', distance: 0.01, op: 'new' })],
    });
    expect(kernel.evaluate(d).bodies[0]!.errors.join()).toMatch(/intersects itself/);
  });
});

describe('plates', () => {
  const mk = (distance: number, thickness: number) => makeDesign({
    sketches: [sketch('s', PLANE_XY, rectProfile(0, 0, 0.1, 0.1))],
    bodies: [body('p', 'al6061', { kind: 'plate', thickness }, { kind: 'extrude', id: 'e', sketch: 's', distance, op: 'new' })],
  });

  it('accepts a first extrude of exactly the thickness, in either direction', () => {
    expect(kernel.evaluate(mk(0.008, 0.008)).bodies[0]!.errors).toEqual([]);
    expect(kernel.evaluate(mk(-0.008, 0.008)).bodies[0]!.errors).toEqual([]);
  });

  it('reports a first extrude of the wrong depth', () => {
    expect(kernel.evaluate(mk(0.01, 0.008)).bodies[0]!.errors.join()).toMatch(/plate "p": the first extrude is 10 mm but the plate thickness is 8 mm/);
  });

  it('reports a first feature that is not an extrude', () => {
    const d = makeDesign({
      sketches: [sketch('s', PLANE_XY, rectProfile(0, 0, 0.1, 0.1))],
      bodies: [body('p', 'al6061', { kind: 'plate', thickness: 0.008 }, {
        kind: 'revolve', id: 'r', sketch: 's', axis: { origin: [-1, 0], direction: [0, 1] }, angle: 1, op: 'new',
      })],
    });
    expect(kernel.evaluate(d).bodies[0]!.errors.join()).toMatch(/first feature must be an extrude/);
  });
});

// ---- components and carving ----

const BLOCK: ComponentDef = {
  id: 'test.block',
  name: 'Test block',
  category: 'sensor',
  mass: 0.5,
  geometry: [{ kind: 'box', size: [0.02, 0.02, 0.02], at: [0, 0, 0] }],
  keepOut: [{ kind: 'box', size: [0.02, 0.02, 0.02], at: [0, 0, 0] }],
  mounts: [{ face: PLANE_XY, holes: [{ at: [0.03, 0], bolt: { size: 'M4', grade: '8.8', length: 0.008 } }] }],
  ports: [],
};

describe('carve', () => {
  const plane: Plane = PLANE_XY;
  const cube = makeDesign({
    sketches: [sketch('s', plane, rectProfile(0, 0, 0.1, 0.1))],
    bodies: [body('b', 'al6061', { kind: 'billet' },
      { kind: 'extrude', id: 'e', sketch: 's', distance: 0.1, op: 'new' },
      { kind: 'carve', id: 'c', component: 'blk', clearance: 0.001 })],
    components: [{ id: 'blk', def: 'test.block', pose: { position: [0.05, 0.05, 0.05], rotation: [0, 0, 0, 1] } }],
  });

  it('removes the keep-out grown by the clearance', async () => {
    const k = await createKernel({ catalog: [BLOCK] });
    const b = k.evaluate(cube).bodies[0]!;
    expect(b.errors).toEqual([]);
    expect(b.mass.volume).toBeCloseTo(1e-3 - 0.022 ** 3, 10);
    k.dispose();
  });

  it('drills bolt clearance holes (bolt + 0.5 mm) when the part is mounted to the body', async () => {
    const k = await createKernel({ catalog: [BLOCK] });
    const loose = k.evaluate(cube).bodies[0]!.mass.volume;
    const mounted = makeDesign({ ...cube, components: [{ ...cube.components[0]!, mountedTo: 'b' }] });
    const v = k.evaluate(mounted).bodies[0]!.mass.volume;
    const r = (0.004 + 0.0005) / 2;
    const n = circleSegments(r);
    const hole = (n / 2) * r * r * Math.sin((2 * Math.PI) / n) * 0.016; // one bolt length each side of the face
    expect(loose - v).toBeCloseTo(hole, 10);
    k.dispose();
  });

  it('drills along the mount face normal of the placed part', async () => {
    // M3 clearance hole radius 1.75 mm fits inside a 4 mm sheet when it lies along the sheet's plane
    const m3: ComponentDef = { ...BLOCK, id: 'test.block-m3', mounts: [{ face: PLANE_XY, holes: [{ at: [0.03, 0], bolt: { size: 'M3', grade: '8.8', length: 0.008 } }] }] };
    const k = await createKernel({ catalog: [m3] });
    const sheet = (rotation: [number, number, number, number]) => makeDesign({
      sketches: [sketch('s', PLANE_XY, rectProfile(0, 0, 0.1, 0.1))],
      bodies: [body('b', 'al6061', { kind: 'billet' },
        { kind: 'extrude', id: 'e', sketch: 's', distance: 0.004, op: 'new' },
        { kind: 'carve', id: 'c', component: 'blk', clearance: 0 })],
      components: [{ id: 'blk', def: 'test.block-m3', pose: { position: [0.05, 0.05, 0.002], rotation }, mountedTo: 'b' }],
    });
    const r = 0.00175;
    const n = circleSegments(r);
    const area = (n / 2) * r * r * Math.sin((2 * Math.PI) / n);
    const cube = 0.02 * 0.02 * 0.004;
    const sheetVolume = 0.1 * 0.1 * 0.004;
    const standing = k.evaluate(sheet([0, 0, 0, 1])).bodies[0]!.mass.volume;
    expect(rel(sheetVolume - standing, cube + area * 0.004)).toBeLessThan(1e-5); // through the 4 mm sheet
    const s = Math.SQRT1_2;
    const lying = k.evaluate(sheet([s, 0, 0, s])).bodies[0]!.mass.volume; // normal turned to y: 16 mm hole inside the sheet
    expect(rel(sheetVolume - lying, cube + area * 0.016)).toBeLessThan(1e-5);
    k.dispose();
  });

  it('carving the catalog gearmotor into a wall drills the output shaft hole through it', () => {
    // wall z 0..6 mm; the mount face sits on the wall's inner face (z = 0), can behind it, shaft out through it
    const wall = makeDesign({
      sketches: [sketch('s', PLANE_XY, rectProfile(0, 0, 0.1, 0.1))],
      bodies: [body('b', 'al6061', { kind: 'billet' },
        { kind: 'extrude', id: 'e', sketch: 's', distance: 0.006, op: 'new' },
        { kind: 'carve', id: 'c', component: 'm', clearance: 0.001 })],
      components: [{ id: 'm', def: 'motor.gearmotor-36mm-27to1', pose: { position: [0.05, 0.05, -0.095], rotation: [0, 0, 0, 1] } }],
    });
    const disc = (r: number) => { const n = circleSegments(r); return (n / 2) * r * r * Math.sin((2 * Math.PI) / n); };
    // the clearance grows each keep-out by 1 mm on every side, so the can pocket is 2 mm + 1 mm deep
    const pocket = disc(0.0205) * 0.003;
    // the shaft (4 mm radius + 1 mm clearance) carries on through the remaining 3 mm of the wall
    const shaftHole = disc(0.005) * 0.003;
    const b = kernel.evaluate(wall).bodies[0]!;
    expect(b.errors).toEqual([]);
    expect(rel(0.1 * 0.1 * 0.006 - b.mass.volume, pocket + shaftHole)).toBeLessThan(1e-5);
    // the hole reaches the wall's outer face: a vertex ring of radius 5 mm sits at z = 6 mm
    let ring = 0;
    const pos = b.mesh.positions;
    for (let i = 0; i < pos.length; i += 3) {
      if (Math.abs(pos[i + 2]! - 0.006) < 1e-6 && Math.abs(Math.hypot(pos[i]! - 0.05, pos[i + 1]! - 0.05) - 0.005) < 1e-4) ring++;
    }
    expect(ring).toBeGreaterThan(8);
  });

  it('reports an unknown component', () => {
    const d = makeDesign({ ...cube, components: [] });
    expect(kernel.evaluate(d).bodies[0]!.errors.join()).toMatch(/component "blk" does not exist/);
  });
});

describe('collisions', () => {
  it('flags envelopes that intersect a body without a carve, and clears them once carved', async () => {
    const k = await createKernel({ catalog: [BLOCK] });
    const features = (carve: boolean) => [
      { kind: 'extrude' as const, id: 'e', sketch: 's', distance: 0.1, op: 'new' as const },
      ...(carve ? [{ kind: 'carve' as const, id: 'c', component: 'blk', clearance: 0 }] : []),
    ];
    const mk = (carve: boolean, x = 0.05) => makeDesign({
      sketches: [sketch('s', PLANE_XY, rectProfile(0, 0, 0.1, 0.1))],
      bodies: [body('b', 'al6061', { kind: 'billet' }, ...features(carve))],
      components: [{ id: 'blk', def: 'test.block', pose: { position: [x, 0.05, 0.05], rotation: [0, 0, 0, 1] } }],
    });
    expect(k.evaluate(mk(false)).components[0]!.collidesWith).toEqual(['b']);
    expect(k.evaluate(mk(true)).components[0]!.collidesWith).toEqual([]);
    expect(k.evaluate(mk(false, 0.5)).components[0]!.collidesWith).toEqual([]);
    // touching faces is not a collision
    expect(k.evaluate(mk(false, 0.11)).components[0]!.collidesWith).toEqual([]);
    k.dispose();
  });

  it('returns component meshes and keep-outs in the design frame', async () => {
    const k = await createKernel({ catalog: [BLOCK] });
    const d = makeDesign({ components: [{ id: 'blk', def: 'test.block', pose: { position: [1, 2, 3], rotation: [0, 0, 0, 1] } }] });
    const c = k.evaluate(d).components[0]!;
    const bb = bounds(c.mesh.positions);
    expect(bb.min).toEqual([0.99, 1.99, 2.99].map((v) => expect.closeTo(v, 6)));
    expect(bounds(c.keepOut.positions).max[0]).toBeCloseTo(1.01, 6);
    k.dispose();
  });
});

describe('whole-robot mass', () => {
  it('sums bodies and components about the combined centre', async () => {
    const k = await createKernel({ catalog: [BLOCK] });
    const d = makeDesign({
      sketches: [sketch('s', PLANE_XY, rectProfile(0, 0, 0.1, 0.1))],
      bodies: [body('b', 'al6061', { kind: 'billet' }, { kind: 'extrude', id: 'e', sketch: 's', distance: 0.1, op: 'new' })],
      components: [{ id: 'blk', def: 'test.block', pose: { position: [0.55, 0.05, 0.05], rotation: [0, 0, 0, 1] } }],
    });
    const ev = k.evaluate(d);
    const bm = ev.bodies[0]!.mass.mass;
    expect(ev.mass.mass).toBeCloseTo(bm + 0.5, 9);
    expect(ev.mass.com[0]).toBeCloseTo((bm * 0.05 + 0.5 * 0.55) / (bm + 0.5), 6);
    expect(ev.timeMs).toBeGreaterThan(0);
    k.dispose();
  });
});
