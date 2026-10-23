import { afterAll, beforeAll, describe, expect, it } from 'vitest';
import { MATERIALS, PLANE_XY, boltRating, weldRating, type ComponentDef, type Profile, type V2 } from '@botfoundry/model';
import { createKernel, type CadKernel } from './kernel.ts';
import { signedArea, tessellateLoop } from './profile.ts';
import { sampleDesign } from './samples.ts';
import { circleLoop, rectProfile } from './sketch.ts';
import { body, makeDesign, sketch } from './testing.ts';
import { wireResistance } from './wiring.ts';

let kernel: CadKernel;
beforeAll(async () => {
  kernel = await createKernel();
});
afterAll(() => kernel.dispose());

const area = (p: Profile): number => {
  const ring = (l: Profile['outer']): V2[] => tessellateLoop(l);
  return Math.abs(signedArea(ring(p.outer))) - p.holes.reduce((s, h) => s + Math.abs(signedArea(ring(h))), 0);
};

describe('plates', () => {
  const plateDesign = () => makeDesign({
    sketches: [sketch('s', { origin: [0, 0.5, 0], normal: [0, 1, 0], xAxis: [1, 0, 0] }, rectProfile(0, 0, 0.2, 0.1, [circleLoop(0.1, -0.05, 0.02)]))],
    bodies: [body('plate', 'al6061', { kind: 'plate', thickness: 0.006 }, { kind: 'extrude', id: 'e', sketch: 's', distance: 0.006, op: 'new' })],
  });

  it('keeps the first extrude plane and slices the carved outline at mid-thickness', () => {
    // the sketch's v axis is -z here, so the hole centre (0.1, -0.05) sits at z = +0.05 and the rectangle spans z in [-0.1, 0]
    const d = plateDesign();
    d.sketches[0]!.profiles = [rectProfile(0, -0.1, 0.2, 0, [circleLoop(0.1, -0.05, 0.02)])];
    const part = kernel.compile(d).parts[0]!;
    expect(part.structure.kind).toBe('plate');
    if (part.structure.kind !== 'plate') return;
    expect(part.structure.thickness).toBe(0.006);
    expect(part.structure.plane).toEqual(d.sketches[0]!.plane);
    expect(part.structure.outline).toHaveLength(1);
    const outline = part.structure.outline[0]!;
    expect(outline.holes).toHaveLength(1);
    // outline area times thickness is the plate volume
    expect(Math.abs(area(outline) * 0.006 - part.mass.volume) / part.mass.volume).toBeLessThan(1e-4);
    // and it is back in sketch coordinates (u in [0, 0.2], v in [-0.1, 0])
    const xs = [outline.outer.start, ...outline.outer.segments.map((s) => s.to)].map((p) => p[0]);
    const ys = [outline.outer.start, ...outline.outer.segments.map((s) => s.to)].map((p) => p[1]);
    expect(Math.min(...xs)).toBeCloseTo(0, 5);
    expect(Math.max(...xs)).toBeCloseTo(0.2, 5);
    expect(Math.min(...ys)).toBeCloseTo(-0.1, 5);
    expect(Math.max(...ys)).toBeCloseTo(0, 5);
  });

  it('includes carved-through holes in the outline', () => {
    const sample = kernel.compile(sampleDesign());
    const base = sample.parts.find((p) => p.id === 'base')!;
    if (base.structure.kind !== 'plate') throw new Error('base is a plate');
    expect(base.structure.outline).toHaveLength(1);
    expect(base.structure.outline[0]!.holes).toHaveLength(1); // the lightening hole
    const wall = sample.parts.find((p) => p.id === 'wall-right')!;
    if (wall.structure.kind !== 'plate') throw new Error('wall is a plate');
    expect(wall.structure.outline[0]!.holes).toHaveLength(1); // the axle hole
  });
});

describe('plate outlines on other planes', () => {
  it('slices the sloped wedge plate (negative extrude, inclined sketch plane) at mid-thickness', () => {
    const wedge = kernel.compile(sampleDesign()).parts.find((p) => p.id === 'wedge')!;
    if (wedge.structure.kind !== 'plate') throw new Error('wedge is a plate');
    expect(wedge.structure.outline).toHaveLength(1);
    const outline = wedge.structure.outline[0]!;
    // trapezoid 0.24 -> 0.16 wide over 0.1228 along the slope, in sketch coordinates; the slice is taken 3 mm below the sketch plane
    expect(Math.abs(area(outline) * 0.006 - wedge.mass.volume) / wedge.mass.volume).toBeLessThan(1e-3);
    expect(area(outline)).toBeCloseTo(((0.24 + 0.16) / 2) * 0.1228, 4);
  });

  it('slices a symmetric plate at its sketch plane', () => {
    const d = makeDesign({
      sketches: [sketch('s', PLANE_XY, rectProfile(0, 0, 0.1, 0.05))],
      bodies: [body('p', 'al6061', { kind: 'plate', thickness: 0.004 }, { kind: 'extrude', id: 'e', sketch: 's', distance: 0.004, symmetric: true, op: 'new' })],
    });
    const part = kernel.compile(d).parts[0]!;
    if (part.structure.kind !== 'plate') throw new Error('plate');
    expect(area(part.structure.outline[0]!)).toBeCloseTo(0.005, 7);
  });
});

describe('structure and parts', () => {
  it('maps billets and tubes, and makes components rigid', () => {
    const d = makeDesign({
      sketches: [sketch('s', PLANE_XY, rectProfile(0, 0, 0.1, 0.1))],
      bodies: [
        body('billet', 'a36', { kind: 'billet' }, { kind: 'extrude', id: 'e', sketch: 's', distance: 0.05, op: 'new' }),
        body('tube', 'a36', { kind: 'tube', wall: 0.003 }, { kind: 'extrude', id: 'e', sketch: 's', distance: 0.05, op: 'new' }),
      ],
      components: [{ id: 'm', def: 'wheel.100x40', pose: { position: [1, 0, 0], rotation: [0, 0, 0, 1] } }],
    });
    const parts = kernel.compile(d).parts;
    expect(parts.map((p) => p.structure.kind)).toEqual(['billet', 'tube', 'rigid']);
    expect(parts[1]!.structure).toEqual({ kind: 'tube', wall: 0.003 });
    expect(parts[2]!.material).toBe('rubber');
    expect(parts[2]!.mass.mass).toBe(0.3);
  });

  it('builds one convex hull per body that contains the whole body', () => {
    const d = makeDesign({
      sketches: [sketch('s', PLANE_XY, rectProfile(0, 0, 0.1, 0.1, [circleLoop(0.05, 0.05, 0.02)]))],
      bodies: [body('b', 'a36', { kind: 'billet' }, { kind: 'extrude', id: 'e', sketch: 's', distance: 0.05, op: 'new' })],
    });
    const p = kernel.compile(d).parts[0]!;
    expect(p.collision).toHaveLength(1);
    expect(p.collision[0]!.vertices).toHaveLength(8); // the hole disappears into the hull
  });

  it('gives each component primitive its own exact hull', () => {
    const d = makeDesign({ components: [{ id: 'm', def: 'motor.gearmotor-36mm-27to1', pose: { position: [0, 0, 0], rotation: [0, 0, 0, 1] } }] });
    const p = kernel.compile(d).parts[0]!;
    expect(p.collision).toHaveLength(2); // can and shaft
    const canZ = p.collision[0]!.vertices.map((v) => v[2]);
    expect(Math.min(...canZ)).toBeCloseTo(0, 9);
    expect(Math.max(...canZ)).toBeCloseTo(0.095, 9);
  });

  it('skips bodies with no geometry', () => {
    const d = makeDesign({
      sketches: [sketch('s', PLANE_XY, rectProfile(0, 0, 0.1, 0.1))],
      bodies: [body('empty', 'a36', { kind: 'billet' }, { kind: 'extrude', id: 'e', sketch: 'missing', distance: 0.05, op: 'new' })],
      fasteners: [{ id: 'f', bolt: { size: 'M5', grade: '8.8', length: 0.02 }, position: [0, 0, 0], axis: [0, 0, 1], joins: ['empty', 'empty'] }],
    });
    const m = kernel.compile(d);
    expect(m.parts).toEqual([]);
    expect(m.connections).toEqual([]);
    expect(m.mass.mass).toBe(0);
  });
});

describe('connections', () => {
  const two = (extra: object = {}) => makeDesign({
    sketches: [sketch('s', PLANE_XY, rectProfile(0, 0, 0.1, 0.1))],
    bodies: [
      body('alu', 'al6061', { kind: 'billet' }, { kind: 'extrude', id: 'e', sketch: 's', distance: 0.01, op: 'new' }),
      body('steel', 'a36', { kind: 'billet' }, { kind: 'extrude', id: 'e', sketch: 's', distance: 0.01, op: 'new' }),
      body('armor', 'ar500', { kind: 'billet' }, { kind: 'extrude', id: 'e', sketch: 's', distance: 0.01, op: 'new' }),
      body('armor2', 'ar500', { kind: 'billet' }, { kind: 'extrude', id: 'e', sketch: 's', distance: 0.01, op: 'new' }),
    ],
    ...extra,
  });

  it('rates bolts from the standards', () => {
    const bolt = { size: 'M6', grade: '10.9', length: 0.02 } as const;
    const d = two({ fasteners: [{ id: 'f1', bolt, position: [0.01, 0.02, 0.03], axis: [0, 0, 1], joins: ['alu', 'steel'] }] });
    const c = kernel.compile(d).connections[0]!;
    expect(c).toEqual({ kind: 'bolt', id: 'f1', a: 'alu', b: 'steel', position: [0.01, 0.02, 0.03], axis: [0, 0, 1], rating: boltRating(bolt) });
  });

  it('emits a bolt axis pointing from b toward a, whichever way it was authored', () => {
    const bolt = { size: 'M6', grade: '10.9', length: 0.02 } as const;
    // low sits at z in [0, 0.01], high at z in [0.5, 0.51]
    const d = makeDesign({
      sketches: [
        sketch('s-low', PLANE_XY, rectProfile(0, 0, 0.1, 0.1)),
        sketch('s-high', { ...PLANE_XY, origin: [0, 0, 0.5] }, rectProfile(0, 0, 0.1, 0.1)),
      ],
      bodies: [
        body('low', 'a36', { kind: 'billet' }, { kind: 'extrude', id: 'e', sketch: 's-low', distance: 0.01, op: 'new' }),
        body('high', 'a36', { kind: 'billet' }, { kind: 'extrude', id: 'e', sketch: 's-high', distance: 0.01, op: 'new' }),
      ],
      fasteners: [
        { id: 'up-high-first', bolt, position: [0, 0, 0], axis: [0, 0, 1], joins: ['high', 'low'] },
        { id: 'down-high-first', bolt, position: [0, 0, 0], axis: [0, 0, -1], joins: ['high', 'low'] },
        { id: 'up-low-first', bolt, position: [0, 0, 0], axis: [0, 0, 1], joins: ['low', 'high'] },
        { id: 'down-low-first', bolt, position: [0, 0, 0], axis: [0, 0, -1], joins: ['low', 'high'] },
      ],
    });
    const axis = (id: string) => {
      const c = kernel.compile(d).connections.find((x) => x.id === id);
      if (!c || c.kind !== 'bolt') throw new Error(`no bolt ${id}`);
      return c.axis;
    };
    // a is the high part: b -> a is +z
    expect(axis('up-high-first')).toEqual([0, 0, 1]);
    expect(axis('down-high-first')).toEqual([0, 0, 1]);
    // a is the low part: b -> a is -z
    expect(axis('up-low-first')).toEqual([0, 0, -1]);
    expect(axis('down-low-first')).toEqual([0, 0, -1]);
  });

  it('keeps a bolt axis as authored when the centres are level along it', () => {
    const bolt = { size: 'M6', grade: '10.9', length: 0.02 } as const;
    const d = two({ fasteners: [
      { id: 'p', bolt, position: [0, 0, 0], axis: [0, 0, 1], joins: ['alu', 'steel'] },
      { id: 'n', bolt, position: [0, 0, 0], axis: [0, 0, -1], joins: ['alu', 'steel'] },
    ] });
    const axes = kernel.compile(d).connections.map((c) => (c.kind === 'bolt' ? c.axis : null));
    expect(axes).toEqual([[0, 0, 1], [0, 0, -1]]);
  });

  it('rates welds with the weaker parent and the length of the seam', () => {
    const path = [[0, 0, 0], [0.05, 0, 0], [0.05, 0.05, 0]] as const;
    const mk = (a: string, b: string) => two({ welds: [{ id: 'w', joins: [a, b], path: [...path], throat: 0.004 }] });
    const w = (a: string, b: string) => {
      const c = kernel.compile(mk(a, b)).connections[0]!;
      if (c.kind !== 'weld') throw new Error('expected a weld');
      return c.rating.strength;
    };
    const L = 0.1;
    expect(w('steel', 'steel')).toBeCloseTo(weldRating(0.004, L, MATERIALS.a36.ultimate).strength, 6);
    // aluminium: parent fu with the heat-affected-zone knockdown
    expect(w('alu', 'steel')).toBeCloseTo(weldRating(0.004, L, MATERIALS.al6061.ultimate, 0.9, 0.6).strength, 6);
    // hardened armour is welded with mild-steel filler, so the seam is no stronger than the filler
    expect(w('armor', 'armor2')).toBeCloseTo(weldRating(0.004, L, 480e6).strength, 6);
    expect(w('armor', 'armor2')).toBeLessThan(weldRating(0.004, L, MATERIALS.ar500.ultimate).strength);
  });

  it('passes joints through', () => {
    const d = two({ joints: [{ id: 'j', kind: 'hinge', a: 'alu', b: 'steel', anchor: [0, 0, 0], axis: [0, 1, 0], limits: [-1, 1] }] });
    expect(kernel.compile(d).connections[0]).toEqual({ kind: 'joint', id: 'j', joint: 'hinge', a: 'alu', b: 'steel', anchor: [0, 0, 0], axis: [0, 1, 0], limits: [-1, 1] });
  });

  it('bolts a mounted component through its mount pattern unless fasteners are listed', () => {
    const d = sampleDesign();
    const motorBolts = kernel.compile(d).connections.filter((c) => c.id.startsWith('motor-right.mount'));
    expect(motorBolts).toHaveLength(4);
    const first = motorBolts[0]!;
    if (first.kind !== 'bolt') throw new Error('expected bolts');
    expect(first.a).toBe('motor-right');
    expect(first.b).toBe('wall-right');
    // mount face at the wall's inner face (z = 0.114), hole at u = +0.014 from the shaft axis; the motor sits
    // inboard of the wall, so the axis from b toward a points along -z
    expect(first.position[2]).toBeCloseTo(0.114, 6);
    expect(first.position[0]).toBeCloseTo(-0.04 + 0.014, 6);
    expect(first.axis[2]).toBeCloseTo(-1, 9);
    expect(first.rating).toEqual(boltRating({ size: 'M3', grade: '10.9', length: 0.010 }));
    // listing a fastener between the pair replaces the implicit ones
    d.fasteners.push({ id: 'mine', bolt: { size: 'M3', grade: '8.8', length: 0.01 }, position: [0, 0, 0], axis: [0, 0, 1], joins: ['wall-right', 'motor-right'] });
    expect(kernel.compile(d).connections.filter((c) => c.id.startsWith('motor-right.mount'))).toHaveLength(0);
  });
});

describe('power and actuators', () => {
  it('builds packs, escs with wiring resistance, and actuators from driven joints', () => {
    const m = kernel.compile(sampleDesign());
    expect(m.power.packs[0]!.spec.series).toBe(6);
    expect(m.power.escs.map((e) => e.pack)).toEqual(['battery', 'battery']);
    for (const e of m.power.escs) expect(m.power.wiringResistance[e.id]).toBeCloseTo(wireResistance(10), 12);
    expect(m.power.wiringResistance['esc-left']).toBeCloseTo(0.3 * 0.003277, 5);
    expect(m.actuators).toEqual([
      expect.objectContaining({ component: 'motor-right', joint: 'wheel-right-bearing', esc: 'esc-right' }),
      expect.objectContaining({ component: 'motor-left', joint: 'wheel-left-bearing', esc: 'esc-left' }),
    ]);
    expect(m.actuators[0]!.motor.gearRatio).toBe(27);
    expect(m.controls).toEqual(sampleDesign().controls);
  });

  it('combines parallel supply wires and ignores unwired escs and undriven joints', () => {
    const d = sampleDesign();
    d.wiring.push({ id: 'extra', from: { component: 'battery', port: 'power' }, to: { component: 'esc-left', port: 'power' }, gauge: 10 });
    d.wiring = d.wiring.filter((w) => w.id !== 'w-pack-right' && w.id !== 'w-motor-left');
    const m = kernel.compile(d);
    expect(m.power.wiringResistance['esc-left']).toBeCloseTo(wireResistance(10) / 2, 12);
    expect(m.power.escs.map((e) => e.id)).toEqual(['esc-left']);
    expect(m.actuators.map((a) => a.component)).toEqual(['motor-right']);
  });

  it('wire gauge sets the resistance', () => {
    const d = sampleDesign();
    d.wiring.find((w) => w.id === 'w-pack-left')!.gauge = 14;
    const m = kernel.compile(d);
    expect(m.power.wiringResistance['esc-left']).toBeCloseTo(wireResistance(14), 12);
    expect(m.power.wiringResistance['esc-left']!).toBeGreaterThan(m.power.wiringResistance['esc-right']!);
  });
});

describe('custom catalogs', () => {
  it('compiles a design that uses parts outside the starter catalog', async () => {
    const def: ComponentDef = {
      id: 'x', name: 'x', category: 'sensor', mass: 0.1,
      geometry: [{ kind: 'box', size: [0.01, 0.01, 0.01], at: [0, 0, 0] }], keepOut: [], mounts: [], ports: [],
    };
    const k = await createKernel({ catalog: [def] });
    const m = k.compile(makeDesign({ components: [{ id: 'c', def: 'x', pose: { position: [0, 0, 0], rotation: [0, 0, 0, 1] } }] }));
    expect(m.parts[0]!.mass.mass).toBeCloseTo(0.1, 12);
    expect(() => k.evaluate(makeDesign({ components: [{ id: 'c', def: 'nope', pose: { position: [0, 0, 0], rotation: [0, 0, 0, 1] } }] }))).toThrow(/unknown catalog part "nope"/);
    k.dispose();
  });
});
