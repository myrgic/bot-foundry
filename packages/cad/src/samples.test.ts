import { afterAll, beforeAll, describe, expect, it } from 'vitest';
import { boltRating } from '@botfoundry/model';
import { createKernel, type CadKernel } from './kernel.ts';
import { sampleDesign } from './samples.ts';

let kernel: CadKernel;
beforeAll(async () => {
  kernel = await createKernel();
});
afterAll(() => kernel.dispose());

describe('sampleDesign', () => {
  it('evaluates cleanly: no errors, no uncarved collisions', () => {
    const ev = kernel.evaluate(sampleDesign());
    for (const b of ev.bodies) {
      expect(b.errors, b.id).toEqual([]);
      expect(b.mass.mass, b.id).toBeGreaterThan(0);
    }
    for (const c of ev.components) expect(c.collidesWith, c.id).toEqual([]);
  });

  it('is a plausible 30-lb-class robot', () => {
    const ev = kernel.evaluate(sampleDesign());
    expect(ev.mass.mass).toBeGreaterThan(5);
    expect(ev.mass.mass).toBeLessThan(14);
    // centre of mass sits low, near the middle, slightly rearward of the wedge
    expect(ev.mass.com[1]).toBeGreaterThan(0.02);
    expect(ev.mass.com[1]).toBeLessThan(0.06);
    expect(Math.abs(ev.mass.com[2])).toBeLessThan(1e-3);
  });

  it('carving removes material from the walls, base and core', () => {
    const d = sampleDesign();
    const carved = kernel.evaluate(d);
    for (const b of d.bodies) b.features = b.features.filter((f) => f.kind !== 'carve');
    const plain = kernel.evaluate(d);
    for (const id of ['base', 'wall-right', 'wall-left', 'core']) {
      expect(carved.bodies.find((b) => b.id === id)!.mass.volume, id).toBeLessThan(plain.bodies.find((b) => b.id === id)!.mass.volume);
    }
    expect(carved.bodies.find((b) => b.id === 'wedge')!.mass.volume).toBe(plain.bodies.find((b) => b.id === 'wedge')!.mass.volume);
    // without the carves the parts do collide
    expect(plain.components.find((c) => c.id === 'battery')!.collidesWith).toEqual(['core']);
  });

  it('carving the gearmotors drills the axle holes in the walls, with no hole sketched', () => {
    const d = sampleDesign();
    for (const id of ['sk-wall-right', 'sk-wall-left']) {
      expect(d.sketches.find((s) => s.id === id)!.profiles.every((p) => p.holes.length === 0), id).toBe(true);
    }
    const ev = kernel.evaluate(d);
    for (const [wallId, sign] of [['wall-right', 1], ['wall-left', -1]] as const) {
      const pos = ev.bodies.find((b) => b.id === wallId)!.mesh.positions;
      // wall outer face z = +-0.120: a ring of vertices 5 mm from the axle axis (-0.04, 0.05)
      let ring = 0;
      for (let i = 0; i < pos.length; i += 3) {
        if (Math.abs(pos[i + 2]! - sign * 0.12) < 1e-6 && Math.abs(Math.hypot(pos[i]! + 0.04, pos[i + 1]! - 0.05) - 0.005) < 1e-4) ring++;
      }
      expect(ring, wallId).toBeGreaterThan(8);
    }
  });

  it('compiles to a self-consistent physical model', () => {
    const d = sampleDesign();
    const m = kernel.compile(d);
    expect(m.schema).toBe('botfoundry/physical@0');
    expect(m.parts.map((p) => p.id).sort()).toEqual([...d.bodies.map((b) => b.id), ...d.components.map((c) => c.id)].sort());
    const ids = new Set(m.parts.map((p) => p.id));
    for (const c of m.connections) {
      expect(ids.has(c.a), `${c.id}.a`).toBe(true);
      expect(ids.has(c.b), `${c.id}.b`).toBe(true);
    }
    expect(m.mass.mass).toBeCloseTo(kernel.evaluate(d).mass.mass, 9);
    expect(m.actuators).toHaveLength(2);
    expect(m.power.packs).toHaveLength(1);
    expect(m.power.escs).toHaveLength(2);
    // every explicit fastener plus 4 mount bolts for each of the two gearmotors
    expect(m.connections.filter((c) => c.kind === 'bolt')).toHaveLength(d.fasteners.length + 8);
    const m6 = m.connections.find((c) => c.id === 'bolt-core-0r');
    expect(m6?.kind === 'bolt' && m6.rating).toEqual(boltRating({ size: 'M6', grade: '12.9', length: 0.02 }));
  });

  it('serialises to JSON and back unchanged', () => {
    const d = sampleDesign();
    expect(JSON.parse(JSON.stringify(d))).toEqual(d);
    const m = kernel.compile(d);
    expect(JSON.parse(JSON.stringify(m)).parts).toHaveLength(m.parts.length);
  });
});
