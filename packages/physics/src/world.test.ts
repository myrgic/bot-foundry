import type { Quat } from '@botfoundry/model';
import { beforeAll, describe, expect, it } from 'vitest';
import { AT_ORIGIN, BACKENDS, bolt, boxPart, model } from './fixtures.ts';
import { Fnv64 } from './hash.ts';
import { buildHull } from './hull.ts';
import type { WorldOptions } from './index.ts';
import { datan2 } from './math.ts';
import { loadWasmSolver } from './wasm.ts';
import { createWorld as create } from './world.ts';

describe.each(BACKENDS)('world (%s backend)', (backend) => {
  beforeAll(loadWasmSolver);
  const createWorld = (options: WorldOptions = {}) => create({ ...options, backend });

  function arena(tilt = 0) {
    const world = createWorld();
    world.addGround();
    for (let i = 0; i < 12; i++) {
      const a = 0.37 * i + tilt;
      const rotation: Quat = [Math.sin(a / 2) * 0.6, Math.sin(a / 2) * 0.8, 0, Math.cos(a / 2)];
      world.addRobot(model(`box${i}`, [boxPart('b', [0.1, 0.08, 0.12], [0, 0, 0])]), {
        position: [0.06 * (i % 4) - 0.1, 0.1 + 0.12 * Math.floor(i / 4), 0.03 * (i % 3)],
        rotation,
      });
    }
    world.addRobot(
      model('bot', [
        boxPart('chassis', [0.4, 0.05, 0.3], [0, 0.4, 0]),
        boxPart('arm', [0.3, 0.03, 0.03], [0.3, 0.45, 0]),
        boxPart('bracket', [0.04, 0.04, 0.04], [-0.1, 0.445, 0.1], { mass: 0.05 }),
      ], [
        { kind: 'joint', id: 'hinge', joint: 'hinge', a: 'chassis', b: 'arm', anchor: [0.15, 0.45, 0], axis: [0, 0, 1], limits: [-1, 1] },
        bolt('m5', 'chassis', 'bracket', [-0.1, 0.425, 0.1], [0, -1, 0]),
      ]),
      { position: [0.5, 0, 0], rotation: [0, 0, 0, 1] },
    );
    return world;
  }

  it('two worlds with identical inputs hash identically after 2000 steps', () => {
    const a = arena(), b = arena();
    for (let i = 0; i < 2000; i++) {
      a.step();
      b.step();
    }
    expect(a.hash()).toBe(b.hash());
    expect(a.hash()).toMatch(/^[0-9a-f]{16}$/);
    // and the hash sees a different input
    const c = arena(1e-9);
    for (let i = 0; i < 2000; i++) c.step();
    expect(c.hash()).not.toBe(a.hash());
  });

  it('reports bodies and connections with world-unique ids', () => {
    const world = createWorld();
    const part = boxPart('p', [0.1, 0.1, 0.1], [0, 0, 0]);
    const r0 = world.addRobot(model('a', [part]), AT_ORIGIN);
    const r1 = world.addRobot(model('a', [part]), { position: [1, 0, 0], rotation: [0, 0, 0, 1] });
    expect(r0.parts).toEqual(['r0/p']);
    expect(r1.parts).toEqual(['r1/p']);
    expect(world.bodies().map((b) => b.id)).toEqual(['r0/p', 'r1/p']);
    expect(world.bodies()[1]!.position).toEqual([1, 0, 0]);
  });

  it('places parts by the robot pose with the body frame at the centre of mass', () => {
    const world = createWorld({ gravity: [0, 0, 0] });
    const quarter: Quat = [0, Math.SQRT1_2, 0, Math.SQRT1_2]; // +90° about y: x → −z
    world.addRobot(model('a', [boxPart('p', [0.1, 0.1, 0.1], [1, 0, 0])]), { position: [0, 2, 0], rotation: quarter });
    const p = world.bodies()[0]!.position;
    expect(p[0]).toBeCloseTo(0, 12);
    expect(p[1]).toBeCloseTo(2, 12);
    expect(p[2]).toBeCloseTo(-1, 12);
  });

  it('rejects connections to unknown parts', () => {
    const world = createWorld();
    expect(() => world.addRobot(model('a', [boxPart('p', [0.1, 0.1, 0.1], [0, 0, 0])], [bolt('x', 'p', 'q', [0, 0, 0], [0, 1, 0])]), AT_ORIGIN)).toThrow(/unknown part q/);
  });
});

describe('helpers', () => {
  it('datan2 matches Math.atan2 to 1e-14 in every quadrant', () => {
    for (let k = 0; k < 720; k++) {
      const a = (k / 720) * 2 * Math.PI - Math.PI + 1e-3;
      for (const r of [1e-3, 1, 1e3]) {
        const y = r * Math.sin(a), x = r * Math.cos(a);
        expect(Math.abs(datan2(y, x) - Math.atan2(y, x))).toBeLessThan(1e-14);
      }
    }
    expect(datan2(0, -1)).toBeCloseTo(Math.PI, 15);
    expect(datan2(1, 0)).toBeCloseTo(Math.PI / 2, 15);
  });

  it('builds a box hull as six quads with three edge directions', () => {
    const v: number[][] = [];
    for (const x of [-1, 1]) for (const y of [-2, 2]) for (const z of [-3, 3]) v.push([x, y, z]);
    const hull = buildHull(v);
    expect(hull.faces).toHaveLength(6);
    expect(hull.faces.every((f) => f.verts.length === 4)).toBe(true);
    expect(hull.edges).toHaveLength(12);
    expect(hull.edgeDirs.length / 3).toBe(3);
    expect(hull.radius).toBeCloseTo(Math.sqrt(14), 12);
  });

  it('rejects a flat hull', () => {
    expect(() => buildHull([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]])).toThrow(/flat/);
  });

  it('FNV-1a 64 matches the published test vector', () => {
    const h = new Fnv64();
    h.byte('a'.charCodeAt(0));
    expect(h.hex()).toBe('af63dc4c8601ec8c');
  });
});
