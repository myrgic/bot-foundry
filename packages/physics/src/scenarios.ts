import type { Connection, PhysPart, Quat } from '@botfoundry/model';
import { AT_ORIGIN, bolt, boxPart, model } from './fixtures.ts';
import type { World, WorldOptions } from './index.ts';
import { createWorld } from './world.ts';

// Shared scenarios for the bench and the cross-backend tests. Not exported from the package entry
// point. Every scenario runs at dt = 1/1000 s with 4 iterations unless the options say otherwise.

export interface Scenario {
  name: string;
  bodies: number;
  constraints: number;
  build(options?: WorldOptions): World;
}

const S = 0.1;

function world(options: WorldOptions | undefined): World {
  return createWorld({ dt: 1 / 1000, iterations: 4, ...options });
}

/** N boxes in a grid of columns standing on the ground, 1 mm apart, so the pile is all contacts. */
export function pile(n: number): Scenario {
  const side = Math.ceil(Math.sqrt(n / 10));
  return {
    name: `pile of ${n} boxes`,
    bodies: n,
    constraints: 0,
    build(options) {
      const w = world(options);
      w.addGround();
      for (let i = 0; i < n; i++) {
        const col = i % (side * side), layer = Math.floor(i / (side * side));
        const x = (col % side) * (S + 0.001), z = Math.floor(col / side) * (S + 0.001);
        w.addRobot(model(`b${i}`, [boxPart('b', [S, S, S], [0, 0, 0])]), {
          position: [x, S / 2 + layer * (S + 0.001), z],
          rotation: [0, 0, 0, 1],
        });
      }
      return w;
    },
  };
}

/**
 * N boxes of mixed sizes dropped with assorted tilts onto the ground and onto each other: face,
 * edge and vertex contacts, impacts, sliding and sticking.
 */
export function tumble(n: number): Scenario {
  return {
    name: `${n} tumbling boxes`,
    bodies: n,
    constraints: 0,
    build(options) {
      const w = world(options);
      w.addGround();
      for (let i = 0; i < n; i++) {
        // a rotation of 0.37·i rad about the unit axis (0.6, 0.8 cos i, 0.8 sin i)
        const a = 0.37 * i;
        const s = Math.sin(a / 2);
        const rotation: Quat = [s * 0.6, s * 0.8 * Math.cos(i), s * 0.8 * Math.sin(i), Math.cos(a / 2)];
        const size: [number, number, number] = [0.06 + 0.01 * (i % 3), 0.05 + 0.01 * (i % 4), 0.07 + 0.01 * (i % 5)];
        w.addRobot(model(`box${i}`, [boxPart('b', size, [0, 0, 0])]), {
          position: [0.15 * (i % 4) - 0.2, 0.08 + 0.13 * Math.floor(i / 12), 0.15 * (Math.floor(i / 4) % 3) - 0.15 + 0.02 * (i % 2)],
          rotation,
        });
      }
      return w;
    },
  };
}

/** Chains of 20 links joined by fixed joints, each hanging from a static anchor: n links in all. */
export function chains(n: number): Scenario {
  const perChain = 20;
  const count = Math.round(n / perChain);
  return {
    name: `${count} chains × ${perChain} fixed joints`,
    bodies: count * perChain,
    constraints: count * perChain,
    build(options) {
      const w = world(options);
      for (let c = 0; c < count; c++) {
        const parts: PhysPart[] = [boxPart('anchor', [0.05, 0.05, 0.05], [0, 0, 0], { mass: 0 })];
        const joints: Connection[] = [];
        let prev = 'anchor';
        for (let k = 0; k < perChain; k++) {
          const id = `l${k}`;
          // links run sideways, so the chain is a cantilever loaded in bending
          parts.push(boxPart(id, [0.05, 0.02, 0.02], [0.05 * (k + 1), 0, 0], { mass: 0.2 }));
          joints.push({ kind: 'joint', id: `j${k}`, joint: 'fixed', a: prev, b: id, anchor: [0.05 * k + 0.025, 0, 0] });
          prev = id;
        }
        w.addRobot(model(`chain${c}`, parts, joints), { position: [0, 2, 0.1 * c], rotation: [0, 0, 0, 1] });
      }
      return w;
    },
  };
}

/**
 * A robot-like assembly: a 10 × 15 grid of small plates, each bolted to its eight neighbours
 * (275 edge bolts and 252 diagonal ones), with two robots side by side, on the ground or in free fall.
 * With `drop` above zero they start that many metres above the ground instead of resting on it.
 */
export function robots(ground = true, drop = 0): Scenario {
  const nx = 10, nz = 15, cell = 0.04;
  const parts: PhysPart[] = [];
  const conns: Connection[] = [];
  const id = (i: number, k: number) => `p${i}_${k}`;
  for (let i = 0; i < nx; i++) {
    for (let k = 0; k < nz; k++) parts.push(boxPart(id(i, k), [cell, 0.01, cell], [i * cell, 0.005, k * cell], { mass: 0.15 }));
  }
  for (let i = 0; i < nx; i++) {
    for (let k = 0; k < nz; k++) {
      const links: [number, number][] = [[1, 0], [0, 1], [1, 1], [1, -1]];
      for (const [di, dk] of links) {
        const i2 = i + di, k2 = k + dk;
        if (i2 < 0 || i2 >= nx || k2 < 0 || k2 >= nz) continue;
        const at: [number, number, number] = [(i + i2) * cell / 2, 0.005, (k + k2) * cell / 2];
        conns.push(bolt(`${id(i, k)}-${id(i2, k2)}`, id(i, k), id(i2, k2), at, [0, 1, 0]));
      }
    }
  }
  return {
    name: !ground
      ? 'the same two robots in free fall (no contacts)'
      : drop > 0
        ? `the same two robots dropped from ${(drop * 100).toFixed(0)} cm onto the ground`
        : 'two robots: 2 × 150 bolted plates on the ground',
    bodies: 2 * parts.length,
    constraints: 2 * conns.length,
    build(options) {
      const w = world(options);
      if (ground) w.addGround();
      w.addRobot(model('a', parts, conns), { position: [0, drop, 0], rotation: [0, 0, 0, 1] });
      w.addRobot(model('b', parts, conns), { position: [0.6, drop, 0], rotation: [0, 0, 0, 1] });
      return w;
    },
  };
}

/**
 * Every joint kind and both ratings in one world, on the ground: a chassis with a limited hinge arm,
 * a limited slider, a ball-jointed pendulum, a bolted bracket, a weld, and a bolt loaded to break.
 */
export function mechanism(): Scenario {
  return {
    name: 'mechanism: hinge, slider, ball, bolts, weld, a breaking bolt',
    bodies: 9,
    constraints: 7,
    build(options) {
      const w = world(options);
      w.addGround();
      w.addRobot(
        model('bot', [
          boxPart('chassis', [0.4, 0.05, 0.3], [0, 0.03, 0]),
          boxPart('arm', [0.3, 0.03, 0.03], [0.3, 0.08, 0]),
          boxPart('bracket', [0.04, 0.04, 0.04], [-0.1, 0.075, 0.1], { mass: 0.05 }),
          boxPart('rail', [0.05, 0.05, 0.05], [-0.15, 0.6, -0.1], { mass: 0 }),
          boxPart('carriage', [0.05, 0.05, 0.05], [-0.15, 0.5, -0.1], { mass: 2 }),
          boxPart('rod', [0.02, 0.3, 0.02], [0.2, 0.45, -0.1], { mass: 1 }),
          boxPart('plate', [0.1, 0.01, 0.1], [-0.1, 0.06, -0.1], { mass: 0.4 }),
        ], [
          { kind: 'joint', id: 'hinge', joint: 'hinge', a: 'chassis', b: 'arm', anchor: [0.15, 0.08, 0], axis: [0, 0, 1], limits: [-0.3, 0.3] },
          { kind: 'joint', id: 'slider', joint: 'slider', a: 'rail', b: 'carriage', anchor: [-0.15, 0.5, -0.1], axis: [1, -1, 0], limits: [-1, 0.15] },
          { kind: 'joint', id: 'ball', joint: 'ball', a: 'rail', b: 'rod', anchor: [0.2, 0.6, -0.1] },
          bolt('m5', 'chassis', 'bracket', [-0.1, 0.055, 0.1], [0, -1, 0]),
          { kind: 'weld', id: 'seam', a: 'chassis', b: 'plate', path: [[-0.14, 0.055, -0.1], [-0.06, 0.055, -0.1]], rating: { strength: 2e4 } },
        ]),
        { position: [0, 0.02, 0], rotation: [0, 0, 0, 1] },
      );
      // a 700 kg block hung from an M3 bolt (it holds about 570 kg): it fails at once and falls
      w.addRobot(
        model('hang', [
          boxPart('anchor', [0.1, 0.1, 0.1], [0.8, 1, 0], { mass: 0 }),
          boxPart('weight', [0.1, 0.1, 0.1], [0.8, 0.89, 0], { mass: 700 }),
        ], [bolt('m3', 'anchor', 'weight', [0.8, 0.945, 0], [0, 1, 0], 'M3')]),
        AT_ORIGIN,
      );
      return w;
    },
  };
}
