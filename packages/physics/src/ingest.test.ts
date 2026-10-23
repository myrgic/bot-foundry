import type { PhysPart, PhysicalModel, Transform } from '@botfoundry/model';
import { beforeAll, describe, expect, it } from 'vitest';
import { AT_ORIGIN, BACKENDS, boxPart, model } from './fixtures.ts';
import { loadWasmSolver } from './wasm.ts';
import { createWorld } from './world.ts';

// Both backends take robots through the same ingestion (robot.ts), which refuses anything that is
// not finite: one NaN in a pose, mass or inertia would spread through every contact and joint.

const part = (): PhysPart => boxPart('b', [0.1, 0.1, 0.1], [0, 0, 0]);
const withMass = (patch: Partial<PhysPart['mass']>): PhysicalModel => model('bad', [{ ...part(), mass: { ...part().mass, ...patch } }]);

const BAD: [string, PhysicalModel, Transform, RegExp][] = [
  ['a NaN pose position', model('r', [part()]), { position: [0, NaN, 0], rotation: [0, 0, 0, 1] }, /pose position must be finite/],
  ['an infinite pose position', model('r', [part()]), { position: [Infinity, 0, 0], rotation: [0, 0, 0, 1] }, /pose position must be finite/],
  ['a NaN pose rotation', model('r', [part()]), { position: [0, 0, 0], rotation: [0, 0, NaN, 1] }, /pose rotation must be finite/],
  ['a zero-length pose rotation', model('r', [part()]), { position: [0, 0, 0], rotation: [0, 0, 0, 0] }, /pose rotation must be a non-zero quaternion/],
  ['a pose rotation just under the length floor', model('r', [part()]), { position: [0, 0, 0], rotation: [0, 0, 0, 9e-10] }, /pose rotation must be a non-zero quaternion/],
  ['a NaN mass', withMass({ mass: NaN }), AT_ORIGIN, /part b: mass must be finite/],
  ['an infinite inertia', withMass({ inertia: [1, 1, Infinity, 0, 0, 0] }), AT_ORIGIN, /part b: inertia must be finite/],
  ['a NaN product of inertia', withMass({ inertia: [1, 1, 1, 0, NaN, 0] }), AT_ORIGIN, /part b: inertia must be finite/],
];

describe.each(BACKENDS)('ingestion (%s backend)', (backend) => {
  beforeAll(loadWasmSolver);

  it('accepts a small but normalizable rotation and an unnormalized one', () => {
    for (const rotation of [[0, 0, 0, 2e-9], [0, 0, 0, 5]] as const) {
      const world = createWorld({ backend });
      world.addGround();
      world.addRobot(model('ok', [part()]), { position: [0, 0.05, 0], rotation: [...rotation] });
      world.step();
      for (const b of world.bodies()) expect(Number.isFinite(b.position[1])).toBe(true);
    }
  });

  it.each(BAD)('rejects %s and leaves the world as it was', (_what, robot, pose, message) => {
    const world = createWorld({ backend });
    world.addGround();
    expect(() => world.addRobot(robot, pose)).toThrow(message);
    expect(world.bodies()).toEqual([]);
    // the world is still usable, and the failed add did not use up a body slot or a robot id
    const ok = world.addRobot(model('ok', [part()]), { position: [0, 0.05, 0], rotation: [0, 0, 0, 1] });
    expect(ok.id).toBe('r0');
    world.step();
    expect(world.bodies()).toHaveLength(1);
  });
});
