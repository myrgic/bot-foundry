import { boltRating } from '@botfoundry/model';
import { beforeAll, describe, expect, it } from 'vitest';
import { AT_ORIGIN, BACKENDS, bolt, boxPart, model } from './fixtures.ts';
import type { WorldOptions } from './index.ts';
import { loadWasmSolver } from './wasm.ts';
import { createWorld as create } from './world.ts';

const G = 9.81;

describe.each(BACKENDS)('bolts (%s backend)', (backend) => {
  beforeAll(loadWasmSolver);
  const createWorld = (options: WorldOptions = {}) => create({ ...options, backend });

  /** A mass hanging from a static anchor by one M5 8.8 bolt with a vertical axis. */
  function hanging(mass: number) {
    const world = createWorld();
    world.addRobot(
      model('hang', [
        boxPart('anchor', [0.1, 0.1, 0.1], [0, 1, 0], { mass: 0 }),
        boxPart('weight', [0.1, 0.1, 0.1], [0, 0.89, 0], { mass }),
      ], [bolt('m5', 'anchor', 'weight', [0, 0.945, 0], [0, 1, 0])]),
      AT_ORIGIN,
    );
    return world;
  }

  it('a mass hanging from one M5 bolt reads axial force m·g within 1 %', () => {
    // this pins the solver's multiplier-to-newton convention: AVBD's λ is already a force
    for (const mass of [0.2, 10, 500]) {
      const world = hanging(mass);
      for (let i = 0; i < 1000; i++) world.step();
      const c = world.connections()[0]!;
      expect(Math.abs(c.axial / (mass * G) - 1)).toBeLessThan(0.01);
      expect(c.shear).toBeLessThan(1e-6 * mass * G);
      expect(c.broken).toBe(false);
    }
  });

  it('reads tension positive whichever part is named first', () => {
    const world = createWorld();
    world.addRobot(
      model('hang', [
        boxPart('anchor', [0.1, 0.1, 0.1], [0, 1, 0], { mass: 0 }),
        boxPart('weight', [0.1, 0.1, 0.1], [0, 0.89, 0], { mass: 10 }),
      ], [bolt('m5', 'weight', 'anchor', [0, 0.945, 0], [0, -1, 0])]),
      AT_ORIGIN,
    );
    for (let i = 0; i < 500; i++) world.step();
    expect(world.connections()[0]!.axial).toBeCloseTo(10 * G, 1);
  });

  it('uses the axis as given: level centres still read tension positive when the axis points b to a', () => {
    // the weight hangs from the anchor through a bolt whose parts sit at the same height, so the
    // centres of mass say nothing about which way the axis points; only the stated convention does
    const level = (axis: [number, number, number]) => {
      const world = createWorld();
      world.addRobot(
        model('level', [
          boxPart('anchor', [0.1, 0.1, 0.1], [0, 1, 0], { mass: 0 }),
          boxPart('weight', [0.1, 0.1, 0.1], [0.12, 1, 0], { mass: 10 }),
        ], [bolt('m5', 'anchor', 'weight', [0.06, 1, 0], axis)]),
        AT_ORIGIN,
      );
      for (let i = 0; i < 500; i++) world.step();
      return world.connections()[0]!.axial;
    };
    // a = anchor is the head side, which is up for a hanging load: b -> a = +y
    expect(level([0, 1, 0])).toBeCloseTo(10 * G, 1);
    // the same bolt with the axis stated the other way round reads the opposite sign: nothing is flipped for it
    expect(level([0, -1, 0])).toBeCloseTo(-10 * G, 1);
  });

  it('breaks once the hanging load exceeds its rating, and not before', () => {
    // pure tension fails at 1.4 · Ft,R (EN 1993-1-8 Table 3.4 interaction)
    const breakingLoad = 1.4 * boltRating({ size: 'M5', grade: '8.8', length: 0.02 }).tension;
    const below = hanging((0.98 * breakingLoad) / G);
    let peak = 0;
    for (let i = 0; i < 2000; i++) {
      below.step();
      peak = Math.max(peak, below.connections()[0]!.utilisation);
    }
    expect(below.connections()[0]!.broken).toBe(false);
    expect(peak).toBeLessThan(1);
    expect(peak).toBeGreaterThan(0.97);

    const above = hanging((1.02 * breakingLoad) / G);
    let brokeAt = -1;
    for (let i = 0; i < 2000 && brokeAt < 0; i++) {
      above.step();
      if (above.connections()[0]!.broken) brokeAt = i;
    }
    expect(brokeAt).toBeGreaterThanOrEqual(0);
    const c = above.connections()[0]!;
    expect(c.utilisation).toBeGreaterThanOrEqual(1);
    // once broken it carries nothing and the weight falls freely
    for (let i = 0; i < 100; i++) above.step();
    expect(above.connections()[0]!.axial).toBe(0);
    expect(above.bodies()[1]!.linearVelocity[1]).toBeCloseTo(-G * 0.1, 1);
  });

  it('carries a 100 kg plate through a 0.05 kg bracket with the analytic load path', () => {
    // anchor —bolt— 0.05 kg bracket —bolt— 100 kg plate: a 2000:1 mass ratio across the bracket
    const world = createWorld();
    world.addRobot(
      model('chain', [
        boxPart('anchor', [0.1, 0.1, 0.1], [0, 1.05, 0], { mass: 0 }),
        boxPart('bracket', [0.03, 0.03, 0.03], [0, 0.985, 0], { mass: 0.05 }),
        boxPart('plate', [0.6, 0.02, 0.6], [0, 0.96, 0], { mass: 100 }),
      ], [bolt('top', 'anchor', 'bracket', [0, 1.0, 0], [0, 1, 0]), bolt('bottom', 'bracket', 'plate', [0, 0.97, 0], [0, 1, 0])]),
      AT_ORIGIN,
    );
    let vmax = 0;
    for (let i = 0; i < 3000; i++) {
      world.step();
      if (i > 500) for (const b of world.bodies()) vmax = Math.max(vmax, Math.hypot(...b.linearVelocity), Math.hypot(...b.angularVelocity));
    }
    const [top, bottom] = world.connections();
    expect(Math.abs(top!.axial / (100.05 * G) - 1)).toBeLessThan(0.01);
    expect(Math.abs(bottom!.axial / (100 * G) - 1)).toBeLessThan(0.01);
    expect(vmax).toBeLessThan(1e-3);
  });

  it('a 100 kg plate bolted to a 0.05 kg bracket survives a drop without exploding', () => {
    const world = createWorld();
    world.addGround();
    world.addRobot(
      model('plate', [
        boxPart('plate', [0.6, 0.02, 0.6], [0, 0.11, 0], { mass: 100 }),
        boxPart('bracket', [0.04, 0.03, 0.04], [0.2, 0.135, 0.2], { mass: 0.05 }),
      ], [bolt('a', 'plate', 'bracket', [0.19, 0.12, 0.2], [0, -1, 0]), bolt('b', 'plate', 'bracket', [0.21, 0.12, 0.2], [0, -1, 0])]),
      { position: [0, 0, 0], rotation: [0.02, 0, 0.01, 1].map((v, _, q) => v / Math.hypot(...q)) as [number, number, number, number] },
    );
    let vmaxLate = 0, vmaxAll = 0;
    for (let i = 0; i < 3000; i++) {
      world.step();
      for (const b of world.bodies()) {
        const v = Math.max(Math.hypot(...b.linearVelocity), Math.hypot(...b.angularVelocity));
        vmaxAll = Math.max(vmaxAll, v);
        if (i > 2000) vmaxLate = Math.max(vmaxLate, v);
      }
    }
    // falling 0.1 m reaches 1.4 m/s; nothing should go faster than a few times that
    expect(vmaxAll).toBeLessThan(5);
    expect(vmaxLate).toBeLessThan(1e-2);
    const [plate, bracket] = world.bodies();
    expect(plate!.position[1]).toBeCloseTo(0.01, 2);
    // the bracket stays where it was bolted, 0.025 m above the plate's centre plane
    expect(bracket!.position[1] - plate!.position[1]).toBeCloseTo(0.025, 3);
    expect(world.connections().every((c) => !c.broken)).toBe(true);
  });
});
