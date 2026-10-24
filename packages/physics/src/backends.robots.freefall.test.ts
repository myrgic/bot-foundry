import { beforeAll, describe, expect, it } from 'vitest';
import { compare } from './compare-backends.ts';
import { BACKENDS } from './fixtures.ts';
import { robots } from './scenarios.ts';
import { loadWasmSolver } from './wasm.ts';

// The two-robot free-fall scenario, one of the slow cross-backend checks, in its own file so
// Vitest runs it in parallel with the on-the-ground scenario (backends.robots.ground.test.ts) and
// the rest (backends.test.ts).

describe('backends, two robots', () => {
  beforeAll(loadWasmSolver);

  it('agree bit for bit on the same robots in free fall', () => {
    compare(robots(false));
  }, 120_000);

  // A rigid assembly in free fall carries no internal load (issue #1): it must not spin up, and
  // its bolts must read next to nothing, over the same 2000 substeps as the comparison above.
  it.each(BACKENDS)('bolted robots in free fall stay still and unloaded (%s backend)', (backend) => {
    const world = robots(false).build({ backend });
    const y0 = world.bodies().map((b) => b.position[1]);
    const n = 2000, dt = 1e-3, g = 9.81;
    let spin = 0, load = 0;
    for (let i = 0; i < n; i++) {
      world.step();
      for (const b of world.bodies()) spin = Math.max(spin, Math.hypot(...b.angularVelocity));
      for (const c of world.connections()) load = Math.max(load, c.utilisation);
    }
    expect(spin).toBeLessThan(1e-3);
    expect(load).toBeLessThan(0.01);
    expect(world.connections().some((c) => c.broken)).toBe(false);
    // and every plate falls as a free body does under BDF1: g·dt²·n(n+1)/2
    const drop = g * dt * dt * n * (n + 1) / 2;
    for (const [i, b] of world.bodies().entries()) expect(Math.abs((y0[i]! - b.position[1]) / drop - 1)).toBeLessThan(1e-9);
  }, 120_000);
});
