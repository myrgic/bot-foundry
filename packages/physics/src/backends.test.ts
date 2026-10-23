import { beforeAll, describe, expect, it } from 'vitest';
import { compare } from './compare-backends.ts';
import type { World } from './index.ts';
import { mechanism, pile, tumble } from './scenarios.ts';
import { loadWasmSolver } from './wasm.ts';

// The cross-backend checks other than the two-robot scenarios (the backends.robots.*.test.ts
// files): the WebAssembly and TypeScript backends must agree bit for bit, same inputs and same hash.

describe('backends', () => {
  beforeAll(loadWasmSolver);

  it('agree bit for bit on contact-heavy scenes: 40 tumbling boxes and a pile of 60', () => {
    compare(tumble(40));
    compare(pile(60));
  }, 120_000);

  it('agree bit for bit with every joint kind, limits, a weld and a bolt that breaks', () => {
    compare(mechanism());
  }, 120_000);

  it('keep separate worlds separate, whatever order they are created and stepped in', () => {
    const steps = (w: World, n: number) => {
      for (let i = 0; i < n; i++) w.step();
      return w.hash();
    };
    const a = tumble(12).build({ backend: 'wasm' });
    const b = mechanism().build({ backend: 'wasm' });
    steps(a, 100);
    // a third world growing its storage in between must not disturb the other two
    const c = pile(60).build({ backend: 'wasm' });
    steps(c, 50);
    expect(steps(a, 400)).toBe(steps(tumble(12).build({ backend: 'ts' }), 500));
    expect(steps(b, 500)).toBe(steps(mechanism().build({ backend: 'ts' }), 500));
  }, 120_000);
});
