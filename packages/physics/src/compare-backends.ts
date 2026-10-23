import { expect } from 'vitest';
import type { Scenario } from './scenarios.ts';

// Shared by the cross-backend tests (backends.test.ts and the backends.robots.*.test.ts files). The
// WebAssembly backend runs the same operations in the same order as the TypeScript one, in f64 with
// no fused multiply-add, so the two must agree bit for bit: same inputs, same hash.

const STEPS = 2000;
const CHECK_EVERY = 250;

export function compare(s: Scenario): void {
  const ts = s.build({ backend: 'ts' });
  const wasm = s.build({ backend: 'wasm' });
  for (let i = 1; i <= STEPS; i++) {
    ts.step();
    wasm.step();
    if (i % CHECK_EVERY === 0) expect(wasm.hash(), `${s.name}, substep ${i}`).toBe(ts.hash());
  }
  expect(wasm.bodies()).toEqual(ts.bodies());
  expect(wasm.connections()).toEqual(ts.connections());
}
