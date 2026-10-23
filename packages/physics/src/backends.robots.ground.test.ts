import { beforeAll, describe, it } from 'vitest';
import { compare } from './compare-backends.ts';
import { robots } from './scenarios.ts';
import { loadWasmSolver } from './wasm.ts';

// The two-robot bench scenario, one of the slow cross-backend checks, in its own file so Vitest
// runs it in parallel with the free-fall scenario (backends.robots.freefall.test.ts) and the rest
// (backends.test.ts).

describe('backends, two robots', () => {
  beforeAll(loadWasmSolver);

  it('agree bit for bit on the two-robot bench scenario (300 bodies, 1054 bolts, on the ground)', () => {
    compare(robots());
  }, 120_000);
});
