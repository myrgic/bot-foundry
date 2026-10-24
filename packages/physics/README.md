# @botfoundry/physics

Deterministic rigid-body solver for Bot Foundry: Augmented Vertex Block Descent (Giles, Diaz, Yuksel, SIGGRAPH 2025) with hull contacts and friction, fixed, hinge, slider and ball joints, and bolts and welds that report the load they carry and break when it exceeds their rating.

## Backends

The solver has two backends that produce the same results bit for bit.

- `ts` (the default) is the reference implementation in plain TypeScript, under `src/`.
- `wasm` runs the same solver compiled from Rust to WebAssembly, about four to five times faster. Load it once, then ask for it by name:

```ts
import { createWorld, loadWasmSolver } from '@botfoundry/physics';

await loadWasmSolver();
const world = createWorld({ backend: 'wasm' });
```

`createWorld({ backend: 'wasm' })` throws if `loadWasmSolver()` has not finished. Everything else about a world (adding robots, stepping, reading bodies, connections and the hash) is the same on both. `addRobot` throws on both backends if the pose, a part's mass or a part's inertia is not a finite number.

The two backends agree exactly because the Rust code performs the same float64 operations in the same order: no fused multiply-add, the same hand-written `atan2`, and SIMD only where each lane does one value's ordinary scalar arithmetic. `src/backends.test.ts` and the `src/backends.robots.*.test.ts` files check this by comparing state hashes over 2,000 substeps. If you change the solver, change both backends and keep those tests passing.

## Benchmark

```sh
pnpm --filter @botfoundry/physics bench
```

prints the mean time per 1 ms substep for each scenario on both backends, side by side. It then runs the two gate scenarios, each 7 times on fresh worlds, and prints the median, minimum and maximum per backend. The gate is 0.5 ms per substep on the WebAssembly backend, judged on the median, and the two verdicts are printed separately:

- resting: the two robots (about 300 bodies and 1,000 bolts) standing on the ground.
- dynamic: the same robots dropped from 10 cm, timed over substeps 100 to 500, which covers the impact and the settling.

## Rebuilding the WebAssembly module

The compiled module is committed at `wasm/solver.wasm`, so you only need Rust if you change the code under `native/`. To rebuild it you need a Rust toolchain with the WebAssembly target:

```sh
rustup target add wasm32-unknown-unknown
pnpm --filter @botfoundry/physics build:wasm
```

That runs `cargo build --release` for `wasm32-unknown-unknown` with 128-bit WebAssembly SIMD turned on (`-C target-feature=+simd128`) and copies the result to `wasm/solver.wasm`. Commit the new file along with your Rust changes. The crate has no dependencies and uses no bindings generator: it exports plain functions, and `src/wasm-world.ts` reads and writes the solver state through typed arrays on the module's memory. The list of state fields in `native/src/store.rs` and the `FIELDS` table in `src/wasm-world.ts` must match, in the same order. The module exports a layout signature generated from the Rust list (each field's name, type, capacity group and number of components), and a world refuses to start if it differs from `FIELDS`, with an error naming the first field that does.

The exports that take counts from JavaScript (`world_reserve` and `world_step`) check them once on entry against the allocated capacities, along with each joint's two body slots and each body's hull ranges. They return 0 on success and a non-zero code otherwise, and `src/wasm-world.ts` throws on a non-zero code. The inner loops are not checked.

Node loads the module from disk. A Vite browser build picks it up as an asset through the `?url` import in `src/wasm.ts`.
