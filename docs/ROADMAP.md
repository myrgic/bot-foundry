# Roadmap

Each step has a check that decides whether it's done. Numbers below are measured on a laptop (Apple Silicon, Node 22) unless noted.

## Done

- **Contract.** Design and physical-model schema, material table, bolt and weld ratings, starter component catalog.
- **CAD kernel.** Sketch profiles with arcs, extrude with draft, revolve, mirror, pattern, booleans, component carving with bolt holes, exact mass properties, compile to the physical model. Moving a carved motor re-evaluates in about 3 ms (target: under 16 ms).
- **Workshop.** Sketching on planes and faces, extrude and revolve dialogs, drag-in parts with a gizmo and carve toggle, live mass properties, undo, save and JSON import/export.
- **Solver core.** Rigid bodies, hull contacts with friction, fixed, hinge, slider and ball joints, rated bolts and welds that break after the solve, deterministic stepping. A mass hanging from one bolt reads its weight within 1 %.
- **WebAssembly solver.** A Rust core with struct-of-arrays storage and SIMD, about 5× faster than the TypeScript solver and bit-identical to it. The TypeScript solver stays as the reference. Two robots (300 bodies, about 1,050 bolts) step in a median of about 0.49 ms resting and 0.50 ms while landing from a 10 cm drop, against a 0.5 ms target. That's right at the line, so headroom is item 1 below.
- **Free fall.** A bolted assembly in free fall now carries no load: no spin-up, no broken bolts, and every part falls at exactly g. New bodies start with gravity in their velocity history, warm starting is weighted per bolted assembly rather than per body, and angle locks carry over less of last step's force. ([#1](https://github.com/myrgic/bot-foundry/issues/1))

## Next

1. **Solver headroom.** Get the two-robot step comfortably under 0.5 ms. Ideas in order: solve paired bodies' systems in SIMD lanes (estimated about 0.42 ms), colour the body graph for wider lane groups, and move stepping into a Web Worker so the frame budget doesn't depend on the main thread. ([#21](https://github.com/myrgic/bot-foundry/issues/21))
2. **Plastic hinges.** Plates as cell lattices joined by yielding hinges with hardening and fracture. Check: a three-point bend and a drop-hammer dent within 15 % of reference force-displacement and absorbed energy. ([#22](https://github.com/myrgic/bot-foundry/issues/22))
3. **Spinners.** Analytic rotors with gyroscopic and reaction torque, tooth-strike impulses, rotation-aware collision checks. Check: no pass-throughs in 1,000 randomized strikes at 250 rad/s, energy error under 2 %. ([#23](https://github.com/myrgic/bot-foundry/issues/23))
4. **Electrical and control.** Battery, motor and speed-controller models, the control graph with PID, key and gamepad mapping. Check: spin-up time and current within 10 % of a published reference case. ([#24](https://github.com/myrgic/bot-foundry/issues/24))
5. **Bolt behaviour.** Preload, friction slip before shear, and bending moment in the rating check. ([#25](https://github.com/myrgic/bot-foundry/issues/25))
6. **Convex decomposition** of carved bodies for contact, instead of one hull per body. ([#26](https://github.com/myrgic/bot-foundry/issues/26))
7. **The arena.** Wire the workshop's compiled robots into the solver, with drivers, an AI pilot and match rules. ([#27](https://github.com/myrgic/bot-foundry/issues/27))

## Issues

Bugs, contract follow-ups and smaller gaps live in the [issue tracker](https://github.com/myrgic/bot-foundry/issues). Each one says what's wrong, how to reproduce it, and the check a fix has to pass. The [`good first issue`](https://github.com/myrgic/bot-foundry/issues?q=is%3Aopen+label%3A%22good+first+issue%22) label marks small, self-contained ones, and [`contract`](https://github.com/myrgic/bot-foundry/issues?q=is%3Aopen+label%3Acontract) marks changes to the CAD and physics contract.
