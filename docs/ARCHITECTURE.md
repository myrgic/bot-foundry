# Architecture

Bot Foundry has two halves that meet at one contract. The CAD side turns what you draw into exact geometry and mass. The physics side turns that into a machine that can be pushed, flipped, bent and broken. Neither half reaches into the other.

```
  Design  ──(cad: evaluate, compile)──▶  PhysicalModel  ──(physics: addRobot, step)──▶  state, forces
  sketches, features,                     parts with mass and collision,
  components, bolts,                      plates with outlines and thickness,
  welds, joints, wiring,                  rated connections, actuators,
  control graph                           power system, control graph
```

Both types live in [`packages/model`](../packages/model/src). Everything is plain JSON in SI units. The design frame has y up, x forward and z to the robot's right.

## CAD: sketch, feature, carve

A design is a feature tree, not an editable mesh. Each body records the steps that made it (extrude this sketch 8 mm, subtract that pocket), and the kernel re-runs them whenever an input changes. Moving a motor therefore moves its pocket, its bolt holes and the frame's mass properties with it.

Bodies are typed by how real robots are built, because the physics needs to know:

- **Plate:** a sketch extruded by its thickness. AR500 wedges, titanium top armor, aluminium side rails.
- **Billet:** a machined solid, like a milled aluminium unibody.
- **Tube:** a profile swept along a path, for frames.

Components (motors, batteries, speed controllers, wheels) carry their own geometry, a keep-out envelope (the space they need to fit and run, including an output shaft, so a carve drills its hole), a bolt pattern, and electrical specs. A carve feature subtracts the envelope and drills clearance holes for the bolt pattern, so a part cuts its own space into the frame.

The kernel uses [manifold](https://github.com/elalish/manifold) for booleans. It always produces watertight solids, which means volume, centre of mass and inertia can be integrated straight from the mesh. Each body's result is cached against a hash of everything it depends on, so dragging one motor only re-evaluates the bodies that motor touches.

## Physics: everything is a constraint

The solver follows Augmented Vertex Block Descent (Giles, Diaz and Yuksel, SIGGRAPH 2025). Each rigid body is solved as a small 6×6 block, and every coupling (contact, hinge, bolt, weld) is a constraint with an augmented-Lagrangian multiplier. That multiplier is the force the constraint carries, in newtons. This is the property the rest of the design leans on:

- A **bolt** knows its rating from its size and grade. Every substep it compares axial and shear load against that rating with the standard interaction check, and breaks after the solve when the check fails.
- A **weld** does the same with the fillet-weld rule for its throat and length.
- A **plastic hinge** (planned) is a joint that holds until the bending moment reaches the plate's plastic moment, σy · b · t² / 4, then rotates and keeps the new angle. A plate becomes a lattice of cells joined by hinges like this, so it dents and folds where it's hit and stays bent.

Stepping is deterministic: fixed substeps, fixed iteration counts, stable ordering, and no platform-dependent math in the state. Two runs with the same inputs give the same state hash, which makes replays, network play and reproducible tournaments possible.

## Why not simulate the metal directly

Continuum solvers can model steel deforming, but not at interactive rates. Real steel is stiff enough that resolving its stress waves needs microsecond time steps, and the fastest recent methods are either tested far below steel's stiffness or need long precomputation that a dent invalidates.

What a player sees is a dent, a bent bar or a sheared bolt. Those come from yield strength and absorbed energy, not from the elastic field. So the runtime models yield and fracture exactly and leaves invisible elastic micro-deflection out. The expensive physics still has a place: it runs offline to calibrate and test the cheap runtime elements against reference results.

## Spinning weapons

A weapon tooth at 60 to 100 m/s moves several centimetres per millisecond, further than an armor plate is thick, so ordinary collision checks miss hits. Spinners will be modelled as rotors with exact inertia and angular momentum: the motor's reaction torque and the gyroscopic torque act on the chassis, and each tooth strike exchanges a real impulse between rotor and target. Contact that has to be geometric will use rotation-aware collision checks.

## Electrical and control (planned)

- **Battery:** an equivalent circuit (open-circuit voltage by state of charge, series resistance, one RC pair).
- **Motors:** brushless or brushed DC equations from Kv, resistance and no-load current.
- **Speed controllers:** averaged duty with a current limit.

Together they solve as one small algebraic loop per step. That's enough for voltage sag and current limiting to matter in a fight.

The control graph runs on its own fixed clock. Inputs (keys, gamepad axes, AI pilots, scripts) feed blocks (curves, mixers, rate limiters, PID with anti-windup) that drive speed controllers and actuators. Every pilot, human or not, drives a robot through the same named inputs.
