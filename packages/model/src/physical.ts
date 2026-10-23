import type { BatterySpec, EscSpec, MotorSpec } from './components.ts';
import type { ControlGraph } from './controls.ts';
import type { BoltRating, WeldRating } from './fasteners.ts';
import type { ConvexHull, Id, MassProperties, MeshData, Plane, Profile, V3 } from './geometry.ts';
import type { MaterialId } from './materials.ts';

// The contract between CAD and physics. The CAD kernel compiles a Design into a PhysicalModel;
// the solver consumes only this. Neither side sees the other's internals.
// Everything is in the design frame, SI units.

export type PartStructure =
  | { kind: 'rigid' }
  | { kind: 'plate'; thickness: number; plane: Plane; outline: Profile[] }
  | { kind: 'billet' }
  | { kind: 'tube'; wall: number };

export interface PhysPart {
  id: Id;
  source: { kind: 'body' | 'component'; id: Id };
  material: MaterialId;
  mass: MassProperties;
  /** convex decomposition for contact */
  collision: ConvexHull[];
  /** carved geometry for rendering and for building deformation lattices */
  mesh: MeshData;
  structure: PartStructure;
}

export type Connection =
  | {
      kind: 'bolt'; id: Id; a: Id; b: Id; position: V3;
      /**
       * Direction in the design frame (need not be unit length), pointing from b toward a (the head side). The solver
       * uses it as given: axial force is measured along it, so a bolt in tension reads positive
       * when the axis points this way. Compile emits it this way; anything else that builds a
       * bolt connection must too.
       */
      axis: V3; rating: BoltRating;
    }
  | { kind: 'weld'; id: Id; a: Id; b: Id; path: V3[]; rating: WeldRating }
  | {
      kind: 'joint'; id: Id; joint: 'fixed' | 'hinge' | 'bearing' | 'slider' | 'ball';
      a: Id; b: Id; anchor: V3; axis?: V3; limits?: [number, number];
    };

export interface Actuator {
  id: Id;
  /** the motor component instance */
  component: Id;
  /** joint the motor drives (a wheel bearing, a weapon hinge) */
  joint: Id;
  motor: MotorSpec;
  /** speed controller feeding it */
  esc: Id;
}

export interface PowerSystem {
  packs: { id: Id; component: Id; spec: BatterySpec }[];
  escs: { id: Id; component: Id; spec: EscSpec; pack: Id }[];
  /** total wiring resistance from each ESC back to its pack, Ω */
  wiringResistance: Record<Id, number>;
}

export interface PhysicalModel {
  schema: 'botfoundry/physical@0';
  name: string;
  parts: PhysPart[];
  connections: Connection[];
  actuators: Actuator[];
  power: PowerSystem;
  controls: ControlGraph;
  /** whole-robot mass properties */
  mass: MassProperties;
}
