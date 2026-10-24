import type { BoltSpec } from './fasteners.ts';
import type { ControlGraph } from './controls.ts';
import type { Id, Plane, Profile, Transform, V2, V3 } from './geometry.ts';
import type { MaterialId } from './materials.ts';

// What a builder authors in the workshop. Everything here is parametric: the CAD kernel
// re-evaluates the feature tree whenever an input changes, so carving follows the parts.

export interface Sketch {
  id: Id;
  name: string;
  plane: Plane;
  profiles: Profile[];
}

/**
 * How a body is built, which is also how the physics discretises it:
 * plates become hinge lattices at their thickness, billets become coarse cell lattices,
 * tubes become beam chains.
 */
export type Structure =
  | { kind: 'plate'; thickness: number }
  | { kind: 'billet' }
  | { kind: 'tube'; wall: number };

export type BooleanOp = 'new' | 'add' | 'subtract' | 'intersect';

export type Feature =
  | {
      kind: 'extrude'; id: Id;
      sketch: Id;
      /** which profiles of the sketch, default all */
      profiles?: number[];
      /** along the sketch normal, m; negative extrudes the other way */
      distance: number;
      /** draft angle, rad; positive narrows the far face (wedges, tapered armor) */
      taper?: number;
      /** extrude half each way from the sketch plane */
      symmetric?: boolean;
      op: BooleanOp;
    }
  | {
      kind: 'revolve'; id: Id;
      sketch: Id;
      profiles?: number[];
      /** axis in sketch coordinates */
      axis: { origin: V2; direction: V2 };
      /** rad, 2π for a full revolution */
      angle: number;
      op: BooleanOp;
    }
  | { kind: 'mirror'; id: Id; plane: Plane }
  | { kind: 'pattern'; id: Id; feature: Id; count: number; step: Transform }
  /** subtract a component's keep-out envelope and drill its mounting holes */
  | { kind: 'carve'; id: Id; component: Id; clearance: number };

export interface Body {
  id: Id;
  name: string;
  material: MaterialId;
  structure: Structure;
  /** ordered; the first feature must create material (op 'new') */
  features: Feature[];
}

export interface ComponentInstance {
  id: Id;
  /** catalog id */
  def: Id;
  name?: string;
  pose: Transform;
  /** body the component is bolted to through its mount pattern */
  mountedTo?: Id;
}

export interface FastenerInstance {
  id: Id;
  bolt: BoltSpec;
  position: V3;
  /** along the bolt; its sign is free, compile orients the connection axis from `joins[1]` toward `joins[0]` */
  axis: V3;
  /** the two things it clamps together: body or component instance ids */
  joins: [Id, Id];
}

export interface Weld {
  id: Id;
  joins: [Id, Id];
  /** seam centreline, design frame */
  path: V3[];
  /** fillet throat, m */
  throat: number;
}

export interface JointDef {
  id: Id;
  kind: 'fixed' | 'hinge' | 'bearing' | 'slider' | 'ball';
  a: Id;
  b: Id;
  anchor: V3;
  axis?: V3;
  /** rad for hinges, m for sliders */
  limits?: [number, number];
  /** motor component instance that drives this joint */
  drive?: Id;
}

export interface Wire {
  id: Id;
  from: { component: Id; port: string };
  to: { component: Id; port: string };
  /** American wire gauge */
  gauge: number;
}

export interface Design {
  schema: 'botfoundry/design@0';
  name: string;
  sketches: Sketch[];
  bodies: Body[];
  components: ComponentInstance[];
  fasteners: FastenerInstance[];
  welds: Weld[];
  joints: JointDef[];
  wiring: Wire[];
  controls: ControlGraph;
}
