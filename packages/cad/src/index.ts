import type { Design, Id, MassProperties, MeshData, PhysicalModel, Transform } from '@botfoundry/model';

// Public API of the CAD kernel. The workshop app and the physics compiler depend only on this.

export interface EvaluatedBody {
  id: Id;
  mesh: MeshData;
  mass: MassProperties;
  /** problems with this body's feature tree, e.g. a self-intersecting profile */
  errors: string[];
}

export interface EvaluatedComponent {
  id: Id;
  pose: Transform;
  /** component geometry in the design frame */
  mesh: MeshData;
  /** keep-out envelope in the design frame, for highlighting clearance problems */
  keepOut: MeshData;
  /** bodies this component's envelope intersects without a carve feature */
  collidesWith: Id[];
}

export interface Evaluation {
  bodies: EvaluatedBody[];
  components: EvaluatedComponent[];
  /** whole robot: bodies plus components */
  mass: MassProperties;
  /** wall-clock time spent evaluating, ms */
  timeMs: number;
}

export interface Kernel {
  /** re-evaluate the whole feature tree */
  evaluate(design: Design): Evaluation;
  /** compile to the physics contract */
  compile(design: Design): PhysicalModel;
  /** release WASM memory held by caches */
  dispose(): void;
}

export { createKernel } from './kernel.ts';
export type { CadKernel, KernelOptions, KernelStats } from './kernel.ts';
export { sampleDesign } from './samples.ts';
export { circleLoop, circleProfile, polygonLoop, rectLoop, rectProfile } from './sketch.ts';
export { arcGeometry, bulgeThrough, sampleArc } from './arc.ts';
export type { ArcGeometry } from './arc.ts';
export { planeFrame, planeToWorld, worldToPlane } from './vec.ts';
export type { PlaneFrame } from './vec.ts';
