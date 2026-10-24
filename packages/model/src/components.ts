import type { BoltSpec } from './fasteners.ts';
import type { Id, Plane, Primitive, V2, V3 } from './geometry.ts';

export interface MotorSpec {
  /** speed constant, rpm per volt */
  kv: number;
  /** winding resistance, Ω */
  resistance: number;
  /** current drawn spinning free, A */
  noLoadCurrent: number;
  /** what the windings survive for a few seconds, A */
  peakCurrent: number;
  /** output reduction after the motor, 1 for direct drive */
  gearRatio: number;
  /** gearbox efficiency, 0..1 */
  gearEfficiency: number;
}

/** Thevenin equivalent circuit of a pack: open-circuit voltage, series resistance, one RC pair. */
export interface BatterySpec {
  /** cells in series */
  series: number;
  /** cells in parallel */
  parallel: number;
  /** capacity of one cell, Ah */
  cellCapacity: number;
  /** open-circuit voltage of one cell at full and empty charge, V */
  cellVoltageFull: number;
  cellVoltageEmpty: number;
  /** series resistance of one cell, Ω */
  cellResistance: number;
  /** RC pair of one cell: resistance Ω and capacitance F */
  cellPolarisationResistance: number;
  cellPolarisationCapacitance: number;
}

export interface EscSpec {
  /** current limit the controller enforces, A */
  currentLimit: number;
  /** highest pack voltage it accepts, V */
  maxVoltage: number;
}

export type ComponentCategory = 'motor' | 'battery' | 'esc' | 'wheel' | 'controller' | 'actuator' | 'sensor';

/** A bolt pattern on one face of a component, hole centres in that face's plane coordinates. */
export interface MountPattern {
  face: Plane;
  holes: { at: V2; bolt: BoltSpec }[];
}

/** Named connection points: an output shaft, power terminals, a signal input. */
export type Port =
  | { name: string; kind: 'shaft'; at: V3; axis: V3; diameter: number }
  | { name: string; kind: 'power-in' | 'power-out' | 'signal-in' | 'signal-out'; at: V3 };

export interface ComponentDef {
  id: Id;
  name: string;
  category: ComponentCategory;
  /** kg */
  mass: number;
  /** what you see and what collides, local frame */
  geometry: Primitive[];
  /** space the part needs to fit and run (spinning cans, cable bends, an output shaft's full length); carved out of the frame */
  keepOut: Primitive[];
  mounts: MountPattern[];
  ports: Port[];
  motor?: MotorSpec;
  battery?: BatterySpec;
  esc?: EscSpec;
}
