import type { ComponentDef } from './components.ts';
import { PLANE_XY } from './geometry.ts';

// A starter catalog with representative specs for common combat-robot parts.
// Names are generic: these describe classes of part, not any one product.

const m4 = { size: 'M4', grade: '10.9', length: 0.012 } as const;
const m3 = { size: 'M3', grade: '10.9', length: 0.010 } as const;

export const CATALOG: ComponentDef[] = [
  {
    id: 'motor.outrunner-63-190kv',
    name: 'Brushless outrunner, 63 mm, 190 kv',
    category: 'motor',
    mass: 0.85,
    // can along +z from the mount face at z = 0, shaft out of the far end
    geometry: [
      { kind: 'cylinder', radius: 0.0315, length: 0.074, axis: 'z', at: [0, 0, 0.037] },
      { kind: 'cylinder', radius: 0.005, length: 0.025, axis: 'z', at: [0, 0, 0.0865] },
    ],
    keepOut: [{ kind: 'cylinder', radius: 0.036, length: 0.080, axis: 'z', at: [0, 0, 0.040] }],
    mounts: [{
      face: PLANE_XY,
      holes: [
        { at: [0.015, 0], bolt: m4 }, { at: [-0.015, 0], bolt: m4 },
        { at: [0, 0.0125], bolt: m4 }, { at: [0, -0.0125], bolt: m4 },
      ],
    }],
    ports: [
      { name: 'shaft', kind: 'shaft', at: [0, 0, 0.099], axis: [0, 0, 1], diameter: 0.010 },
      { name: 'power', kind: 'power-in', at: [0, 0.03, 0] },
    ],
    motor: { kv: 190, resistance: 0.035, noLoadCurrent: 1.8, peakCurrent: 120, gearRatio: 1, gearEfficiency: 1 },
  },
  {
    id: 'motor.gearmotor-36mm-27to1',
    name: 'Brushless gearmotor, 36 mm, 27:1 planetary',
    category: 'motor',
    mass: 0.42,
    geometry: [
      { kind: 'cylinder', radius: 0.018, length: 0.095, axis: 'z', at: [0, 0, 0.0475] },
      { kind: 'cylinder', radius: 0.004, length: 0.020, axis: 'z', at: [0, 0, 0.105] },
    ],
    // the can plus 2 mm past the mount face, and the output shaft over its full length so a carve drills its hole
    keepOut: [
      { kind: 'cylinder', radius: 0.0195, length: 0.097, axis: 'z', at: [0, 0, 0.0485] },
      { kind: 'cylinder', radius: 0.004, length: 0.020, axis: 'z', at: [0, 0, 0.105] },
    ],
    mounts: [{
      face: { origin: [0, 0, 0.095], normal: [0, 0, 1], xAxis: [1, 0, 0] },
      holes: [
        { at: [0.014, 0], bolt: m3 }, { at: [-0.014, 0], bolt: m3 },
        { at: [0, 0.014], bolt: m3 }, { at: [0, -0.014], bolt: m3 },
      ],
    }],
    ports: [
      { name: 'shaft', kind: 'shaft', at: [0, 0, 0.115], axis: [0, 0, 1], diameter: 0.008 },
      { name: 'power', kind: 'power-in', at: [0, 0.018, 0.01] },
    ],
    motor: { kv: 1400, resistance: 0.06, noLoadCurrent: 0.9, peakCurrent: 60, gearRatio: 27, gearEfficiency: 0.85 },
  },
  {
    id: 'battery.lipo-6s-5000',
    name: 'LiPo pack, 6S 5000 mAh',
    category: 'battery',
    mass: 0.78,
    geometry: [{ kind: 'box', size: [0.155, 0.048, 0.050], at: [0, 0, 0.025] }],
    keepOut: [{ kind: 'box', size: [0.165, 0.056, 0.056], at: [0, 0, 0.028] }],
    mounts: [],
    ports: [{ name: 'power', kind: 'power-out', at: [0.0775, 0, 0.025] }],
    battery: {
      series: 6, parallel: 1, cellCapacity: 5.0, cellVoltageFull: 4.2, cellVoltageEmpty: 3.3,
      cellResistance: 0.003, cellPolarisationResistance: 0.002, cellPolarisationCapacitance: 8000,
    },
  },
  {
    id: 'esc.120a',
    name: 'Speed controller, 120 A',
    category: 'esc',
    mass: 0.12,
    geometry: [{ kind: 'box', size: [0.070, 0.040, 0.020], at: [0, 0, 0.010] }],
    keepOut: [{ kind: 'box', size: [0.080, 0.046, 0.026], at: [0, 0, 0.013] }],
    mounts: [],
    ports: [
      { name: 'power', kind: 'power-in', at: [-0.035, 0, 0.01] },
      { name: 'motor', kind: 'power-out', at: [0.035, 0, 0.01] },
      { name: 'signal', kind: 'signal-in', at: [0.035, 0.015, 0.01] },
    ],
    esc: { currentLimit: 120, maxVoltage: 50.4 },
  },
  {
    id: 'wheel.100x40',
    name: 'Wheel, 100 × 40 mm, neoprene tread',
    category: 'wheel',
    mass: 0.30,
    geometry: [{ kind: 'cylinder', radius: 0.050, length: 0.040, axis: 'z', at: [0, 0, 0.020] }],
    keepOut: [{ kind: 'cylinder', radius: 0.056, length: 0.046, axis: 'z', at: [0, 0, 0.020] }],
    mounts: [],
    ports: [{ name: 'hub', kind: 'shaft', at: [0, 0, 0], axis: [0, 0, 1], diameter: 0.008 }],
  },
];
