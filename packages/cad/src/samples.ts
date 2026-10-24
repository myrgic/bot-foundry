import { EMPTY_CONTROLS, type Body, type ControlGraph, type Design, type Plane, type Quat, type Sketch } from '@botfoundry/model';
import { circleLoop, polygonLoop, rectLoop } from './sketch.ts';

// A sample robot: a 30-lb-class wedge pusher. Design frame: x forward, y up, z to the right, the
// ground is y = 0. Two gearmotors drive rear wheels outboard of the side walls; a wedge of AR500
// plate sits on the front; the battery sits in a pocket machined into an aluminium core block.

const BASE_BOTTOM = 0.02;
const BASE_T = 0.008;
const BASE_TOP = BASE_BOTTOM + BASE_T;
const WALL_T = 0.006;
const WALL_INNER = 0.114;
const AXLE_X = -0.04;
const AXLE_Y = 0.05;

/** 180° about y: flips a part end for end across the chassis. */
const TURN_Y: Quat = [0, 1, 0, 0];
/** -90° about x: takes a part's local z axis to design up (+y). */
const LAY_FLAT: Quat = [-Math.SQRT1_2, 0, 0, Math.SQRT1_2];

const up = (y: number): Plane => ({ origin: [0, y, 0], normal: [0, 1, 0], xAxis: [1, 0, 0] });
/** Vertical plane facing +z at the given z; sketch u = x, v = y. */
const across = (z: number): Plane => ({ origin: [0, 0, z], normal: [0, 0, 1], xAxis: [1, 0, 0] });

function controls(): ControlGraph {
  return {
    ...EMPTY_CONTROLS,
    nodes: [
      { id: 'in-drive', kind: 'input', name: 'drive' },
      { id: 'in-turn', kind: 'input', name: 'turn' },
      { id: 'mix-left', kind: 'mix', weights: [1, 1] },
      { id: 'mix-right', kind: 'mix', weights: [1, -1] },
    ],
    edges: [
      { from: 'in-drive', to: 'mix-left', slot: 0 },
      { from: 'in-turn', to: 'mix-left', slot: 1 },
      { from: 'in-drive', to: 'mix-right', slot: 0 },
      { from: 'in-turn', to: 'mix-right', slot: 1 },
    ],
    outputs: [
      { component: 'esc-left', from: 'mix-left' },
      { component: 'esc-right', from: 'mix-right' },
    ],
  };
}

export function sampleDesign(): Design {
  // sketch u = x, v = -z on horizontal planes (v = normal × xAxis)
  const sketches: Sketch[] = [
    {
      id: 'sk-base', name: 'Base outline', plane: up(BASE_BOTTOM),
      profiles: [{
        outer: polygonLoop([[-0.15, -0.09], [-0.12, -0.12], [0.2, -0.12], [0.2, 0.12], [-0.12, 0.12], [-0.15, 0.09]]),
        holes: [circleLoop(-0.11, 0, 0.025)],
      }],
    },
    {
      id: 'sk-wall-right', name: 'Right wall', plane: across(WALL_INNER),
      profiles: [{
        outer: polygonLoop([[-0.15, BASE_TOP], [0.2, BASE_TOP], [0.2, 0.07], [0.09, 0.08], [-0.12, 0.08], [-0.15, 0.07]]),
        holes: [],
      }],
    },
    {
      id: 'sk-wall-left', name: 'Left wall', plane: across(-WALL_INNER - WALL_T),
      profiles: [{
        outer: polygonLoop([[-0.15, BASE_TOP], [0.2, BASE_TOP], [0.2, 0.07], [0.09, 0.08], [-0.12, 0.08], [-0.15, 0.07]]),
        holes: [],
      }],
    },
    {
      id: 'sk-core', name: 'Core block', plane: up(BASE_TOP),
      profiles: [{ outer: rectLoop(-0.005, -0.045, 0.19, 0.045), holes: [] }],
    },
    {
      // sloped plane through the front edge of the base, normal tilted forward by the 12° ramp angle
      id: 'sk-wedge', name: 'Wedge plate',
      plane: { origin: [0.2, BASE_TOP, 0], normal: [Math.sin(0.213), Math.cos(0.213), 0], xAxis: [0, 0, 1] },
      profiles: [{ outer: polygonLoop([[-0.12, 0], [0.12, 0], [0.08, 0.1228], [-0.08, 0.1228]]), holes: [] }],
    },
  ];

  const bodies: Body[] = [
    {
      id: 'base', name: 'Base plate', material: 'al6061', structure: { kind: 'plate', thickness: BASE_T },
      features: [
        { kind: 'extrude', id: 'base-ext', sketch: 'sk-base', distance: BASE_T, op: 'new' },
        { kind: 'carve', id: 'base-esc-l', component: 'esc-left', clearance: 0.001 },
        { kind: 'carve', id: 'base-esc-r', component: 'esc-right', clearance: 0.001 },
      ],
    },
    {
      id: 'wall-right', name: 'Right wall', material: 'al6061', structure: { kind: 'plate', thickness: WALL_T },
      features: [
        { kind: 'extrude', id: 'wr-ext', sketch: 'sk-wall-right', distance: WALL_T, op: 'new' },
        { kind: 'carve', id: 'wr-motor', component: 'motor-right', clearance: 0.001 },
      ],
    },
    {
      id: 'wall-left', name: 'Left wall', material: 'al6061', structure: { kind: 'plate', thickness: WALL_T },
      features: [
        { kind: 'extrude', id: 'wl-ext', sketch: 'sk-wall-left', distance: WALL_T, op: 'new' },
        { kind: 'carve', id: 'wl-motor', component: 'motor-left', clearance: 0.001 },
      ],
    },
    {
      id: 'core', name: 'Core block', material: 'al6061', structure: { kind: 'billet' },
      features: [
        { kind: 'extrude', id: 'core-ext', sketch: 'sk-core', distance: 0.06, op: 'new' },
        { kind: 'carve', id: 'core-battery', component: 'battery', clearance: 0.001 },
      ],
    },
    {
      id: 'wedge', name: 'Wedge', material: 'ar500', structure: { kind: 'plate', thickness: 0.006 },
      features: [{ kind: 'extrude', id: 'wedge-ext', sketch: 'sk-wedge', distance: -0.006, op: 'new' }],
    },
  ];

  const motor = (id: string, side: 1 | -1) => ({
    id, def: 'motor.gearmotor-36mm-27to1', name: id,
    pose: {
      position: [AXLE_X, AXLE_Y, side * (WALL_INNER - 0.095)] as const,
      rotation: side === 1 ? ([0, 0, 0, 1] as Quat) : TURN_Y,
    },
    mountedTo: side === 1 ? 'wall-right' : 'wall-left',
  });
  const wheel = (id: string, side: 1 | -1) => ({
    id, def: 'wheel.100x40', name: id,
    pose: {
      position: [AXLE_X, AXLE_Y, side * (WALL_INNER + WALL_T + 0.004)] as const,
      rotation: side === 1 ? ([0, 0, 0, 1] as Quat) : TURN_Y,
    },
  });
  const esc = (id: string, side: 1 | -1) => ({
    id, def: 'esc.120a', name: id,
    pose: { position: [0.09, BASE_TOP - 0.0025, side * 0.0775] as const, rotation: LAY_FLAT },
    mountedTo: 'base',
  });

  return {
    schema: 'botfoundry/design@0',
    name: 'Sample wedge',
    sketches,
    bodies,
    components: [
      motor('motor-right', 1), motor('motor-left', -1),
      wheel('wheel-right', 1), wheel('wheel-left', -1),
      {
        id: 'battery', def: 'battery.lipo-6s-5000', name: 'Battery',
        pose: { position: [0.0925, BASE_TOP + 0.004, 0], rotation: LAY_FLAT }, mountedTo: 'core',
      },
      esc('esc-right', 1), esc('esc-left', -1),
    ],
    fasteners: [
      // walls to base: M5 along the lower edge
      ...[-0.1, 0.02, 0.16].flatMap((x, i) => [1, -1].map((side) => ({
        id: `bolt-wall-${side === 1 ? 'r' : 'l'}${i}`, bolt: { size: 'M5', grade: '10.9', length: 0.016 } as const,
        position: [x, BASE_TOP, side * (WALL_INNER + WALL_T / 2)] as const, axis: [0, 1, 0] as const,
        joins: [side === 1 ? 'wall-right' : 'wall-left', 'base'] as [string, string],
      }))),
      // core to base: M6 through the block
      ...[0.003, 0.18].flatMap((x, i) => [1, -1].map((side) => ({
        id: `bolt-core-${i}${side === 1 ? 'r' : 'l'}`, bolt: { size: 'M6', grade: '12.9', length: 0.02 } as const,
        position: [x, BASE_TOP, side * 0.037] as const, axis: [0, 1, 0] as const,
        joins: ['core', 'base'] as [string, string],
      }))),
      // wedge to base: M6 into the front edge
      ...[-0.08, 0, 0.08].map((z, i) => ({
        id: `bolt-wedge-${i}`, bolt: { size: 'M6', grade: '10.9', length: 0.02 } as const,
        position: [0.2, BASE_TOP - 0.004, z] as const, axis: [1, 0, 0] as const,
        joins: ['wedge', 'base'] as [string, string],
      })),
    ],
    welds: [],
    joints: (['right', 'left'] as const).map((side) => ({
      id: `wheel-${side}-bearing`, kind: 'hinge' as const,
      a: `wall-${side}`, b: `wheel-${side}`,
      anchor: [AXLE_X, AXLE_Y, (side === 'right' ? 1 : -1) * (WALL_INNER + WALL_T)] as const,
      // both wheels turn the same way to drive forward (x), which is about -z
      axis: [0, 0, -1] as const,
      drive: `motor-${side}`,
    })),
    wiring: (['right', 'left'] as const).flatMap((side) => [
      { id: `w-pack-${side}`, from: { component: 'battery', port: 'power' }, to: { component: `esc-${side}`, port: 'power' }, gauge: 10 },
      { id: `w-motor-${side}`, from: { component: `esc-${side}`, port: 'motor' }, to: { component: `motor-${side}`, port: 'power' }, gauge: 12 },
    ]),
    controls: controls(),
  };
}
