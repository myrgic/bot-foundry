import { MATERIALS, type Connection as ModelConnection, type PhysicalModel, type Transform } from '@botfoundry/model';
import { Body } from './body.ts';
import { Connection, type Rating } from './connection.ts';
import { buildHull } from './hull.ts';
import { Joint, type JointKind } from './joint.ts';
import { normalize3, quat, quatNormalize, rotate, vec3, type Mat3, type Vec3 } from './math.ts';
import type { SolverParams } from './params.ts';

// Model ingestion shared by both backends: turns a compiled robot at a pose into bodies, joints and
// rated connections. The WebAssembly backend copies what this builds into its own memory, so the
// creation-time arithmetic is the same code on both.

export const MAX_HULLS = 64;
export const MAX_BODIES = 65536;

export interface BuiltRobot {
  bodies: Body[];
  connections: Connection[];
  parts: string[];
}

export function groundBody(): Body {
  // the ground is in no body list; its index only has to differ from every part's
  return new Body({
    index: MAX_BODIES - 1,
    id: 'ground',
    mass: 0,
    inertia: new Float64Array(9),
    friction: 1,
    hulls: [],
    isGround: true,
  });
}

function requireFinite(what: string, values: readonly number[]): void {
  if (!values.every(Number.isFinite)) throw new Error(`${what} must be finite numbers, got [${values.join(', ')}]`);
}

/**
 * Bodies get indices firstIndex, firstIndex + 1, … in part order. Throws on a pose, mass or inertia
 * that is not finite: a NaN or infinity in the state would spread through every contact and joint.
 */
export function buildRobot(model: PhysicalModel, pose: Transform, robotId: string, firstIndex: number, params: SolverParams): BuiltRobot {
  requireFinite('the pose position', pose.position);
  requireFinite('the pose rotation', pose.rotation);
  // a finite but zero-length quaternion normalizes to a degenerate identity/NaN state: refuse it
  const rotLenSq =
    pose.rotation[0]! ** 2 + pose.rotation[1]! ** 2 + pose.rotation[2]! ** 2 + pose.rotation[3]! ** 2;
  if (rotLenSq < 1e-18) throw new Error(`the pose rotation must have length at least 1e-9, got ${Math.sqrt(rotLenSq)}`);
  for (const part of model.parts) {
    requireFinite(`part ${part.id}: mass`, [part.mass.mass]);
    requireFinite(`part ${part.id}: inertia`, part.mass.inertia);
  }
  const poseRot = quatNormalize(quat(...pose.rotation));
  const posePos = vec3(...pose.position);
  const toWorld = (p: readonly number[], out: Vec3): Vec3 => {
    rotate(out, poseRot, vec3(p[0], p[1], p[2]));
    out[0] = out[0]! + posePos[0]!;
    out[1] = out[1]! + posePos[1]!;
    out[2] = out[2]! + posePos[2]!;
    return out;
  };

  const byPart = new Map<string, Body>();
  const bodies: Body[] = [];
  const parts: string[] = [];
  for (const part of model.parts) {
    if (firstIndex + bodies.length >= MAX_BODIES - 1) throw new Error(`a world holds at most ${MAX_BODIES} bodies`);
    if (part.collision.length > MAX_HULLS) throw new Error(`part ${part.id}: at most ${MAX_HULLS} collision hulls`);
    const com = part.mass.com;
    const hulls = part.collision.map((h) => buildHull(h.vertices.map((v) => [v[0] - com[0], v[1] - com[1], v[2] - com[2]])));
    const [ixx, iyy, izz, ixy, ixz, iyz] = part.mass.inertia;
    const inertia: Mat3 = new Float64Array([ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz]);
    const body = new Body({
      index: firstIndex + bodies.length,
      id: `${robotId}/${part.id}`,
      mass: part.mass.mass,
      inertia,
      friction: MATERIALS[part.material]?.friction ?? 0.5,
      hulls,
    });
    // the body frame sits at the COM with the design-frame axes, carried by the pose
    toWorld(com, body.pos);
    body.rot.set(poseRot);
    // A new body has no velocity history for the adaptive warm start to read (world.ts). Give it
    // one that reads as free fall, so its first guess is the plain inertial target y (AVBD Eq. 2):
    // started from a guess without gravity, the bodies of a falling assembly are pulled apart by
    // the first solve and set spinning (issue #1).
    for (let i = 0; i < 3; i++) body.prevVel[i] = -params.dt * params.gravity[i]!;
    bodies.push(body);
    byPart.set(part.id, body);
    parts.push(body.id);
  }

  const connections = model.connections.map((c) => buildConnection(robotId, c, byPart, toWorld, params));
  // actuators, power and controls belong to later solver lanes and are ignored here
  return { bodies, connections, parts };
}

function buildConnection(
  robotId: string,
  c: ModelConnection,
  byPart: Map<string, Body>,
  toWorld: (p: readonly number[], out: Vec3) => Vec3,
  params: SolverParams,
): Connection {
  const a = byPart.get(c.a), b = byPart.get(c.b);
  if (!a || !b) throw new Error(`connection ${c.id}: unknown part ${a ? c.b : c.a}`);
  if (a === b) throw new Error(`connection ${c.id}: joins part ${c.a} to itself`);
  const origin = toWorld([0, 0, 0], vec3());
  const worldDir = (d: readonly number[]): Vec3 => {
    const out = toWorld(d, vec3());
    for (let i = 0; i < 3; i++) out[i] = out[i]! - origin[i]!;
    return normalize3(out, out);
  };

  let kind: JointKind = 'fixed';
  let anchor: Vec3;
  let axis: Vec3 | null = null;
  let limits: readonly [number, number] | undefined;
  let rating: Rating = { kind: 'none' };
  switch (c.kind) {
    case 'bolt': {
      anchor = toWorld(c.position, vec3());
      axis = worldDir(c.axis);
      // the axis points from b toward a (see Connection in @botfoundry/model); used as given
      rating = { kind: 'bolt', rating: c.rating };
      break;
    }
    case 'weld': {
      if (c.path.length === 0) throw new Error(`weld ${c.id}: empty path`);
      const sum = [0, 0, 0];
      for (const p of c.path) for (let i = 0; i < 3; i++) sum[i]! += p[i]!;
      anchor = toWorld(sum.map((s) => s / c.path.length), vec3());
      const first = c.path[0]!, last = c.path[c.path.length - 1]!;
      const seam = [last[0] - first[0], last[1] - first[1], last[2] - first[2]];
      axis = seam[0]! ** 2 + seam[1]! ** 2 + seam[2]! ** 2 > 0 ? worldDir(seam) : null;
      rating = { kind: 'weld', rating: c.rating };
      break;
    }
    case 'joint': {
      kind = c.joint === 'bearing' ? 'hinge' : c.joint;
      anchor = toWorld(c.anchor, vec3());
      if (c.axis) axis = worldDir(c.axis);
      else if (kind === 'hinge' || kind === 'slider') throw new Error(`joint ${c.id}: a ${c.joint} needs an axis`);
      limits = c.limits;
      break;
    }
  }
  const joint = new Joint({ kind, a, b, anchor, axis: axis ?? undefined, limits, params });
  return new Connection(`${robotId}/${c.id}`, joint, rating, axis);
}

export function pairKey(a: Body, b: Body): number {
  return a.index < b.index ? a.index * MAX_BODIES + b.index : b.index * MAX_BODIES + a.index;
}

/** Key of the manifold between hull ha of A and hull hb of B; sorting by it fixes the solve order. */
export function manifoldKey(A: Body, ha: number, B: Body, hb: number): number {
  return ((A.index * MAX_HULLS + ha) * MAX_BODIES + B.index) * MAX_HULLS + hb;
}
