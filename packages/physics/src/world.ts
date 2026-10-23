import type { PhysicalModel, Transform } from '@botfoundry/model';
import type { Body, Constraint } from './body.ts';
import { collideAll, createContactOut } from './collide.ts';
import type { Connection } from './connection.ts';
import { Manifold } from './contact.ts';
import { Fnv64 } from './hash.ts';
import type { BodyState, ConnectionState, RobotHandle, World, WorldOptions } from './index.ts';
import { quatAddRotation, quatDiff, vec3, type Vec3 } from './math.ts';
import { defaultParams, type SolverParams } from './params.ts';
import { buildRobot, groundBody, manifoldKey, pairKey } from './robot.ts';
import { createSystem, solveSystem } from './system.ts';
import { WasmWorld } from './wasm-world.ts';

// The world: Augmented Vertex Block Descent for rigid bodies (Giles, Diaz, Yuksel, SIGGRAPH 2025,
// https://graphics.cs.utah.edu/research/projects/avbd/), structured after the authors' MIT
// avbd-demo3d. Each substep: collide, warm-start constraints, predict bodies, then a fixed number
// of Gauss-Seidel sweeps of per-body 6x6 solves (primal) each followed by a multiplier update
// (dual), then BDF1 velocities and the connection ratings check.
//
// Determinism: fixed substep and iteration count, bodies and constraints always visited in
// creation / sorted-key order, no Math.random, no clock, and no transcendental functions in state.

export function createWorld(options: WorldOptions = {}): World {
  return options.backend === 'wasm' ? new WasmWorld(options) : new PhysicsWorld(options);
}

class PhysicsWorld implements World {
  readonly params: SolverParams;
  private readonly bodyList: Body[] = [];
  private ground: Body | null = null;
  private readonly connectionList: Connection[] = [];
  private readonly manifolds = new Map<number, Manifold>();
  private activeManifolds: Manifold[] = [];
  /** live connection count per body pair; connected bodies do not collide */
  private readonly linked = new Map<number, number>();
  private robots = 0;
  private stepCount = 0;
  private readonly sys = createSystem();
  private readonly dx = new Float64Array(6);
  private readonly contactOut = createContactOut();
  // warm-start scratch, per body (see assemblyWeights)
  private root = new Int32Array(0);
  private weight = new Float64Array(0);
  private massSum = new Float64Array(0);

  constructor(options: WorldOptions) {
    const g = options.gravity ?? [0, -9.81, 0];
    this.params = defaultParams(options.dt ?? 1 / 1000, options.iterations ?? 4, vec3(g[0], g[1], g[2]));
  }

  addGround(): void {
    if (this.ground) return;
    this.ground = groundBody();
  }

  addRobot(model: PhysicalModel, pose: Transform): RobotHandle {
    const robotId = `r${this.robots}`;
    const robot = buildRobot(model, pose, robotId, this.bodyList.length, this.params);
    this.robots++;
    for (const body of robot.bodies) this.bodyList.push(body);
    for (const c of robot.connections) {
      const j = c.joint;
      this.connectionList.push(c);
      j.a.joints.push(j);
      j.b.joints.push(j);
      const key = pairKey(j.a, j.b);
      this.linked.set(key, (this.linked.get(key) ?? 0) + 1);
    }
    return { id: robotId, parts: robot.parts };
  }

  step(): void {
    const p = this.params;
    const dt = p.dt, h2 = dt * dt;
    const g = p.gravity;
    const gLen = Math.sqrt(g[0]! * g[0]! + g[1]! * g[1]! + g[2]! * g[2]!);
    this.stepCount++;

    this.collide();

    // warm-start constraints at x⁻ (Eq. 19); per-body lists in deterministic order
    for (const body of this.bodyList) {
      body.constraints.length = 0;
      for (const j of body.joints) body.constraints.push(j);
    }
    for (const c of this.connectionList) if (!c.broken) c.joint.prepare();
    const live: Manifold[] = [];
    for (const m of this.activeManifolds) {
      if (!m.prepare()) continue;
      live.push(m);
      m.a.constraints.push(m);
      if (!m.b.isStatic) m.b.constraints.push(m);
    }
    this.activeManifolds = live;

    // The adaptive warm start of VBD (Chen et al. 2024) scales gravity in the starting guess by how
    // much of it a body felt last substep. Measured per body, it starts the bodies of one jointed
    // assembly from guesses that disagree with each other; four Gauss-Seidel sweeps settle that
    // only partly, which leaves the bodies' accelerations unequal again, and the error feeds itself
    // (issue #1: a 0.1 rad/s nudge to one plate of a free-falling robot grew to 1.5 rad/s and loaded
    // bolts to 97 %). The weight is therefore taken per assembly, from the acceleration of its
    // centre of mass, in which the internal forces cancel.
    if (gLen > 0) this.assemblyWeights(dt, g, gLen);

    // predict: inertial target y (Eq. 2) and the warm start above
    for (const body of this.bodyList) {
      body.pos0.set(body.pos);
      body.rot0.set(body.rot);
      if (body.isStatic) continue;
      body.updateWorldInertia();
      for (let i = 0; i < 3; i++) body.inertialPos[i] = body.pos[i]! + dt * body.vel[i]! + h2 * g[i]!;
      quatAddRotation(body.inertialRot, body.rot, body.angVel, dt);
      const w = gLen > 0 ? this.weight[this.root[body.index]!]! : 0;
      for (let i = 0; i < 3; i++) body.pos[i] = body.pos[i]! + dt * body.vel[i]! + w * h2 * g[i]!;
      quatAddRotation(body.rot, body.rot, body.angVel, dt);
    }

    for (let it = 0; it < p.iterations; it++) {
      for (const body of this.bodyList) if (!body.isStatic) this.solveBody(body, h2);
      for (const c of this.connectionList) if (!c.broken) c.joint.dual();
      for (const m of this.activeManifolds) m.dual();
    }

    // BDF1 velocities
    const inv = 1 / dt;
    for (const body of this.bodyList) {
      if (body.isStatic) continue;
      body.prevVel.set(body.vel);
      for (let i = 0; i < 3; i++) body.vel[i] = (body.pos[i]! - body.pos0[i]!) * inv;
      quatDiff(body.angVel, body.rot, body.rot0);
      for (let i = 0; i < 3; i++) body.angVel[i] = body.angVel[i]! * inv;
    }

    // ratings are checked after the solve; a connection that fails stops acting next substep
    for (const c of this.connectionList) {
      if (!c.measure()) continue;
      const j = c.joint;
      j.a.joints = j.a.joints.filter((x) => x !== j);
      j.b.joints = j.b.joints.filter((x) => x !== j);
      const key = pairKey(j.a, j.b);
      this.linked.set(key, this.linked.get(key)! - 1);
    }
  }

  /**
   * Group the moving bodies into assemblies joined by unbroken connections (static bodies join
   * nothing) and give each assembly one warm-start weight: the share of gravity its centre of mass
   * accelerated with last substep, clamped to [0, 1]. Fills root (each body's assembly, named by its
   * lowest body index) and weight (indexed by root).
   */
  private assemblyWeights(dt: number, g: Vec3, gLen: number): void {
    const n = this.bodyList.length;
    if (this.root.length < n) {
      this.root = new Int32Array(n);
      this.weight = new Float64Array(n);
      this.massSum = new Float64Array(n);
    }
    const root = this.root, weight = this.weight, massSum = this.massSum;
    for (let i = 0; i < n; i++) root[i] = i;
    for (const c of this.connectionList) {
      if (c.broken || c.joint.a.isStatic || c.joint.b.isStatic) continue;
      const ra = findRoot(root, c.joint.a.index), rb = findRoot(root, c.joint.b.index);
      if (ra < rb) root[rb] = ra;
      else if (rb < ra) root[ra] = rb;
    }
    for (let i = 0; i < n; i++) {
      root[i] = findRoot(root, i);
      weight[i] = 0;
      massSum[i] = 0;
    }
    // mass-weighted acceleration along gravity, summed in body order
    for (const body of this.bodyList) {
      if (body.isStatic) continue;
      let along = 0;
      for (let i = 0; i < 3; i++) along += ((body.vel[i]! - body.prevVel[i]!) / dt) * (g[i]! / gLen);
      const r = root[body.index]!;
      weight[r] = weight[r]! + body.mass * along;
      massSum[r] = massSum[r]! + body.mass;
    }
    for (let i = 0; i < n; i++) {
      if (root[i] !== i || !(massSum[i]! > 0)) continue;
      const w = weight[i]! / massSum[i]! / gLen;
      weight[i] = w > 1 ? 1 : w > 0 ? w : 0;
    }
  }

  /** One primal step for one body: assemble and solve its 6x6 system, apply the update (Eq. 4-6). */
  private solveBody(body: Body, h2: number): void {
    const { lhs, rhs } = this.sys;
    lhs.fill(0);
    const m = body.mass / h2;
    lhs[0] = m;
    lhs[7] = m;
    lhs[14] = m;
    const I = body.inertiaWorld;
    for (let r = 0; r < 3; r++) for (let c = 0; c <= r; c++) lhs[(3 + r) * 6 + 3 + c] = I[r * 3 + c]! / h2;
    for (let i = 0; i < 3; i++) rhs[i] = m * (body.pos[i]! - body.inertialPos[i]!);
    quatDiff(D, body.rot, body.inertialRot);
    for (let r = 0; r < 3; r++) rhs[3 + r] = (I[r * 3]! * D[0]! + I[r * 3 + 1]! * D[1]! + I[r * 3 + 2]! * D[2]!) / h2;

    const cs: Constraint[] = body.constraints;
    for (let k = 0; k < cs.length; k++) cs[k]!.stamp(body, this.sys);

    const dx = this.dx;
    solveSystem(this.sys, dx);
    body.pos[0] = body.pos[0]! + dx[0]!;
    body.pos[1] = body.pos[1]! + dx[1]!;
    body.pos[2] = body.pos[2]! + dx[2]!;
    DTHETA[0] = dx[3]!;
    DTHETA[1] = dx[4]!;
    DTHETA[2] = dx[5]!;
    quatAddRotation(body.rot, body.rot, DTHETA, 1);
  }

  /** Run collision detection and refresh the manifold set. */
  private collide(): void {
    const stamp = this.stepCount;
    const out = this.contactOut;
    collideAll(this.bodyList, this.ground, this.linked, this.params.margin, stamp, out, (A, ha, B, hb) => {
      this.manifold(A, ha, B, hb).update(out, stamp);
    });
    const active: Manifold[] = [];
    for (const [key, m] of this.manifolds) {
      if (m.seen === stamp) active.push(m);
      else this.manifolds.delete(key);
    }
    active.sort((u, v) => u.key - v.key);
    this.activeManifolds = active;
  }

  private manifold(A: Body, ha: number, B: Body, hb: number): Manifold {
    const key = manifoldKey(A, ha, B, hb);
    let m = this.manifolds.get(key);
    if (!m) {
      m = new Manifold(A, B, key, this.params);
      this.manifolds.set(key, m);
    }
    return m;
  }

  bodies(): BodyState[] {
    return this.bodyList.map((b) => ({
      id: b.id,
      position: [b.pos[0]!, b.pos[1]!, b.pos[2]!],
      rotation: [b.rot[0]!, b.rot[1]!, b.rot[2]!, b.rot[3]!],
      linearVelocity: [b.vel[0]!, b.vel[1]!, b.vel[2]!],
      angularVelocity: [b.angVel[0]!, b.angVel[1]!, b.angVel[2]!],
    }));
  }

  connections(): ConnectionState[] {
    return this.connectionList.map((c) => ({
      id: c.id,
      axial: c.axial,
      shear: c.shear,
      utilisation: c.utilisation,
      broken: c.broken,
    }));
  }

  hash(): string {
    const h = new Fnv64();
    for (const b of this.bodyList) {
      h.floats(b.pos);
      h.floats(b.rot);
      h.floats(b.vel);
      h.floats(b.angVel);
    }
    for (const c of this.connectionList) {
      h.float(c.axial);
      h.float(c.shear);
      h.float(c.utilisation);
      h.byte(c.broken ? 1 : 0);
    }
    return h.hex();
  }
}

const D = vec3();
const DTHETA = vec3();

/** Union-find root with path halving; roots are always the lowest index in their set. */
function findRoot(parent: Int32Array, i: number): number {
  while (parent[i] !== i) {
    parent[i] = parent[parent[i]!]!;
    i = parent[i]!;
  }
  return i;
}
