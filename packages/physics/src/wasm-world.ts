import type { PhysicalModel, Transform } from '@botfoundry/model';
import type { Body } from './body.ts';
import type { Connection } from './connection.ts';
import { Fnv64 } from './hash.ts';
import type { BodyState, ConnectionState, RobotHandle, World, WorldOptions } from './index.ts';
import type { Joint } from './joint.ts';
import { vec3 } from './math.ts';
import { defaultParams, type SolverParams } from './params.ts';
import { buildRobot } from './robot.ts';
import { wasmSolver, type SolverExports } from './wasm.ts';

// The WebAssembly backend. Model ingestion runs in TypeScript exactly as in the reference backend
// (world.ts, robot.ts); each substep (collision, the solve and the ratings) then runs in native/
// over struct-of-arrays state in WebAssembly memory. Every field of that state is a typed-array
// view here, so setup writes straight into it and readout reads straight out of it, and a substep
// passes nothing across the boundary but two counts.

// [name, f64 or u32, capacity group, components], in the order of the store! list in
// native/src/store.rs; the constructor checks the whole table against the module's layout signature.
const FIELDS = [
  ['params', 'f', 'one', 16],
  ['b_pos', 'f', 'b', 3],
  ['b_rot', 'f', 'b', 4],
  ['b_vel', 'f', 'b', 3],
  ['b_ang', 'f', 'b', 3],
  ['b_prev_vel', 'f', 'b', 3],
  ['b_pos0', 'f', 'b', 3],
  ['b_rot0', 'f', 'b', 4],
  ['b_ipos', 'f', 'b', 3],
  ['b_irot', 'f', 'b', 4],
  ['b_inertia', 'f', 'b', 9],
  ['b_iworld', 'f', 'b', 9],
  ['b_mh2', 'f', 'b', 7],
  ['b_mass', 'f', 'b', 1],
  ['b_dq', 'f', 'b', 6],
  ['b_hull_off', 'u', 'b', 1],
  ['b_hull_n', 'u', 'b', 1],
  ['b_radius', 'f', 'b', 1],
  ['b_friction', 'f', 'b', 1],
  ['b_stamp', 'u', 'b', 1],
  ['b_nstamp', 'u', 'b', 1],
  ['j_a', 'u', 'j', 1],
  ['j_b', 'u', 'j', 1],
  ['j_kind', 'u', 'j', 1],
  ['j_flags', 'u', 'j', 1],
  ['j_broken', 'u', 'j', 1],
  ['j_ra', 'f', 'j', 3],
  ['j_rb', 'f', 'j', 3],
  ['j_axis_a', 'f', 'j', 3],
  ['j_axis_b', 'f', 'j', 3],
  ['j_perp1a', 'f', 'j', 3],
  ['j_perp2a', 'f', 'j', 3],
  ['j_perp1b', 'f', 'j', 3],
  ['j_qrel', 'f', 'j', 4],
  ['j_k0', 'f', 'j', 2],
  ['j_stiff', 'f', 'j', 2],
  ['j_limits', 'f', 'j', 2],
  ['j_point_k', 'f', 'j', 1],
  ['j_point_lambda', 'f', 'j', 3],
  ['j_point_c0', 'f', 'j', 3],
  ['j_lock_k', 'f', 'j', 1],
  ['j_lock_lambda', 'f', 'j', 3],
  ['j_lock_c0', 'f', 'j', 3],
  ['j_axis_k', 'f', 'j', 2],
  ['j_axis_lambda', 'f', 'j', 2],
  ['j_axis_c0', 'f', 'j', 2],
  ['j_slide_k', 'f', 'j', 2],
  ['j_slide_lambda', 'f', 'j', 2],
  ['j_slide_c0', 'f', 'j', 2],
  ['j_limit', 'f', 'j', 4],
  ['j_limit_ja', 'f', 'j', 6],
  ['j_limit_jb', 'f', 'j', 6],
  ['j_raw', 'f', 'j', 3],
  ['j_rbw', 'f', 'j', 3],
  ['j_qaq', 'f', 'j', 4],
  ['j_lockc', 'f', 'j', 3],
  ['j_u', 'f', 'j', 6],
  ['j_v', 'f', 'j', 3],
  ['c_rating', 'u', 'j', 1],
  ['c_rating_v', 'f', 'j', 2],
  ['c_axis', 'f', 'j', 3],
  ['c_out', 'f', 'j', 3],
  ['m_a', 'u', 'm', 1],
  ['m_b', 'u', 'm', 1],
  ['m_count', 'u', 'm', 1],
  ['m_stick', 'u', 'm', 4],
  ['m_friction', 'f', 'm', 1],
  ['m_k0', 'f', 'm', 1],
  ['m_feature', 'f', 'm', 4],
  ['m_ra', 'f', 'm', 12],
  ['m_rb', 'f', 'm', 12],
  ['m_basis', 'f', 'm', 9],
  ['m_c0', 'f', 'm', 12],
  ['m_pen', 'f', 'm', 12],
  ['m_lambda', 'f', 'm', 12],
  ['m_ja', 'f', 'm', 72],
  ['m_jb', 'f', 'm', 72],
  ['m_jat', 'f', 'm', 72],
  ['m_jbt', 'f', 'm', 72],
  ['m_seen', 'u', 'm', 1],
  ['active', 'u', 'm', 1],
  ['h_count', 'u', 'h', 1],
  ['h_vert_off', 'u', 'h', 1],
  ['h_face_off', 'u', 'h', 1],
  ['h_face_n', 'u', 'h', 1],
  ['h_edge_off', 'u', 'h', 1],
  ['h_edge_n', 'u', 'h', 1],
  ['h_dir_off', 'u', 'h', 1],
  ['h_dir_n', 'u', 'h', 1],
  ['hv_local', 'f', 'hv', 3],
  ['hv_world', 'f', 'hv', 3],
  ['f_normal', 'f', 'f', 3],
  ['f_world', 'f', 'f', 3],
  ['f_vert_off', 'u', 'f', 1],
  ['f_vert_n', 'u', 'f', 1],
  ['fv', 'u', 'fv', 1],
  ['e', 'u', 'e', 3],
  ['d', 'f', 'd', 3],
  ['broken_out', 'u', 'j', 1],
  ['adj_off', 'u', 'b1', 1],
  ['adj_cur', 'u', 'b1', 1],
  ['adj', 'u', 'adj', 1],
] as const;

/** Rust's spelling of each FIELDS type and capacity group, as the module's layout signature has them. */
const SIGNATURE_TYPES = { f: 'f64', u: 'u32' } as const;
const SIGNATURE_GROUPS = {
  one: 'One', b: 'Body', b1: 'BodyPlusOne', j: 'Joint', m: 'Manifold', adj: 'Adjacency',
  h: 'Hull', hv: 'HullVert', f: 'Face', fv: 'FaceVert', e: 'Edge', d: 'Dir',
} as const;

/**
 * Compares the module's layout signature (`name:type:group:width,` per field, generated in Rust from
 * the same table that lays the arrays out) with FIELDS. Returns null on a match, else a message
 * naming the first field that differs.
 */
export function layoutMismatch(signature: string): string | null {
  const have = signature.split(',').filter((e) => e !== '');
  const want = FIELDS.map(([name, type, group, width]) => `${name}:${SIGNATURE_TYPES[type]}:${SIGNATURE_GROUPS[group]}:${width}`);
  for (let i = 0; i < Math.max(have.length, want.length); i++) {
    if (have[i] === want[i]) continue;
    const name = (want[i] ?? have[i]!).split(':')[0];
    const detail = have[i] === undefined ? 'the module has no such field' : want[i] === undefined ? 'wasm-world.ts has no such field' : `the module has ${have[i]}, wasm-world.ts has ${want[i]}`;
    return `solver.wasm does not match wasm-world.ts at field ${i} (${name}): ${detail}. Rebuild it with build:wasm`;
  }
  return null;
}

/** Status codes of world_reserve and world_step (native/src/world.rs); 0 is success. */
const ABI_ERRORS: Record<number, string> = {
  1: 'the body count is past the allocated body slots',
  2: 'the joint count is past the allocated joints',
  3: 'a joint names a body slot above the body count',
  4: 'a body or hull range is outside its pool',
  5: 'the requested capacity is too large',
};

function abiCheck(call: string, code: number): void {
  if (code !== 0) throw new Error(`${call} rejected its arguments: ${ABI_ERRORS[code] ?? 'unknown error'} (code ${code})`);
}

type Field = (typeof FIELDS)[number];
type Views = { [F in Field as F[0]]: F[1] extends 'f' ? Float64Array : Uint32Array };

const KIND = { fixed: 0, hinge: 1, slider: 2, ball: 3 } as const;
const F_POINT = 1, F_LOCK = 2, F_AXIS = 4, F_SLIDE = 8, F_LIMITS = 16;
const RATING = { none: 0, bolt: 1, weld: 2 } as const;
/** capacity groups, in world_reserve's argument order */
const GROUPS = ['b', 'j', 'm', 'h', 'hv', 'f', 'fv', 'e', 'd'] as const;
type Counts = Record<(typeof GROUPS)[number], number>;

const freeWorlds = new FinalizationRegistry<{ ex: SolverExports; w: number }>(({ ex, w }) => ex.world_free(w));

export class WasmWorld implements World {
  readonly params: SolverParams;
  private readonly ex: SolverExports;
  private readonly w: number;
  private v!: Views;
  private buffer: ArrayBuffer | null = null;
  private version = -1;

  private readonly bodyList: Body[] = [];
  private readonly connectionList: Connection[] = [];
  /** used entries of each capacity group (body slot 0 is the ground) */
  private readonly used: Counts = { b: 1, j: 0, m: 0, h: 0, hv: 0, f: 0, fv: 0, e: 0, d: 0 };
  private robots = 0;
  private hasGround = false;

  constructor(options: WorldOptions) {
    const g = options.gravity ?? [0, -9.81, 0];
    this.params = defaultParams(options.dt ?? 1 / 1000, options.iterations ?? 4, vec3(g[0], g[1], g[2]));
    this.ex = wasmSolver();
    const signature = new TextDecoder().decode(new Uint8Array(this.ex.memory.buffer, this.ex.world_layout_signature(), this.ex.world_layout_signature_len()));
    const mismatch = layoutMismatch(signature);
    if (mismatch) throw new Error(mismatch);
    this.w = this.ex.world_new();
    freeWorlds.register(this, { ex: this.ex, w: this.w });
    this.reserve({ ...this.used, m: 64 });
  }

  /** Grow the native capacities to at least `need` (doubling), then refresh the views. */
  private reserve(need: Counts): void {
    const caps = this.caps();
    const w = GROUPS.map((g) => (need[g] <= caps[g] ? caps[g] : Math.max(need[g], 2 * caps[g])));
    abiCheck('world_reserve', this.ex.world_reserve(this.w, w[0]!, w[1]!, w[2]!, w[3]!, w[4]!, w[5]!, w[6]!, w[7]!, w[8]!));
    this.views();
  }

  private caps(): Counts {
    const c = new Uint32Array(this.ex.memory.buffer, this.ex.world_caps(this.w), GROUPS.length);
    return { b: c[0]!, j: c[1]!, m: c[2]!, h: c[3]!, hv: c[4]!, f: c[5]!, fv: c[6]!, e: c[7]!, d: c[8]! };
  }

  /** The views, rebuilt when any field moved or the memory grew (another world can grow it too). */
  private views(): Views {
    const buffer = this.ex.memory.buffer;
    const version = this.ex.world_version(this.w);
    if (buffer === this.buffer && version === this.version) return this.v;
    const c = this.caps();
    const table = new Uint32Array(buffer, this.ex.world_layout(this.w), FIELDS.length);
    const size = { ...c, one: 1, b1: c.b + 1, adj: 2 * c.j + 2 * c.m };
    const v: Record<string, Float64Array | Uint32Array> = {};
    FIELDS.forEach(([name, type, group, width], i) => {
      const n = size[group] * width;
      v[name] = type === 'f' ? new Float64Array(buffer, table[i]!, n) : new Uint32Array(buffer, table[i]!, n);
    });
    this.v = v as Views;
    this.buffer = buffer;
    this.version = version;
    return this.v;
  }

  addGround(): void {
    if (this.hasGround) return;
    this.hasGround = true;
    this.ex.world_add_ground(this.w);
  }

  addRobot(model: PhysicalModel, pose: Transform): RobotHandle {
    const robotId = `r${this.robots}`;
    const robot = buildRobot(model, pose, robotId, this.bodyList.length, this.params);
    this.robots++;

    const need = { ...this.used };
    need.b += robot.bodies.length;
    need.j += robot.connections.length;
    for (const body of robot.bodies) {
      for (const h of body.hulls) {
        need.h++;
        need.hv += h.count;
        need.f += h.faces.length;
        for (const f of h.faces) need.fv += f.verts.length;
        need.e += h.edges.length;
        need.d += h.edgeDirs.length / 3;
      }
    }
    this.reserve(need);

    const v = this.v, u = this.used;
    for (const body of robot.bodies) {
      // part i of the world is slot i + 1; slot 0 is the ground
      const s = body.index + 1;
      v.b_pos.set(body.pos, 3 * s);
      v.b_rot.set(body.rot, 4 * s);
      v.b_prev_vel.set(body.prevVel, 3 * s);
      v.b_mass[s] = body.mass;
      v.b_inertia.set(body.inertiaLocal, 9 * s);
      v.b_radius[s] = body.radius;
      v.b_friction[s] = body.friction;
      v.b_hull_off[s] = u.h;
      v.b_hull_n[s] = body.hulls.length;
      for (const h of body.hulls) {
        const hi = u.h++;
        v.h_count[hi] = h.count;
        v.h_vert_off[hi] = u.hv;
        v.hv_local.set(h.verts, 3 * u.hv);
        u.hv += h.count;
        v.h_face_off[hi] = u.f;
        v.h_face_n[hi] = h.faces.length;
        for (const f of h.faces) {
          v.f_normal.set(f.normal, 3 * u.f);
          v.f_vert_off[u.f] = u.fv;
          v.f_vert_n[u.f] = f.verts.length;
          v.fv.set(f.verts, u.fv);
          u.fv += f.verts.length;
          u.f++;
        }
        v.h_edge_off[hi] = u.e;
        v.h_edge_n[hi] = h.edges.length;
        for (const e of h.edges) {
          v.e[3 * u.e] = e.i0;
          v.e[3 * u.e + 1] = e.i1;
          v.e[3 * u.e + 2] = e.dir;
          u.e++;
        }
        v.h_dir_off[hi] = u.d;
        v.h_dir_n[hi] = h.edgeDirs.length / 3;
        v.d.set(h.edgeDirs, 3 * u.d);
        u.d += h.edgeDirs.length / 3;
      }
      this.bodyList.push(body);
      u.b++;
    }
    for (const c of robot.connections) {
      this.uploadConnection(u.j++, c);
      this.connectionList.push(c);
    }
    return { id: robotId, parts: robot.parts };
  }

  private uploadConnection(i: number, c: Connection): void {
    const v = this.v;
    const j: Joint = c.joint;
    v.j_a[i] = j.a.index + 1;
    v.j_b[i] = j.b.index + 1;
    v.j_kind[i] = KIND[j.kind];
    v.j_flags[i] =
      (j.hasPoint ? F_POINT : 0) | (j.hasLock ? F_LOCK : 0) | (j.hasAxisRows ? F_AXIS : 0) | (j.hasSlideRows ? F_SLIDE : 0) | (j.limits ? F_LIMITS : 0);
    v.j_broken[i] = 0;
    v.j_ra.set(j.rA, 3 * i);
    v.j_rb.set(j.rB, 3 * i);
    v.j_axis_a.set(j.axisA, 3 * i);
    v.j_axis_b.set(j.axisB, 3 * i);
    v.j_perp1a.set(j.perp1A, 3 * i);
    v.j_perp2a.set(j.perp2A, 3 * i);
    v.j_perp1b.set(j.perp1B, 3 * i);
    v.j_qrel.set(j.qRel, 4 * i);
    v.j_k0[2 * i] = j.k0Lin;
    v.j_k0[2 * i + 1] = j.k0Ang;
    v.j_stiff[2 * i] = j.stiffnessLin;
    v.j_stiff[2 * i + 1] = j.stiffnessAng;
    if (j.limits) {
      v.j_limits[2 * i] = j.limits[0];
      v.j_limits[2 * i + 1] = j.limits[1];
    }
    const r = c.rating;
    v.c_rating[i] = RATING[r.kind];
    if (r.kind === 'bolt') {
      v.c_rating_v[2 * i] = r.rating.shear;
      v.c_rating_v[2 * i + 1] = r.rating.tension;
    } else if (r.kind === 'weld') {
      v.c_rating_v[2 * i] = r.rating.strength;
    }
    v.c_axis.set(c.axisA, 3 * i);
  }

  step(): void {
    const p = this.params;
    const params = this.views().params;
    params[0] = p.dt;
    params[1] = p.iterations;
    params.set(p.gravity, 2);
    params[5] = p.alpha;
    params[6] = p.gamma;
    params[7] = p.betaLin;
    params[8] = p.betaAng;
    params[9] = p.penaltyMin;
    params[10] = p.contactBeta;
    params[11] = p.contactPenaltyMin;
    params[12] = p.penaltyMax;
    params[13] = p.margin;
    params[14] = p.stickSpeed;
    abiCheck('world_step', this.ex.world_step(this.w, this.bodyList.length, this.connectionList.length));
  }

  bodies(): BodyState[] {
    const v = this.views();
    return this.bodyList.map((b) => {
      const s = b.index + 1;
      return {
        id: b.id,
        position: [v.b_pos[3 * s]!, v.b_pos[3 * s + 1]!, v.b_pos[3 * s + 2]!],
        rotation: [v.b_rot[4 * s]!, v.b_rot[4 * s + 1]!, v.b_rot[4 * s + 2]!, v.b_rot[4 * s + 3]!],
        linearVelocity: [v.b_vel[3 * s]!, v.b_vel[3 * s + 1]!, v.b_vel[3 * s + 2]!],
        angularVelocity: [v.b_ang[3 * s]!, v.b_ang[3 * s + 1]!, v.b_ang[3 * s + 2]!],
      };
    });
  }

  connections(): ConnectionState[] {
    const v = this.views();
    return this.connectionList.map((c, i) => ({
      id: c.id,
      axial: v.c_out[3 * i]!,
      shear: v.c_out[3 * i + 1]!,
      utilisation: v.c_out[3 * i + 2]!,
      broken: v.j_broken[i] !== 0,
    }));
  }

  hash(): string {
    const v = this.views();
    const h = new Fnv64();
    for (let s = 1; s <= this.bodyList.length; s++) {
      for (let k = 0; k < 3; k++) h.float(v.b_pos[3 * s + k]!);
      for (let k = 0; k < 4; k++) h.float(v.b_rot[4 * s + k]!);
      for (let k = 0; k < 3; k++) h.float(v.b_vel[3 * s + k]!);
      for (let k = 0; k < 3; k++) h.float(v.b_ang[3 * s + k]!);
    }
    for (let i = 0; i < this.connectionList.length; i++) {
      h.float(v.c_out[3 * i]!);
      h.float(v.c_out[3 * i + 1]!);
      h.float(v.c_out[3 * i + 2]!);
      h.byte(v.j_broken[i] ? 1 : 0);
    }
    return h.hex();
  }
}
