// One substep, a port of PhysicsWorld.step in src/world.ts: collide, warm-start constraints at x⁻
// (AVBD Eq. 19), predict bodies (Eq. 2 with the adaptive warm start of VBD, Chen et al. 2024),
// Gauss-Seidel sweeps of per-body 6x6 solves each followed by the dual updates, BDF1 velocities,
// then the connection ratings. Joints and manifolds are visited in the TypeScript backend's order
// and bodies in an order equivalent to it (see Schedule), so the two produce bit-identical states.

use std::cmp::Reverse;
use std::collections::BinaryHeap;

use crate::collide::{self, Collision};
use crate::contact;
use crate::joint;
use crate::math::*;
use crate::store::*;
use crate::system::{solve, solve2, Sys};

/// Status codes of the C ABI: 0 is success. src/wasm-world.ts names the others.
pub const OK: u32 = 0;
/// bodies + the ground's slot exceed the body capacity
pub const ERR_BODIES: u32 = 1;
/// the joint count exceeds the joint capacity
pub const ERR_JOINTS: u32 = 2;
/// a joint names a body slot above the body count
pub const ERR_JOINT_BODY: u32 = 3;
/// a body's hull range or a hull's vertex, face, edge or direction range leaves its pool
pub const ERR_HULL_POOL: u32 = 4;
/// a reserve would pass MAX_CAP or MAX_STORE_BYTES
pub const ERR_CAPACITY: u32 = 5;

pub struct World {
    pub caps: Caps,
    pub s: Store,
    pub layout: Vec<u32>,
    /// bumped whenever a field may have moved, so TypeScript knows to rebuild its views
    pub version: u32,
    pub col: Collision,
    pub caps_out: [u32; 9],
    pub sched: Schedule,
    pub warm: Warm,
}

/// Scratch for the per-assembly warm-start weight, indexed by body slot (see assembly_weights).
#[derive(Default)]
pub struct Warm {
    root: Vec<u32>,
    weight: Vec<f64>,
    mass_sum: Vec<f64>,
}

/// The order of body solves within a sweep, as (body, partner) with partner 0 for a body solved
/// alone. Gauss-Seidel in index order gives the same result under any order that keeps every
/// pair of bodies sharing a constraint in index order, since a body's solve reads only itself and
/// its neighbours; two ready bodies that share no constraint can therefore be solved together.
#[derive(Default)]
pub struct Schedule {
    order: Vec<(u32, u32)>,
    pending: Vec<u32>,
    ready: BinaryHeap<Reverse<u32>>,
    held: Vec<u32>,
    /// the adjacency the order was built from; an unchanged one (a resting scene) reuses it
    built_off: Vec<u32>,
    built_adj: Vec<u32>,
}

fn resize(caps: &mut Caps, s: &mut Store, layout: &mut Vec<u32>, version: &mut u32, want: &[usize; 9]) {
    for (c, &w) in caps.0.iter_mut().zip(want) {
        *c = (*c).max(w);
    }
    s.resize(caps);
    s.table(layout);
    *version += 1;
}

impl World {
    pub fn new() -> World {
        let mut w = World { caps: Caps([0; 9]), s: Store::new(), layout: Vec::new(), version: 0, col: Collision::new(), caps_out: [0; 9], sched: Schedule::default(), warm: Warm::default() };
        w.reserve(&[1, 0, 0, 0, 0, 0, 0, 0, 0]);
        // the ground: static, at the origin, unrotated
        w.s.b_rot[3] = 1.0;
        w.s.b_rot0[3] = 1.0;
        w
    }

    pub fn reserve(&mut self, want: &[usize; 9]) {
        resize(&mut self.caps, &mut self.s, &mut self.layout, &mut self.version, want);
    }

    /// Whether a reserve of `want` is within MAX_CAP and MAX_STORE_BYTES (it only ever grows).
    pub fn check_reserve(&self, want: &[usize; 9]) -> u32 {
        let mut caps = self.caps.0;
        for (c, &w) in caps.iter_mut().zip(want) {
            *c = (*c).max(w);
            if *c > MAX_CAP {
                return ERR_CAPACITY;
            }
        }
        if store_bytes(&Caps(caps)) > MAX_STORE_BYTES {
            return ERR_CAPACITY;
        }
        OK
    }

    /// Whether step(bodies, joints) stays inside the allocated arrays: the counts against the
    /// capacities, then every index the step takes from them (a joint's two body slots, a body's
    /// hulls and each hull's ranges in the pools). Checked once per step; the inner loops index
    /// unchecked. Indices deeper in the geometry (face vertices, edge endpoints) are not rechecked:
    /// they are written by addRobot alongside the ranges checked here.
    pub fn check_step(&self, bodies: usize, joints: usize) -> u32 {
        let (c, s) = (&self.caps.0, &self.s);
        // slots 0..=bodies must exist, slot 0 being the ground
        if bodies >= c[0] {
            return ERR_BODIES;
        }
        if joints > c[1] {
            return ERR_JOINTS;
        }
        for j in 0..joints {
            if s.j_a[j] as usize > bodies || s.j_b[j] as usize > bodies {
                return ERR_JOINT_BODY;
            }
        }
        // u64 sums: two u32 offsets and counts cannot wrap there
        let fits = |off: u32, n: u32, cap: usize| off as u64 + n as u64 <= cap as u64;
        for i in 1..=bodies {
            let (off, n) = (s.b_hull_off[i], s.b_hull_n[i]);
            if !fits(off, n, c[3]) {
                return ERR_HULL_POOL;
            }
            for h in off as usize..(off + n) as usize {
                if !fits(s.h_vert_off[h], s.h_count[h], c[4])
                    || !fits(s.h_face_off[h], s.h_face_n[h], c[5])
                    || !fits(s.h_edge_off[h], s.h_edge_n[h], c[7])
                    || !fits(s.h_dir_off[h], s.h_dir_n[h], c[8])
                {
                    return ERR_HULL_POOL;
                }
            }
        }
        OK
    }

    /// One substep for slots 1..=bodies and joints 0..joints: collision, then the solve.
    /// Returns how many connections broke; their indices are in broken_out.
    pub fn step(&mut self, bodies: usize, joints: usize) -> usize {
        let World { caps, s, layout, version, col, sched, warm, .. } = self;
        let p = Params::read(&s.params);
        col.link_new_joints(s, joints);
        let mut grow = |s: &mut Store, manifolds: usize| {
            let mut want = caps.0;
            want[2] = manifolds.max(2 * want[2]);
            resize(caps, s, layout, version, &want);
        };
        let active = collide::collide(col, s, &p, bodies, &mut grow);

        let dt = p.dt;
        let h2 = dt * dt;
        let g = p.g;
        let g_len = (g[0] * g[0] + g[1] * g[1] + g[2] * g[2]).sqrt();

        for j in 0..joints {
            if at!(s.j_broken, j) == 0 {
                joint::prepare(s, &p, j);
            }
        }
        let mut live = 0;
        for k in 0..active {
            let m = at!(s.active, k) as usize;
            if contact::prepare(s, &p, m) {
                at_mut!(s.active, live) = m as u32;
                live += 1;
            }
        }
        build_adjacency(s, bodies, joints, live);
        build_schedule(s, bodies, sched);

        // one adaptive warm-start weight per jointed assembly (see PhysicsWorld.step)
        if g_len > 0.0 {
            assembly_weights(warm, s, bodies, joints, dt, g, g_len);
        }

        for i in 1..=bodies {
            for c in 0..3 {
                at_mut!(s.b_pos0, 3 * i + c) = at!(s.b_pos, 3 * i + c);
            }
            for c in 0..4 {
                at_mut!(s.b_rot0, 4 * i + c) = at!(s.b_rot, 4 * i + c);
            }
            if at!(s.b_mass, i) == 0.0 {
                continue;
            }
            let rot = q4(&s.b_rot, i);
            let iw = world_inertia(rot, &s.b_inertia[9 * i..9 * i + 9]);
            s.b_iworld[9 * i..9 * i + 9].copy_from_slice(&iw);
            let mh2 = [at!(s.b_mass, i) / h2, iw[0] / h2, iw[3] / h2, iw[4] / h2, iw[6] / h2, iw[7] / h2, iw[8] / h2];
            s.b_mh2[7 * i..7 * i + 7].copy_from_slice(&mh2);
            let pos = v3(&s.b_pos, i);
            let vel = v3(&s.b_vel, i);
            let ang = v3(&s.b_ang, i);
            set3(&mut s.b_ipos, i, [pos[0] + dt * vel[0] + h2 * g[0], pos[1] + dt * vel[1] + h2 * g[1], pos[2] + dt * vel[2] + h2 * g[2]]);
            set4(&mut s.b_irot, i, quat_add_rotation(rot, ang, dt));
            let w = if g_len > 0.0 { warm.weight[warm.root[i] as usize] } else { 0.0 };
            set3(&mut s.b_pos, i, [pos[0] + dt * vel[0] + w * h2 * g[0], pos[1] + dt * vel[1] + w * h2 * g[1], pos[2] + dt * vel[2] + w * h2 * g[2]]);
            set4(&mut s.b_rot, i, quat_add_rotation(rot, ang, dt));
        }

        for i in 0..=bodies {
            let d = joint::body_delta(s, i);
            s.b_dq[6 * i..6 * i + 6].copy_from_slice(&d);
        }
        for j in 0..joints {
            if at!(s.j_broken, j) == 0 {
                joint::refresh(s, j, true, true);
            }
        }

        for _ in 0..p.iterations {
            for k in 0..sched.order.len() {
                let (x, y) = sched.order[k];
                if y == 0 {
                    solve_body(s, x as usize, h2);
                } else {
                    solve_pair(s, x as usize, y as usize, h2);
                }
            }
            for j in 0..joints {
                if at!(s.j_broken, j) == 0 {
                    joint::dual(s, &p, j);
                }
            }
            for k in 0..live {
                let m = at!(s.active, k) as usize;
                contact::dual(s, &p, m);
            }
        }

        let inv = 1.0 / dt;
        for i in 1..=bodies {
            if at!(s.b_mass, i) == 0.0 {
                continue;
            }
            for c in 0..3 {
                at_mut!(s.b_prev_vel, 3 * i + c) = at!(s.b_vel, 3 * i + c);
                at_mut!(s.b_vel, 3 * i + c) = (at!(s.b_pos, 3 * i + c) - at!(s.b_pos0, 3 * i + c)) * inv;
            }
            let w = quat_diff(q4(&s.b_rot, i), q4(&s.b_rot0, i));
            set3(&mut s.b_ang, i, [w[0] * inv, w[1] * inv, w[2] * inv]);
        }

        let mut broken = 0;
        for j in 0..joints {
            if joint::measure(s, j) {
                at_mut!(s.broken_out, broken) = j as u32;
                broken += 1;
                col.unlink(s, j);
            }
        }
        broken
    }
}

/// Per-body constraint lists in the order the TypeScript backend stamps them: the body's live
/// joints by creation order, then its manifolds in active (sorted-key) order. Static bodies are
/// never solved, so they get no list.
/// Union-find root with path halving; roots are always the lowest slot in their set.
fn find_root(parent: &mut [u32], mut i: usize) -> usize {
    while parent[i] as usize != i {
        parent[i] = parent[parent[i] as usize];
        i = parent[i] as usize;
    }
    i
}

/// A port of PhysicsWorld.assemblyWeights: group the moving bodies into assemblies joined by
/// unbroken joints (static bodies join nothing) and give each one warm-start weight, the share of
/// gravity its centre of mass accelerated with last substep, clamped to [0, 1]. Fills warm.root
/// (each body's assembly, named by its lowest slot) and warm.weight (indexed by root).
#[inline(never)]
fn assembly_weights(warm: &mut Warm, s: &Store, bodies: usize, joints: usize, dt: f64, g: [f64; 3], g_len: f64) {
    let n = bodies + 1;
    if warm.root.len() < n {
        warm.root.resize(n, 0);
        warm.weight.resize(n, 0.0);
        warm.mass_sum.resize(n, 0.0);
    }
    let Warm { root, weight, mass_sum } = warm;
    for i in 0..n {
        root[i] = i as u32;
    }
    for j in 0..joints {
        if at!(s.j_broken, j) != 0 {
            continue;
        }
        let (a, b) = (at!(s.j_a, j) as usize, at!(s.j_b, j) as usize);
        if at!(s.b_mass, a) == 0.0 || at!(s.b_mass, b) == 0.0 {
            continue;
        }
        let (ra, rb) = (find_root(root, a), find_root(root, b));
        if ra < rb {
            root[rb] = ra as u32;
        } else if rb < ra {
            root[ra] = rb as u32;
        }
    }
    for i in 0..n {
        root[i] = find_root(root, i) as u32;
        weight[i] = 0.0;
        mass_sum[i] = 0.0;
    }
    // mass-weighted acceleration along gravity, summed in slot order
    for i in 1..=bodies {
        let m = at!(s.b_mass, i);
        if m == 0.0 {
            continue;
        }
        let vel = v3(&s.b_vel, i);
        let prev = v3(&s.b_prev_vel, i);
        let mut along = 0.0;
        for c in 0..3 {
            along += ((vel[c] - prev[c]) / dt) * (g[c] / g_len);
        }
        let r = root[i] as usize;
        weight[r] = weight[r] + m * along;
        mass_sum[r] = mass_sum[r] + m;
    }
    for i in 0..n {
        if root[i] as usize != i || !(mass_sum[i] > 0.0) {
            continue;
        }
        let w = weight[i] / mass_sum[i] / g_len;
        weight[i] = if w > 1.0 {
            1.0
        } else if w > 0.0 {
            w
        } else {
            0.0
        };
    }
}

#[inline(never)]
fn build_adjacency(s: &mut Store, bodies: usize, joints: usize, live: usize) {
    let off = &mut s.adj_off;
    for i in 0..=bodies + 1 {
        off[i] = 0;
    }
    let dynamic = |mass: &[f64], i: usize| mass[i] != 0.0;
    for j in 0..joints {
        if at!(s.j_broken, j) != 0 {
            continue;
        }
        let (a, b) = (at!(s.j_a, j) as usize, at!(s.j_b, j) as usize);
        if dynamic(&s.b_mass, a) {
            off[a + 1] += 1;
        }
        if dynamic(&s.b_mass, b) {
            off[b + 1] += 1;
        }
    }
    for k in 0..live {
        let m = at!(s.active, k) as usize;
        let (a, b) = (at!(s.m_a, m) as usize, at!(s.m_b, m) as usize);
        if dynamic(&s.b_mass, a) {
            off[a + 1] += 1;
        }
        if dynamic(&s.b_mass, b) {
            off[b + 1] += 1;
        }
    }
    for i in 1..=bodies + 1 {
        off[i] += off[i - 1];
    }
    let cur = &mut s.adj_cur;
    cur[..=bodies].copy_from_slice(&off[..=bodies]);
    for j in 0..joints {
        if at!(s.j_broken, j) != 0 {
            continue;
        }
        let (a, b) = (at!(s.j_a, j) as usize, at!(s.j_b, j) as usize);
        for x in [a, b] {
            if dynamic(&s.b_mass, x) {
                s.adj[cur[x] as usize] = (j as u32) << 1;
                cur[x] += 1;
            }
        }
    }
    for k in 0..live {
        let m = at!(s.active, k) as usize;
        let (a, b) = (at!(s.m_a, m) as usize, at!(s.m_b, m) as usize);
        for x in [a, b] {
            if dynamic(&s.b_mass, x) {
                s.adj[cur[x] as usize] = ((m as u32) << 1) | 1;
                cur[x] += 1;
            }
        }
    }
}

/// The other body of adjacency entry c of body i, if it is dynamic.
#[inline(always)]
fn neighbour(s: &Store, c: u32, i: usize) -> Option<usize> {
    let k = (c >> 1) as usize;
    let (a, b) = if c & 1 == 0 { (at!(s.j_a, k), at!(s.j_b, k)) } else { (at!(s.m_a, k), at!(s.m_b, k)) };
    let o = if a as usize == i { b } else { a } as usize;
    if at!(s.b_mass, o) != 0.0 {
        Some(o)
    } else {
        None
    }
}

/// Pair up bodies as Schedule describes: repeatedly take the lowest-index body whose
/// lower-index neighbours are all solved, and with it the next such body it shares no
/// constraint with, if any.
#[inline(never)]
fn build_schedule(s: &Store, bodies: usize, sched: &mut Schedule) {
    let Schedule { order, pending, ready, held, built_off, built_adj } = sched;
    let off = &s.adj_off[..bodies + 2];
    let adj = &s.adj[..at!(s.adj_off, bodies + 1) as usize];
    if !order.is_empty() && built_off[..] == *off && built_adj[..] == *adj {
        return;
    }
    built_off.clear();
    built_off.extend_from_slice(off);
    built_adj.clear();
    built_adj.extend_from_slice(adj);
    order.clear();
    ready.clear();
    pending.clear();
    pending.resize(bodies + 1, 0);
    for i in 1..=bodies {
        if at!(s.b_mass, i) == 0.0 {
            continue;
        }
        for e in at!(s.adj_off, i)..at!(s.adj_off, i + 1) {
            if neighbour(s, at!(s.adj, e as usize), i).is_some_and(|o| o < i) {
                pending[i] += 1;
            }
        }
        if pending[i] == 0 {
            ready.push(Reverse(i as u32));
        }
    }
    let adjacent = |x: usize, y: usize| (at!(s.adj_off, x)..at!(s.adj_off, x + 1)).any(|e| neighbour(s, at!(s.adj, e as usize), x) == Some(y));
    while let Some(Reverse(x)) = ready.pop() {
        held.clear();
        let mut y = 0;
        while let Some(Reverse(c)) = ready.pop() {
            if adjacent(x as usize, c as usize) {
                held.push(c);
            } else {
                y = c;
                break;
            }
        }
        for &c in held.iter() {
            ready.push(Reverse(c));
        }
        order.push((x, y));
        for z in [x, y] {
            if z == 0 {
                continue;
            }
            let z = z as usize;
            for e in at!(s.adj_off, z)..at!(s.adj_off, z + 1) {
                if let Some(o) = neighbour(s, at!(s.adj, e as usize), z) {
                    if o > z {
                        pending[o] -= 1;
                        if pending[o] == 0 {
                            ready.push(Reverse(o as u32));
                        }
                    }
                }
            }
        }
    }
}

/// Assemble body i's 6x6 system for one primal step (Eq. 4-6).
#[inline(always)]
fn assemble(s: &Store, i: usize, h2: f64) -> Sys {
    let mut sys = Sys { lhs: [0.0; 36], rhs: [0.0; 6] };
    let mh2 = &s.b_mh2[7 * i..7 * i + 7];
    let m = mh2[0];
    sys.lhs[0] = m;
    sys.lhs[7] = m;
    sys.lhs[14] = m;
    sys.lhs[21] = mh2[1];
    sys.lhs[27] = mh2[2];
    sys.lhs[28] = mh2[3];
    sys.lhs[33] = mh2[4];
    sys.lhs[34] = mh2[5];
    sys.lhs[35] = mh2[6];
    let iw = &s.b_iworld[9 * i..9 * i + 9];
    let pos = v3(&s.b_pos, i);
    let ip = v3(&s.b_ipos, i);
    for c in 0..3 {
        sys.rhs[c] = m * (pos[c] - ip[c]);
    }
    let d = quat_diff(q4(&s.b_rot, i), q4(&s.b_irot, i));
    for r in 0..3 {
        sys.rhs[3 + r] = (iw[r * 3] * d[0] + iw[r * 3 + 1] * d[1] + iw[r * 3 + 2] * d[2]) / h2;
    }
    for e in at!(s.adj_off, i)..at!(s.adj_off, i + 1) {
        let c = at!(s.adj, e as usize);
        if c & 1 == 0 {
            joint::stamp(s, (c >> 1) as usize, i, &mut sys);
        } else {
            contact::stamp(s, (c >> 1) as usize, i, &mut sys);
        }
    }
    sys
}

/// Apply body i's step and refresh what depends on its pose.
#[inline(always)]
fn apply(s: &mut Store, i: usize, dx: [f64; 6]) {
    let pos = v3(&s.b_pos, i);
    set3(&mut s.b_pos, i, [pos[0] + dx[0], pos[1] + dx[1], pos[2] + dx[2]]);
    let rot = quat_add_rotation(q4(&s.b_rot, i), [dx[3], dx[4], dx[5]], 1.0);
    set4(&mut s.b_rot, i, rot);

    // keep what depends on this body's pose current for the constraints that read it next
    let d = joint::body_delta(s, i);
    s.b_dq[6 * i..6 * i + 6].copy_from_slice(&d);
    for e in at!(s.adj_off, i)..at!(s.adj_off, i + 1) {
        let c = at!(s.adj, e as usize);
        if c & 1 == 0 {
            let j = (c >> 1) as usize;
            joint::refresh(s, j, at!(s.j_a, j) as usize == i, at!(s.j_b, j) as usize == i);
        }
    }
}

/// One primal step for body i: assemble and solve its 6x6 system, apply the update.
#[inline(never)]
fn solve_body(s: &mut Store, i: usize, h2: f64) {
    let sys = assemble(s, i, h2);
    apply(s, i, solve(&sys));
}

/// solve_body for two bodies that share no constraint, with both factorisations in SIMD lanes.
#[inline(never)]
fn solve_pair(s: &mut Store, x: usize, y: usize, h2: f64) {
    let sx = assemble(s, x, h2);
    let sy = assemble(s, y, h2);
    let (dx, dy) = solve2(&sx, &sy);
    apply(s, x, dx);
    apply(s, y, dy);
}
