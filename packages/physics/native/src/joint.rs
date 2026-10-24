// Hard joints and rated connections: a port of src/joint.ts and src/connection.ts over the
// struct-of-arrays store (Giles, Diaz, Yuksel, "Augmented Vertex Block Descent", SIGGRAPH 2025;
// structure after the authors' MIT avbd-demo3d). Sign convention and row definitions are documented
// in joint.ts; the arithmetic here keeps its order exactly.

use crate::math::*;
use crate::store::*;
use crate::system::{stamp_angular_row, stamp_row, Sys};

const ANGLE_MARGIN: f64 = 0.02;

/// World anchor offsets and p_A − p_B.
#[inline(always)]
fn point_c(s: &Store, j: usize, a: usize, b: usize) -> (V3, V3, V3) {
    let ra = rotate(q4(&s.b_rot, a), v3(&s.j_ra, j));
    let rb = rotate(q4(&s.b_rot, b), v3(&s.j_rb, j));
    let pa = v3(&s.b_pos, a);
    let pb = v3(&s.b_pos, b);
    let c = [pa[0] + ra[0] - pb[0] - rb[0], pa[1] + ra[1] - pb[1] - rb[1], pa[2] + ra[2] - pb[2] - rb[2]];
    (ra, rb, c)
}

/// World anchor offsets and d = p_B − p_A.
#[inline(always)]
fn separation(s: &Store, j: usize, a: usize, b: usize) -> (V3, V3, V3) {
    let ra = rotate(q4(&s.b_rot, a), v3(&s.j_ra, j));
    let rb = rotate(q4(&s.b_rot, b), v3(&s.j_rb, j));
    let pa = v3(&s.b_pos, a);
    let pb = v3(&s.b_pos, b);
    let d = [pb[0] + rb[0] - pa[0] - ra[0], pb[1] + rb[1] - pa[1] - ra[1], pb[2] + rb[2] - pa[2] - ra[2]];
    (ra, rb, d)
}

/// point_c from the refreshed anchors.
#[inline(always)]
fn point_c_cached(s: &Store, j: usize, a: usize, b: usize) -> (V3, V3, V3) {
    let ra = v3(&s.j_raw, j);
    let rb = v3(&s.j_rbw, j);
    let pa = v3(&s.b_pos, a);
    let pb = v3(&s.b_pos, b);
    let c = [pa[0] + ra[0] - pb[0] - rb[0], pa[1] + ra[1] - pb[1] - rb[1], pa[2] + ra[2] - pb[2] - rb[2]];
    (ra, rb, c)
}

/// separation from the refreshed anchors.
#[inline(always)]
fn separation_cached(s: &Store, j: usize, a: usize, b: usize) -> (V3, V3, V3) {
    let ra = v3(&s.j_raw, j);
    let rb = v3(&s.j_rbw, j);
    let pa = v3(&s.b_pos, a);
    let pb = v3(&s.b_pos, b);
    let d = [pb[0] + rb[0] - pa[0] - ra[0], pb[1] + rb[1] - pa[1] - ra[1], pb[2] + rb[2] - pa[2] - ra[2]];
    (ra, rb, d)
}

/// Recompute joint j's world-frame quantities after body A moved (side_a), body B moved (side_b),
/// or both. They are exactly what stamp and dual would compute from the current rotations.
#[inline(never)]
pub fn refresh(s: &mut Store, j: usize, side_a: bool, side_b: bool) {
    let flags = at!(s.j_flags, j);
    if side_a {
        let qa = q4(&s.b_rot, at!(s.j_a, j) as usize);
        if flags & (F_POINT | F_SLIDE) != 0 {
            let r = rotate(qa, v3(&s.j_ra, j));
            set3(&mut s.j_raw, j, r);
        }
        if flags & F_LOCK != 0 {
            let q = quat_mul(qa, q4(&s.j_qrel, j));
            set4(&mut s.j_qaq, j, q);
        }
        if flags & (F_AXIS | F_SLIDE) != 0 {
            let u0 = rotate(qa, v3(&s.j_perp1a, j));
            let u1 = rotate(qa, v3(&s.j_perp2a, j));
            set3(&mut s.j_u, 2 * j, u0);
            set3(&mut s.j_u, 2 * j + 1, u1);
        }
    }
    let qb = q4(&s.b_rot, at!(s.j_b, j) as usize);
    if side_b {
        if flags & (F_POINT | F_SLIDE) != 0 {
            let r = rotate(qb, v3(&s.j_rb, j));
            set3(&mut s.j_rbw, j, r);
        }
        if flags & F_AXIS != 0 {
            let v = rotate(qb, v3(&s.j_axis_b, j));
            set3(&mut s.j_v, j, v);
        }
    }
    if flags & F_LOCK != 0 {
        let c = quat_diff(q4(&s.j_qaq, j), qb);
        set3(&mut s.j_lockc, j, c);
    }
}

#[inline(always)]
fn lock_c(s: &Store, j: usize, a: usize, b: usize) -> V3 {
    quat_diff(quat_mul(q4(&s.b_rot, a), q4(&s.j_qrel, j)), q4(&s.b_rot, b))
}

#[inline(always)]
fn perp(s: &Store, j: usize, k: usize) -> V3 {
    if k == 0 {
        v3(&s.j_perp1a, j)
    } else {
        v3(&s.j_perp2a, j)
    }
}

/// Jacobian of u · (p_B − p_A) for body A or B.
#[inline(always)]
fn slide_jacobian(u: V3, ra: V3, rb: V3, d: V3, is_a: bool) -> [f64; 6] {
    if is_a {
        let c = cross(u, [ra[0] + d[0], ra[1] + d[1], ra[2] + d[2]]);
        [-u[0], -u[1], -u[2], c[0], c[1], c[2]]
    } else {
        let c = cross(rb, u);
        [u[0], u[1], u[2], c[0], c[1], c[2]]
    }
}

/// The refreshed body_delta of body i.
#[inline(always)]
pub fn dq(s: &Store, i: usize) -> [f64; 6] {
    debug_assert!(6 * i + 5 < s.b_dq.len());
    unsafe { *(s.b_dq.as_ptr().add(6 * i) as *const [f64; 6]) }
}

/// [x − x₀, θ ⊖ θ₀] since the start of the substep.
#[inline(always)]
pub fn body_delta(s: &Store, i: usize) -> [f64; 6] {
    let p = v3(&s.b_pos, i);
    let p0 = v3(&s.b_pos0, i);
    let t = quat_diff(q4(&s.b_rot, i), q4(&s.b_rot0, i));
    [p[0] - p0[0], p[1] - p0[1], p[2] - p0[2], t[0], t[1], t[2]]
}

fn limit_c(s: &Store, j: usize, a: usize, b: usize) -> f64 {
    let dqa = dq(s, a);
    let dqb = dq(s, b);
    let mut c = at!(s.j_limit, 4 * j + 3);
    for i in 0..6 {
        c += at!(s.j_limit_ja, 6 * j + i) * dqa[i] + at!(s.j_limit_jb, 6 * j + i) * dqb[i];
    }
    c
}

/// Hinge angle (rad) or slider travel (m) relative to the pose at creation.
fn position(s: &Store, j: usize, a: usize, b: usize) -> f64 {
    let qa = q4(&s.b_rot, a);
    if at!(s.j_kind, j) == KIND_SLIDER {
        let (_, _, d) = separation(s, j, a, b);
        let u = rotate(qa, v3(&s.j_axis_a, j));
        return dot(u, d);
    }
    let u = rotate(qa, v3(&s.j_perp1a, j));
    let v = rotate(q4(&s.b_rot, b), v3(&s.j_perp1b, j));
    let t = rotate(qa, v3(&s.j_axis_a, j));
    let f = cross(u, v);
    datan2(dot(t, f), dot(u, v))
}

#[inline(never)]
pub fn prepare(s: &mut Store, p: &Params, j: usize) {
    let a = at!(s.j_a, j) as usize;
    let b = at!(s.j_b, j) as usize;
    let flags = at!(s.j_flags, j);
    let decay = p.gamma;
    // the angle lock keeps AVBD's alpha * gamma (Eq. 19); see Joint.prepare in src/joint.ts
    let lock_decay = p.alpha * p.gamma;
    let (k0_lin, k0_ang) = (at!(s.j_k0, 2 * j), at!(s.j_k0, 2 * j + 1));
    let (st_lin, st_ang) = (at!(s.j_stiff, 2 * j), at!(s.j_stiff, 2 * j + 1));
    if flags & F_POINT != 0 {
        let (_, _, c) = point_c(s, j, a, b);
        set3(&mut s.j_point_c0, j, [p.alpha * c[0], p.alpha * c[1], p.alpha * c[2]]);
        for i in 0..3 {
            at_mut!(s.j_point_lambda, 3 * j + i) = at!(s.j_point_lambda, 3 * j + i) * decay;
        }
        at_mut!(s.j_point_k, j) = js_min(p.warm(at!(s.j_point_k, j), k0_lin, p.penalty_min), st_lin);
    }
    if flags & F_LOCK != 0 {
        let c = lock_c(s, j, a, b);
        set3(&mut s.j_lock_c0, j, [p.alpha * c[0], p.alpha * c[1], p.alpha * c[2]]);
        for i in 0..3 {
            at_mut!(s.j_lock_lambda, 3 * j + i) = at!(s.j_lock_lambda, 3 * j + i) * lock_decay;
        }
        at_mut!(s.j_lock_k, j) = js_min(p.warm(at!(s.j_lock_k, j), k0_ang, p.penalty_min), st_ang);
    }
    if flags & F_AXIS != 0 {
        let v = rotate(q4(&s.b_rot, b), v3(&s.j_axis_b, j));
        for k in 0..2 {
            let u = rotate(q4(&s.b_rot, a), perp(s, j, k));
            at_mut!(s.j_axis_c0, 2 * j + k) = p.alpha * dot(u, v);
            at_mut!(s.j_axis_lambda, 2 * j + k) = at!(s.j_axis_lambda, 2 * j + k) * decay;
            at_mut!(s.j_axis_k, 2 * j + k) = js_min(p.warm(at!(s.j_axis_k, 2 * j + k), k0_ang, p.penalty_min), st_ang);
        }
    }
    if flags & F_SLIDE != 0 {
        let (_, _, d) = separation(s, j, a, b);
        for k in 0..2 {
            let u = rotate(q4(&s.b_rot, a), perp(s, j, k));
            at_mut!(s.j_slide_c0, 2 * j + k) = p.alpha * dot(u, d);
            at_mut!(s.j_slide_lambda, 2 * j + k) = at!(s.j_slide_lambda, 2 * j + k) * decay;
            at_mut!(s.j_slide_k, 2 * j + k) = js_min(p.warm(at!(s.j_slide_k, 2 * j + k), k0_lin, p.penalty_min), st_lin);
        }
    }
    if flags & F_LIMITS != 0 {
        prepare_limit(s, p, j, a, b);
    }
}

fn prepare_limit(s: &mut Store, p: &Params, j: usize, a: usize, b: usize) {
    let lo = at!(s.j_limits, 2 * j);
    let hi = at!(s.j_limits, 2 * j + 1);
    let hinge = at!(s.j_kind, j) == KIND_HINGE;
    let x = position(s, j, a, b);
    let margin = if hinge { ANGLE_MARGIN } else { p.margin };
    let side: f64 = if x < lo + margin {
        -1.0
    } else if x > hi - margin {
        1.0
    } else {
        0.0
    };
    if side != at!(s.j_limit, 4 * j) {
        at_mut!(s.j_limit, 4 * j + 2) = 0.0;
        at_mut!(s.j_limit, 4 * j + 1) = 0.0;
    }
    at_mut!(s.j_limit, 4 * j) = side;
    if side == 0.0 {
        return;
    }
    let c0 = if side < 0.0 { x - lo } else { hi - x };
    at_mut!(s.j_limit, 4 * j + 3) = if c0 > 0.0 { c0 } else { (1.0 - p.alpha) * c0 };
    let qa = q4(&s.b_rot, a);
    if hinge {
        let t = rotate(qa, v3(&s.j_axis_a, j));
        for i in 0..6 {
            at_mut!(s.j_limit_ja, 6 * j + i) = 0.0;
            at_mut!(s.j_limit_jb, 6 * j + i) = 0.0;
        }
        for i in 0..3 {
            at_mut!(s.j_limit_ja, 6 * j + 3 + i) = -side * -t[i];
            at_mut!(s.j_limit_jb, 6 * j + 3 + i) = -side * t[i];
        }
        at_mut!(s.j_limit, 4 * j + 1) = p.warm(at!(s.j_limit, 4 * j + 1), at!(s.j_k0, 2 * j + 1), p.penalty_min);
    } else {
        let (ra, rb, d) = separation(s, j, a, b);
        let u = rotate(qa, v3(&s.j_axis_a, j));
        let ja = slide_jacobian(u, ra, rb, d, true);
        let jb = slide_jacobian(u, ra, rb, d, false);
        for i in 0..6 {
            at_mut!(s.j_limit_ja, 6 * j + i) = -side * ja[i];
            at_mut!(s.j_limit_jb, 6 * j + i) = -side * jb[i];
        }
        at_mut!(s.j_limit, 4 * j + 1) = p.warm(at!(s.j_limit, 4 * j + 1), at!(s.j_k0, 2 * j), p.penalty_min);
    }
    at_mut!(s.j_limit, 4 * j + 2) = at!(s.j_limit, 4 * j + 2) * p.gamma;
}

/// |g f_j − (g·f) e_j| for j = 0, 1, 2: the diagonalised geometric stiffness (AVBD Sec. 3.5).
#[cfg(not(target_feature = "simd128"))]
#[inline(always)]
fn geometric_norms(gx: f64, gy: f64, gz: f64, f: [f64; 3], gf: f64) -> [f64; 3] {
    let mut n = [0.0; 3];
    for jj in 0..3 {
        let fj = f[jj];
        let cx = gx * fj - (if jj == 0 { gf } else { 0.0 });
        let cy = gy * fj - (if jj == 1 { gf } else { 0.0 });
        let cz = gz * fj - (if jj == 2 { gf } else { 0.0 });
        n[jj] = (cx * cx + cy * cy + cz * cz).sqrt();
    }
    n
}

/// geometric_norms with j = 0 and 1 in f64x2 lanes, each doing that j's scalar operations.
#[cfg(target_feature = "simd128")]
#[inline(always)]
fn geometric_norms(gx: f64, gy: f64, gz: f64, f: [f64; 3], gf: f64) -> [f64; 3] {
    use core::arch::wasm32::*;
    let f01 = f64x2(f[0], f[1]);
    let cx = f64x2_sub(f64x2_mul(f64x2_splat(gx), f01), f64x2(gf, 0.0));
    let cy = f64x2_sub(f64x2_mul(f64x2_splat(gy), f01), f64x2(0.0, gf));
    let cz = f64x2_sub(f64x2_mul(f64x2_splat(gz), f01), f64x2_splat(0.0));
    let n01 = f64x2_sqrt(f64x2_add(f64x2_add(f64x2_mul(cx, cx), f64x2_mul(cy, cy)), f64x2_mul(cz, cz)));
    let f2 = f[2];
    let (cx2, cy2, cz2) = (gx * f2 - 0.0, gy * f2 - 0.0, gz * f2 - gf);
    [f64x2_extract_lane::<0>(n01), f64x2_extract_lane::<1>(n01), (cx2 * cx2 + cy2 * cy2 + cz2 * cz2).sqrt()]
}

/// Add joint j's force and stiffness to the system of `body` (one of its two bodies).
#[inline(never)]
pub fn stamp(s: &Store, j: usize, body: usize, sys: &mut Sys) {
    let a = at!(s.j_a, j) as usize;
    let b = at!(s.j_b, j) as usize;
    let is_a = body == a;
    let flags = at!(s.j_flags, j);
    let st_lin_inf = at!(s.j_stiff, 2 * j) == f64::INFINITY;
    let st_ang_inf = at!(s.j_stiff, 2 * j + 1) == f64::INFINITY;
    let lhs = &mut sys.lhs;
    let rhs = &mut sys.rhs;

    if flags & F_POINT != 0 && at!(s.j_point_k, j) > 0.0 {
        let k = at!(s.j_point_k, j);
        let (raw, rbw, mut c) = point_c_cached(s, j, a, b);
        if st_lin_inf {
            for i in 0..3 {
                c[i] = c[i] - at!(s.j_point_c0, 3 * j + i);
            }
        }
        let mut f = [0.0; 3];
        for i in 0..3 {
            f[i] = k * c[i] + at!(s.j_point_lambda, 3 * j + i);
        }
        let r = if is_a { raw } else { rbw };
        let (rx, ry, rz) = (r[0], r[1], r[2]);
        let rr = rx * rx + ry * ry + rz * rz;
        lhs[0] = lhs[0] + k;
        lhs[7] = lhs[7] + k;
        lhs[14] = lhs[14] + k;
        lhs[21] = lhs[21] + k * (rr - rx * rx);
        lhs[27] = lhs[27] - k * ry * rx;
        lhs[28] = lhs[28] + k * (rr - ry * ry);
        lhs[33] = lhs[33] - k * rz * rx;
        lhs[34] = lhs[34] - k * rz * ry;
        lhs[35] = lhs[35] + k * (rr - rz * rz);
        lhs[19] = lhs[19] - k * rz;
        lhs[20] = lhs[20] + k * ry;
        lhs[24] = lhs[24] + k * rz;
        lhs[26] = lhs[26] - k * rx;
        lhs[30] = lhs[30] - k * ry;
        lhs[31] = lhs[31] + k * rx;
        let t = cross(r, f);
        let sg = if is_a { 1.0 } else { -1.0 };
        for i in 0..3 {
            rhs[i] = rhs[i] + sg * f[i];
            rhs[3 + i] = rhs[3 + i] + sg * t[i];
        }
        // geometric stiffness of the rotated anchor, diagonalised (AVBD Sec. 3.5)
        let (gx, gy, gz) = (sg * rx, sg * ry, sg * rz);
        let gf = gx * f[0] + gy * f[1] + gz * f[2];
        let n = geometric_norms(gx, gy, gz, f, gf);
        for jj in 0..3 {
            lhs[(3 + jj) * 7] = lhs[(3 + jj) * 7] + n[jj];
        }
    }

    if flags & F_LOCK != 0 && at!(s.j_lock_k, j) > 0.0 {
        let k = at!(s.j_lock_k, j);
        let c3 = v3(&s.j_lockc, j);
        let sg = if is_a { 1.0 } else { -1.0 };
        for i in 0..3 {
            let c = if st_ang_inf { c3[i] - at!(s.j_lock_c0, 3 * j + i) } else { c3[i] };
            let f = k * c + at!(s.j_lock_lambda, 3 * j + i);
            lhs[(3 + i) * 7] = lhs[(3 + i) * 7] + k;
            rhs[3 + i] = rhs[3 + i] + sg * f;
        }
    }

    if flags & F_AXIS != 0 {
        let v = v3(&s.j_v, j);
        for k in 0..2 {
            let kk = at!(s.j_axis_k, 2 * j + k);
            if kk <= 0.0 {
                continue;
            }
            let u = v3(&s.j_u, 2 * j + k);
            let c = dot(u, v) - (if st_ang_inf { at!(s.j_axis_c0, 2 * j + k) } else { 0.0 });
            let f = kk * c + at!(s.j_axis_lambda, 2 * j + k);
            let t = cross(u, v);
            let sg = if is_a { 1.0 } else { -1.0 };
            stamp_angular_row(sys, sg * t[0], sg * t[1], sg * t[2], kk, f);
        }
    }

    if flags & F_SLIDE != 0 {
        let (ra, rb, d) = separation_cached(s, j, a, b);
        for k in 0..2 {
            let kk = at!(s.j_slide_k, 2 * j + k);
            if kk <= 0.0 {
                continue;
            }
            let u = v3(&s.j_u, 2 * j + k);
            let c = dot(u, d) - (if st_lin_inf { at!(s.j_slide_c0, 2 * j + k) } else { 0.0 });
            let f = kk * c + at!(s.j_slide_lambda, 2 * j + k);
            let jac = slide_jacobian(u, ra, rb, d, is_a);
            stamp_row(sys, &jac, kk, f);
        }
    }

    let side = at!(s.j_limit, 4 * j);
    let lk = at!(s.j_limit, 4 * j + 1);
    if side != 0.0 && lk > 0.0 {
        let f = js_min(lk * limit_c(s, j, a, b) + at!(s.j_limit, 4 * j + 2), 0.0);
        let o = 6 * j;
        let jac: [f64; 6] = if is_a {
            s.j_limit_ja[o..o + 6].try_into().unwrap()
        } else {
            s.j_limit_jb[o..o + 6].try_into().unwrap()
        };
        stamp_row(sys, &jac, lk, f);
    }
}

/// Multiplier and penalty update (AVBD Eq. 11 and 16), once per iteration.
#[inline(never)]
pub fn dual(s: &mut Store, p: &Params, j: usize) {
    let a = at!(s.j_a, j) as usize;
    let b = at!(s.j_b, j) as usize;
    let flags = at!(s.j_flags, j);
    let (k0_lin, k0_ang) = (at!(s.j_k0, 2 * j), at!(s.j_k0, 2 * j + 1));
    let (st_lin, st_ang) = (at!(s.j_stiff, 2 * j), at!(s.j_stiff, 2 * j + 1));
    let st_lin_inf = st_lin == f64::INFINITY;
    let st_ang_inf = st_ang == f64::INFINITY;

    if flags & F_POINT != 0 && at!(s.j_point_k, j) > 0.0 {
        let (_, _, c3) = point_c_cached(s, j, a, b);
        let mut err = 0.0;
        for i in 0..3 {
            let c = if st_lin_inf { c3[i] - at!(s.j_point_c0, 3 * j + i) } else { c3[i] };
            if st_lin_inf {
                at_mut!(s.j_point_lambda, 3 * j + i) = at!(s.j_point_k, j) * c + at!(s.j_point_lambda, 3 * j + i);
            }
            err = js_max(err, c.abs());
        }
        at_mut!(s.j_point_k, j) = js_min(p.ramp(at!(s.j_point_k, j), err, p.beta_lin, k0_lin), st_lin);
    }
    if flags & F_LOCK != 0 && at!(s.j_lock_k, j) > 0.0 {
        let c3 = v3(&s.j_lockc, j);
        let mut err = 0.0;
        for i in 0..3 {
            let c = if st_ang_inf { c3[i] - at!(s.j_lock_c0, 3 * j + i) } else { c3[i] };
            if st_ang_inf {
                at_mut!(s.j_lock_lambda, 3 * j + i) = at!(s.j_lock_k, j) * c + at!(s.j_lock_lambda, 3 * j + i);
            }
            err = js_max(err, c.abs());
        }
        at_mut!(s.j_lock_k, j) = js_min(p.ramp(at!(s.j_lock_k, j), err, p.beta_ang, k0_ang), st_ang);
    }
    if flags & F_AXIS != 0 {
        let v = v3(&s.j_v, j);
        for k in 0..2 {
            let kk = at!(s.j_axis_k, 2 * j + k);
            if kk <= 0.0 {
                continue;
            }
            let u = v3(&s.j_u, 2 * j + k);
            let c = dot(u, v) - (if st_ang_inf { at!(s.j_axis_c0, 2 * j + k) } else { 0.0 });
            if st_ang_inf {
                at_mut!(s.j_axis_lambda, 2 * j + k) = kk * c + at!(s.j_axis_lambda, 2 * j + k);
            }
            at_mut!(s.j_axis_k, 2 * j + k) = js_min(p.ramp(kk, c.abs(), p.beta_ang, k0_ang), st_ang);
        }
    }
    if flags & F_SLIDE != 0 {
        let (_, _, d) = separation_cached(s, j, a, b);
        for k in 0..2 {
            let kk = at!(s.j_slide_k, 2 * j + k);
            if kk <= 0.0 {
                continue;
            }
            let u = v3(&s.j_u, 2 * j + k);
            let c = dot(u, d) - (if st_lin_inf { at!(s.j_slide_c0, 2 * j + k) } else { 0.0 });
            if st_lin_inf {
                at_mut!(s.j_slide_lambda, 2 * j + k) = kk * c + at!(s.j_slide_lambda, 2 * j + k);
            }
            at_mut!(s.j_slide_k, 2 * j + k) = js_min(p.ramp(kk, c.abs(), p.beta_lin, k0_lin), st_lin);
        }
    }
    let side = at!(s.j_limit, 4 * j);
    let lk = at!(s.j_limit, 4 * j + 1);
    if side != 0.0 && lk > 0.0 {
        let c = limit_c(s, j, a, b);
        let f = js_min(lk * c + at!(s.j_limit, 4 * j + 2), 0.0);
        at_mut!(s.j_limit, 4 * j + 2) = f;
        if f < 0.0 {
            let hinge = at!(s.j_kind, j) == KIND_HINGE;
            let beta = if hinge { p.beta_ang } else { p.beta_lin };
            let k0 = if hinge { k0_ang } else { k0_lin };
            at_mut!(s.j_limit, 4 * j + 1) = p.ramp(lk, c.abs(), beta, k0);
        }
    }
}

/// Read the force connection j carried this substep and test its rating; true when it breaks now.
#[inline(never)]
pub fn measure(s: &mut Store, j: usize) -> bool {
    if at!(s.j_broken, j) != 0 {
        at_mut!(s.c_out, 3 * j) = 0.0;
        at_mut!(s.c_out, 3 * j + 1) = 0.0;
        return false;
    }
    let qa = q4(&s.b_rot, at!(s.j_a, j) as usize);
    // linear force on B: the point multiplier, or −Σ λ_k perp_k for a slider
    let f = if at!(s.j_flags, j) & F_POINT != 0 {
        v3(&s.j_point_lambda, j)
    } else {
        let mut out = [0.0; 3];
        if at!(s.j_flags, j) & F_SLIDE != 0 {
            for k in 0..2 {
                let u = rotate(qa, perp(s, j, k));
                let l = at!(s.j_slide_lambda, 2 * j + k);
                for i in 0..3 {
                    out[i] = out[i] - l * u[i];
                }
            }
        }
        out
    };
    let ax = rotate(qa, v3(&s.c_axis, j));
    let axial = f[0] * ax[0] + f[1] * ax[1] + f[2] * ax[2];
    let sx = f[0] - axial * ax[0];
    let sy = f[1] - axial * ax[1];
    let sz = f[2] - axial * ax[2];
    let shear = (sx * sx + sy * sy + sz * sz).sqrt();
    at_mut!(s.c_out, 3 * j) = axial;
    at_mut!(s.c_out, 3 * j + 1) = shear;
    let u = match at!(s.c_rating, j) {
        // EN 1993-1-8 Table 3.4 interaction, as boltUtilisation in @botfoundry/model
        RATING_BOLT => shear.abs() / at!(s.c_rating_v, 2 * j) + js_max(0.0, axial) / (1.4 * at!(s.c_rating_v, 2 * j + 1)),
        RATING_WELD => (axial * axial + shear * shear).sqrt() / at!(s.c_rating_v, 2 * j),
        _ => 0.0,
    };
    at_mut!(s.c_out, 3 * j + 2) = u;
    if u >= 1.0 {
        at_mut!(s.j_broken, j) = 1;
        return true;
    }
    false
}
