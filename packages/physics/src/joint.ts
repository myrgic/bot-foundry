import type { Body, BodySystem, Constraint } from './body.ts';
import {
  cross3,
  datan2,
  dot3,
  quat,
  quatConj,
  quatDiff,
  quatMul,
  rotate,
  rotateInv,
  sub3,
  tangentBasis,
  vec3,
  type Vec3,
} from './math.ts';
import { rampPenalty, warmPenalty, type SolverParams } from './params.ts';
import { stampAngularRow, stampRow } from './system.ts';

// Hard joints for AVBD, after the reference Joint in avbd-demo3d (Giles, MIT licence;
// Giles, Diaz, Yuksel, "Augmented Vertex Block Descent", SIGGRAPH 2025), extended from its
// ball-socket + angle lock to hinge, slider and ball joints with optional limits.
//
// Sign convention, shared with contacts: each row has a constraint value C, Jacobians J_A and J_B,
// and a multiplier λ in newtons (or N·m for angular rows): the AVBD energy is M/2h²‖x − y‖² plus
// K/2·C² + λC per row, so the row's force F = K·C + λ is already a force, and the generalised force
// the row applies to body X is −J_Xᵀ·λ. For the point block C = p_A − p_B (anchor on A minus anchor
// on B), so the force on B is +λ and the force on A is −λ.

export type JointKind = 'fixed' | 'hinge' | 'slider' | 'ball';

export interface JointInit {
  kind: JointKind;
  a: Body;
  b: Body;
  /** world anchor at creation */
  anchor: Vec3;
  /** world axis at creation (unit); required for hinge and slider */
  axis?: Vec3;
  /** rad for hinges, m for sliders, measured from the pose at creation */
  limits?: readonly [number, number];
  params: SolverParams;
}

const rAw = vec3(), rBw = vec3(), C3 = vec3(), F3 = vec3(), T3 = vec3(), U3 = vec3(), V3 = vec3(), D3 = vec3();
const tmpQ = quat();
const J6 = new Float64Array(6);
const dqA = new Float64Array(6), dqB = new Float64Array(6);

/** angular limit rows become active this close to the stop, rad */
const ANGLE_MARGIN = 0.02;

export class Joint implements Constraint {
  readonly kind: JointKind;
  readonly a: Body;
  readonly b: Body;
  readonly params: SolverParams;
  broken = false;

  /** anchor in each body's frame */
  readonly rA = vec3();
  readonly rB = vec3();
  /** joint axis in A's frame and B's frame, and two perpendiculars in A's frame */
  readonly axisA = vec3();
  readonly axisB = vec3();
  readonly perp1A = vec3();
  readonly perp2A = vec3();
  /** perp1A carried into B's frame at creation: the hinge angle's zero */
  readonly perp1B = vec3();
  /** rest relative rotation q_A⁻¹ q_B */
  readonly qRel = quat();

  readonly hasPoint: boolean;
  readonly hasLock: boolean;
  readonly hasAxisRows: boolean;
  readonly hasSlideRows: boolean;
  readonly limits: readonly [number, number] | null;

  /** base stiffness for penalty scaling: lighter body's m/h² and I/h² */
  readonly k0Lin: number;
  readonly k0Ang: number;
  /** finite values make the blocks soft springs (no multiplier); Infinity is a hard constraint */
  stiffnessLin = Infinity;
  stiffnessAng = Infinity;

  // point block: 3 rows on world axes, one scalar penalty
  pointK = 0;
  readonly pointLambda = vec3();
  readonly pointC0 = vec3();
  // angle lock: 3 rows on world axes
  lockK = 0;
  readonly lockLambda = vec3();
  readonly lockC0 = vec3();
  // hinge axis alignment: 2 rows, perp_k(A) · axis(B) = 0
  readonly axisK = new Float64Array(2);
  readonly axisLambda = new Float64Array(2);
  readonly axisC0 = new Float64Array(2);
  // slider: 2 rows, perp_k(A) · (p_B − p_A) = 0
  readonly slideK = new Float64Array(2);
  readonly slideLambda = new Float64Array(2);
  readonly slideC0 = new Float64Array(2);
  // limit: one inequality row C ≥ 0, linearised at the start of the substep like a contact
  limitSide = 0;
  limitK = 0;
  limitLambda = 0;
  limitC0 = 0;
  readonly limitJA = new Float64Array(6);
  readonly limitJB = new Float64Array(6);

  constructor(init: JointInit) {
    const { a, b, kind, params } = init;
    this.kind = kind;
    this.a = a;
    this.b = b;
    this.params = params;
    this.hasPoint = kind !== 'slider';
    this.hasLock = kind === 'fixed' || kind === 'slider';
    this.hasAxisRows = kind === 'hinge';
    this.hasSlideRows = kind === 'slider';
    this.limits = (kind === 'hinge' || kind === 'slider') && init.limits ? init.limits : null;

    rotateInv(this.rA, a.rot, sub3(T3, init.anchor, a.pos));
    rotateInv(this.rB, b.rot, sub3(T3, init.anchor, b.pos));
    const axis = init.axis ?? vec3(0, 1, 0);
    rotateInv(this.axisA, a.rot, axis);
    rotateInv(this.axisB, b.rot, axis);
    tangentBasis(this.axisA, this.perp1A, this.perp2A);
    rotateInv(this.perp1B, b.rot, rotate(T3, a.rot, this.perp1A));
    quatMul(this.qRel, quatConj(tmpQ, a.rot), b.rot);

    const h2 = params.dt * params.dt;
    const m = Math.min(a.isStatic ? Infinity : a.mass, b.isStatic ? Infinity : b.mass);
    const i = Math.min(a.isStatic ? Infinity : a.inertiaScale, b.isStatic ? Infinity : b.inertiaScale);
    this.k0Lin = Number.isFinite(m) ? m / h2 : 0;
    this.k0Ang = Number.isFinite(i) ? i / h2 : 0;
  }

  prepare(): boolean {
    if (this.broken) return false;
    const p = this.params;
    // The paper warm-starts λ ← α·γ·λ (Eq. 19). For a joint under a steady load that leaves a
    // persistent error which re-grows the lost α share every substep, and the multiplier read out
    // as the connection's force is then biased (0.2 % on a hanging 100 kg plate). Decaying by γ alone
    // keeps the readout within a few ppm of the static load.
    // The angle lock keeps the paper's α·γ. Carried over at γ alone, its multiplier feeds a
    // vibration of the plates of a bolted assembly that grows until bolts break (issue #1: in the
    // free-falling two-robot scene, a 1e-6 rad/s nudge to one plate grows past 1 rad/s within 1500
    // substeps; at α·γ it dies away). Its λ is a moment, which the ratings do not read, and the
    // extra sag it costs a 3 kg, 0.5 m cantilever on a fixed joint is 13 µm.
    const decay = p.gamma;
    const lockDecay = p.alpha * p.gamma;
    const a = this.a, b = this.b;
    if (this.hasPoint) {
      this.pointC(this.pointC0);
      for (let i = 0; i < 3; i++) this.pointLambda[i] = this.pointLambda[i]! * decay;
      this.pointK = Math.min(warmPenalty(this.pointK, p, this.k0Lin), this.stiffnessLin);
    }
    if (this.hasLock) {
      this.lockC(this.lockC0);
      for (let i = 0; i < 3; i++) this.lockLambda[i] = this.lockLambda[i]! * lockDecay;
      this.lockK = Math.min(warmPenalty(this.lockK, p, this.k0Ang), this.stiffnessAng);
    }
    if (this.hasAxisRows) {
      rotate(V3, b.rot, this.axisB);
      for (let k = 0; k < 2; k++) {
        rotate(U3, a.rot, k === 0 ? this.perp1A : this.perp2A);
        this.axisC0[k] = dot3(U3, V3);
        this.axisLambda[k] = this.axisLambda[k]! * decay;
        this.axisK[k] = Math.min(warmPenalty(this.axisK[k]!, p, this.k0Ang), this.stiffnessAng);
      }
    }
    if (this.hasSlideRows) {
      this.separation(D3);
      for (let k = 0; k < 2; k++) {
        rotate(U3, a.rot, k === 0 ? this.perp1A : this.perp2A);
        this.slideC0[k] = dot3(U3, D3);
        this.slideLambda[k] = this.slideLambda[k]! * decay;
        this.slideK[k] = Math.min(warmPenalty(this.slideK[k]!, p, this.k0Lin), this.stiffnessLin);
      }
    }
    if (this.limits) this.prepareLimit();
    return true;
  }

  stamp(body: Body, sys: BodySystem): void {
    const isA = body === this.a;
    const a = this.a, b = this.b;
    const alpha = this.params.alpha;
    const lhs = sys.lhs, rhs = sys.rhs;

    if (this.hasPoint && this.pointK > 0) {
      const k = this.pointK;
      this.pointC(C3);
      if (this.stiffnessLin === Infinity) for (let i = 0; i < 3; i++) C3[i] = C3[i]! - alpha * this.pointC0[i]!;
      for (let i = 0; i < 3; i++) F3[i] = k * C3[i]! + this.pointLambda[i]!;
      // J_lin = ±I, J_ang = −skew(r_A) for A and +skew(r_B) for B
      const r = isA ? rAw : rBw;
      const rx = r[0]!, ry = r[1]!, rz = r[2]!;
      const rr = rx * rx + ry * ry + rz * rz;
      lhs[0] = lhs[0]! + k;
      lhs[7] = lhs[7]! + k;
      lhs[14] = lhs[14]! + k;
      // angular block k (|r|² I − r rᵀ)
      lhs[21] = lhs[21]! + k * (rr - rx * rx);
      lhs[27] = lhs[27]! - k * ry * rx;
      lhs[28] = lhs[28]! + k * (rr - ry * ry);
      lhs[33] = lhs[33]! - k * rz * rx;
      lhs[34] = lhs[34]! - k * rz * ry;
      lhs[35] = lhs[35]! + k * (rr - rz * rz);
      // cross block J_angᵀ K J_lin = k skew(r) for both bodies
      lhs[19] = lhs[19]! - k * rz;
      lhs[20] = lhs[20]! + k * ry;
      lhs[24] = lhs[24]! + k * rz;
      lhs[26] = lhs[26]! - k * rx;
      lhs[30] = lhs[30]! - k * ry;
      lhs[31] = lhs[31]! + k * rx;
      cross3(T3, r, F3);
      const s = isA ? 1 : -1;
      for (let i = 0; i < 3; i++) {
        rhs[i] = rhs[i]! + s * F3[i]!;
        rhs[3 + i] = rhs[3 + i]! + s * T3[i]!;
      }
      // Geometric stiffness of the rotated anchor, diagonalised to stay positive (AVBD Sec. 3.5,
      // as in the reference): H = −(g·F) I + g Fᵀ with g = r_A for A and −r_B for B.
      const gx = s * rx, gy = s * ry, gz = s * rz;
      const gf = gx * F3[0]! + gy * F3[1]! + gz * F3[2]!;
      for (let j = 0; j < 3; j++) {
        const fj = F3[j]!;
        const cx = gx * fj - (j === 0 ? gf : 0);
        const cy = gy * fj - (j === 1 ? gf : 0);
        const cz = gz * fj - (j === 2 ? gf : 0);
        lhs[(3 + j) * 7] = lhs[(3 + j) * 7]! + Math.sqrt(cx * cx + cy * cy + cz * cz);
      }
    }

    if (this.hasLock && this.lockK > 0) {
      const k = this.lockK;
      this.lockC(C3);
      const s = isA ? 1 : -1;
      for (let i = 0; i < 3; i++) {
        const c = this.stiffnessAng === Infinity ? C3[i]! - alpha * this.lockC0[i]! : C3[i]!;
        const f = k * c + this.lockLambda[i]!;
        lhs[(3 + i) * 7] = lhs[(3 + i) * 7]! + k;
        rhs[3 + i] = rhs[3 + i]! + s * f;
      }
    }

    if (this.hasAxisRows) {
      rotate(V3, b.rot, this.axisB);
      for (let k = 0; k < 2; k++) {
        const kk = this.axisK[k]!;
        if (kk <= 0) continue;
        rotate(U3, a.rot, k === 0 ? this.perp1A : this.perp2A);
        const c = dot3(U3, V3) - (this.stiffnessAng === Infinity ? alpha * this.axisC0[k]! : 0);
        const f = kk * c + this.axisLambda[k]!;
        cross3(T3, U3, V3);
        const s = isA ? 1 : -1;
        stampAngularRow(sys, s * T3[0]!, s * T3[1]!, s * T3[2]!, kk, f);
      }
    }

    if (this.hasSlideRows) {
      this.separation(D3);
      for (let k = 0; k < 2; k++) {
        const kk = this.slideK[k]!;
        if (kk <= 0) continue;
        rotate(U3, a.rot, k === 0 ? this.perp1A : this.perp2A);
        const c = dot3(U3, D3) - (this.stiffnessLin === Infinity ? alpha * this.slideC0[k]! : 0);
        const f = kk * c + this.slideLambda[k]!;
        this.slideJacobian(U3, isA, J6);
        stampRow(sys, J6, kk, f);
      }
    }

    if (this.limitSide !== 0 && this.limitK > 0) {
      const f = Math.min(this.limitK * this.limitC() + this.limitLambda, 0);
      stampRow(sys, isA ? this.limitJA : this.limitJB, this.limitK, f);
    }
  }

  dual(): void {
    const p = this.params;
    const alpha = p.alpha;
    const a = this.a, b = this.b;
    if (this.hasPoint && this.pointK > 0) {
      this.pointC(C3);
      let err = 0;
      for (let i = 0; i < 3; i++) {
        const c = this.stiffnessLin === Infinity ? C3[i]! - alpha * this.pointC0[i]! : C3[i]!;
        if (this.stiffnessLin === Infinity) this.pointLambda[i] = this.pointK * c + this.pointLambda[i]!;
        err = Math.max(err, Math.abs(c));
      }
      this.pointK = Math.min(rampPenalty(this.pointK, err, p.betaLin, p, this.k0Lin), this.stiffnessLin);
    }
    if (this.hasLock && this.lockK > 0) {
      this.lockC(C3);
      let err = 0;
      for (let i = 0; i < 3; i++) {
        const c = this.stiffnessAng === Infinity ? C3[i]! - alpha * this.lockC0[i]! : C3[i]!;
        if (this.stiffnessAng === Infinity) this.lockLambda[i] = this.lockK * c + this.lockLambda[i]!;
        err = Math.max(err, Math.abs(c));
      }
      // Plastic hinges (a later lane) clamp |lockLambda| to the yield moment here and move qRel to
      // the current relative rotation, as three-avbd's yielding fixed joint does; the penalty of a
      // clamped row is then left un-ramped.
      this.lockK = Math.min(rampPenalty(this.lockK, err, p.betaAng, p, this.k0Ang), this.stiffnessAng);
    }
    if (this.hasAxisRows) {
      rotate(V3, b.rot, this.axisB);
      for (let k = 0; k < 2; k++) {
        const kk = this.axisK[k]!;
        if (kk <= 0) continue;
        rotate(U3, a.rot, k === 0 ? this.perp1A : this.perp2A);
        const c = dot3(U3, V3) - (this.stiffnessAng === Infinity ? alpha * this.axisC0[k]! : 0);
        if (this.stiffnessAng === Infinity) this.axisLambda[k] = kk * c + this.axisLambda[k]!;
        this.axisK[k] = Math.min(rampPenalty(kk, Math.abs(c), p.betaAng, p, this.k0Ang), this.stiffnessAng);
      }
    }
    if (this.hasSlideRows) {
      this.separation(D3);
      for (let k = 0; k < 2; k++) {
        const kk = this.slideK[k]!;
        if (kk <= 0) continue;
        rotate(U3, a.rot, k === 0 ? this.perp1A : this.perp2A);
        const c = dot3(U3, D3) - (this.stiffnessLin === Infinity ? alpha * this.slideC0[k]! : 0);
        if (this.stiffnessLin === Infinity) this.slideLambda[k] = kk * c + this.slideLambda[k]!;
        this.slideK[k] = Math.min(rampPenalty(kk, Math.abs(c), p.betaLin, p, this.k0Lin), this.stiffnessLin);
      }
    }
    if (this.limitSide !== 0 && this.limitK > 0) {
      const c = this.limitC();
      const f = Math.min(this.limitK * c + this.limitLambda, 0);
      this.limitLambda = f;
      if (f < 0) {
        const beta = this.kind === 'hinge' ? p.betaAng : p.betaLin;
        const k0 = this.kind === 'hinge' ? this.k0Ang : this.k0Lin;
        this.limitK = rampPenalty(this.limitK, Math.abs(c), beta, p, k0);
      }
    }
  }

  /** Linear force this joint applies to body B, N, world frame. */
  forceOnB(out: Vec3): Vec3 {
    if (this.hasPoint) {
      out.set(this.pointLambda);
      return out;
    }
    out.fill(0);
    if (this.hasSlideRows) {
      // force on B = −J_B,linᵀ λ = −Σ λ_k perp_k
      for (let k = 0; k < 2; k++) {
        rotate(U3, this.a.rot, k === 0 ? this.perp1A : this.perp2A);
        const l = this.slideLambda[k]!;
        for (let i = 0; i < 3; i++) out[i] = out[i]! - l * U3[i]!;
      }
    }
    return out;
  }

  /** p_A − p_B in world space; leaves the world anchor offsets in rAw, rBw. */
  private pointC(out: Vec3): void {
    const a = this.a, b = this.b;
    rotate(rAw, a.rot, this.rA);
    rotate(rBw, b.rot, this.rB);
    for (let i = 0; i < 3; i++) out[i] = a.pos[i]! + rAw[i]! - b.pos[i]! - rBw[i]!;
  }

  /** world rotation vector from B's orientation to where the rest pose wants it */
  private lockC(out: Vec3): void {
    quatMul(tmpQ, this.a.rot, this.qRel);
    quatDiff(out, tmpQ, this.b.rot);
  }

  /** d = p_B − p_A in world space; leaves rAw, rBw */
  private separation(out: Vec3): void {
    const a = this.a, b = this.b;
    rotate(rAw, a.rot, this.rA);
    rotate(rBw, b.rot, this.rB);
    for (let i = 0; i < 3; i++) out[i] = b.pos[i]! + rBw[i]! - a.pos[i]! - rAw[i]!;
  }

  /** Jacobian of u · (p_B − p_A) for body A or B; needs rAw, rBw and D3 from separation() */
  private slideJacobian(u: Vec3, isA: boolean, out: Float64Array): void {
    if (isA) {
      // δ/δθ_A = u × (r_A + d)
      for (let i = 0; i < 3; i++) T3[i] = rAw[i]! + D3[i]!;
      cross3(C3, u, T3);
      out[0] = -u[0]!;
      out[1] = -u[1]!;
      out[2] = -u[2]!;
    } else {
      cross3(C3, rBw, u);
      out[0] = u[0]!;
      out[1] = u[1]!;
      out[2] = u[2]!;
    }
    out[3] = C3[0]!;
    out[4] = C3[1]!;
    out[5] = C3[2]!;
  }

  /** Hinge angle (rad) or slider travel (m) relative to the pose at creation. */
  position(): number {
    const a = this.a, b = this.b;
    if (this.kind === 'slider') {
      this.separation(D3);
      rotate(U3, a.rot, this.axisA);
      return dot3(U3, D3);
    }
    rotate(U3, a.rot, this.perp1A);
    rotate(V3, b.rot, this.perp1B);
    rotate(T3, a.rot, this.axisA);
    cross3(F3, U3, V3);
    return datan2(dot3(T3, F3), dot3(U3, V3));
  }

  private prepareLimit(): void {
    const p = this.params;
    const [lo, hi] = this.limits!;
    const x = this.position();
    const margin = this.kind === 'hinge' ? ANGLE_MARGIN : p.margin;
    const side = x < lo + margin ? -1 : x > hi - margin ? 1 : 0;
    if (side !== this.limitSide) {
      this.limitLambda = 0;
      this.limitK = 0;
    }
    this.limitSide = side;
    if (side === 0) return;
    const c0 = side < 0 ? x - lo : hi - x;
    // keep a real gap, soften only the correction of a violation (as for contacts)
    this.limitC0 = c0 > 0 ? c0 : (1 - p.alpha) * c0;
    const ja = this.limitJA, jb = this.limitJB;
    const a = this.a;
    if (this.kind === 'hinge') {
      // dθ/dθ_B = axis, dθ/dθ_A = −axis
      rotate(T3, a.rot, this.axisA);
      ja.fill(0);
      jb.fill(0);
      for (let i = 0; i < 3; i++) {
        ja[3 + i] = -side * -T3[i]!;
        jb[3 + i] = -side * T3[i]!;
      }
      this.limitK = warmPenalty(this.limitK, p, this.k0Ang);
    } else {
      this.separation(D3);
      rotate(U3, a.rot, this.axisA);
      this.slideJacobian(U3, true, ja);
      this.slideJacobian(U3, false, jb);
      for (let i = 0; i < 6; i++) {
        ja[i] = -side * ja[i]!;
        jb[i] = -side * jb[i]!;
      }
      this.limitK = warmPenalty(this.limitK, p, this.k0Lin);
    }
    this.limitLambda = this.limitLambda * p.gamma;
  }

  /** linearised limit value: C0 + J_A·Δq_A + J_B·Δq_B */
  private limitC(): number {
    bodyDelta(this.a, dqA);
    bodyDelta(this.b, dqB);
    let c = this.limitC0;
    for (let i = 0; i < 6; i++) c += this.limitJA[i]! * dqA[i]! + this.limitJB[i]! * dqB[i]!;
    return c;
  }
}

/** [x − x₀, θ ⊖ θ₀] since the start of the substep */
export function bodyDelta(body: Body, out: Float64Array): void {
  out[0] = body.pos[0]! - body.pos0[0]!;
  out[1] = body.pos[1]! - body.pos0[1]!;
  out[2] = body.pos[2]! - body.pos0[2]!;
  quatDiff(T3, body.rot, body.rot0);
  out[3] = T3[0]!;
  out[4] = T3[1]!;
  out[5] = T3[2]!;
}
