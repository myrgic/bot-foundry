import type { ManifoldToplevel } from 'manifold-3d';
import {
  MATERIALS, boltRating, weldRating,
  type Actuator, type ComponentDef, type Connection, type Design, type Id, type Loop, type MaterialId,
  type PartStructure, type PhysPart, type PhysicalModel, type PowerSystem, type Profile, type V2, type V3, type Wire,
} from '@botfoundry/model';
import type { BodyEntry, ComponentEntry, Run } from './kernel.ts';
import type { CatalogMap, FirstExtrude } from './body.ts';
import { convexHull } from './hull.ts';
import { combineMass } from './mass.ts';
import { primitiveVertices } from './primitives.ts';
import { signedArea } from './profile.ts';
import { Scope } from './scope.ts';
import { applyDir, applyPoint, asMat4, fromTransform, invertRigid, len3, mul4, planeFrame, sub3, translation } from './vec.ts';
import { wireResistance } from './wiring.ts';

// Compile a Design into the PhysicalModel the solver consumes.

/** Parent materials ranked by how a weld behaves. Aluminium loses strength in the heat-affected zone. */
const ALUMINIUM = new Set<MaterialId>(['al6061', 'al7075']);
const HAZ_KNOCKDOWN = 0.6;
/** Hardened steels are welded with mild-steel filler; ER70S-6 minimum tensile strength is 480 MPa (AWS A5.18). */
const FILLER_ULTIMATE: Partial<Record<MaterialId, number>> = { ar500: 480e6, s7: 480e6 };

/**
 * A bolt connection's axis points from part b toward part a, the head side, so a bolt in tension
 * reads positive axial force. The authored sign is arbitrary, so flip it when part a's centre of
 * mass sits on the negative side of part b's along the axis. With the centres level along the
 * axis there is nothing to go by and the axis is kept as given.
 */
function boltAxis(axis: V3, comA: V3, comB: V3): V3 {
  const side = (comA[0] - comB[0]) * axis[0] + (comA[1] - comB[1]) * axis[1] + (comA[2] - comB[2]) * axis[2];
  return side < 0 ? [0 - axis[0], 0 - axis[1], 0 - axis[2]] : axis;
}

/** What a catalog part is made of, for contact and failure purposes. The catalog carries no material. */
function componentMaterial(def: ComponentDef): MaterialId {
  switch (def.category) {
    case 'wheel': return 'rubber';
    case 'battery':
    case 'esc':
    case 'controller':
    case 'sensor': return 'pc';
    default: return 'al6061';
  }
}

function polylineLength(path: readonly V3[]): number {
  let l = 0;
  for (let i = 1; i < path.length; i++) l += len3(sub3(path[i] as V3, path[i - 1] as V3));
  return l;
}

function ringToLoop(ring: readonly V2[]): Loop {
  const start = ring[0] as V2;
  return { start, segments: [...ring.slice(1).map((to) => ({ to })), { to: start }] };
}

/** Mid-thickness slice of a plate in its sketch plane, as profiles in sketch coordinates. */
export function plateOutline(entry: BodyEntry, fe: FirstExtrude): Profile[] {
  if (!entry.manifold) return [];
  const scope = new Scope();
  try {
    const frame = planeFrame(fe.plane);
    const local = scope.track(entry.manifold.transform(asMat4(invertRigid(frame.matrix))));
    const section = scope.track(scope.track(local.slice(fe.symmetric ? 0 : fe.distance / 2)).simplify(1e-6));
    const profiles: Profile[] = [];
    for (const piece of section.decompose()) {
      scope.track(piece);
      const rings = piece.toPolygons().map((poly) => poly.map((p) => [p[0], p[1]] as V2));
      // CrossSection output is CCW for outer boundaries and CW for holes
      const outer = rings.filter((r) => signedArea(r) > 0).sort((a, b) => signedArea(b) - signedArea(a))[0];
      if (!outer) continue;
      profiles.push({ outer: ringToLoop(outer), holes: rings.filter((r) => r !== outer).map(ringToLoop) });
    }
    return profiles;
  } finally {
    scope.dispose();
  }
}

export function compileModel(_w: ManifoldToplevel, design: Design, catalog: CatalogMap, run: Run): PhysicalModel {
  const parts: PhysPart[] = [];
  const materialOf = new Map<Id, MaterialId>();

  for (const body of design.bodies) {
    const entry = run.bodies.get(body.id) as BodyEntry;
    if (!entry.manifold) continue; // nothing to simulate
    let structure: PartStructure;
    switch (body.structure.kind) {
      case 'plate':
        structure = {
          kind: 'plate',
          thickness: body.structure.thickness,
          plane: entry.firstExtrude?.plane ?? { origin: [0, 0, 0], normal: [0, 0, 1], xAxis: [1, 0, 0] },
          outline: entry.firstExtrude ? plateOutline(entry, entry.firstExtrude) : [],
        };
        break;
      case 'billet': structure = { kind: 'billet' }; break;
      case 'tube': structure = { kind: 'tube', wall: body.structure.wall }; break;
    }
    const pts: V3[] = [];
    for (let i = 0; i < entry.mesh.positions.length; i += 3) {
      pts.push([entry.mesh.positions[i] as number, entry.mesh.positions[i + 1] as number, entry.mesh.positions[i + 2] as number]);
    }
    parts.push({
      id: body.id,
      source: { kind: 'body', id: body.id },
      material: body.material,
      mass: entry.mass,
      // one hull per body for now; concave bodies (plates with large cut-outs, U-channels) are over-approximated
      collision: [convexHull(pts)],
      mesh: entry.mesh,
      structure,
    });
    materialOf.set(body.id, body.material);
  }

  for (const inst of design.components) {
    const comp = run.components.get(inst.id) as ComponentEntry;
    const material = componentMaterial(comp.def);
    parts.push({
      id: inst.id,
      source: { kind: 'component', id: inst.id },
      material,
      mass: comp.mass,
      // catalog geometry is already a union of convex primitives, so keep one exact hull per primitive
      collision: comp.def.geometry.map((p) => convexHull(primitiveVertices(p, comp.matrix))),
      mesh: comp.mesh,
      structure: { kind: 'rigid' },
    });
    materialOf.set(inst.id, material);
  }
  const known = new Set(parts.map((p) => p.id));
  const comOf = new Map(parts.map((p) => [p.id, p.mass.com] as const));

  // ---- connections ----
  const connections: Connection[] = [];
  for (const f of design.fasteners) {
    if (!known.has(f.joins[0]) || !known.has(f.joins[1])) continue;
    connections.push({
      kind: 'bolt', id: f.id, a: f.joins[0], b: f.joins[1], position: f.position, axis: boltAxis(f.axis, comOf.get(f.joins[0]) as V3, comOf.get(f.joins[1]) as V3), rating: boltRating(f.bolt),
    });
  }
  // a component bolted through its mount pattern gets one bolt per mount hole unless the design lists fasteners for it
  for (const inst of design.components) {
    if (!inst.mountedTo || !known.has(inst.mountedTo)) continue;
    const explicit = design.fasteners.some(
      (f) => (f.joins[0] === inst.id && f.joins[1] === inst.mountedTo) || (f.joins[1] === inst.id && f.joins[0] === inst.mountedTo),
    );
    if (explicit) continue;
    const def = (run.components.get(inst.id) as ComponentEntry).def;
    const pose = fromTransform(inst.pose);
    def.mounts.forEach((mount, mi) => {
      const face = planeFrame(mount.face);
      mount.holes.forEach((hole, hi) => {
        connections.push({
          kind: 'bolt',
          id: `${inst.id}.mount${mi}.${hi}`,
          a: inst.id,
          b: inst.mountedTo as Id,
          position: applyPoint(mul4(pose, mul4(face.matrix, translation([hole.at[0], hole.at[1], 0]))), [0, 0, 0]),
          axis: boltAxis(applyDir(pose, face.normal), comOf.get(inst.id) as V3, comOf.get(inst.mountedTo as Id) as V3),
          rating: boltRating(hole.bolt),
        });
      });
    });
  }
  for (const weld of design.welds) {
    if (!known.has(weld.joins[0]) || !known.has(weld.joins[1])) continue;
    const parents = weld.joins.map((id) => materialOf.get(id) as MaterialId);
    const ultimate = Math.min(...parents.map((m) => Math.min(MATERIALS[m].ultimate, FILLER_ULTIMATE[m] ?? Infinity)));
    const knockdown = parents.some((m) => ALUMINIUM.has(m)) ? HAZ_KNOCKDOWN : 1;
    connections.push({
      kind: 'weld', id: weld.id, a: weld.joins[0], b: weld.joins[1], path: weld.path,
      rating: weldRating(weld.throat, polylineLength(weld.path), ultimate, 0.9, knockdown),
    });
  }
  for (const j of design.joints) {
    if (!known.has(j.a) || !known.has(j.b)) continue;
    const c: Connection = { kind: 'joint', id: j.id, joint: j.kind, a: j.a, b: j.b, anchor: j.anchor };
    if (j.axis) c.axis = j.axis;
    if (j.limits) c.limits = j.limits;
    connections.push(c);
  }

  // ---- wiring: supply paths and drive chains ----
  const defOf = (id: Id): ComponentDef | undefined => {
    const inst = design.components.find((c) => c.id === id);
    return inst ? catalog.get(inst.def) : undefined;
  };
  const portKind = (comp: Id, port: string) => defOf(comp)?.ports.find((p) => p.name === port)?.kind;
  const supplyConductance = new Map<string, number>(); // `${esc}|${pack}` -> 1/R of the parallel supply wires
  const driveEsc = new Map<Id, Id>(); // motor component -> esc component
  for (const wire of design.wiring as Wire[]) {
    const ends = [wire.from, wire.to];
    const cat = ends.map((e) => defOf(e.component)?.category);
    const pack = ends.findIndex((_, i) => cat[i] === 'battery');
    const esc = ends.findIndex((_, i) => cat[i] === 'esc');
    if (pack >= 0 && esc >= 0 && pack !== esc) {
      const escEnd = ends[esc] as Wire['from'];
      if (portKind(escEnd.component, escEnd.port) === 'power-in') {
        const key = `${escEnd.component}|${(ends[pack] as Wire['from']).component}`;
        supplyConductance.set(key, (supplyConductance.get(key) ?? 0) + 1 / wireResistance(wire.gauge));
      }
    }
    const motor = ends.findIndex((_, i) => defOf((ends[i] as Wire['from']).component)?.motor !== undefined);
    if (motor >= 0 && esc >= 0) driveEsc.set((ends[motor] as Wire['from']).component, (ends[esc] as Wire['from']).component);
  }

  const power: PowerSystem = { packs: [], escs: [], wiringResistance: {} };
  for (const inst of design.components) {
    const def = (run.components.get(inst.id) as ComponentEntry).def;
    if (def.battery) power.packs.push({ id: inst.id, component: inst.id, spec: def.battery });
  }
  for (const inst of design.components) {
    const def = (run.components.get(inst.id) as ComponentEntry).def;
    if (!def.esc) continue;
    const pack = power.packs.find((p) => supplyConductance.has(`${inst.id}|${p.component}`));
    if (!pack) continue; // not wired to any pack: it cannot be powered
    power.escs.push({ id: inst.id, component: inst.id, spec: def.esc, pack: pack.id });
    power.wiringResistance[inst.id] = 1 / (supplyConductance.get(`${inst.id}|${pack.component}`) as number);
  }

  const actuators: Actuator[] = [];
  for (const j of design.joints) {
    if (!j.drive || !known.has(j.a) || !known.has(j.b)) continue;
    const motor = defOf(j.drive)?.motor;
    const esc = driveEsc.get(j.drive);
    if (!motor || !esc) continue; // an undriven joint stays passive
    actuators.push({ id: `${j.id}.drive`, component: j.drive, joint: j.id, motor, esc });
  }

  return {
    schema: 'botfoundry/physical@0',
    name: design.name,
    parts,
    connections,
    actuators,
    power,
    controls: design.controls,
    mass: combineMass(parts.map((p) => p.mass)),
  };
}
