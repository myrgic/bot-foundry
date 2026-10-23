import * as THREE from 'three';
import type { Loop, Plane, Segment, Sketch, V2 } from '@botfoundry/model';
import { arcGeometry, bulgeThrough } from '@botfoundry/cad';
import { sampleSegment } from '../geom/arc.ts';
import { loopsToProfiles, nearestEdge, profileLoops, rectangleLoop, circleAt } from '../geom/loops.ts';
import { planeToWorld } from '../geom/plane.ts';
import { GRID_COARSE, GRID_FINE, parsePair, parseTyped, snapPoint, type SnapKind } from '../geom/snap.ts';
import { allIds, nextId } from '../state/ops.ts';
import type { Store } from '../state/store.ts';
import { h } from '../dom.ts';
import { clearGroup, loopLine, pointMarkers, polyline } from '../view/lines.ts';
import { COLORS, type Viewport } from '../view/viewport.ts';

type DrawTool = 'line' | 'rect' | 'circle';

export interface SketchHooks {
  onFinish(sketch: Sketch): void;
  onPickAxis(axis: { origin: V2; direction: V2 }): void;
  onPickCancelled(): void;
  /** the sketch view opened: freeze the rest of the workshop */
  onEnter(): void;
  onExit(): void;
  toast(message: string): void;
}

interface Label {
  el: HTMLElement;
  at: V2;
}

const SNAP_PIXELS = 9;

const isTyping = (t: EventTarget | null): boolean => t instanceof HTMLInputElement || t instanceof HTMLTextAreaElement || t instanceof HTMLSelectElement;
const dist = (a: V2, b: V2): number => Math.hypot(a[0] - b[0], a[1] - b[1]);
const mm = (v: number, d = 2): string => `${+(v * 1000).toFixed(d)}`;

/**
 * Sketch mode: an orthographic view onto a plane in which the user draws closed loops. Lines are
 * placed by clicking (grid snap 1 mm, 5 mm with Shift, endpoint snap); holding A while placing a
 * point turns that segment into an arc whose bulge follows the drag; clicking the start point
 * closes the loop. A second mode picks an existing sketch edge to use as a revolve axis.
 */
export class SketchTool {
  active = false;

  private readonly vp: Viewport;
  private readonly store: Store;
  private readonly hooks: SketchHooks;
  private readonly group = new THREE.Group();
  private readonly bar: HTMLElement;
  private readonly status: HTMLElement;
  private readonly toolButtons = new Map<DrawTool, HTMLButtonElement>();
  private readonly labels: Label[] = [];

  private mode: 'draw' | 'pick' = 'draw';
  private plane!: Plane;
  private tool: DrawTool = 'line';
  private loops: Loop[] = [];
  private path: { start: V2; segs: Segment[] } | null = null;
  private anchor: V2 | null = null;
  private cursor: { point: V2; raw: V2; kind: SnapKind } | null = null;
  private arcHeld = false;
  private arcDrag: { to: V2; bulge: number } | null = null;
  private typed = '';
  private pickSketch: Sketch | null = null;
  private hover: { from: V2; to: V2 } | null = null;

  constructor(vp: Viewport, store: Store, hooks: SketchHooks, host: HTMLElement) {
    this.vp = vp;
    this.store = store;
    this.hooks = hooks;
    vp.overlay.add(this.group);
    this.status = h('div', { class: 'sk-status' });
    this.bar = h('div', { class: 'sketchbar', hidden: true });
    host.append(this.bar);
    vp.onFrame(() => this.placeLabels());
    this.onMove = this.onMove.bind(this);
    this.onDown = this.onDown.bind(this);
    this.onUp = this.onUp.bind(this);
    this.onKeyDown = this.onKeyDown.bind(this);
    this.onKeyUp = this.onKeyUp.bind(this);
  }

  // ----- entering and leaving --------------------------------------------------------------

  startDraw(plane: Plane, label: string): void {
    this.begin('draw', plane);
    this.loops = [];
    this.path = null;
    this.anchor = null;
    this.tool = 'line';
    this.renderBar(label);
    this.redraw();
  }

  startPickAxis(sketch: Sketch): void {
    this.begin('pick', sketch.plane);
    this.pickSketch = sketch;
    this.hover = null;
    this.renderBar(sketch.name);
    this.redraw();
  }

  private begin(mode: 'draw' | 'pick', plane: Plane): void {
    if (this.active) this.teardown();
    this.mode = mode;
    this.plane = plane;
    this.active = true;
    this.typed = '';
    this.cursor = null;
    this.arcDrag = null;
    this.arcHeld = false;
    this.hooks.onEnter();
    this.vp.enterSketchView(plane, this.vp.contentRadius());
    this.vp.dom.addEventListener('pointermove', this.onMove);
    this.vp.dom.addEventListener('pointerdown', this.onDown);
    this.vp.dom.addEventListener('pointerup', this.onUp);
    this.vp.dom.addEventListener('contextmenu', preventDefault);
    window.addEventListener('keydown', this.onKeyDown);
    window.addEventListener('keyup', this.onKeyUp);
    this.vp.dom.classList.add('drawing');
    this.bar.hidden = false;
  }

  private teardown(): void {
    this.active = false;
    this.vp.dom.removeEventListener('pointermove', this.onMove);
    this.vp.dom.removeEventListener('pointerdown', this.onDown);
    this.vp.dom.removeEventListener('pointerup', this.onUp);
    this.vp.dom.removeEventListener('contextmenu', preventDefault);
    window.removeEventListener('keydown', this.onKeyDown);
    window.removeEventListener('keyup', this.onKeyUp);
    this.vp.dom.classList.remove('drawing');
    this.bar.hidden = true;
    clearGroup(this.group);
    this.clearLabels();
    this.vp.exitSketchView();
    this.hooks.onExit();
  }

  stop(): void {
    if (!this.active) return;
    const picking = this.mode === 'pick';
    this.teardown();
    if (picking) this.hooks.onPickCancelled();
  }

  // ----- toolbar ---------------------------------------------------------------------------

  private renderBar(label: string): void {
    this.bar.replaceChildren();
    if (this.mode === 'pick') {
      this.bar.append(
        h('span', { class: 'sk-title', text: `Pick an axis on ${label}` }),
        h('button', { type: 'button', text: 'Sketch X axis', on: { click: () => this.axis([0, 0], [1, 0]) } }),
        h('button', { type: 'button', text: 'Sketch Y axis', on: { click: () => this.axis([0, 0], [0, 1]) } }),
        h('button', { type: 'button', class: 'ghost', text: 'Cancel', on: { click: () => this.stop() } }),
        h('div', { class: 'sk-hint', text: 'Click a line of the sketch to revolve about it. Esc cancels.' }),
      );
      return;
    }
    this.toolButtons.clear();
    const tools: [DrawTool, string, string][] = [['line', 'Line / arc', 'L'], ['rect', 'Rectangle', 'R'], ['circle', 'Circle', 'C']];
    this.bar.append(h('span', { class: 'sk-title', text: `Sketch on ${label}` }));
    for (const [id, name, key] of tools) {
      const b = h('button', { type: 'button', title: `${name} (${key})`, text: name, on: { click: () => this.setTool(id) } });
      this.toolButtons.set(id, b);
      this.bar.append(b);
    }
    this.bar.append(
      h('button', { type: 'button', class: 'primary', text: 'Finish sketch', on: { click: () => this.finish() } }),
      h('button', { type: 'button', class: 'ghost', text: 'Cancel', on: { click: () => this.stop() } }),
      this.status,
      h('div', { class: 'sk-hint', text: 'Click to place points, click the start point to close a loop. Hold A while placing a point for an arc (drag the bulge). Shift snaps to 5 mm. Type a length and press Enter. Loops inside a loop become holes.' }),
    );
    this.syncToolButtons();
  }

  private syncToolButtons(): void {
    for (const [id, b] of this.toolButtons) b.classList.toggle('on', id === this.tool);
  }

  private setTool(tool: DrawTool): void {
    if (this.mode !== 'draw') return;
    this.tool = tool;
    this.path = null;
    this.anchor = null;
    this.typed = '';
    this.syncToolButtons();
    this.redraw();
  }

  private axis(origin: V2, direction: V2): void {
    this.teardown();
    this.hooks.onPickAxis({ origin, direction });
  }

  private finish(): void {
    if (this.path) return this.hooks.toast('Close the open loop first (click its start point) or press Esc to drop it.');
    if (this.loops.length === 0) return this.hooks.toast('Draw at least one closed loop.');
    const d = this.store.design;
    const id = nextId('sketch', allIds(d));
    const sketch: Sketch = { id, name: `Sketch ${d.sketches.length + 1}`, plane: this.plane, profiles: loopsToProfiles(this.loops) };
    this.teardown();
    this.hooks.onFinish(sketch);
  }

  // ----- pointer ---------------------------------------------------------------------------

  private endpoints(): V2[] {
    const pts: V2[] = [[0, 0]];
    const addLoop = (l: Loop): void => {
      pts.push(l.start, ...l.segments.map((s) => s.to));
    };
    if (this.mode === 'pick' && this.pickSketch) return profileLoops(this.pickSketch.profiles).flatMap((l) => [l.start, ...l.segments.map((s) => s.to)]);
    this.loops.forEach(addLoop);
    if (this.path) pts.push(this.path.start, ...this.path.segs.map((s) => s.to));
    for (const sk of this.store.design.sketches) {
      if (JSON.stringify(sk.plane) === JSON.stringify(this.plane)) profileLoops(sk.profiles).forEach(addLoop);
    }
    return pts;
  }

  private locate(e: PointerEvent): void {
    const raw = this.vp.pointOnPlane(e, this.plane);
    if (!raw) return;
    const snap = snapPoint(raw, {
      grid: this.mode === 'pick' ? 0 : e.shiftKey ? GRID_COARSE : GRID_FINE,
      endpoints: this.endpoints(),
      radius: SNAP_PIXELS * this.vp.pixelSize(),
    });
    this.cursor = { point: snap.point, raw, kind: snap.kind };
  }

  private onMove(e: PointerEvent): void {
    this.locate(e);
    if (this.mode === 'pick') {
      this.updateHover();
    } else if (this.arcDrag && this.cursor) {
      const p0 = this.lastPoint();
      if (p0) this.arcDrag.bulge = bulgeThrough(p0, this.arcDrag.to, this.cursor.raw);
    }
    this.redraw();
  }

  private updateHover(): void {
    const hit = this.cursor && this.pickSketch ? nearestEdge(profileLoops(this.pickSketch.profiles), this.cursor.raw) : null;
    this.hover = hit && hit.distance < 14 * this.vp.pixelSize() ? { from: hit.from, to: hit.to } : null;
  }

  private onDown(e: PointerEvent): void {
    if (e.button !== 0) return;
    this.locate(e);
    const c = this.cursor;
    if (!c) return;
    if (this.mode === 'pick') {
      this.updateHover();
      if (this.hover) {
        const { from, to } = this.hover;
        const len = dist(from, to);
        this.axis(from, [(to[0] - from[0]) / len, (to[1] - from[1]) / len]);
      }
      return;
    }
    this.typed = '';
    if (this.tool === 'rect') return this.clickTwoPoint(c.point, (a, b) => rectangleLoop(a, b));
    if (this.tool === 'circle') return this.clickTwoPoint(c.point, (a, b) => (dist(a, b) > 1e-6 ? circleAt(a, dist(a, b)) : null));
    if (!this.path) {
      this.path = { start: c.point, segs: [] };
      return this.redraw();
    }
    const last = this.lastPoint() as V2;
    if (dist(c.point, last) < 1e-6) return;
    if (this.arcHeld) {
      try {
        this.vp.dom.setPointerCapture(e.pointerId);
      } catch {
        // synthetic or already-released pointer: the drag still works while the pointer stays over the canvas
      }
      this.arcDrag = { to: c.point, bulge: 0 };
      return this.redraw();
    }
    this.addSegment(c.point, 0);
  }

  private onUp(): void {
    if (!this.arcDrag) return;
    const { to, bulge } = this.arcDrag;
    this.arcDrag = null;
    this.addSegment(to, bulge);
  }

  private clickTwoPoint(p: V2, make: (a: V2, b: V2) => Loop | null): void {
    if (!this.anchor) {
      this.anchor = p;
      return this.redraw();
    }
    const loop = make(this.anchor, p);
    if (!loop) return;
    this.loops.push(loop);
    this.anchor = null;
    this.redraw();
  }

  private lastPoint(): V2 | null {
    if (!this.path) return null;
    return this.path.segs[this.path.segs.length - 1]?.to ?? this.path.start;
  }

  private addSegment(to: V2, bulge: number): void {
    const path = this.path;
    if (!path) return;
    const closing = dist(to, path.start) < 1e-9;
    const canClose = path.segs.length >= 2 || (path.segs.length === 1 && (bulge !== 0 || (path.segs[0]?.bulge ?? 0) !== 0));
    if (closing && !canClose) return this.hooks.toast('A loop needs at least three points, or an arc.');
    path.segs.push(bulge !== 0 ? { to, bulge } : { to });
    if (closing) {
      this.loops.push({ start: path.start, segments: path.segs });
      this.path = null;
    }
    this.redraw();
  }

  // ----- keyboard --------------------------------------------------------------------------

  private onKeyUp(e: KeyboardEvent): void {
    if (e.key.toLowerCase() === 'a') this.arcHeld = false;
    this.redraw();
  }

  private onKeyDown(e: KeyboardEvent): void {
    if (isTyping(e.target) || e.metaKey || e.ctrlKey) return;
    const key = e.key;
    if (this.mode === 'pick') {
      if (key === 'Escape') this.stop();
      return;
    }
    if (key === 'a' || key === 'A') {
      this.arcHeld = true;
      return this.redraw();
    }
    if (!this.typed && !this.path && !this.anchor) {
      const t = { l: 'line', r: 'rect', c: 'circle' }[key.toLowerCase()] as DrawTool | undefined;
      if (t) return this.setTool(t);
    }
    if (/^[0-9.,<x-]$/.test(key) && (this.path || this.anchor)) {
      this.typed += key;
      e.preventDefault();
      return this.redraw();
    }
    if (key === 'Backspace') {
      e.preventDefault();
      if (this.typed) this.typed = this.typed.slice(0, -1);
      else if (this.path) {
        if (this.path.segs.length > 0) this.path.segs.pop();
        else this.path = null;
      } else if (this.anchor) this.anchor = null;
      else this.loops.pop();
      return this.redraw();
    }
    if (key === 'Escape') {
      if (this.typed) this.typed = '';
      else if (this.path || this.anchor) {
        this.path = null;
        this.anchor = null;
      } else return this.stop();
      return this.redraw();
    }
    if (key === 'Enter') {
      e.preventDefault();
      if (this.typed) this.commitTyped();
      else if (this.path && this.path.segs.length >= 2) this.addSegment(this.path.start, 0);
      else if (!this.path && !this.anchor && this.loops.length > 0) this.finish();
    }
  }

  /** Enter with a typed value: place the next point from it, along the cursor direction unless an angle was typed. */
  private commitTyped(): void {
    const text = this.typed;
    const c = this.cursor;
    this.typed = '';
    if (!c) return;
    if (this.tool === 'rect' && this.anchor) {
      const pair = parsePair(text);
      if (!pair) return this.hooks.toast('Type width and height in mm, like 40x20.');
      const sx = c.raw[0] >= this.anchor[0] ? 1 : -1, sy = c.raw[1] >= this.anchor[1] ? 1 : -1;
      this.loops.push(rectangleLoop(this.anchor, [this.anchor[0] + sx * Math.abs(pair[0]), this.anchor[1] + sy * Math.abs(pair[1])]));
      this.anchor = null;
    } else if (this.tool === 'circle' && this.anchor) {
      const t = parseTyped(text);
      if (!t) return this.hooks.toast('Type a radius in mm.');
      this.loops.push(circleAt(this.anchor, Math.abs(t.length)));
      this.anchor = null;
    } else if (this.tool === 'line' && this.path) {
      const t = parseTyped(text);
      const from = this.lastPoint() as V2;
      if (!t) return this.hooks.toast('Type a length in mm, optionally with an angle: 25 or 25,45.');
      const angle = t.angle ?? Math.atan2(c.raw[1] - from[1], c.raw[0] - from[0]);
      const len = Math.abs(t.length);
      const to: V2 = [from[0] + len * Math.cos(angle), from[1] + len * Math.sin(angle)];
      // land exactly on the start point when the typed segment returns to it
      this.addSegment(dist(to, this.path.start) < 1e-7 ? this.path.start : to, 0);
    }
    this.redraw();
  }

  // ----- drawing ---------------------------------------------------------------------------

  private redraw(): void {
    clearGroup(this.group);
    this.clearLabels();
    const plane = this.plane;
    if (this.mode === 'pick' && this.pickSketch) {
      for (const l of profileLoops(this.pickSketch.profiles)) this.group.add(loopLine(plane, l, COLORS.amber, 0.9));
      if (this.hover) this.group.add(polyline(plane, [this.hover.from, this.hover.to], 0xffffff, 1));
      this.vp.requestRender();
      return;
    }
    for (const l of this.loops) this.group.add(loopLine(plane, l, COLORS.amber));
    const vertices: V2[] = this.loops.flatMap((l) => [l.start, ...l.segments.map((s) => s.to)]);
    const c = this.cursor;

    if (this.path) {
      const pts: V2[] = [this.path.start];
      let from = this.path.start;
      for (const s of this.path.segs) {
        pts.push(...sampleSegment(from, s));
        from = s.to;
      }
      this.group.add(polyline(plane, pts, 0xffe0a0));
      vertices.push(this.path.start, ...this.path.segs.map((s) => s.to));
      if (c) this.previewSegment(from, c);
    } else if (this.anchor && c) {
      this.previewShape(this.anchor, c.point);
    }
    if (vertices.length > 0) this.group.add(pointMarkers(plane, vertices, 0xffe0a0, 5));
    if (c) this.group.add(pointMarkers(plane, [c.point], c.kind === 'endpoint' ? COLORS.blue : COLORS.amber, c.kind === 'endpoint' ? 10 : 7));
    this.syncStatus();
    this.vp.requestRender();
  }

  private previewSegment(from: V2, c: { point: V2; raw: V2 }): void {
    const typed = this.typed ? parseTyped(this.typed) : null;
    if (this.arcDrag) {
      const seg: Segment = { to: this.arcDrag.to, bulge: this.arcDrag.bulge };
      this.group.add(polyline(this.plane, [from, ...sampleSegment(from, seg)], 0xffffff));
      const g = arcGeometry(from, seg.to, seg.bulge ?? 0);
      const mid: V2 = [(from[0] + seg.to[0]) / 2, (from[1] + seg.to[1]) / 2];
      this.label(mid, g ? `R ${mm(g.radius)}  ${+((Math.abs(g.sweep) * 180) / Math.PI).toFixed(1)}°` : `${mm(dist(from, seg.to))}`);
      return;
    }
    let to = c.point;
    if (typed) {
      const angle = typed.angle ?? Math.atan2(c.raw[1] - from[1], c.raw[0] - from[0]);
      to = [from[0] + Math.abs(typed.length) * Math.cos(angle), from[1] + Math.abs(typed.length) * Math.sin(angle)];
    }
    const len = dist(from, to);
    this.group.add(polyline(this.plane, [from, to], 0xffffff, 0.85));
    const deg = (Math.atan2(to[1] - from[1], to[0] - from[0]) * 180) / Math.PI;
    const text = this.typed ? `${this.typed}▏` : `${mm(len)} mm  ∠${+deg.toFixed(1)}°${this.arcHeld ? '  arc' : ''}`;
    this.label([(from[0] + to[0]) / 2, (from[1] + to[1]) / 2], text);
  }

  private previewShape(a: V2, b: V2): void {
    if (this.tool === 'rect') {
      const typed = this.typed ? parsePair(this.typed) : null;
      const sx = b[0] >= a[0] ? 1 : -1, sy = b[1] >= a[1] ? 1 : -1;
      const corner: V2 = typed ? [a[0] + sx * Math.abs(typed[0]), a[1] + sy * Math.abs(typed[1])] : b;
      this.group.add(loopLine(this.plane, rectangleLoop(a, corner), 0xffffff, 0.85));
      this.label([(a[0] + corner[0]) / 2, Math.max(a[1], corner[1])], this.typed ? `${this.typed}▏` : `${mm(Math.abs(corner[0] - a[0]))} × ${mm(Math.abs(corner[1] - a[1]))} mm`);
    } else if (this.tool === 'circle') {
      const typed = this.typed ? parseTyped(this.typed) : null;
      const r = typed ? Math.abs(typed.length) : dist(a, b);
      if (r > 1e-6) this.group.add(loopLine(this.plane, circleAt(a, r), 0xffffff, 0.85));
      this.label([a[0], a[1] + r], this.typed ? `${this.typed}▏` : `R ${mm(r)}  Ø ${mm(2 * r)} mm`);
    }
  }

  private syncStatus(): void {
    const n = this.loops.length;
    const open = this.path ? ` · open loop, ${this.path.segs.length} segment${this.path.segs.length === 1 ? '' : 's'}` : '';
    this.status.textContent = `${n} closed loop${n === 1 ? '' : 's'}${open}`;
    this.status.classList.toggle('arm', this.arcHeld);
  }

  // ----- labels ----------------------------------------------------------------------------

  private label(at: V2, text: string): void {
    const el = h('div', { class: 'dim', text });
    this.vp.labels.append(el);
    this.labels.push({ el, at });
  }

  private clearLabels(): void {
    for (const l of this.labels) l.el.remove();
    this.labels.length = 0;
  }

  private placeLabels(): void {
    for (const l of this.labels) {
      const p = this.vp.project(planeToWorld(this.plane, l.at));
      l.el.style.transform = `translate(${p.x}px, ${p.y}px) translate(-50%, -140%)`;
    }
  }
}

function preventDefault(e: Event): void {
  e.preventDefault();
}
