import { planeFrame } from '@botfoundry/cad';
import { EMPTY_CONTROLS, type Design, type Plane } from '@botfoundry/model';

// Saving and loading designs as JSON, in localStorage and through files.

export const SAVE_KEY = 'botfoundry.workshop.saved@0';
export const AUTOSAVE_KEY = 'botfoundry.workshop.autosave@0';

export type KeyValueStore = Pick<Storage, 'getItem' | 'setItem'>;

const isObject = (v: unknown): v is Record<string, unknown> => typeof v === 'object' && v !== null && !Array.isArray(v);
const isVec = (v: unknown, n: number): boolean => Array.isArray(v) && v.length === n && v.every((x) => typeof x === 'number' && Number.isFinite(x));

function fail(message: string): never {
  throw new Error(`not a Bot Foundry design: ${message}`);
}

function checkPlane(p: unknown, where: string): void {
  if (!isObject(p) || !isVec(p.origin, 3) || !isVec(p.normal, 3) || !isVec(p.xAxis, 3)) fail(`${where} has no valid plane`);
  const plane = p as unknown as Plane;
  if (Math.hypot(plane.normal[0], plane.normal[1], plane.normal[2]) < 1e-9) fail(`${where} has a zero normal`);
  // The kernel throws for an x axis parallel to the normal; reject it here so no render path meets it.
  try {
    planeFrame(plane);
  } catch {
    fail(`${where} has an x axis parallel to its normal`);
  }
}

/** Parse and sanity-check design JSON. Throws an Error with a readable message. */
export function parseDesign(text: string): Design {
  let raw: unknown;
  try {
    raw = JSON.parse(text);
  } catch (e) {
    throw new Error(`not valid JSON: ${e instanceof Error ? e.message : String(e)}`);
  }
  if (!isObject(raw)) fail('expected a JSON object');
  if (raw.schema !== 'botfoundry/design@0') fail(`unknown schema ${JSON.stringify(raw.schema)}`);
  const list = (key: string): unknown[] => {
    const v = raw[key];
    if (v === undefined) return [];
    if (!Array.isArray(v)) fail(`"${key}" must be a list`);
    return v;
  };
  const sketches = list('sketches');
  sketches.forEach((s, i) => {
    if (!isObject(s) || typeof s.id !== 'string' || !Array.isArray(s.profiles)) fail(`sketch ${i} is malformed`);
    checkPlane(s.plane, `sketch "${s.id}"`);
  });
  const bodies = list('bodies');
  bodies.forEach((b, i) => {
    if (!isObject(b) || typeof b.id !== 'string' || !Array.isArray(b.features) || !isObject(b.structure)) fail(`body ${i} is malformed`);
  });
  const components = list('components');
  components.forEach((c, i) => {
    if (!isObject(c) || typeof c.id !== 'string' || typeof c.def !== 'string' || !isObject(c.pose)) fail(`component ${i} is malformed`);
    if (!isVec(c.pose.position, 3) || !isVec(c.pose.rotation, 4)) fail(`component "${c.id}" has an invalid pose`);
  });
  return {
    schema: 'botfoundry/design@0',
    name: typeof raw.name === 'string' ? raw.name : 'Untitled bot',
    sketches: sketches as Design['sketches'],
    bodies: bodies as Design['bodies'],
    components: components as Design['components'],
    fasteners: list('fasteners') as Design['fasteners'],
    welds: list('welds') as Design['welds'],
    joints: list('joints') as Design['joints'],
    wiring: list('wiring') as Design['wiring'],
    controls: isObject(raw.controls) ? (raw.controls as unknown as Design['controls']) : EMPTY_CONTROLS,
  };
}

export function stringifyDesign(d: Design): string {
  return JSON.stringify(d, null, 2);
}

/** localStorage can be missing or throw (private windows, blocked storage); callers get null instead. */
export function browserStorage(): KeyValueStore | null {
  try {
    return globalThis.localStorage ?? null;
  } catch {
    return null;
  }
}

export function writeStored(key: string, d: Design, store: KeyValueStore | null = browserStorage()): boolean {
  if (!store) return false;
  try {
    store.setItem(key, stringifyDesign(d));
    return true;
  } catch {
    return false;
  }
}

export function readStored(key: string, store: KeyValueStore | null = browserStorage()): Design | null {
  if (!store) return null;
  try {
    const text = store.getItem(key);
    return text === null ? null : parseDesign(text);
  } catch {
    return null;
  }
}
