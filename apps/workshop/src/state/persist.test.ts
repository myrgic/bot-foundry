import { describe, expect, it } from 'vitest';
import { sampleDesign } from '@botfoundry/cad';
import { emptyDesign } from './ops.ts';
import { parseDesign, readStored, stringifyDesign, writeStored, type KeyValueStore } from './persist.ts';

function memory(): KeyValueStore & { data: Map<string, string> } {
  const data = new Map<string, string>();
  return { data, getItem: (k) => data.get(k) ?? null, setItem: (k, v) => void data.set(k, v) };
}

describe('persist', () => {
  it('round-trips the sample design through JSON', () => {
    const d = sampleDesign();
    expect(parseDesign(stringifyDesign(d))).toEqual(d);
  });

  it('round-trips through storage', () => {
    const store = memory();
    expect(writeStored('k', emptyDesign('x'), store)).toBe(true);
    expect(readStored('k', store)?.name).toBe('x');
    expect(readStored('missing', store)).toBeNull();
  });

  it('reports bad input with a readable message', () => {
    expect(() => parseDesign('{')).toThrow(/not valid JSON/);
    expect(() => parseDesign('[]')).toThrow(/expected a JSON object/);
    expect(() => parseDesign('{"schema":"other"}')).toThrow(/unknown schema/);
    expect(() => parseDesign(JSON.stringify({ schema: 'botfoundry/design@0', bodies: [{ id: 1 }] }))).toThrow(/body 0/);
    expect(() => parseDesign(JSON.stringify({ schema: 'botfoundry/design@0', components: [{ id: 'c', def: 'd', pose: { position: [0, 0], rotation: [0, 0, 0, 1] } }] }))).toThrow(/invalid pose/);
  });

  it('rejects degenerate sketch planes on load with a readable message', () => {
    const sketch = (plane: object) => JSON.stringify({ schema: 'botfoundry/design@0', sketches: [{ id: 's', profiles: [], plane }] });
    const origin = [0, 0, 0];
    expect(() => parseDesign(sketch({ origin, normal: [0, 0, 0], xAxis: [1, 0, 0] }))).toThrow(/zero normal/);
    expect(() => parseDesign(sketch({ origin, normal: [0, 0, 1], xAxis: [0, 0, 2] }))).toThrow(/x axis parallel to its normal/);
    expect(parseDesign(sketch({ origin, normal: [0, 0, 1], xAxis: [1, 0, 0] })).sketches).toHaveLength(1);
  });

  it('fills missing lists so a minimal file loads', () => {
    const d = parseDesign('{"schema":"botfoundry/design@0"}');
    expect(d.bodies).toEqual([]);
    expect(d.controls.nodes).toEqual([]);
  });

  it('survives storage that throws or returns junk', () => {
    const throwing: KeyValueStore = { getItem: () => { throw new Error('blocked'); }, setItem: () => { throw new Error('blocked'); } };
    expect(writeStored('k', emptyDesign(), throwing)).toBe(false);
    expect(readStored('k', throwing)).toBeNull();
    const junk: KeyValueStore = { getItem: () => 'not json', setItem: () => {} };
    expect(readStored('k', junk)).toBeNull();
    expect(readStored('k', null)).toBeNull();
  });
});
