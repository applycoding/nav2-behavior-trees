// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

import { readFileSync } from "node:fs";
import { resolve } from "node:path";
import { describe, expect, it } from "vitest";

import {
  CHECKPOINT_INTERVAL,
  buildReplayIndex,
  fullReplayAt,
  seekStatus,
} from "../src/replay/engine";
import { parseTraceJsonl } from "../src/trace/parse";
import type { NodeStatus } from "../src/trace/types";

function traceFixture(): string {
  const candidates = [
    resolve(import.meta.dirname, "../public/traces/critical_battery/trace.jsonl"),
    resolve(import.meta.dirname, "../../traces/web_gen/critical_battery/trace.jsonl"),
  ];
  for (const p of candidates) {
    try {
      return readFileSync(p, "utf8");
    } catch {
      /* try next */
    }
  }
  throw new Error("critical_battery trace not found; run npm run copy-traces");
}

function equalMaps(a: Map<number, NodeStatus>, b: Map<number, NodeStatus>): boolean {
  if (a.size !== b.size) return false;
  for (const [k, v] of a) {
    if (b.get(k) !== v) return false;
  }
  return true;
}

describe("replay engine", () => {
  const trace = parseTraceJsonl(traceFixture());
  const index = buildReplayIndex(trace);

  it("builds checkpoints every 100 ticks", () => {
    for (let t = 0; t <= trace.maxTick; t += CHECKPOINT_INTERVAL) {
      expect(index.checkpoints.has(t)).toBe(true);
    }
  });

  it("seek matches full replay at scattered ticks", () => {
    const probes = [0, 1, 50, 99, 100, 101, 1202, 1380, 1381, trace.maxTick];
    for (const tick of probes) {
      const sought = seekStatus(index, trace, tick);
      const full = fullReplayAt(trace, tick);
      expect(equalMaps(sought, full)).toBe(true);
    }
  });

  it("backward seek reproduces earlier tick state", () => {
    const forward = seekStatus(index, trace, 2500);
    const back = seekStatus(index, trace, 1100);
    const direct = seekStatus(index, trace, 1100);
    expect(equalMaps(back, direct)).toBe(true);
    expect(equalMaps(forward, seekStatus(index, trace, 2500))).toBe(true);
    expect(equalMaps(back, direct)).toBe(true);
    const full = fullReplayAt(trace, 1100);
    expect(equalMaps(back, full)).toBe(true);
  });
});
