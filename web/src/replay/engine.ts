// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

import type { LoadedTrace, NodeStatus, TickSample, TraceNode } from "../trace/types";

export const CHECKPOINT_INTERVAL = 100;

export type StatusMap = Map<number, NodeStatus>;

export interface ReplayIndex {
  checkpoints: Map<number, StatusMap>;
  transitionsByTick: Map<number, Array<{ uid: number; to: NodeStatus; seq: number }>>;
}

function cloneStatus(src: StatusMap): StatusMap {
  return new Map(src);
}

export function buildReplayIndex(trace: LoadedTrace): ReplayIndex {
  const allUids = trace.header.nodes.map((n) => n.uid);
  const current = new Map<number, NodeStatus>(allUids.map((uid) => [uid, "IDLE"]));
  const checkpoints = new Map<number, StatusMap>();
  const transitionsByTick = new Map<number, Array<{ uid: number; to: NodeStatus; seq: number }>>();

  for (const t of trace.transitions) {
    let bucket = transitionsByTick.get(t.tick);
    if (!bucket) {
      bucket = [];
      transitionsByTick.set(t.tick, bucket);
    }
    bucket.push({ uid: t.uid, to: t.to, seq: t.seq });
  }

  for (const bucket of transitionsByTick.values()) {
    bucket.sort((a, b) => a.seq - b.seq);
  }

  checkpoints.set(0, cloneStatus(current));

  for (let tick = 0; tick <= trace.maxTick; tick++) {
    const bucket = transitionsByTick.get(tick);
    if (bucket) {
      for (const tr of bucket) {
        current.set(tr.uid, tr.to);
      }
    }
    if (tick % CHECKPOINT_INTERVAL === 0) {
      checkpoints.set(tick, cloneStatus(current));
    }
  }

  return { checkpoints, transitionsByTick };
}

function applyRange(
  status: StatusMap,
  transitionsByTick: ReplayIndex["transitionsByTick"],
  fromExclusive: number,
  toInclusive: number,
): void {
  for (let tick = fromExclusive + 1; tick <= toInclusive; tick++) {
    const bucket = transitionsByTick.get(tick);
    if (!bucket) continue;
    for (const tr of bucket) {
      status.set(tr.uid, tr.to);
    }
  }
}

export function seekStatus(index: ReplayIndex, trace: LoadedTrace, tick: number): StatusMap {
  const clamped = Math.max(0, Math.min(trace.maxTick, tick));
  const base = Math.floor(clamped / CHECKPOINT_INTERVAL) * CHECKPOINT_INTERVAL;
  const start = index.checkpoints.get(base);
  const status = cloneStatus(
    start ?? new Map(trace.header.nodes.map((n) => [n.uid, "IDLE" as NodeStatus])),
  );
  applyRange(status, index.transitionsByTick, base, clamped);
  return status;
}

export function fullReplayAt(trace: LoadedTrace, tick: number): StatusMap {
  const allUids = trace.header.nodes.map((n) => n.uid);
  const status = new Map<number, NodeStatus>(allUids.map((uid) => [uid, "IDLE"]));
  for (const t of trace.transitions) {
    if (t.tick > tick) break;
    status.set(t.uid, t.to);
  }
  return status;
}

export function activeLeaf(
  status: StatusMap,
  nodesByUid: Map<number, TraceNode>,
): TraceNode | null {
  let best: TraceNode | null = null;
  let bestDepth = -1;
  for (const [uid, st] of status) {
    if (st !== "RUNNING") continue;
    const node = nodesByUid.get(uid);
    if (!node) continue;
    const depth = node.path.length;
    if (depth > bestDepth || (depth === bestDepth && uid > (best?.uid ?? -1))) {
      best = node;
      bestDepth = depth;
    }
  }
  return best;
}

export function sampleAt(trace: LoadedTrace, tick: number): TickSample | null {
  const exact = trace.samples.get(tick);
  if (exact) return exact;
  const keys = [...trace.samples.keys()].filter((k) => k <= tick).sort((a, b) => b - a);
  return keys.length > 0 ? (trace.samples.get(keys[0]) ?? null) : null;
}
