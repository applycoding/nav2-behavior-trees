// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

import { generateLanes } from "../world/lanes";
import type { TraceHeader, TraceLane, TraceNode } from "./types";

function parentUidForNode(nodes: TraceNode[], node: TraceNode): number {
  if (node.parent !== undefined) return node.parent;
  const parts = node.path.split("/");
  if (parts.length <= 1) return 0;
  const parentPath = parts.slice(0, -1).join("/");
  const parent = nodes.find((n) => n.path === parentPath);
  return parent?.uid ?? 0;
}

/** Fills parent uid and lanes when absent from the trace header (pose is in cell-center units). */
export function enrichTraceHeader(header: TraceHeader): TraceHeader {
  const nodes: TraceNode[] = header.nodes.map((n) => ({
    ...n,
    parent: parentUidForNode(header.nodes, n),
  }));
  const lanes: TraceLane[] = header.lanes?.length ? header.lanes : generateLanes();
  return { ...header, nodes, lanes };
}

export function nodeDepth(nodes: TraceNode[], uid: number): number {
  const byUid = new Map(nodes.map((n) => [n.uid, n]));
  let depth = 0;
  let cur = byUid.get(uid);
  while (cur && cur.parent && cur.parent !== cur.uid) {
    depth++;
    cur = byUid.get(cur.parent);
  }
  return depth;
}
