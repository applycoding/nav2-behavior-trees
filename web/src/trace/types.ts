// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

export type NodeStatus = "IDLE" | "RUNNING" | "SUCCESS" | "FAILURE";

export interface TraceNode {
  uid: number;
  path: string;
  type: string;
}

export interface TraceHeader {
  schema_version: number;
  scenario: string;
  dt_ms: number;
  tree_sha256: string;
  nodes: TraceNode[];
  map: { dock: [number, number] };
}

export interface TickSample {
  k: "s";
  tick: number;
  pose: [number, number];
  battery: number;
  docked: boolean;
  brushes: boolean;
  cov: [number, number, number];
}

export interface TickTransition {
  k: "t";
  tick: number;
  seq: number;
  uid: number;
  from: NodeStatus;
  to: NodeStatus;
}

export type TraceLine = TickSample | TickTransition;

export interface ScenarioManifest {
  scenarios: Array<{ id: string; scenario: string; dt_ms: number }>;
}

export interface LoadedTrace {
  header: TraceHeader;
  samples: Map<number, TickSample>;
  transitions: TickTransition[];
  maxTick: number;
  nodesByUid: Map<number, TraceNode>;
}
