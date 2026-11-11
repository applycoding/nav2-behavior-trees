// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

import { enrichTraceHeader } from "./enrich";
import type { LoadedTrace, TickSample, TickTransition, TraceHeader } from "./types";

export function parseTraceJsonl(text: string): LoadedTrace {
  const lines = text.split("\n").filter((l) => l.trim().length > 0);
  if (lines.length === 0) {
    throw new Error("empty trace");
  }
  const header = enrichTraceHeader(JSON.parse(lines[0]) as TraceHeader);
  const samples = new Map<number, TickSample>();
  const transitions: TickTransition[] = [];
  let maxTick = 0;

  for (let i = 1; i < lines.length; i++) {
    const row = JSON.parse(lines[i]) as TickSample | TickTransition;
    if (row.k === "s") {
      samples.set(row.tick, row);
      maxTick = Math.max(maxTick, row.tick);
    } else {
      transitions.push(row);
      maxTick = Math.max(maxTick, row.tick);
    }
  }

  const nodesByUid = new Map(header.nodes.map((n) => [n.uid, n]));

  return { header, samples, transitions, maxTick, nodesByUid };
}

export async function fetchManifest(): Promise<{ id: string }[]> {
  const base = import.meta.env.BASE_URL;
  const res = await fetch(`${base}traces/manifest.json`);
  if (!res.ok) {
    throw new Error(`manifest fetch failed: ${res.status}`);
  }
  const body = (await res.json()) as { scenarios: { id: string }[] };
  return body.scenarios;
}

export async function loadTrace(scenarioId: string): Promise<LoadedTrace> {
  const base = import.meta.env.BASE_URL;
  const res = await fetch(`${base}traces/${scenarioId}/trace.jsonl`);
  if (!res.ok) {
    throw new Error(`trace fetch failed: ${res.status}`);
  }
  return parseTraceJsonl(await res.text());
}
