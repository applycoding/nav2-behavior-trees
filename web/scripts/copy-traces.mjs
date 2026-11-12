// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

import { copyFileSync, existsSync, mkdirSync, readdirSync, readFileSync, writeFileSync } from "node:fs";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const root = resolve(dirname(fileURLToPath(import.meta.url)), "../..");
const outRoot = resolve(dirname(fileURLToPath(import.meta.url)), "../public/traces");

const scenarios = [
  "nominal",
  "low_battery",
  "critical_battery",
  "blocked_corridor",
  "phantom_obstacle",
  "stuck",
  "unreachable_zone",
  "lane_obstacle",
  "lane_blocked_perm",
];

const sources = [join(root, "traces"), join(root, "traces/web_gen")];

function resolveScenarioDir(id) {
  for (const base of sources) {
    const dir = join(base, id);
    if (existsSync(join(dir, "trace.jsonl"))) return dir;
  }
  return null;
}

mkdirSync(outRoot, { recursive: true });

const manifest = { scenarios: [] };

for (const id of scenarios) {
  const src = resolveScenarioDir(id);
  if (!src) {
    throw new Error(`missing trace for ${id}; run bt_mission or populate traces/web_gen`);
  }
  const dest = join(outRoot, id);
  mkdirSync(dest, { recursive: true });
  copyFileSync(join(src, "trace.jsonl"), join(dest, "trace.jsonl"));
  if (existsSync(join(src, "metrics.json"))) {
    copyFileSync(join(src, "metrics.json"), join(dest, "metrics.json"));
  }
  const header = JSON.parse(readFileSync(join(dest, "trace.jsonl"), "utf8").split("\n")[0]);
  manifest.scenarios.push({ id, dt_ms: header.dt_ms, scenario: header.scenario });
}

writeFileSync(join(outRoot, "manifest.json"), `${JSON.stringify(manifest, null, 2)}\n`);
