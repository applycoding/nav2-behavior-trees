// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

import { readFileSync, writeFileSync, existsSync, readdirSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";

const __dirname = dirname(fileURLToPath(import.meta.url));
const webRoot = join(__dirname, "..");

const tracesRoot =
  process.env.TRACE_ROOT && existsSync(process.env.TRACE_ROOT)
    ? process.env.TRACE_ROOT
    : join(webRoot, "public/traces");

const scenarioIds = [
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

const scenarios = [];
for (const id of scenarioIds) {
  const metricsPath = join(tracesRoot, id, "metrics.json");
  if (!existsSync(metricsPath)) {
    throw new Error(`missing metrics for report: ${metricsPath}`);
  }
  const m = JSON.parse(readFileSync(metricsPath, "utf8"));
  const entry = {
    id,
    scenario: m.scenario ?? id,
    success: Boolean(m.success),
    sim_time_sec: m.sim_time_sec,
  };
  if (m.cov_pct !== undefined) {
    entry.cov_pct = m.cov_pct;
  }
  scenarios.push(entry);
}

const gitSha = process.env.GITHUB_SHA ?? process.env.GIT_SHA ?? "0".repeat(40);
const runId = process.env.GITHUB_RUN_ID ?? "local";

const report = {
  schema_version: 1,
  git_sha: gitSha,
  sample: false,
  run_id: String(runId),
  scenarios,
};

writeFileSync(join(webRoot, "public/report.json"), `${JSON.stringify(report, null, 2)}\n`);
console.log(`wrote ${join(webRoot, "public/report.json")} (${scenarios.length} scenarios from ${tracesRoot})`);
