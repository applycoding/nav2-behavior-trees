// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

import { readFileSync } from "node:fs";
import { resolve } from "node:path";

import { expect, test } from "@playwright/test";

import { activeLeaf, buildReplayIndex, sampleAt, seekStatus } from "../src/replay/engine";
import { parseTraceJsonl } from "../src/trace/parse";

const SCENARIOS = [
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

// Matches bt_core::kCriticalBatteryFollowLaneHaltTick in C++ gtests.
const CRITICAL_HALT_TICK = 1381;
const CRITICAL_EVENT_TICK = 1380;

function loadCriticalTrace() {
  const path = resolve(import.meta.dirname, "../public/traces/critical_battery/trace.jsonl");
  return parseTraceJsonl(readFileSync(path, "utf8"));
}

function expectedAtTick(tick: number) {
  const trace = loadCriticalTrace();
  const index = buildReplayIndex(trace);
  const status = seekStatus(index, trace, tick);
  const sample = sampleAt(trace, tick);
  const leaf = activeLeaf(status, trace.nodesByUid);
  return { status, sample, leaf };
}

test.describe("viewer criterion 21", () => {
  test("loads manifest and all traces", async ({ page }) => {
    const errors: string[] = [];
    const failed: string[] = [];
    page.on("console", (msg) => {
      if (msg.type() === "error") errors.push(msg.text());
    });
    page.on("requestfailed", (req) => failed.push(req.url()));

    await page.goto("./");
    await expect(page.getByTestId("scenario-select")).toBeVisible();
    await expect(page).toHaveTitle(/simulation only/i);

    for (const id of SCENARIOS) {
      await page.getByTestId("scenario-select").selectOption(id);
      await expect(page.getByTestId("readout-tick")).toHaveText("0", { timeout: 15_000 });
    }

    expect(errors).toEqual([]);
    expect(failed).toEqual([]);
  });

  test("transport controls and critical_battery readouts", async ({ page }) => {
    const errors: string[] = [];
    page.on("console", (msg) => {
      if (msg.type() === "error") errors.push(msg.text());
    });

    await page.goto("./");
    await page.getByTestId("scenario-select").selectOption("critical_battery");

    const scrub = page.getByTestId("tick-scrub");

    await scrub.evaluate(
      (el, tick) => {
        (el as HTMLInputElement).value = String(tick);
        el.dispatchEvent(new Event("input", { bubbles: true }));
      },
      CRITICAL_EVENT_TICK,
    );
  await expect(page.getByTestId("readout-tick")).toHaveText(String(CRITICAL_EVENT_TICK));

    await scrub.evaluate(
      (el, tick) => {
        (el as HTMLInputElement).value = String(tick);
        el.dispatchEvent(new Event("input", { bubbles: true }));
      },
      CRITICAL_HALT_TICK,
    );
    const haltExp = expectedAtTick(CRITICAL_HALT_TICK);
    await expect(page.getByTestId("readout-tick")).toHaveText(String(CRITICAL_HALT_TICK));
    await expect(page.getByTestId("readout-battery")).toHaveText(haltExp.sample!.battery.toFixed(3));

    const trace = loadCriticalTrace();
    const followLaneUid = trace.header.nodes.find((n) => n.type === "FollowLane")?.uid ?? 34;
    const followStatus = haltExp.status.get(followLaneUid) ?? "IDLE";
    await expect(page.locator(`#node-${followLaneUid}`)).toHaveAttribute("data-status", followStatus);

    const dockUid = trace.header.nodes.find((n) => n.type === "NavigateToDock")?.uid;
    expect(dockUid).toBeDefined();
    const postDockTick = CRITICAL_HALT_TICK + 80;
    await scrub.evaluate(
      (el, tick) => {
        (el as HTMLInputElement).value = String(tick);
        el.dispatchEvent(new Event("input", { bubbles: true }));
      },
      postDockTick,
    );
    const dockExp = expectedAtTick(postDockTick);
    const dockStatus = dockExp.status.get(dockUid!) ?? "IDLE";
    expect(["RUNNING", "SUCCESS"]).toContain(dockStatus);

    await page.getByTestId("btn-play").click();
    await page.waitForTimeout(300);
    await page.getByTestId("btn-pause").click();

    const before = Number(await page.getByTestId("readout-tick").innerText());
    await page.getByTestId("btn-step").click();
    const after = Number(await page.getByTestId("readout-tick").innerText());
    expect(after).toBe(before + 1);

    await scrub.evaluate(
      (el, tick) => {
        (el as HTMLInputElement).value = String(tick);
        el.dispatchEvent(new Event("input", { bubbles: true }));
      },
      900,
    );
    await expect(page.getByTestId("readout-tick")).toHaveText("900");

    await page.screenshot({ path: resolve(import.meta.dirname, "../../docs/demo.png") });

    expect(errors).toEqual([]);
  });
});
