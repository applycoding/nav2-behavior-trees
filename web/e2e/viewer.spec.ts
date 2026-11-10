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
    await scrub.evaluate((el) => {
      (el as HTMLInputElement).value = "1202";
      el.dispatchEvent(new Event("input", { bubbles: true }));
    });

    const expectTick = 1202;
    const exp = expectedAtTick(expectTick);
    await expect(page.getByTestId("readout-tick")).toHaveText(String(expectTick));
    await expect(page.getByTestId("readout-battery")).toHaveText(exp.sample!.battery.toFixed(3));
    await expect(page.getByTestId("readout-active-leaf")).toHaveText(exp.leaf?.type ?? "—");

    const trace = loadCriticalTrace();
    const followLaneUid = trace.header.nodes.find((n) => n.type === "FollowLane")?.uid ?? 34;
    const followStatus = exp.status.get(followLaneUid) ?? "IDLE";
    await expect(page.locator(`#node-${followLaneUid}`)).toHaveAttribute("data-status", followStatus);

    await page.getByTestId("btn-play").click();
    await page.waitForTimeout(300);
    await page.getByTestId("btn-pause").click();

    const before = Number(await page.getByTestId("readout-tick").innerText());
    await page.getByTestId("btn-step").click();
    const after = Number(await page.getByTestId("readout-tick").innerText());
    expect(after).toBe(before + 1);

    const haltTick = 1381;
    await scrub.evaluate(
      (el, tick) => {
        (el as HTMLInputElement).value = String(tick);
        el.dispatchEvent(new Event("input", { bubbles: true }));
      },
      haltTick,
    );
    const haltExp = expectedAtTick(haltTick);
    await expect(page.getByTestId("readout-battery")).toHaveText(haltExp.sample!.battery.toFixed(3));
    await expect(page.getByTestId("readout-active-leaf")).toHaveText("RetryUntilSuccessful");

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
