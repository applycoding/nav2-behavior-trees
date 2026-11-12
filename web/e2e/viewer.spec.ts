// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

import { resolve } from "node:path";

import { expect, test } from "@playwright/test";

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

// From traces/critical_battery/metrics.json and C++ gtests (bt_core::kCriticalBattery*).
const CRITICAL_EVENT_TICK = 1380;
const CRITICAL_HALT_TICK = 1381;
const CRITICAL_EVENT_BATTERY = "81.112";
const CRITICAL_HALT_BATTERY = "7.986";
const CRITICAL_HALT_ROBOT_CELL = "35.500,5.500";
const FOLLOW_LANE_UID = 34;
const NAVIGATE_TO_DOCK_UID = 55;

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

  test("report.json is a non-sample build manifest", async ({ request, baseURL }) => {
    const res = await request.get(`${baseURL}report.json`);
    expect(res.ok()).toBeTruthy();
    const body = (await res.json()) as { sample?: boolean; git_sha?: string };
    expect(body.sample).toBe(false);
    expect(body.git_sha).toMatch(/^[0-9a-f]{40}$/);
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
    await expect(page.getByTestId("readout-battery")).toHaveText(CRITICAL_EVENT_BATTERY);

    await scrub.evaluate(
      (el, tick) => {
        (el as HTMLInputElement).value = String(tick);
        el.dispatchEvent(new Event("input", { bubbles: true }));
      },
      CRITICAL_HALT_TICK,
    );
    await expect(page.getByTestId("readout-tick")).toHaveText(String(CRITICAL_HALT_TICK));
    await expect(page.getByTestId("readout-battery")).toHaveText(CRITICAL_HALT_BATTERY);
    await expect(page.getByTestId("map-canvas")).toHaveAttribute("data-robot-cell", CRITICAL_HALT_ROBOT_CELL);

    await expect(page.locator(`#node-${FOLLOW_LANE_UID}`)).toHaveAttribute("data-status", "IDLE");

    await expect(page.locator(`#node-${NAVIGATE_TO_DOCK_UID}`)).toHaveAttribute("data-status", "RUNNING");

    await page.getByTestId("btn-play").click();
    await page.waitForTimeout(300);
    await page.getByTestId("btn-pause").click();

    const before = Number(await page.getByTestId("readout-tick").innerText());
    await page.getByTestId("btn-step").click();
    const after = Number(await page.getByTestId("readout-tick").innerText());
    expect(after).toBe(before + 1);

    await page.screenshot({ path: resolve(import.meta.dirname, "../../docs/demo.png") });

    expect(errors).toEqual([]);
  });
});
