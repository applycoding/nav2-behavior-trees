// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

import { drawBatteryGauge, drawMapScene } from "./canvas/draw";
import {
  activeLeaf,
  buildReplayIndex,
  sampleAt,
  seekStatus,
  type ReplayIndex,
} from "./replay/engine";
import { fetchManifest, loadTrace } from "./trace/parse";
import type { LoadedTrace, NodeStatus } from "./trace/types";

const statusClass: Record<NodeStatus, string> = {
  IDLE: "status-idle",
  RUNNING: "status-running",
  SUCCESS: "status-success",
  FAILURE: "status-failure",
};

export class ViewerApp {
  private trace: LoadedTrace | null = null;
  private index: ReplayIndex | null = null;
  private tick = 0;
  private playing = false;
  private playTimer: number | null = null;

  private readonly scenarioSelect = document.querySelector<HTMLSelectElement>("#scenario-select")!;
  private readonly tickScrub = document.querySelector<HTMLInputElement>("#tick-scrub")!;
  private readonly readoutTick = document.querySelector<HTMLSpanElement>("#readout-tick")!;
  private readonly readoutBattery = document.querySelector<HTMLSpanElement>("#readout-battery")!;
  private readonly readoutLeaf = document.querySelector<HTMLSpanElement>("#readout-active-leaf")!;
  private readonly readoutCov = document.querySelector<HTMLSpanElement>("#readout-cov")!;
  private readonly treeList = document.querySelector<HTMLUListElement>("#tree-list")!;
  private readonly mapCanvas = document.querySelector<HTMLCanvasElement>("#map-canvas")!;
  private readonly batteryCanvas = document.querySelector<HTMLCanvasElement>("#battery-canvas")!;
  private readonly mapCtx = this.mapCanvas.getContext("2d")!;
  private readonly batteryCtx = this.batteryCanvas.getContext("2d")!;

  async init(): Promise<void> {
    this.bindControls();
    const scenarios = await fetchManifest();
    for (const s of scenarios) {
      const opt = document.createElement("option");
      opt.value = s.id;
      opt.textContent = s.id;
      this.scenarioSelect.append(opt);
    }
    const defaultId = scenarios.some((s) => s.id === "critical_battery")
      ? "critical_battery"
      : scenarios[0]?.id;
    if (defaultId) {
      this.scenarioSelect.value = defaultId;
      await this.loadScenario(defaultId);
    }
  }

  private bindControls(): void {
    document.querySelector("#btn-play")?.addEventListener("click", () => this.play());
    document.querySelector("#btn-pause")?.addEventListener("click", () => this.pause());
    document.querySelector("#btn-step")?.addEventListener("click", () => this.stepOnce());
    this.scenarioSelect.addEventListener("change", () => {
      void this.loadScenario(this.scenarioSelect.value);
    });
    this.tickScrub.addEventListener("input", () => {
      this.pause();
      this.setTick(Number(this.tickScrub.value));
    });
  }

  async loadScenario(id: string): Promise<void> {
    this.pause();
    this.trace = await loadTrace(id);
    this.index = buildReplayIndex(this.trace);
    this.tickScrub.min = "0";
    this.tickScrub.max = String(this.trace.maxTick);
    this.buildTreeDom();
    this.setTick(0);
  }

  private buildTreeDom(): void {
    if (!this.trace) return;
    this.treeList.replaceChildren();
    for (const node of this.trace.header.nodes) {
      const li = document.createElement("li");
      li.id = `node-${node.uid}`;
      li.dataset.testid = `node-${node.uid}`;
      li.dataset.uid = String(node.uid);
      li.dataset.status = "IDLE";
      li.className = "status-idle";
      const swatch = document.createElement("span");
      swatch.className = "swatch";
      const label = document.createElement("span");
      label.textContent = `${node.type} (${node.uid})`;
      li.append(swatch, label);
      this.treeList.append(li);
    }
  }

  getCurrentTick(): number {
    return this.tick;
  }

  setTick(tick: number): void {
    if (!this.trace || !this.index) return;
    const clamped = Math.max(0, Math.min(this.trace.maxTick, tick));
    this.tick = clamped;
    this.tickScrub.value = String(clamped);
    const status = seekStatus(this.index, this.trace, clamped);
    const sample = sampleAt(this.trace, clamped);
    const leaf = activeLeaf(status, this.trace.nodesByUid);

    this.readoutTick.textContent = String(clamped);
    this.readoutBattery.textContent = sample ? sample.battery.toFixed(3) : ", ";
    this.readoutLeaf.textContent = leaf ? leaf.type : ", ";
    if (sample) {
      this.readoutCov.textContent = sample.cov.map((c) => c.toFixed(1)).join(" / ");
    }

    for (const node of this.trace.header.nodes) {
      const st = status.get(node.uid) ?? "IDLE";
      const li = document.getElementById(`node-${node.uid}`);
      if (!li) continue;
      li.dataset.status = st;
      li.className = statusClass[st];
    }

    drawMapScene(this.mapCtx, sample, this.trace.header.map.dock);
    drawBatteryGauge(this.batteryCtx, sample?.battery ?? 0);
  }

  play(): void {
    if (!this.trace || this.playing) return;
    this.playing = true;
    const dt = this.trace.header.dt_ms;
    this.playTimer = window.setInterval(() => {
      if (!this.trace) return;
      if (this.tick >= this.trace.maxTick) {
        this.pause();
        return;
      }
      this.setTick(this.tick + 1);
    }, dt);
  }

  pause(): void {
    this.playing = false;
    if (this.playTimer !== null) {
      window.clearInterval(this.playTimer);
      this.playTimer = null;
    }
  }

  stepOnce(): void {
    this.pause();
    if (!this.trace) return;
    this.setTick(Math.min(this.trace.maxTick, this.tick + 1));
  }
}
