// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

import { GRID_H, GRID_W, ZONES, isWallCell } from "../world/map";
import type { TickSample, TraceLane } from "../trace/types";

const ZONE_FILL = ["#1e3a5f", "#1f4d3a", "#4a3a1e"];
const ZONE_STROKE = ["#60a5fa", "#4ade80", "#fbbf24"];
const COV_TINT = ["rgba(96, 165, 250, 0.35)", "rgba(74, 222, 128, 0.35)", "rgba(251, 191, 36, 0.35)"];

export function drawMapScene(
  ctx: CanvasRenderingContext2D,
  sample: TickSample | null,
  dock: [number, number],
  lanes: TraceLane[],
): void {
  const w = ctx.canvas.width;
  const h = ctx.canvas.height;
  const scale = Math.min(w / GRID_W, h / GRID_H);
  const ox = (w - GRID_W * scale) / 2;
  const oy = (h - GRID_H * scale) / 2;

  ctx.fillStyle = "#121820";
  ctx.fillRect(0, 0, w, h);

  for (const z of ZONES) {
    ctx.fillStyle = ZONE_FILL[z.id] ?? "#333";
    ctx.strokeStyle = ZONE_STROKE[z.id] ?? "#888";
    ctx.lineWidth = 1;
    ctx.fillRect(ox + z.x0 * scale, oy + z.y0 * scale, (z.x1 - z.x0 + 1) * scale, (z.y1 - z.y0 + 1) * scale);
    ctx.strokeRect(ox + z.x0 * scale, oy + z.y0 * scale, (z.x1 - z.x0 + 1) * scale, (z.y1 - z.y0 + 1) * scale);
    if (sample && sample.cov[z.id] !== undefined) {
      const pct = Math.max(0, Math.min(100, sample.cov[z.id])) / 100;
      ctx.fillStyle = COV_TINT[z.id] ?? "rgba(200,200,200,0.3)";
      ctx.globalAlpha = 0.15 + 0.55 * pct;
      ctx.fillRect(ox + z.x0 * scale, oy + z.y0 * scale, (z.x1 - z.x0 + 1) * scale, (z.y1 - z.y0 + 1) * scale);
      ctx.globalAlpha = 1;
    }
    ctx.fillStyle = "#cbd5e1";
    ctx.font = `${Math.max(10, scale * 0.55)}px sans-serif`;
    ctx.fillText(
      z.name,
      ox + z.x0 * scale + 4,
      oy + z.y0 * scale + Math.max(12, scale * 0.7),
    );
  }

  ctx.strokeStyle = "rgba(148, 163, 184, 0.55)";
  ctx.lineWidth = Math.max(1, scale * 0.12);
  for (const lane of lanes) {
    ctx.beginPath();
    ctx.moveTo(ox + lane.start_x * scale, oy + lane.start_y * scale);
    ctx.lineTo(ox + lane.end_x * scale, oy + lane.end_y * scale);
    ctx.stroke();
  }

  for (let y = 0; y < GRID_H; y++) {
    for (let x = 0; x < GRID_W; x++) {
      if (!isWallCell(x, y)) continue;
      ctx.fillStyle = "#2d3748";
      ctx.fillRect(ox + x * scale, oy + y * scale, scale, scale);
    }
  }

  const [dockX, dockY] = dock;
  ctx.fillStyle = "#a78bfa";
  ctx.fillRect(ox + dockX * scale, oy + dockY * scale, scale * 1.2, scale * 1.2);

  if (sample) {
    const [rx, ry] = sample.pose;
    ctx.fillStyle = sample.brushes ? "#f97316" : "#38bdf8";
    ctx.beginPath();
    ctx.arc(ox + rx * scale, oy + ry * scale, scale * 0.55, 0, Math.PI * 2);
    ctx.fill();
    if (sample.docked) {
      ctx.strokeStyle = "#e2e8f0";
      ctx.lineWidth = 2;
      ctx.stroke();
    }
  }
}

export function formatRobotCell(sample: TickSample | null): string {
  if (!sample) return "none";
  return `${sample.pose[0].toFixed(3)},${sample.pose[1].toFixed(3)}`;
}

export function drawBatteryGauge(ctx: CanvasRenderingContext2D, battery: number): void {
  const w = ctx.canvas.width;
  const h = ctx.canvas.height;
  const pct = Math.max(0, Math.min(100, battery)) / 100;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#334155";
  ctx.fillRect(0, 0, w, h);
  const color = battery < 10 ? "#ef4444" : battery < 25 ? "#f59e0b" : "#22c55e";
  ctx.fillStyle = color;
  ctx.fillRect(0, 0, w * pct, h);
  ctx.strokeStyle = "#64748b";
  ctx.strokeRect(0.5, 0.5, w - 1, h - 1);
}
