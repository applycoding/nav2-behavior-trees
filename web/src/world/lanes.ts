// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

import { ZONES, isWallCell } from "./map";

export interface TraceLane {
  zone: number;
  index: number;
  row_y: number;
  start_x: number;
  start_y: number;
  end_x: number;
  end_y: number;
  reverse: boolean;
}

function cellCenter(c: number): number {
  return c + 0.5;
}

function cellFree(x: number, y: number): boolean {
  return !isWallCell(x, y);
}

/** Mirrors GridSim::generateLanes (pose and lane endpoints are in cell-center units). */
export function generateLanes(): TraceLane[] {
  const lanes: TraceLane[] = [];
  for (const z of ZONES) {
    let laneInZone = 0;
    let reverseRow = false;
    for (let y = z.y0 + 1; y <= z.y1; y += 2) {
      const segs: Array<{ x0: number; x1: number }> = [];
      let x = z.x0;
      while (x <= z.x1) {
        while (x <= z.x1 && !cellFree(x, y)) x++;
        if (x > z.x1) break;
        const xstart = x;
        while (x <= z.x1 && cellFree(x, y)) x++;
        const len = x - xstart;
        if (len >= 2) segs.push({ x0: xstart, x1: x - 1 });
      }
      if (reverseRow) segs.reverse();
      for (const s of segs) {
        if (!reverseRow) {
          lanes.push({
            zone: z.id,
            index: laneInZone++,
            row_y: y,
            start_x: cellCenter(s.x0),
            start_y: cellCenter(y),
            end_x: cellCenter(s.x1),
            end_y: cellCenter(y),
            reverse: false,
          });
        } else {
          lanes.push({
            zone: z.id,
            index: laneInZone++,
            row_y: y,
            start_x: cellCenter(s.x1),
            start_y: cellCenter(y),
            end_x: cellCenter(s.x0),
            end_y: cellCenter(y),
            reverse: true,
          });
        }
      }
      reverseRow = !reverseRow;
    }
  }
  return lanes;
}
