// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

export const GRID_W = 60;
export const GRID_H = 40;
export const CELL_M = 0.2;

export interface ZoneRect {
  id: number;
  name: string;
  x0: number;
  y0: number;
  x1: number;
  y1: number;
}

export const ZONES: ZoneRect[] = [
  { id: 0, name: "A", x0: 4, y0: 4, x1: 24, y1: 15 },
  { id: 1, name: "B", x0: 35, y0: 4, x1: 54, y1: 15 },
  { id: 2, name: "C", x0: 4, y0: 24, x1: 24, y1: 34 },
];

export function isWallCell(x: number, y: number): boolean {
  if (x < 0 || y < 0 || x >= GRID_W || y >= GRID_H) return true;
  if (x === 0 || y === 0 || x === GRID_W - 1 || y === GRID_H - 1) return true;
  if (x === 30 && y >= 1 && y < GRID_H - 1) {
    if (y >= 10 && y <= 12) return false;
    if (y >= 28 && y <= 30) return false;
    return true;
  }
  if (y === 20 && x >= 1 && x < GRID_W - 1) {
    if (x >= 14 && x <= 16) return false;
    if (x >= 44 && x <= 46) return false;
    return true;
  }
  if (x >= 12 && x <= 15 && y >= 8 && y <= 10) return true;
  if (x >= 40 && x <= 42 && y >= 12 && y <= 13) return true;
  return false;
}
