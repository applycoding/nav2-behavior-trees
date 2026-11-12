// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#pragma once

namespace bt_core {

struct MissionParams {
  static constexpr double DT_SEC = 0.1;
  static constexpr int DT_MS = 100;
  static constexpr int MAX_TICKS = 15000;
  static constexpr int MAX_STATUS_CHANGES_PER_TICK = 2000;

  static constexpr double LOW_BATTERY = 25.0;
  static constexpr double CRIT_BATTERY = 10.0;
  static constexpr double CHARGED_BATTERY = 95.0;

  static constexpr double MOTION_DRAIN_PER_M = 0.3;
  static constexpr double TIME_DRAIN_PER_SEC = 0.02;
  static constexpr double CHARGE_PER_SEC = 1.0;

  static constexpr double CELL_SIZE_M = 0.2;
  static constexpr double SPEED_MPS = 0.4;
  static constexpr double CELLS_PER_TICK = SPEED_MPS * DT_SEC / CELL_SIZE_M;

  static constexpr double DOCK_SEC = 3.0;
  static constexpr double UNDOCK_SEC = 2.0;
  static constexpr double BACKUP_SEC = 2.0;
};

}  // namespace bt_core
