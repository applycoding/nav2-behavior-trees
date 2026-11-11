// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace bt_core {

struct RunMetrics {
  std::string scenario;
  bool success{false};
  double sim_time_sec{0};
  int ticks{0};

  std::vector<double> cov_pct{0, 0, 0};
  int lanes_done{0};
  int zones_skipped{0};
  int lanes_skipped{0};
  int charge_cycles{0};
  int dock_count{0};
  double min_battery{100};

  int clear_costmap{0};
  int sim_wait{0};
  int backup{0};
  int navigate_to_zone_attempts_zone_c{0};
  int recovery_count{0};
  int follow_lane_starts_b3{0};
  int critical_battery_event_tick{-1};

  int collisions{0};
  int goal_overlap{0};

  int halted_follow_lane{0};
  int follow_lane_halt_tick{-1};
  int next_goal_after_halt_kind{-1};  // encoded GoalKind

  int status_changes_max_per_tick{0};
  bool ended_docked{false};
  bool battery_zero{false};

  int halted_nodes{0};
};

}  // namespace bt_core
