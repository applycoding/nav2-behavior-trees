// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#pragma once

#include <deque>
#include <string>
#include <vector>

namespace bt_core {

struct RunMetrics;

class MissionState {
 public:
  void resetZones(const std::deque<int>& zones);

  int activeZone() const { return active_zone_; }
  int laneIndex() const { return lane_idx_; }
  bool needCharge() const { return need_charge_; }
  bool atBoundary() const { return at_boundary_; }

  void setNeedCharge(bool v) { need_charge_ = v; }
  void setAtBoundary(bool v) { at_boundary_ = v; }

  bool queueEmpty() const;
  bool hasNextLane(int lanes_in_zone) const;

  // Node side effects
  void onSelectZone(int lanes_in_zone);
  void onAdvanceLane(int lanes_in_zone);
  void onSkipLane();
  void onMarkZoneDone();
  void onMarkZoneSkipped();
  void onFollowLaneStart();

  RunMetrics& metrics();
  const RunMetrics& metrics() const;

 private:
  std::deque<int> zone_queue_;
  int active_zone_{-1};
  int lane_idx_{0};
  bool need_charge_{false};
  bool at_boundary_{false};
  RunMetrics* metrics_{nullptr};

  friend void missionStateBindMetrics(MissionState& state, RunMetrics& m);
};

void missionStateBindMetrics(MissionState& state, RunMetrics& m);

}  // namespace bt_core
