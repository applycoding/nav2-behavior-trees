// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/mission_state.hpp"

#include "bt_core/metrics.hpp"

namespace bt_core {

void missionStateBindMetrics(MissionState& state, RunMetrics& m) {
  state.metrics_ = &m;
}

void MissionState::resetZones(const std::deque<int>& zones) {
  zone_queue_ = zones;
  active_zone_ = -1;
  lane_idx_ = 0;
  need_charge_ = false;
  at_boundary_ = false;
}

bool MissionState::queueEmpty() const {
  return active_zone_ < 0 && zone_queue_.empty();
}

bool MissionState::hasNextLane(int lanes_in_zone) const {
  return lane_idx_ < lanes_in_zone;
}

void MissionState::onSelectZone(int /*lanes_in_zone*/) {
  if (active_zone_ >= 0) {
#ifdef MUTANT_RESET_LANE
    lane_idx_ = 0;
#endif
    return;
  }
  if (zone_queue_.empty()) {
    return;
  }
  active_zone_ = zone_queue_.front();
  zone_queue_.pop_front();
  lane_idx_ = 0;
}

void MissionState::onAdvanceLane(int lanes_in_zone) {
  at_boundary_ = true;
  ++lane_idx_;
  if (metrics_) {
    ++metrics_->lanes_done;
  }
  (void)lanes_in_zone;
}

void MissionState::onSkipLane() {
  at_boundary_ = true;
  ++lane_idx_;
  if (metrics_) {
    ++metrics_->lanes_skipped;
  }
}

void MissionState::onMarkZoneDone() {
  active_zone_ = -1;
}

void MissionState::onMarkZoneSkipped() {
  active_zone_ = -1;
  if (metrics_) {
    ++metrics_->zones_skipped;
  }
}

void MissionState::onFollowLaneStart() {
  at_boundary_ = false;
}

RunMetrics& MissionState::metrics() {
  static RunMetrics dummy;
  return metrics_ ? *metrics_ : dummy;
}

const RunMetrics& MissionState::metrics() const {
  return const_cast<MissionState*>(this)->metrics();
}

}  // namespace bt_core
