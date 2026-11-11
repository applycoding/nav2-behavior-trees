// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace bt_core {

enum class GoalKind {
  NavigateToPose,
  FollowLane,
  BackUp,
  Wait,
  Dock,
  Undock,
  Charge,
};

enum class GoalStatus { RUNNING, SUCCEEDED, FAILED, CANCELED };

enum class FailReason { NONE, NO_PATH, BLOCKED, STUCK };

struct GoalArgs {
  double target_x{0};
  double target_y{0};
  int lane_zone{-1};
  int lane_index{-1};
  double wait_sec{0};
};

using GoalId = std::uint64_t;

struct GoalState {
  GoalStatus status{GoalStatus::RUNNING};
  FailReason reason{FailReason::NONE};
};

struct WorldSample {
  double x{0};
  double y{0};
  double battery{0};
  bool docked{false};
  bool brushes{false};
  std::vector<double> cov_pct;  // per zone A,B,C
};

class NavBackend {
 public:
  virtual ~NavBackend() = default;

  virtual std::optional<GoalId> send(GoalKind kind, GoalArgs args) = 0;
  virtual GoalState poll(GoalId id) = 0;
  virtual void cancel(GoalId id) = 0;
  virtual void clearCostmap() = 0;
  virtual WorldSample snapshot() const = 0;
  virtual void step() = 0;

  virtual int tick() const = 0;
  virtual double simTimeSec() const = 0;

  virtual bool isStuckCondition() const { return false; }
};

}  // namespace bt_core
