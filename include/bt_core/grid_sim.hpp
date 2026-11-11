// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#pragma once

#include "bt_core/mission_params.hpp"
#include "bt_core/nav_backend.hpp"

#include <cstdint>
#include <deque>
#include <functional>
#include <string>
#include <vector>

namespace bt_core {

struct Lane {
  int zone{0};
  int index{0};
  int row_y{0};
  double start_x{0};
  double start_y{0};
  double end_x{0};
  double end_y{0};
  bool reverse{false};
};

struct ZoneDef {
  int id{0};
  char name{'?'};
  int x0{0}, y0{0}, x1{0}, y1{0};
  int free_cells{0};
  int lane_count{0};
};

enum class ScenarioId {
  nominal,
  low_battery,
  critical_battery,
  blocked_corridor,
  phantom_obstacle,
  stuck,
  unreachable_zone,
  lane_obstacle,
  lane_blocked_perm,
};

struct GridSimConfig {
  int width{60};
  int height{40};
  double battery0{100};
  std::deque<int> zones{0, 1, 2};
  ScenarioId scenario{ScenarioId::nominal};
};

class GridSim : public NavBackend {
 public:
  static GridSim makeWorld(const GridSimConfig& cfg);

  // Test-only 5x5 grid factory
  static GridSim makeTestGrid(int w, int h,
                              const std::vector<std::uint8_t>& real,
                              const std::vector<std::uint8_t>& cost = {});

  std::optional<GoalId> send(GoalKind kind, GoalArgs args) override;
  GoalState poll(GoalId id) override;
  void cancel(GoalId id) override;
  void clearCostmap() override;
  WorldSample snapshot() const override;
  void step() override;

  int tick() const override { return tick_; }
  double simTimeSec() const override { return tick_ * MissionParams::DT_SEC; }

  const std::vector<ZoneDef>& zones() const { return zones_; }
  const std::vector<Lane>& lanes() const { return lanes_; }
  int dockCellX() const { return dock_x_; }
  int dockCellY() const { return dock_y_; }

  int collisions() const { return collisions_; }
  int goalOverlap() const { return goal_overlap_; }
  int lanesCompleted() const { return lanes_completed_; }
  double minBattery() const { return min_battery_; }

  const std::vector<std::uint8_t>& realMap() const { return real_; }
  const std::vector<std::uint8_t>& costMap() const { return cost_; }

  // Test-only: force movement into occupied cell
  void testForceCollision();

  int lanesForZone(int zone_id) const;
  int freeCellsInZone(int zone_id) const;
  Lane laneAt(int zone_id, int lane_idx) const;

  GoalKind lastSentGoalKind() const { return last_sent_kind_; }
  void onGoalSent(GoalKind k);

  int followLaneHaltTick() const { return follow_lane_halt_tick_; }
  int criticalBatteryEventTick() const { return critical_battery_event_tick_; }
  int nextGoalAfterHaltKind() const { return next_goal_after_halt_kind_; }

  bool isStuckCondition() const override { return last_fail_reason_ == FailReason::STUCK; }

  void recordHaltedFollowLane(int at_tick);
  void recordLaneAdvanced();
  void recordZoneSkipped();
  void recordLaneSkipped();
  void recordChargeCycleComplete();
  void recordClearCostmap();
  void recordSimWait();
  void recordBackup();
  void recordNavigateToZoneAttempt(int zone);
  void recordRecovery();

 private:
  GridSim() = default;
  void buildFrozenWorld();
  void applyScenarioHooks();
  void generateLanes();
  int zoneIndex(char name) const;

  bool cellFreeReal(int x, int y) const;
  bool cellBlockedCost(int x, int y) const;
  bool cellWalkableCost(int x, int y) const;

  struct ActiveGoal {
    GoalId id{0};
    GoalKind kind{GoalKind::NavigateToPose};
    GoalArgs args{};
    GoalStatus status{GoalStatus::RUNNING};
    FailReason reason{FailReason::NONE};
    std::vector<std::pair<int, int>> path;
    std::size_t path_i{0};
    int lane_segment{0};  // 0=to start, 1=along lane
    int wait_ticks_left{0};
    int stuck_no_progress_ticks{0};
    double last_x{0};
    double last_y{0};
    bool brushes{false};
  };

  bool planAStar(double sx, double sy, double gx, double gy,
                 std::vector<std::pair<int, int>>& out);
  void advanceMotion(ActiveGoal& g);
  void applyBattery();
  void markCoverageIfBrushing(int cx, int cy);

  GoalId next_id_{1};
  std::optional<ActiveGoal> active_;
  std::uint64_t canceled_ids_{0};

  std::vector<std::uint8_t> real_;
  std::vector<std::uint8_t> cost_;
  std::vector<std::vector<std::vector<bool>>> covered_;  // [zone][y][x]

  int width_{0};
  int height_{0};
  double x_{0};
  double y_{0};
  double battery_{100};
  bool docked_{true};
  bool charging_{false};
  bool stuck_flag_{false};
  bool stuck_scenario_latched_{false};

  int tick_{0};
  int collisions_{0};
  int goal_overlap_{0};
  int lanes_completed_{0};
  double min_battery_{100};

  std::vector<ZoneDef> zones_;
  std::vector<Lane> lanes_;
  int dock_x_{55};
  int dock_y_{30};

  GridSimConfig cfg_;

  FailReason last_fail_reason_{FailReason::NONE};
  GoalKind last_sent_kind_{GoalKind::Wait};
  int halted_follow_lane_count_{0};
  int follow_lane_halt_tick_{-1};
  int critical_battery_event_tick_{-1};
  int next_goal_after_halt_kind_{-1};

  // Scenario state
  bool doors_ac_closed_{false};
  bool corridor_blocked_{false};
  bool corridor_phantom_{false};
  bool phantom_costmap_cleared_{false};
  int lane_b_obstacle_ticks_{0};
  bool lane_b_perm_{false};
  bool lane_b3_obstacle_armed_{false};
  bool critical_battery_pending_{false};
};

std::string scenarioName(ScenarioId id);
ScenarioId scenarioFromName(const std::string& name);
GridSimConfig configForScenario(ScenarioId id);

}  // namespace bt_core
