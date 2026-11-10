// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/runner.hpp"
#include "bt_core/scenario_gates.hpp"

#include <gtest/gtest.h>

namespace {

bt_core::RunMetrics runProd(bt_core::ScenarioId id) {
  return bt_core::runMission(id, BT_SOURCE_DIR "/trees/mission.xml", "/tmp/bt_test_out", false).metrics;
}

}  // namespace

TEST(Scenarios, Nominal) {
  auto m = runProd(bt_core::ScenarioId::nominal);
  EXPECT_TRUE(m.success);
  for (double c : m.cov_pct) EXPECT_GE(c, 95.0);
  EXPECT_LE(m.sim_time_sec, 400.0);
  EXPECT_EQ(m.charge_cycles, 0);
  EXPECT_GE(m.min_battery, 50.0);
  EXPECT_EQ(m.lanes_done, 19);
  bt_core::test_gates::expectGlobalInvariants(m);
}

TEST(Scenarios, LowBattery) {
  auto m = runProd(bt_core::ScenarioId::low_battery);
  EXPECT_TRUE(m.success);
  EXPECT_EQ(m.charge_cycles, 1);
  EXPECT_EQ(m.halted_follow_lane, 0);
  EXPECT_EQ(m.lanes_done, 19);
  for (double c : m.cov_pct) EXPECT_GE(c, 95.0);
  EXPECT_GE(m.min_battery, 15.0);
  EXPECT_LE(m.sim_time_sec, 560.0);
  bt_core::test_gates::expectGlobalInvariants(m);
}

TEST(Scenarios, CriticalBattery) {
  auto m = runProd(bt_core::ScenarioId::critical_battery);
  EXPECT_TRUE(m.success);
  EXPECT_EQ(m.halted_follow_lane, 1);
  EXPECT_EQ(m.charge_cycles, 1);
  EXPECT_EQ(m.lanes_done, 19);
  for (double c : m.cov_pct) EXPECT_GE(c, 95.0);
  EXPECT_GT(m.min_battery, 2.0);
  EXPECT_LE(m.sim_time_sec, 590.0);
  bt_core::test_gates::expectGlobalInvariants(m);
}

TEST(Scenarios, BlockedCorridor) {
  auto m = runProd(bt_core::ScenarioId::blocked_corridor);
  EXPECT_TRUE(m.success);
  EXPECT_GE(m.clear_costmap, 1);
  EXPECT_GE(m.sim_wait, 1);
  EXPECT_EQ(m.zones_skipped, 0);
  for (double c : m.cov_pct) EXPECT_GE(c, 95.0);
  EXPECT_LE(m.sim_time_sec, 410.0);
  bt_core::test_gates::expectGlobalInvariants(m);
}

TEST(Scenarios, PhantomObstacle) {
  auto m = runProd(bt_core::ScenarioId::phantom_obstacle);
  EXPECT_TRUE(m.success);
  EXPECT_EQ(m.clear_costmap, 1);
  EXPECT_EQ(m.zones_skipped, 0);
  for (double c : m.cov_pct) EXPECT_GE(c, 95.0);
  EXPECT_LE(m.sim_time_sec, 405.0);
  bt_core::test_gates::expectGlobalInvariants(m);
}

TEST(Scenarios, Stuck) {
  auto m = runProd(bt_core::ScenarioId::stuck);
  EXPECT_TRUE(m.success);
  EXPECT_EQ(m.backup, 1);
  EXPECT_EQ(m.zones_skipped, 0);
  for (double c : m.cov_pct) EXPECT_GE(c, 95.0);
  EXPECT_LE(m.sim_time_sec, 405.0);
  bt_core::test_gates::expectGlobalInvariants(m);
}

TEST(Scenarios, UnreachableZone) {
  auto m = runProd(bt_core::ScenarioId::unreachable_zone);
  EXPECT_TRUE(m.success);
  EXPECT_EQ(m.zones_skipped, 1);
  EXPECT_GE(m.cov_pct[0], 95.0);
  EXPECT_GE(m.cov_pct[1], 95.0);
  EXPECT_EQ(m.lanes_done, 14);
  bt_core::test_gates::expectGlobalInvariants(m);
}

TEST(Scenarios, LaneObstacle) {
  auto m = runProd(bt_core::ScenarioId::lane_obstacle);
  EXPECT_TRUE(m.success);
  EXPECT_EQ(m.lanes_skipped, 0);
  EXPECT_GE(m.sim_wait, 1);
  for (double c : m.cov_pct) EXPECT_GE(c, 95.0);
  bt_core::test_gates::expectGlobalInvariants(m);
}

TEST(Scenarios, LaneBlockedPerm) {
  auto m = runProd(bt_core::ScenarioId::lane_blocked_perm);
  EXPECT_TRUE(m.success);
  EXPECT_EQ(m.lanes_skipped, 1);
  EXPECT_EQ(m.lanes_done, 18);
  EXPECT_GE(m.cov_pct[1], 90.0);
  EXPECT_GE(m.cov_pct[0], 95.0);
  EXPECT_GE(m.cov_pct[2], 95.0);
  bt_core::test_gates::expectGlobalInvariants(m);
}
