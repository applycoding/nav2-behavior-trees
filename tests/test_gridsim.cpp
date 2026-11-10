// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/grid_sim.hpp"
#include "bt_core/mission_params.hpp"

#include <gtest/gtest.h>

TEST(GridSimWorld, ZoneAndLaneCounts) {
  auto sim = bt_core::GridSim::makeWorld(bt_core::configForScenario(bt_core::ScenarioId::nominal));
  EXPECT_EQ(sim.freeCellsInZone(0), 240);
  EXPECT_EQ(sim.freeCellsInZone(1), 234);
  EXPECT_EQ(sim.freeCellsInZone(2), 231);
  EXPECT_EQ(sim.lanesForZone(0), 7);
  EXPECT_EQ(sim.lanesForZone(1), 7);
  EXPECT_EQ(sim.lanesForZone(2), 5);
}

TEST(GridSimUnit, OverlapRefused) {
  std::vector<std::uint8_t> map(25, 0);
  auto sim = bt_core::GridSim::makeTestGrid(5, 5, map);
  bt_core::GoalArgs wa{};
  wa.wait_sec = 1.0;
  auto a = sim.send(bt_core::GoalKind::Wait, wa);
  ASSERT_TRUE(a);
  auto b = sim.send(bt_core::GoalKind::Wait, wa);
  EXPECT_FALSE(b);
  EXPECT_EQ(sim.goalOverlap(), 1);
}

TEST(GridSimUnit, CancelIdempotent) {
  std::vector<std::uint8_t> map(25, 0);
  auto sim = bt_core::GridSim::makeTestGrid(5, 5, map);
  bt_core::GoalArgs wb{};
  wb.wait_sec = 5.0;
  auto id = *sim.send(bt_core::GoalKind::Wait, wb);
  sim.cancel(id);
  sim.cancel(id);
  EXPECT_EQ(sim.poll(id).status, bt_core::GoalStatus::CANCELED);
}

TEST(GridSimUnit, BatteryClamp) {
  std::vector<std::uint8_t> map(25, 0);
  auto sim = bt_core::GridSim::makeTestGrid(5, 5, map);
  for (int i = 0; i < 10000; ++i) sim.step();
  EXPECT_GT(sim.snapshot().battery, 0);
}
