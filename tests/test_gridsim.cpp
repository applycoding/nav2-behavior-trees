// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/grid_sim.hpp"
#include "bt_core/mission_params.hpp"

#include <gtest/gtest.h>

#include <cmath>

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

TEST(GridSimUnit, NoCornerCutting) {
  std::vector<std::uint8_t> map(25, 0);
  map[6] = 1;
  auto sim = bt_core::GridSim::makeTestGrid(5, 5, map);
  bt_core::GoalArgs nav{};
  nav.target_x = 3.5;
  nav.target_y = 3.5;
  const double sx = sim.snapshot().x;
  const double sy = sim.snapshot().y;
  auto id = *sim.send(bt_core::GoalKind::NavigateToPose, nav);
  ASSERT_NE(id, 0u);
  for (int i = 0; i < 200; ++i) {
    sim.step();
    if (sim.poll(id).status != bt_core::GoalStatus::RUNNING) break;
  }
  EXPECT_NE(sim.poll(id).status, bt_core::GoalStatus::RUNNING);
  EXPECT_EQ(sim.collisions(), 0);
  const auto end = sim.snapshot();
  EXPECT_GT(end.x, sx);
  EXPECT_GT(end.y, sy);
}

TEST(GridSimUnit, BlockedFailsWithoutMoving) {
  std::vector<std::uint8_t> real(9, 0);
  std::vector<std::uint8_t> cost(9, 0);
  real[1] = 1;
  auto sim = bt_core::GridSim::makeTestGrid(3, 3, real, cost);
  const auto start = sim.snapshot();
  const double sx = start.x;
  const double sy = start.y;
  bt_core::GoalArgs nav{};
  nav.target_x = 2.5;
  nav.target_y = 0.5;
  auto id = *sim.send(bt_core::GoalKind::NavigateToPose, nav);
  bt_core::GoalStatus status = bt_core::GoalStatus::RUNNING;
  bt_core::FailReason reason = bt_core::FailReason::NONE;
  for (int i = 0; i < 5; ++i) {
    sim.step();
    auto st = sim.poll(id);
    status = st.status;
    reason = st.reason;
    if (status != bt_core::GoalStatus::RUNNING) break;
  }
  EXPECT_EQ(status, bt_core::GoalStatus::FAILED);
  EXPECT_EQ(reason, bt_core::FailReason::BLOCKED);
  const auto end = sim.snapshot();
  EXPECT_LT(std::hypot(end.x - sx, end.y - sy), 1.2);
  (void)sy;
}

TEST(GridSimUnit, CollisionIncrements) {
  std::vector<std::uint8_t> map(25, 0);
  map[0] = 1;
  auto sim = bt_core::GridSim::makeTestGrid(5, 5, map);
  sim.testForceCollision();
  EXPECT_GE(sim.collisions(), 1);
}

TEST(GridSimUnit, BatteryFailsAtZero) {
  std::vector<std::uint8_t> map(25, 0);
  auto sim = bt_core::GridSim::makeTestGrid(5, 5, map);
  bt_core::GoalArgs w{};
  w.wait_sec = 10000.0;
  auto id = *sim.send(bt_core::GoalKind::Wait, w);
  for (int i = 0; i < 30000; ++i) {
    sim.step();
    if (sim.snapshot().battery <= 0.0) break;
  }
  EXPECT_LE(sim.snapshot().battery, 0.0);
  (void)id;
}

TEST(GridSimUnit, BatteryClampsAtMaxWhileCharging) {
  std::vector<std::uint8_t> map(25, 0);
  auto sim = bt_core::GridSim::makeTestGrid(5, 5, map);
  auto charge_id = *sim.send(bt_core::GoalKind::Charge, {});
  double mid = sim.snapshot().battery;
  for (int i = 0; i < 2000; ++i) {
    sim.step();
  }
  EXPECT_GE(sim.snapshot().battery, mid);
  EXPECT_LE(sim.snapshot().battery, 100.0);
  sim.cancel(charge_id);
  bt_core::GoalArgs w{};
  w.wait_sec = 50.0;
  auto wait_id = *sim.send(bt_core::GoalKind::Wait, w);
  const double before = sim.snapshot().battery;
  sim.step();
  const double after = sim.snapshot().battery;
  EXPECT_LT(after, before);
  (void)wait_id;
}
