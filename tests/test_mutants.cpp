// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/runner.hpp"

#include <gtest/gtest.h>

TEST(Mutants, NoBatteryGuardFailsLowBattery) {
  auto m = bt_core::runMission(bt_core::ScenarioId::low_battery,
                               BT_SOURCE_DIR "/trees/mutants/no_battery_guard.xml", "/tmp/m", false)
               .metrics;
  EXPECT_FALSE(m.success);
  EXPECT_TRUE(m.battery_zero);
}

TEST(Mutants, NoBatteryGuardFailsCritical) {
  auto m = bt_core::runMission(bt_core::ScenarioId::critical_battery,
                               BT_SOURCE_DIR "/trees/mutants/no_battery_guard.xml", "/tmp/m", false)
               .metrics;
  EXPECT_FALSE(m.success);
  EXPECT_TRUE(m.battery_zero);
}

TEST(Mutants, NonreactiveFailsCritical) {
  auto m = bt_core::runMission(bt_core::ScenarioId::critical_battery,
                               BT_SOURCE_DIR "/trees/mutants/nonreactive.xml", "/tmp/m", false)
               .metrics;
  EXPECT_FALSE(m.success);
  EXPECT_TRUE(m.battery_zero);
}

TEST(Mutants, NoRecoveryBlockedCorridor) {
  auto m = bt_core::runMission(bt_core::ScenarioId::blocked_corridor,
                               BT_SOURCE_DIR "/trees/mutants/no_recovery.xml", "/tmp/m", false)
               .metrics;
  EXPECT_EQ(m.zones_skipped, 3);
  for (double c : m.cov_pct) EXPECT_EQ(c, 0.0);
}

TEST(Mutants, NoRecoveryStuck) {
  auto m = bt_core::runMission(bt_core::ScenarioId::stuck,
                               BT_SOURCE_DIR "/trees/mutants/no_recovery.xml", "/tmp/m", false)
               .metrics;
  EXPECT_FALSE(m.success);
  EXPECT_GE(m.lanes_skipped, 6);
  EXPECT_GE(m.zones_skipped, 2);
}

TEST(Mutants, NoRecoveryPhantomObstacle) {
  auto m = bt_core::runMission(bt_core::ScenarioId::phantom_obstacle,
                               BT_SOURCE_DIR "/trees/mutants/no_recovery.xml", "/tmp/m", false)
               .metrics;
  EXPECT_EQ(m.zones_skipped, 3);
  for (double c : m.cov_pct) EXPECT_EQ(c, 0.0);
}
