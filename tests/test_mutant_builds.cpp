// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/runner.hpp"

#include <gtest/gtest.h>

#if defined(MUTANT_NO_CANCEL)
TEST(MutantBuild, NoCancelOverlap) {
  auto m = bt_core::runMission(bt_core::ScenarioId::critical_battery, BT_SOURCE_DIR "/trees/mission.xml",
                               "/tmp/m", false)
               .metrics;
  EXPECT_GE(m.goal_overlap, 1);
}
#endif

#if defined(MUTANT_RESET_LANE)
TEST(MutantBuild, ResetLaneExtraLane) {
  auto m = bt_core::runMission(bt_core::ScenarioId::low_battery, BT_SOURCE_DIR "/trees/mission.xml",
                               "/tmp/m", false)
               .metrics;
  EXPECT_GT(m.lanes_done, 19);
}
#endif
