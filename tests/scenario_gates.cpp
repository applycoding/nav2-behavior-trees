// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/scenario_gates.hpp"

#include "bt_core/mission_params.hpp"

#include <gtest/gtest.h>

namespace bt_core {
namespace test_gates {

void expectGlobalInvariants(const RunMetrics& m) {
  EXPECT_EQ(m.collisions, 0);
  EXPECT_EQ(m.goal_overlap, 0);
  EXPECT_FALSE(m.battery_zero);
  EXPECT_TRUE(m.ended_docked);
  EXPECT_LE(m.status_changes_max_per_tick, MissionParams::MAX_STATUS_CHANGES_PER_TICK);
}

}  // namespace test_gates
}  // namespace bt_core
