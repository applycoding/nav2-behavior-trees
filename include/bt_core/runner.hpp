// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#pragma once

#include "bt_core/grid_sim.hpp"
#include "bt_core/metrics.hpp"
#include "bt_core/mission_context.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <string>

namespace bt_core {

struct RunResult {
  RunMetrics metrics;
};

RunResult runMission(ScenarioId scenario, const std::string& tree_xml_path,
                     const std::string& out_dir, bool write_trace = true);

void registerMissionNodes(BT::BehaviorTreeFactory& factory, MissionContext& ctx);

}  // namespace bt_core
