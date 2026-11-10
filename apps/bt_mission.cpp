// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/grid_sim.hpp"
#include "bt_core/runner.hpp"

#include <iostream>
#include <string>

int main(int argc, char** argv) {
  std::string scenario = "nominal";
  std::string tree = "trees/mission.xml";
  std::string out = "traces/out";

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--scenario" && i + 1 < argc) scenario = argv[++i];
    else if (a == "--tree" && i + 1 < argc) tree = argv[++i];
    else if (a == "--out" && i + 1 < argc) out = argv[++i];
  }

  bt_core::ScenarioId sid = bt_core::scenarioFromName(scenario);
  auto result = bt_core::runMission(sid, tree, out, true);
  std::cout << "success=" << result.metrics.success << " t=" << result.metrics.sim_time_sec << '\n';
  return result.metrics.success ? 0 : 1;
}
