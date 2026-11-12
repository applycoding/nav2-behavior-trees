// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/frozen_world.hpp"

#include "bt_core/grid_sim.hpp"
#include <unordered_map>

namespace bt_core {

namespace {
ScenarioId g_frozen_scenario{ScenarioId::nominal};
}  // namespace

void setFrozenWorldScenario(ScenarioId scenario) { g_frozen_scenario = scenario; }

const GridSim& frozenWorld() {
  static std::unordered_map<int, GridSim> worlds;
  const int key = static_cast<int>(g_frozen_scenario);
  auto it = worlds.find(key);
  if (it == worlds.end()) {
    auto inserted = worlds.emplace(key, GridSim::makeWorld(configForScenario(g_frozen_scenario)));
    return inserted.first->second;
  }
  return it->second;
}

const GridSim& worldRef(NavBackend& backend) {
  if (auto* live = dynamic_cast<GridSim*>(&backend)) {
    return *live;
  }
  return frozenWorld();
}

}  // namespace bt_core
