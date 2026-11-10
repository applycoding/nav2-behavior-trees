// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/frozen_world.hpp"

#include "bt_core/grid_sim.hpp"

namespace bt_core {

const GridSim& frozenWorld() {
  static const GridSim world = GridSim::makeWorld(configForScenario(ScenarioId::nominal));
  return world;
}

const GridSim& worldRef(NavBackend& backend) {
  if (auto* live = dynamic_cast<GridSim*>(&backend)) {
    return *live;
  }
  return frozenWorld();
}

}  // namespace bt_core
