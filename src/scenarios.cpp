// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/grid_sim.hpp"

namespace bt_core {

std::string scenarioName(ScenarioId id) {
  switch (id) {
    case ScenarioId::nominal:
      return "nominal";
    case ScenarioId::low_battery:
      return "low_battery";
    case ScenarioId::critical_battery:
      return "critical_battery";
    case ScenarioId::blocked_corridor:
      return "blocked_corridor";
    case ScenarioId::phantom_obstacle:
      return "phantom_obstacle";
    case ScenarioId::stuck:
      return "stuck";
    case ScenarioId::unreachable_zone:
      return "unreachable_zone";
    case ScenarioId::lane_obstacle:
      return "lane_obstacle";
    case ScenarioId::lane_blocked_perm:
      return "lane_blocked_perm";
  }
  return "nominal";
}

ScenarioId scenarioFromName(const std::string& name) {
  if (name == "low_battery") return ScenarioId::low_battery;
  if (name == "critical_battery") return ScenarioId::critical_battery;
  if (name == "blocked_corridor") return ScenarioId::blocked_corridor;
  if (name == "phantom_obstacle") return ScenarioId::phantom_obstacle;
  if (name == "stuck") return ScenarioId::stuck;
  if (name == "unreachable_zone") return ScenarioId::unreachable_zone;
  if (name == "lane_obstacle") return ScenarioId::lane_obstacle;
  if (name == "lane_blocked_perm") return ScenarioId::lane_blocked_perm;
  return ScenarioId::nominal;
}

GridSimConfig configForScenario(ScenarioId id) {
  GridSimConfig c;
  c.scenario = id;
  c.zones = {0, 1, 2};
  switch (id) {
    case ScenarioId::low_battery:
      c.battery0 = 30;
      break;
    default:
      c.battery0 = 100;
      break;
  }
  return c;
}

}  // namespace bt_core
