// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "nav2_bt_mission/mock_nav2.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::string scenario = "nominal";
  for (int i = 1; i < argc; ++i) {
  if (std::string(argv[i]) == "--scenario" && i + 1 < argc) {
      scenario = argv[++i];
    }
  }
  auto cfg = bt_core::configForScenario(bt_core::scenarioFromName(scenario));
  bt_core::GridSim world = bt_core::GridSim::makeWorld(cfg);
  auto sim = std::make_shared<bt_core::GridSim>(std::move(world));
  auto node = std::make_shared<nav2_bt_mission::MockNav2>(rclcpp::NodeOptions(), sim);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
