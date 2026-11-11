// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "nav2_bt_mission/ros_nav_backend.hpp"
#include "nav2_bt_mission/ros_runner.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "bt_core/frozen_world.hpp"
#include "bt_core/grid_sim.hpp"

#include <iostream>
#include <string>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::string scenario = "nominal";
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--scenario" && i + 1 < argc) {
      scenario = argv[++i];
    }
  }

  auto node = std::make_shared<rclcpp::Node>("bt_mission_node");
  nav2_bt_mission::RosNavBackend::useNodeSpinThread(false);

  auto sid = bt_core::scenarioFromName(scenario);
  bt_core::setFrozenWorldScenario(sid);
  nav2_bt_mission::RosNavBackend backend(node, nullptr);

  std::string pkg_share = ament_index_cpp::get_package_share_directory("nav2_bt_mission");
  std::string tree_xml = pkg_share + "/trees/mission.xml";

  auto metrics =
      nav2_bt_mission::runMissionRos(sid, tree_xml, backend, node, {});

  rclcpp::shutdown();

  std::cout << "success=" << metrics.success << " t=" << metrics.sim_time_sec << " ticks="
            << metrics.ticks << " lanes=" << metrics.lanes_done << " backup=" << metrics.backup
            << " sim_wait=" << metrics.sim_wait << " battery_zero=" << metrics.battery_zero
            << " remote_steps=" << backend.remoteStepCount() << '\n';
  return metrics.success ? 0 : 1;
}
