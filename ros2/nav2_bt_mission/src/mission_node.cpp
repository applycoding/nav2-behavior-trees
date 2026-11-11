// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "nav2_bt_mission/ros_nav_backend.hpp"
#include "nav2_bt_mission/ros_runner.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::string scenario = "nominal";
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--scenario" && i + 1 < argc) {
      scenario = argv[++i];
    }
  }

  auto node = std::make_shared<rclcpp::Node>("bt_mission_node");
  nav2_bt_mission::RosNavBackend backend(node, nullptr);

  std::string pkg_share = ament_index_cpp::get_package_share_directory("nav2_bt_mission");
  std::string tree_xml = pkg_share + "/trees/mission.xml";

  nav2_bt_mission::RosNavBackend::useNodeSpinThread(true);
  std::thread spin_thread([node]() { rclcpp::spin(node); });

  auto sid = bt_core::scenarioFromName(scenario);
  auto metrics =
      nav2_bt_mission::runMissionRos(sid, tree_xml, backend, node, {});

  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  std::cout << "success=" << metrics.success << " t=" << metrics.sim_time_sec << " ticks="
            << metrics.ticks << " lanes=" << metrics.lanes_done << " backup=" << metrics.backup
            << " sim_wait=" << metrics.sim_wait << " battery_zero=" << metrics.battery_zero
            << '\n';
  return metrics.success ? 0 : 1;
}
