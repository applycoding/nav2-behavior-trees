// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#pragma once

#include "bt_core/grid_sim.hpp"
#include "bt_core/metrics.hpp"
#include "nav2_bt_mission/ros_nav_backend.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace nav2_bt_mission {

struct RosRunOptions {
  bool write_trace{false};
  std::string out_dir;
  rclcpp::Executor* executor{nullptr};
  bool skip_remote_ready{false};
};

bt_core::RunMetrics runMissionRos(bt_core::ScenarioId scenario, const std::string& tree_xml,
                                 RosNavBackend& backend, rclcpp::Node::SharedPtr spin_node,
                                 const RosRunOptions& opts = {});

}  // namespace nav2_bt_mission
