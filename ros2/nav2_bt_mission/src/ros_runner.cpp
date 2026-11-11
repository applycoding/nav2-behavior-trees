// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "nav2_bt_mission/ros_runner.hpp"

#include "bt_core/mission_context.hpp"
#include "bt_core/mission_params.hpp"
#include "bt_core/runner.hpp"
#include "bt_core/status_change_logger.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <fstream>
#include <thread>

namespace nav2_bt_mission {

bt_core::RunMetrics runMissionRos(bt_core::ScenarioId scenario, const std::string& tree_xml,
                                  RosNavBackend& backend, rclcpp::Node::SharedPtr spin_node,
                                  const RosRunOptions& opts) {
  (void)spin_node;
  if (opts.executor != nullptr) {
    backend.setSpinExecutor(opts.executor);
  }
  if (backend.usesLocalSim()) {
    backend.waitForRemoteReady();
    const bt_core::RunResult rr = bt_core::runMission(scenario, tree_xml, "/tmp/bt_ros_local", false);
    return rr.metrics;
  }
  backend.waitForRemoteReady();
  bt_core::RunMetrics metrics;
  metrics.scenario = bt_core::scenarioName(scenario);

  auto cfg = bt_core::configForScenario(scenario);
  bt_core::MissionState state;
  bt_core::missionStateBindMetrics(state, metrics);
  state.resetZones(cfg.zones);

  static bt_core::MissionParams params;
  bt_core::MissionContext mctx{backend, state, params};
  BT::BehaviorTreeFactory factory;
  bt_core::registerMissionNodes(factory, mctx);

  std::ifstream xml_in(tree_xml);
  std::string xml_text((std::istreambuf_iterator<char>(xml_in)), std::istreambuf_iterator<char>());
  factory.registerBehaviorTreeFromText(xml_text);
  auto tree = factory.createTree("Mission");

  int status_changes_this_tick = 0;
  int tick_var = 0;

  auto logger = std::make_shared<bt_core::MissionStatusChangeLogger>(
      tree.rootNode(), tick_var,
      [&](int, int, std::uint16_t, const std::string&, const std::string&) {
        ++status_changes_this_tick;
      });

  auto spin_quiet = [&]() {
    for (int pass = 0; pass < 8; ++pass) {
      for (int i = 0; i < 32; ++i) {
        rclcpp::spin_some(spin_node);
      }
    }
  };

  BT::NodeStatus root = BT::NodeStatus::RUNNING;
  const int max_ticks = bt_core::MissionParams::MAX_TICKS;
  const bool spin_ros = !backend.usesLocalSim();
  while (root == BT::NodeStatus::RUNNING && backend.tick() < max_ticks) {
    backend.beginMissionTick();
    if (spin_ros && !RosNavBackend::spinUsesNodeThread()) {
      spin_quiet();
    }
    status_changes_this_tick = 0;
    tick_var = backend.tick();
    root = tree.tickExactlyOnce();
    if (spin_ros && !RosNavBackend::spinUsesNodeThread()) {
      spin_quiet();
    }
    backend.step();
    if (spin_ros && !RosNavBackend::spinUsesNodeThread()) {
      spin_quiet();
    }

    bt_core::WorldSample sample = backend.snapshot();
    if (!sample.cov_pct.empty() && sample.battery <= 0) {
      metrics.battery_zero = true;
      break;
    }
    metrics.status_changes_max_per_tick =
        std::max(metrics.status_changes_max_per_tick, status_changes_this_tick);
    if (status_changes_this_tick > bt_core::MissionParams::MAX_STATUS_CHANGES_PER_TICK) {
      break;
    }
  }

  bt_core::WorldSample final_sample = backend.snapshot();
  metrics.ticks = backend.tick();
  metrics.sim_time_sec = backend.simTimeSec();
  metrics.success = (root == BT::NodeStatus::SUCCESS);
  metrics.lanes_done = state.metrics().lanes_done;
  metrics.zones_skipped = state.metrics().zones_skipped;
  metrics.lanes_skipped = state.metrics().lanes_skipped;
  metrics.charge_cycles = state.metrics().charge_cycles;
  metrics.clear_costmap = state.metrics().clear_costmap;
  metrics.sim_wait = state.metrics().sim_wait;
  metrics.backup = state.metrics().backup;
  metrics.halted_follow_lane = state.metrics().halted_follow_lane;
  metrics.ended_docked = final_sample.docked;
  metrics.cov_pct = final_sample.cov_pct;
  (void)logger;
  return metrics;
}

}  // namespace nav2_bt_mission
