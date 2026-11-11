// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/frozen_world.hpp"
#include "bt_core/scenario_gates.hpp"
#include "nav2_bt_mission/mock_nav2.hpp"
#include "nav2_bt_mission/ros_nav_backend.hpp"
#include "nav2_bt_mission/ros_runner.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>
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

  nav2_bt_mission::RosNavBackend::useNodeSpinThread(false);

  auto sid = bt_core::scenarioFromName(scenario);
  bt_core::setFrozenWorldScenario(sid);
  auto cfg = bt_core::configForScenario(sid);
  auto sim = std::make_shared<bt_core::GridSim>(bt_core::GridSim::makeWorld(cfg));
  auto mock = std::make_shared<nav2_bt_mission::MockNav2>(rclcpp::NodeOptions(), sim);
  auto backend = std::make_shared<nav2_bt_mission::RosNavBackend>(mock, nullptr);
  backend->setCoupledSim(sim);
  backend->setMockPeer(mock);

  rclcpp::executors::SingleThreadedExecutor exec;
  backend->setSpinExecutor(&exec);
  exec.add_node(mock);

  for (int i = 0; i < 12000; ++i) {
    exec.spin_some();
    if (backend->remoteActionServersReadyAfterSpin()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  const std::string tree_xml = std::string(BT_SOURCE_DIR) + "/trees/mission.xml";
  nav2_bt_mission::RosRunOptions opts;
  opts.executor = &exec;
  opts.skip_remote_ready = true;

  auto metrics = nav2_bt_mission::runMissionRos(sid, tree_xml, *backend, mock, opts);

  std::cout << "success=" << metrics.success << " t=" << metrics.sim_time_sec << " ticks="
            << metrics.ticks << " lanes=" << metrics.lanes_done << " backup=" << metrics.backup
            << " sim_wait=" << metrics.sim_wait << " battery_zero=" << metrics.battery_zero
            << " remote_steps=" << backend->remoteStepCount() << std::endl;

  exec.remove_node(mock);
  exec.cancel();
  mock.reset();
  backend.reset();
  rclcpp::shutdown();

  return metrics.success ? 0 : 1;
}
