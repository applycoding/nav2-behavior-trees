// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/scenario_gates.hpp"
#include "nav2_bt_mission/mock_nav2.hpp"
#include "nav2_bt_mission/ros_nav_backend.hpp"
#include "nav2_bt_mission/ros_runner.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <thread>

namespace {

std::string treePath() { return std::string(BT_SOURCE_DIR) + "/trees/mission.xml"; }

bt_core::RunMetrics runRosScenario(bt_core::ScenarioId scenario) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto cfg = bt_core::configForScenario(scenario);
  bt_core::GridSim world = bt_core::GridSim::makeWorld(cfg);
  auto sim = std::make_shared<bt_core::GridSim>(std::move(world));
  auto client = std::make_shared<rclcpp::Node>("ros_test_client_" + bt_core::scenarioName(scenario));
  nav2_bt_mission::RosNavBackend backend(client, sim);
  auto metrics =
      nav2_bt_mission::runMissionRos(scenario, treePath(), backend, client, {});
  rclcpp::shutdown();
  return metrics;
}

}  // namespace

TEST(RosStack, NominalSemanticGates) {
  auto m = runRosScenario(bt_core::ScenarioId::nominal);
  EXPECT_TRUE(m.success);
  for (double c : m.cov_pct) {
    EXPECT_GE(c, 95.0);
  }
  EXPECT_LE(m.sim_time_sec, 400.0);
  EXPECT_EQ(m.charge_cycles, 0);
  EXPECT_EQ(m.lanes_done, 19);
  bt_core::test_gates::expectGlobalInvariants(m);
}

TEST(RosStack, StuckSemanticGates) {
  auto m = runRosScenario(bt_core::ScenarioId::stuck);
  EXPECT_TRUE(m.success);
  EXPECT_EQ(m.backup, 1);
  for (double c : m.cov_pct) {
    EXPECT_GE(c, 95.0);
  }
  bt_core::test_gates::expectGlobalInvariants(m);
}

TEST(RosCriticalStack, CancelBeforeDock) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto cfg = bt_core::configForScenario(bt_core::ScenarioId::critical_battery);
  bt_core::GridSim world = bt_core::GridSim::makeWorld(cfg);
  auto sim = std::make_shared<bt_core::GridSim>(std::move(world));
  auto client = std::make_shared<rclcpp::Node>("ros_test_client_critical_battery");
  nav2_bt_mission::RosNavBackend backend(client, sim);
  auto m = nav2_bt_mission::runMissionRos(bt_core::ScenarioId::critical_battery, treePath(),
                                          backend, client, {});
  EXPECT_TRUE(m.success);
  EXPECT_EQ(m.halted_follow_lane, 1);
  bt_core::test_gates::expectGlobalInvariants(m);
  rclcpp::shutdown();
}

TEST(RosStack, RemoteDdsNominalSemanticGates) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto cfg = bt_core::configForScenario(bt_core::ScenarioId::nominal);
  auto sim = std::make_shared<bt_core::GridSim>(bt_core::GridSim::makeWorld(cfg));
  auto mock = std::make_shared<nav2_bt_mission::MockNav2>(rclcpp::NodeOptions(), sim);
  std::thread mock_spin([mock]() { rclcpp::spin(mock); });

  auto client = std::make_shared<rclcpp::Node>("ros_test_remote_nominal");
  nav2_bt_mission::RosNavBackend::useNodeSpinThread(true);
  std::thread client_spin([client]() { rclcpp::spin(client); });

  nav2_bt_mission::RosNavBackend backend(client, nullptr);
  backend.waitForRemoteReady(std::chrono::seconds(60));
  auto m = nav2_bt_mission::runMissionRos(bt_core::ScenarioId::nominal, treePath(), backend, client,
                                          {});

  nav2_bt_mission::RosNavBackend::useNodeSpinThread(false);
  rclcpp::shutdown();
  if (mock_spin.joinable()) {
    mock_spin.join();
  }
  if (client_spin.joinable()) {
    client_spin.join();
  }

  EXPECT_TRUE(m.success) << "ticks=" << m.ticks << " t=" << m.sim_time_sec;
  EXPECT_GT(m.ticks, 500) << "mission ended too early";
  EXPECT_LE(m.sim_time_sec, 400.0);
  EXPECT_EQ(m.lanes_done, 19);
  bt_core::test_gates::expectGlobalInvariants(m);
}
