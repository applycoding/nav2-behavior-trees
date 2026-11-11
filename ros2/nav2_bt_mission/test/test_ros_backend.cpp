// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/scenario_gates.hpp"
#include "nav2_bt_mission/mock_nav2.hpp"
#include "nav2_bt_mission/ros_nav_backend.hpp"
#include "nav2_bt_mission/ros_runner.hpp"

#include <gtest/gtest.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace {

std::string treePath() { return std::string(BT_SOURCE_DIR) + "/trees/mission.xml"; }

class RosHarness {
 public:
  explicit RosHarness(bt_core::ScenarioId scenario) : scenario_(scenario) {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    auto cfg = bt_core::configForScenario(scenario);
    sim_ = std::make_shared<bt_core::GridSim>(bt_core::GridSim::makeWorld(cfg));
    rclcpp::NodeOptions opts;
    mock_ = std::make_shared<nav2_bt_mission::MockNav2>(opts, sim_);
    client_ =
        std::make_shared<rclcpp::Node>("ros_test_client_" + bt_core::scenarioName(scenario));
    backend_ = std::make_shared<nav2_bt_mission::RosNavBackend>(client_, nullptr);
    exec_.add_node(mock_);
    exec_.add_node(client_);
    nav2_bt_mission::RosNavBackend::useNodeSpinThread(false);
  }

  bt_core::RunMetrics run() {
    nav2_bt_mission::RosRunOptions opts;
    opts.executor = &exec_;
    return nav2_bt_mission::runMissionRos(scenario_, treePath(), *backend_, client_, opts);
  }

  nav2_bt_mission::RosNavBackend& backend() { return *backend_; }

 private:
  bt_core::ScenarioId scenario_;
  std::shared_ptr<bt_core::GridSim> sim_;
  std::shared_ptr<nav2_bt_mission::MockNav2> mock_;
  std::shared_ptr<rclcpp::Node> client_;
  std::shared_ptr<nav2_bt_mission::RosNavBackend> backend_;
  rclcpp::executors::SingleThreadedExecutor exec_;
};

}  // namespace

TEST(RosStack, NominalSemanticGates) {
  RosHarness harness(bt_core::ScenarioId::nominal);
  auto m = harness.run();
  EXPECT_TRUE(m.success);
  for (double c : m.cov_pct) {
    EXPECT_GE(c, 95.0);
  }
  EXPECT_LE(m.sim_time_sec, 400.0);
  EXPECT_EQ(m.charge_cycles, 0);
  EXPECT_EQ(m.lanes_done, 19);
  EXPECT_GT(harness.backend().remoteStepCount(), 0);
  bt_core::test_gates::expectGlobalInvariants(m);
  rclcpp::shutdown();
}

TEST(RosStack, StuckSemanticGates) {
  RosHarness harness(bt_core::ScenarioId::stuck);
  auto m = harness.run();
  EXPECT_TRUE(m.success);
  EXPECT_EQ(m.backup, 1);
  for (double c : m.cov_pct) {
    EXPECT_GE(c, 95.0);
  }
  bt_core::test_gates::expectGlobalInvariants(m);
  rclcpp::shutdown();
}

TEST(RosCriticalStack, CancelBeforeDock) {
  RosHarness harness(bt_core::ScenarioId::critical_battery);
  auto m = harness.run();
  EXPECT_TRUE(m.success);
  EXPECT_EQ(m.halted_follow_lane, 1);
  EXPECT_GE(harness.backend().followPathCanceledBeforeDockCount(), 1);
  bt_core::test_gates::expectGlobalInvariants(m);
  rclcpp::shutdown();
}
