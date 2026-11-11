// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/frozen_world.hpp"
#include "bt_core/scenario_gates.hpp"
#include "nav2_bt_mission/mock_nav2.hpp"
#include "nav2_bt_mission/ros_nav_backend.hpp"
#include "nav2_bt_mission/ros_runner.hpp"

#include <gtest/gtest.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

namespace {

std::string treePath() { return std::string(BT_SOURCE_DIR) + "/trees/mission.xml"; }

class RclcppEnvironment : public ::testing::Environment {
 public:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

const auto* const kRclEnv = ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

class RosHarness {
 public:
  explicit RosHarness(bt_core::ScenarioId scenario) : scenario_(scenario) {
    bt_core::setFrozenWorldScenario(scenario);
    auto cfg = bt_core::configForScenario(scenario);
    sim_ = std::make_shared<bt_core::GridSim>(bt_core::GridSim::makeWorld(cfg));
    mock_ = std::make_shared<nav2_bt_mission::MockNav2>(rclcpp::NodeOptions(), sim_);
    backend_ = std::make_shared<nav2_bt_mission::RosNavBackend>(mock_, nullptr);
    backend_->setCoupledSim(sim_);
    backend_->setMockPeer(mock_);
    backend_->setSpinExecutor(&exec_);
    exec_.add_node(mock_);
    nav2_bt_mission::RosNavBackend::useNodeSpinThread(false);
    for (int i = 0; i < 64; ++i) {
      exec_.spin_some();
    }
    for (int i = 0; i < 12000; ++i) {
      exec_.spin_some();
      if (backend_->remoteActionServersReadyAfterSpin()) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  ~RosHarness() { exec_.cancel(); }

  bt_core::RunMetrics run() {
    nav2_bt_mission::RosRunOptions opts;
    opts.executor = &exec_;
    opts.skip_remote_ready = true;
    return nav2_bt_mission::runMissionRos(scenario_, treePath(), *backend_, mock_, opts);
  }

  nav2_bt_mission::RosNavBackend& backend() { return *backend_; }

 private:
  bt_core::ScenarioId scenario_;
  std::shared_ptr<bt_core::GridSim> sim_;
  std::shared_ptr<nav2_bt_mission::MockNav2> mock_;
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
}

TEST(RosCriticalStack, CancelBeforeDock) {
  RosHarness harness(bt_core::ScenarioId::critical_battery);
  auto m = harness.run();
  EXPECT_TRUE(m.success);
  EXPECT_EQ(m.halted_follow_lane, 1);
  EXPECT_GE(harness.backend().followPathCanceledBeforeDockCount(), 1);
  bt_core::test_gates::expectGlobalInvariants(m);
}
