// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/critical_battery_constants.hpp"
#include "bt_core/frozen_world.hpp"
#include "bt_core/nav_backend.hpp"
#include "bt_core/scenario_gates.hpp"
#include "nav2_bt_mission/mock_nav2.hpp"
#include "nav2_bt_mission/ros_nav_backend.hpp"
#include "nav2_bt_mission/ros_runner.hpp"

#include <gtest/gtest.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <stdexcept>
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
  explicit RosHarness(bt_core::ScenarioId scenario, bool reject_all_goals = false)
      : scenario_(scenario) {
    bt_core::setFrozenWorldScenario(scenario);
    auto cfg = bt_core::configForScenario(scenario);
    sim_ = std::make_shared<bt_core::GridSim>(bt_core::GridSim::makeWorld(cfg));
    mock_ = std::make_shared<nav2_bt_mission::MockNav2>(rclcpp::NodeOptions(), sim_, reject_all_goals);
    backend_ = std::make_shared<nav2_bt_mission::RosNavBackend>(mock_, nullptr);
    backend_->setMockPeer(mock_);
    backend_->setSpinExecutor(&exec_);
    exec_.add_node(mock_);
    nav2_bt_mission::RosNavBackend::useNodeSpinThread(false);
    bool ready = false;
    for (int i = 0; i < 12000; ++i) {
      exec_.spin_some();
      if (backend_->remoteActionServersReadyAfterSpin()) {
        ready = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!ready) {
      throw std::runtime_error("mock Nav2 action servers not ready (hard failure)");
    }
  }

  ~RosHarness() { exec_.cancel(); }

  bt_core::RunMetrics run() {
    nav2_bt_mission::RosRunOptions opts;
    opts.executor = &exec_;
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

TEST(RosStack, StuckSemanticGates) {
  RosHarness harness(bt_core::ScenarioId::stuck);
  auto m = harness.run();
  EXPECT_TRUE(m.success);
  EXPECT_EQ(m.backup, 1);
  EXPECT_EQ(m.zones_skipped, 0);
  EXPECT_LE(m.sim_time_sec, 405.0);
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
  EXPECT_EQ(m.charge_cycles, 1);
  EXPECT_EQ(m.lanes_done, 19);
  for (double c : m.cov_pct) {
    EXPECT_GE(c, 95.0);
  }
  EXPECT_GT(m.min_battery, 2.0);
  EXPECT_LE(m.sim_time_sec, 590.0);
  EXPECT_EQ(m.critical_battery_event_tick, bt_core::kCriticalBatteryEventTick);
  EXPECT_EQ(m.follow_lane_halt_tick, bt_core::kCriticalBatteryFollowLaneHaltTick);
  EXPECT_LE(m.follow_lane_halt_tick - m.critical_battery_event_tick, 2);
  EXPECT_EQ(m.next_goal_after_halt_kind, static_cast<int>(bt_core::GoalKind::NavigateToPose));
  EXPECT_EQ(m.next_goal_after_halt_role, static_cast<int>(bt_core::NavPoseRole::Dock));
  EXPECT_GE(harness.backend().followPathCanceledBeforeDockCount(), 1);
  EXPECT_TRUE(harness.backend().followPathCanceledBeforeDockOrdered());
  bt_core::test_gates::expectGlobalInvariants(m);
}

TEST(RosStack, NominalSemanticGates) {
  RosHarness harness(bt_core::ScenarioId::nominal);
  auto m = harness.run();
  EXPECT_TRUE(m.success);
  for (double c : m.cov_pct) {
    EXPECT_GE(c, 95.0);
  }
  EXPECT_LE(m.sim_time_sec, 400.0);
  EXPECT_EQ(m.charge_cycles, 0);
  EXPECT_EQ(m.halted_follow_lane, 0);
  EXPECT_EQ(m.lanes_done, 19);
  EXPECT_GT(harness.backend().remoteStepCount(), 0);
  bt_core::test_gates::expectGlobalInvariants(m);
}

TEST(RosStack, RejectAllGoalsFailsMission) {
  RosHarness harness(bt_core::ScenarioId::nominal, true);
  auto m = harness.run();
  EXPECT_FALSE(m.success);
}
