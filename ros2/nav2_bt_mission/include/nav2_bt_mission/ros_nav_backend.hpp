// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#pragma once

#include "bt_core/grid_sim.hpp"
#include "bt_core/nav_backend.hpp"

#include "nav2_bt_mission/mock_nav2.hpp"
#include "nav2_bt_mission/srv/get_world_sample.hpp"
#include "nav2_bt_mission/srv/sim_step.hpp"

#include <nav2_msgs/action/back_up.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/wait.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <unordered_map>

namespace nav2_bt_mission {

class RosNavBackend : public bt_core::NavBackend {
 public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using FollowPath = nav2_msgs::action::FollowPath;
  using BackUp = nav2_msgs::action::BackUp;
  using Wait = nav2_msgs::action::Wait;

  RosNavBackend(rclcpp::Node::SharedPtr node, std::shared_ptr<bt_core::GridSim> local_sim);

  std::optional<bt_core::GoalId> send(bt_core::GoalKind kind, bt_core::GoalArgs args) override;
  bt_core::GoalState poll(bt_core::GoalId id) override;
  void cancel(bt_core::GoalId id) override;
  void clearCostmap() override;
  bt_core::WorldSample snapshot() const override;
  void step() override;
  int tick() const override;
  double simTimeSec() const override;

  int followPathCanceledBeforeDockCount() const { return follow_canceled_before_dock_; }

  void waitForRemoteReady(std::chrono::milliseconds timeout = std::chrono::seconds(60));

  void setSpinExecutor(rclcpp::Executor* executor) { spin_executor_ = executor; }

  void setMockPeer(const std::shared_ptr<MockNav2>& mock);
  void setCoupledSim(std::shared_ptr<bt_core::GridSim> sim);

  bool usesLocalSim() const { return static_cast<bool>(local_sim_); }

 private:
  struct GoalRecord {
    bt_core::GoalKind kind{bt_core::GoalKind::Wait};
    std::uint64_t generation{0};
    bt_core::GoalStatus status{bt_core::GoalStatus::RUNNING};
    bt_core::FailReason reason{bt_core::FailReason::NONE};
    bool cancel_pending{false};
    rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr nav_handle;
    rclcpp_action::ClientGoalHandle<FollowPath>::SharedPtr follow_handle;
    rclcpp_action::ClientGoalHandle<BackUp>::SharedPtr backup_handle;
    rclcpp_action::ClientGoalHandle<Wait>::SharedPtr wait_handle;
    bt_core::GoalId sim_goal_id{0};
  };

  template <typename ActionT>
  void spinWaitForGoalHandle(const typename rclcpp_action::Client<ActionT>::SharedPtr& client,
                             GoalRecord& rec);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<bt_core::GridSim> local_sim_;
  rclcpp::Client<nav2_bt_mission::srv::SimStep>::SharedPtr step_client_;
  rclcpp::Client<nav2_bt_mission::srv::GetWorldSample>::SharedPtr sample_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_client_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;
  rclcpp_action::Client<BackUp>::SharedPtr backup_client_;
  rclcpp_action::Client<Wait>::SharedPtr wait_client_;
  rclcpp_action::Client<Wait>::SharedPtr dock_client_;
  rclcpp_action::Client<Wait>::SharedPtr undock_client_;
  rclcpp_action::Client<Wait>::SharedPtr charge_client_;

  std::unordered_map<bt_core::GoalId, GoalRecord> goals_;
  bt_core::GoalId next_id_{1};
  mutable std::mutex mu_;
  int follow_canceled_before_dock_{0};
  bool dock_goal_after_follow_cancel_{false};
  mutable int remote_tick_{0};
  mutable bt_core::WorldSample last_remote_sample_{};
  mutable bool has_remote_sample_{false};
  rclcpp::Executor* spin_executor_{nullptr};
  std::weak_ptr<MockNav2> mock_peer_;
  std::shared_ptr<bt_core::GridSim> coupled_sim_;
};

}  // namespace nav2_bt_mission
