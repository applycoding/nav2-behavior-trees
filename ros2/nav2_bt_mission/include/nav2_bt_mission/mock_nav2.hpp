// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#pragma once

#include "bt_core/grid_sim.hpp"

#include "nav2_bt_mission/srv/get_world_sample.hpp"
#include "nav2_bt_mission/srv/sim_step.hpp"

#include <nav2_msgs/action/back_up.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/wait.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <map>
#include <vector>

namespace nav2_bt_mission {

class MockNav2 : public rclcpp::Node {
 public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using FollowPath = nav2_msgs::action::FollowPath;
  using BackUp = nav2_msgs::action::BackUp;
  using Wait = nav2_msgs::action::Wait;

  explicit MockNav2(const rclcpp::NodeOptions& options, std::shared_ptr<bt_core::GridSim> sim,
                    bool reject_all_goals = false);

  std::shared_ptr<bt_core::GridSim> sim() const { return sim_; }
  void stepSim();

  void stepSimPublic() { stepSim(); }

  bt_core::GoalId followPathServerGoalId() const { return last_follow_gid_; }
  bt_core::GoalId lookupGoalId(const rclcpp_action::GoalUUID& uuid) const;
  void cancelGoal(bt_core::GoalId gid);

 private:
  bool parseLaneTag(const std::string& tag, int& zone, int& lane) const;
  void publishActionResults();
  bt_core::GoalId registerGoal(const rclcpp_action::GoalUUID& uuid, bt_core::GoalKind kind,
                               const bt_core::GoalArgs& args);

  std::shared_ptr<bt_core::GridSim> sim_;
  mutable std::mutex sim_mu_;
  std::map<rclcpp_action::GoalUUID, bt_core::GoalId> uuid_to_gid_;
  bt_core::GoalId last_follow_gid_{0};

  struct NavExec {
    std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateToPose>> handle;
    bt_core::GoalId gid{0};
  };
  struct FollowExec {
    std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowPath>> handle;
    bt_core::GoalId gid{0};
  };
  struct BackUpExec {
    std::shared_ptr<rclcpp_action::ServerGoalHandle<BackUp>> handle;
    bt_core::GoalId gid{0};
  };
  struct WaitExec {
    std::shared_ptr<rclcpp_action::ServerGoalHandle<Wait>> handle;
    bt_core::GoalId gid{0};
  };

  std::vector<NavExec> nav_exec_;
  std::vector<FollowExec> follow_exec_;
  std::vector<BackUpExec> backup_exec_;
  std::vector<WaitExec> wait_exec_;

  rclcpp_action::Server<NavigateToPose>::SharedPtr nav_server_;
  rclcpp_action::Server<FollowPath>::SharedPtr follow_server_;
  rclcpp_action::Server<BackUp>::SharedPtr backup_server_;
  rclcpp_action::Server<Wait>::SharedPtr wait_server_;
  rclcpp_action::Server<Wait>::SharedPtr dock_server_;
  rclcpp_action::Server<Wait>::SharedPtr undock_server_;
  rclcpp_action::Server<Wait>::SharedPtr charge_server_;
  rclcpp::Service<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_srv_;
  rclcpp::Service<nav2_bt_mission::srv::SimStep>::SharedPtr step_srv_;
  rclcpp::Service<nav2_bt_mission::srv::GetWorldSample>::SharedPtr sample_srv_;
  bool reject_all_goals_{false};
};

}  // namespace nav2_bt_mission
