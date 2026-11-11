// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "nav2_bt_mission/mock_nav2.hpp"

#include <algorithm>
#include <sstream>

namespace nav2_bt_mission {

namespace {
constexpr char kGlobalCostmapClear[] = "/global_costmap/clear_entirely_global_costmap";
}  // namespace

MockNav2::MockNav2(const rclcpp::NodeOptions& options, std::shared_ptr<bt_core::GridSim> sim,
                   bool reject_all_goals)
    : Node("mock_nav2_servers", options), sim_(std::move(sim)),
      reject_all_goals_(reject_all_goals) {
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;
  using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;
  using GoalHandleBackUp = rclcpp_action::ServerGoalHandle<BackUp>;
  using GoalHandleWait = rclcpp_action::ServerGoalHandle<Wait>;

  nav_server_ = rclcpp_action::create_server<NavigateToPose>(
      this, "navigate_to_pose",
      [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const NavigateToPose::Goal> goal) {
        if (reject_all_goals_) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        std::lock_guard<std::mutex> lock(sim_mu_);
        bt_core::GoalArgs args{};
        args.target_x = goal->pose.pose.position.x;
        args.target_y = goal->pose.pose.position.y;
        const double dx = args.target_x - (static_cast<double>(sim_->dockCellX()) + 0.5);
        const double dy = args.target_y - (static_cast<double>(sim_->dockCellY()) + 0.5);
        args.nav_role = (dx * dx + dy * dy < 0.01) ? bt_core::NavPoseRole::Dock : bt_core::NavPoseRole::Zone;
        bt_core::GoalId gid = registerGoal(uuid, bt_core::GoalKind::NavigateToPose, args);
        if (gid == 0) {
          RCLCPP_ERROR(get_logger(), "reject NavigateToPose goal (sim busy or failed to start)");
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<GoalHandleNavigateToPose> handle) {
        std::lock_guard<std::mutex> lock(sim_mu_);
        auto it = uuid_to_gid_.find(handle->get_goal_id());
        if (it != uuid_to_gid_.end()) {
          sim_->cancel(it->second);
        }
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<GoalHandleNavigateToPose> handle) {
        std::lock_guard<std::mutex> lock(sim_mu_);
        auto it = uuid_to_gid_.find(handle->get_goal_id());
        if (it == uuid_to_gid_.end()) {
          return;
        }
        nav_exec_.push_back({handle, it->second});
      });

  follow_server_ = rclcpp_action::create_server<FollowPath>(
      this, "follow_path",
      [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const FollowPath::Goal> goal) {
        if (reject_all_goals_) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        int zone = 0;
        int lane = 0;
        if (!parseLaneTag(goal->controller_id, zone, lane)) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        std::lock_guard<std::mutex> lock(sim_mu_);
        bt_core::GoalArgs args{};
        args.lane_zone = zone;
        args.lane_index = lane;
        bt_core::GoalId gid = registerGoal(uuid, bt_core::GoalKind::FollowLane, args);
        if (gid == 0) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        last_follow_gid_ = gid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<GoalHandleFollowPath> handle) {
        std::lock_guard<std::mutex> lock(sim_mu_);
        auto it = uuid_to_gid_.find(handle->get_goal_id());
        if (it != uuid_to_gid_.end()) {
          sim_->cancel(it->second);
        }
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<GoalHandleFollowPath> handle) {
        std::lock_guard<std::mutex> lock(sim_mu_);
        auto it = uuid_to_gid_.find(handle->get_goal_id());
        if (it == uuid_to_gid_.end()) {
          return;
        }
        follow_exec_.push_back({handle, it->second});
      });

  backup_server_ = rclcpp_action::create_server<BackUp>(
      this, "backup",
      [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const BackUp::Goal> goal) {
        (void)goal;
        if (reject_all_goals_) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        std::lock_guard<std::mutex> lock(sim_mu_);
        bt_core::GoalId gid = registerGoal(uuid, bt_core::GoalKind::BackUp, {});
        if (gid == 0) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<GoalHandleBackUp> handle) {
        std::lock_guard<std::mutex> lock(sim_mu_);
        auto it = uuid_to_gid_.find(handle->get_goal_id());
        if (it != uuid_to_gid_.end()) {
          sim_->cancel(it->second);
        }
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<GoalHandleBackUp> handle) {
        std::lock_guard<std::mutex> lock(sim_mu_);
        auto it = uuid_to_gid_.find(handle->get_goal_id());
        if (it == uuid_to_gid_.end()) {
          return;
        }
        backup_exec_.push_back({handle, it->second});
      });

  auto make_wait = [this](const std::string& name, bt_core::GoalKind kind,
                          rclcpp_action::Server<Wait>::SharedPtr& slot) {
    slot = rclcpp_action::create_server<Wait>(
        this, name,
        [this, kind, name](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Wait::Goal> goal) {
          if (reject_all_goals_) {
            return rclcpp_action::GoalResponse::REJECT;
          }
          std::lock_guard<std::mutex> lock(sim_mu_);
          bt_core::GoalArgs args{};
          if (kind == bt_core::GoalKind::Wait) {
            args.wait_sec = static_cast<double>(goal->time.sec) +
                            static_cast<double>(goal->time.nanosec) * 1e-9;
          }
          bt_core::GoalId gid = registerGoal(uuid, kind, args);
          if (gid == 0) {
            RCLCPP_ERROR(get_logger(), "reject %s goal (sim busy or failed to start)", name.c_str());
            return rclcpp_action::GoalResponse::REJECT;
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<GoalHandleWait> handle) {
          std::lock_guard<std::mutex> lock(sim_mu_);
          auto it = uuid_to_gid_.find(handle->get_goal_id());
          if (it != uuid_to_gid_.end()) {
            sim_->cancel(it->second);
          }
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<GoalHandleWait> handle) {
          std::lock_guard<std::mutex> lock(sim_mu_);
          auto it = uuid_to_gid_.find(handle->get_goal_id());
          if (it == uuid_to_gid_.end()) {
            return;
          }
          wait_exec_.push_back({handle, it->second});
        });
  };

  make_wait("wait", bt_core::GoalKind::Wait, wait_server_);
  make_wait("/mock_dock/dock", bt_core::GoalKind::Dock, dock_server_);
  make_wait("/mock_dock/undock", bt_core::GoalKind::Undock, undock_server_);
  make_wait("/mock_dock/charge", bt_core::GoalKind::Charge, charge_server_);

  clear_srv_ = create_service<nav2_msgs::srv::ClearEntireCostmap>(
      kGlobalCostmapClear,
      [this](const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Request>,
             std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Response>) {
        std::lock_guard<std::mutex> lock(sim_mu_);
        sim_->clearCostmap();
      });

  step_srv_ = create_service<nav2_bt_mission::srv::SimStep>(
      "~/step",
      [this](const std::shared_ptr<nav2_bt_mission::srv::SimStep::Request>,
             std::shared_ptr<nav2_bt_mission::srv::SimStep::Response> response) {
        stepSim();
        response->ok = true;
      });
  sample_srv_ = create_service<nav2_bt_mission::srv::GetWorldSample>(
      "~/sample",
      [this](const std::shared_ptr<nav2_bt_mission::srv::GetWorldSample::Request>,
             std::shared_ptr<nav2_bt_mission::srv::GetWorldSample::Response> response) {
        std::lock_guard<std::mutex> lock(sim_mu_);
        auto s = sim_->snapshot();
        response->x = s.x;
        response->y = s.y;
        response->battery = s.battery;
        response->docked = s.docked;
        response->brushes = s.brushes;
        for (std::size_t i = 0; i < 3 && i < s.cov_pct.size(); ++i) {
          response->cov_pct[i] = s.cov_pct[i];
        }
        response->tick = sim_->tick();
        response->collisions = sim_->collisions();
        response->goal_overlap = sim_->goalOverlap();
        response->min_battery = sim_->minBattery();
      });
}

bt_core::GoalId MockNav2::registerGoal(const rclcpp_action::GoalUUID& uuid, bt_core::GoalKind kind,
                                       const bt_core::GoalArgs& args) {
  auto gid = sim_->send(kind, args);
  if (!gid) {
    return 0;
  }
  uuid_to_gid_[uuid] = *gid;
  return *gid;
}

void MockNav2::cancelGoal(bt_core::GoalId gid) { sim_->cancel(gid); }

bt_core::GoalId MockNav2::lookupGoalId(const rclcpp_action::GoalUUID& uuid) const {
  std::lock_guard<std::mutex> lock(sim_mu_);
  auto it = uuid_to_gid_.find(uuid);
  if (it == uuid_to_gid_.end()) {
    return 0;
  }
  return it->second;
}

bool MockNav2::parseLaneTag(const std::string& tag, int& zone, int& lane) const {
  if (tag.rfind("bt_lane:", 0) != 0) {
    return false;
  }
  std::istringstream in(tag.substr(8));
  char sep = 0;
  if (!(in >> zone)) {
    return false;
  }
  if (!(in >> sep) || sep != ':') {
    return false;
  }
  return static_cast<bool>(in >> lane);
}

void MockNav2::stepSim() {
  std::lock_guard<std::mutex> lock(sim_mu_);
  sim_->step();
  publishActionResults();
}

void MockNav2::publishActionResults() {
  auto settle = [this](auto& list, auto make_result) {
    list.erase(std::remove_if(list.begin(), list.end(),
                              [&](auto& item) {
                                auto st = sim_->poll(item.gid);
                                if (st.status == bt_core::GoalStatus::RUNNING) {
                                  return false;
                                }
                                if (st.status == bt_core::GoalStatus::SUCCEEDED) {
                                  item.handle->succeed(make_result());
                                } else if (st.status == bt_core::GoalStatus::CANCELED) {
                                  item.handle->canceled(make_result());
                                } else {
                                  item.handle->abort(make_result());
                                }
                                return true;
                              }),
                   list.end());
  };

  settle(nav_exec_, [] { return std::make_shared<NavigateToPose::Result>(); });
  settle(follow_exec_, [] { return std::make_shared<FollowPath::Result>(); });
  settle(backup_exec_, [] { return std::make_shared<BackUp::Result>(); });
  settle(wait_exec_, [] { return std::make_shared<Wait::Result>(); });
}

}  // namespace nav2_bt_mission
