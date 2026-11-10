// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "nav2_bt_mission/ros_nav_backend.hpp"

#include "nav2_bt_mission/srv/get_world_sample.hpp"
#include "nav2_bt_mission/srv/sim_step.hpp"

#include "bt_core/mission_params.hpp"

#include <action_msgs/msg/goal_status.hpp>
#include <chrono>
#include <thread>

namespace nav2_bt_mission {

namespace {

template <typename FutureT>
bool waitWithSpin(FutureT& fut, const rclcpp::Node::SharedPtr& node, rclcpp::Executor* exec,
                  std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    if (fut.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      return true;
    }
  if (exec != nullptr) {
      exec->spin_some();
    } else {
      rclcpp::spin_some(node);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return fut.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
}

bt_core::GoalStatus statusFromAction(int status) {
  switch (status) {
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      return bt_core::GoalStatus::SUCCEEDED;
    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      return bt_core::GoalStatus::CANCELED;
    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      return bt_core::GoalStatus::FAILED;
    default:
      return bt_core::GoalStatus::RUNNING;
  }
}

template <typename HandleT>
bt_core::GoalStatus statusFromHandle(const HandleT& handle) {
  if (!handle) {
    return bt_core::GoalStatus::FAILED;
  }
  return statusFromAction(handle->get_status());
}

geometry_msgs::msg::PoseStamped makePose(double x, double y) {
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.w = 1.0;
  return p;
}

void spinBackend(const rclcpp::Node::SharedPtr& node, rclcpp::Executor* exec) {
  if (exec != nullptr) {
    exec->spin_some();
  } else {
    rclcpp::spin_some(node);
  }
}

}  // namespace

void RosNavBackend::setMockPeer(const std::shared_ptr<MockNav2>& mock) { mock_peer_ = mock; }

void RosNavBackend::setCoupledSim(std::shared_ptr<bt_core::GridSim> sim) { coupled_sim_ = std::move(sim); }

RosNavBackend::RosNavBackend(rclcpp::Node::SharedPtr node, std::shared_ptr<bt_core::GridSim> local_sim)
    : node_(std::move(node)), local_sim_(std::move(local_sim)) {
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "/mock_nav2_servers/navigate_to_pose");
  follow_client_ =
      rclcpp_action::create_client<FollowPath>(node_, "/mock_nav2_servers/follow_path");
  backup_client_ = rclcpp_action::create_client<BackUp>(node_, "/mock_nav2_servers/backup");
  wait_client_ = rclcpp_action::create_client<Wait>(node_, "/mock_nav2_servers/wait");
  dock_client_ = rclcpp_action::create_client<Wait>(node_, "/mock_dock/dock");
  undock_client_ = rclcpp_action::create_client<Wait>(node_, "/mock_dock/undock");
  charge_client_ = rclcpp_action::create_client<Wait>(node_, "/mock_dock/charge");
  clear_client_ = node_->create_client<nav2_msgs::srv::ClearEntireCostmap>(
      "/global_costmap/clear_entirely_global_costmap");
  if (!local_sim_) {
    step_client_ = node_->create_client<nav2_bt_mission::srv::SimStep>("/mock_nav2_servers/step");
    sample_client_ =
        node_->create_client<nav2_bt_mission::srv::GetWorldSample>("/mock_nav2_servers/sample");
    last_remote_sample_.battery = 100.0;
    last_remote_sample_.cov_pct = {0.0, 0.0, 0.0};
    has_remote_sample_ = false;
  }
}

std::optional<bt_core::GoalId> RosNavBackend::send(bt_core::GoalKind kind, bt_core::GoalArgs args) {
  if (local_sim_) {
    std::lock_guard<std::mutex> lock(mu_);
    return local_sim_->send(kind, args);
  }

  bt_core::GoalId id{};
  {
    std::lock_guard<std::mutex> lock(mu_);
    id = next_id_++;
    GoalRecord rec;
    rec.kind = kind;
    goals_[id] = rec;
  }

  auto mark_failed = [this, id]() {
    std::lock_guard<std::mutex> g(mu_);
    auto it = goals_.find(id);
    if (it != goals_.end()) {
      it->second.status = bt_core::GoalStatus::FAILED;
    }
  };

  const auto result_cb = [this, id, kind](const rclcpp_action::ResultCode code) {
    std::lock_guard<std::mutex> g(mu_);
    auto it = goals_.find(id);
    if (it == goals_.end()) {
      return;
    }
    if (code == rclcpp_action::ResultCode::SUCCEEDED) {
      it->second.status = bt_core::GoalStatus::SUCCEEDED;
    } else if (code == rclcpp_action::ResultCode::CANCELED) {
      it->second.status = bt_core::GoalStatus::CANCELED;
      if (kind == bt_core::GoalKind::FollowLane) {
        dock_goal_after_follow_cancel_ = true;
      }
    } else {
      it->second.status = bt_core::GoalStatus::FAILED;
    }
    if (kind == bt_core::GoalKind::NavigateToPose && dock_goal_after_follow_cancel_) {
      ++follow_canceled_before_dock_;
      dock_goal_after_follow_cancel_ = false;
    }
  };

  auto store_nav_handle = [this, id, mark_failed](
                              const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr& handle) {
    if (!handle) {
      mark_failed();
      return;
    }
    std::lock_guard<std::mutex> lock(mu_);
    auto it = goals_.find(id);
    if (it != goals_.end()) {
      it->second.nav_handle = handle;
    }
  };

  switch (kind) {
    case bt_core::GoalKind::NavigateToPose: {
      NavigateToPose::Goal goal;
      goal.pose = makePose(args.target_x, args.target_y);
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
      opts.goal_response_callback = [store_nav_handle](
          const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr handle) {
        store_nav_handle(handle);
      };
      opts.result_callback = [result_cb](const auto& wrapped) { result_cb(wrapped.code); };
      nav_client_->async_send_goal(goal, opts);
      break;
    }
    case bt_core::GoalKind::FollowLane: {
      FollowPath::Goal goal;
      goal.controller_id =
          "bt_lane:" + std::to_string(args.lane_zone) + ":" + std::to_string(args.lane_index);
      goal.goal_checker_id = "default";
      rclcpp_action::Client<FollowPath>::SendGoalOptions opts;
      opts.goal_response_callback = [this, id, mark_failed](
          const rclcpp_action::ClientGoalHandle<FollowPath>::SharedPtr handle) {
        if (!handle) {
          mark_failed();
          return;
        }
        std::lock_guard<std::mutex> lock(mu_);
        goals_[id].follow_handle = handle;
      };
      opts.result_callback = [result_cb](const auto& wrapped) { result_cb(wrapped.code); };
      follow_client_->async_send_goal(goal, opts);
      break;
    }
    case bt_core::GoalKind::BackUp: {
      BackUp::Goal goal;
      goal.speed = 0.2f;
      rclcpp_action::Client<BackUp>::SendGoalOptions opts;
      opts.goal_response_callback = [this, id, mark_failed](
          const rclcpp_action::ClientGoalHandle<BackUp>::SharedPtr handle) {
        if (!handle) {
          mark_failed();
          return;
        }
        std::lock_guard<std::mutex> lock(mu_);
        goals_[id].backup_handle = handle;
      };
      opts.result_callback = [result_cb](const auto& wrapped) { result_cb(wrapped.code); };
      backup_client_->async_send_goal(goal, opts);
      break;
    }
    case bt_core::GoalKind::Wait: {
      Wait::Goal goal;
      goal.time = rclcpp::Duration::from_seconds(args.wait_sec);
      rclcpp_action::Client<Wait>::SendGoalOptions opts;
      opts.goal_response_callback = [this, id, mark_failed](
          const rclcpp_action::ClientGoalHandle<Wait>::SharedPtr handle) {
        if (!handle) {
          mark_failed();
          return;
        }
        std::lock_guard<std::mutex> lock(mu_);
        goals_[id].wait_handle = handle;
      };
      opts.result_callback = [result_cb](const auto& wrapped) { result_cb(wrapped.code); };
      wait_client_->async_send_goal(goal, opts);
      break;
    }
    case bt_core::GoalKind::Dock: {
      rclcpp_action::Client<Wait>::SendGoalOptions opts;
      opts.goal_response_callback = [this, id, mark_failed](
          const rclcpp_action::ClientGoalHandle<Wait>::SharedPtr handle) {
        if (!handle) {
          mark_failed();
          return;
        }
        std::lock_guard<std::mutex> lock(mu_);
        goals_[id].wait_handle = handle;
      };
      opts.result_callback = [result_cb](const auto& wrapped) { result_cb(wrapped.code); };
      dock_client_->async_send_goal(Wait::Goal{}, opts);
      break;
    }
    case bt_core::GoalKind::Undock: {
      rclcpp_action::Client<Wait>::SendGoalOptions opts;
      opts.goal_response_callback = [this, id, mark_failed](
          const rclcpp_action::ClientGoalHandle<Wait>::SharedPtr handle) {
        if (!handle) {
          mark_failed();
          return;
        }
        std::lock_guard<std::mutex> lock(mu_);
        goals_[id].wait_handle = handle;
      };
      opts.result_callback = [result_cb](const auto& wrapped) { result_cb(wrapped.code); };
      undock_client_->async_send_goal(Wait::Goal{}, opts);
      break;
    }
    case bt_core::GoalKind::Charge: {
      rclcpp_action::Client<Wait>::SendGoalOptions opts;
      opts.goal_response_callback = [this, id, mark_failed](
          const rclcpp_action::ClientGoalHandle<Wait>::SharedPtr handle) {
        if (!handle) {
          mark_failed();
          return;
        }
        std::lock_guard<std::mutex> lock(mu_);
        goals_[id].wait_handle = handle;
      };
      opts.result_callback = [result_cb](const auto& wrapped) { result_cb(wrapped.code); };
      charge_client_->async_send_goal(Wait::Goal{}, opts);
      break;
    }
    default: {
      std::lock_guard<std::mutex> lock(mu_);
      goals_.erase(id);
      return std::nullopt;
    }
  }

  const auto handle_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < handle_deadline) {
    spinBackend(node_, spin_executor_);
    std::lock_guard<std::mutex> lock(mu_);
    auto it = goals_.find(id);
    if (it == goals_.end()) {
      return std::nullopt;
    }
    if (it->second.status == bt_core::GoalStatus::FAILED) {
      return std::nullopt;
    }
    const GoalRecord& rec = it->second;
    const bool has_handle = static_cast<bool>(rec.nav_handle) || static_cast<bool>(rec.follow_handle) ||
                            static_cast<bool>(rec.backup_handle) || static_cast<bool>(rec.wait_handle);
    if (has_handle) {
      return id;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  mark_failed();
  return std::nullopt;
}

bt_core::GoalState RosNavBackend::poll(bt_core::GoalId id) {
  if (!local_sim_) {
    spinBackend(node_, spin_executor_);
  }
  std::lock_guard<std::mutex> lock(mu_);
  if (local_sim_) {
    return local_sim_->poll(id);
  }
  auto it = goals_.find(id);
  if (it == goals_.end()) {
    return {bt_core::GoalStatus::FAILED, bt_core::FailReason::NONE};
  }
  GoalRecord& rec = it->second;
  if (coupled_sim_ && rec.sim_goal_id != 0) {
    auto sim_st = coupled_sim_->poll(rec.sim_goal_id);
    if (sim_st.status != bt_core::GoalStatus::RUNNING) {
      rec.status = sim_st.status;
      rec.reason = sim_st.reason;
    }
  }
  if (rec.status == bt_core::GoalStatus::RUNNING) {
    if (rec.nav_handle) {
      rec.status = statusFromHandle(rec.nav_handle);
    } else if (rec.follow_handle) {
      rec.status = statusFromHandle(rec.follow_handle);
    } else if (rec.backup_handle) {
      rec.status = statusFromHandle(rec.backup_handle);
    } else if (rec.wait_handle) {
      rec.status = statusFromHandle(rec.wait_handle);
    }
  }
  return {rec.status, rec.reason};
}

void RosNavBackend::cancel(bt_core::GoalId id) {
  if (local_sim_) {
    std::lock_guard<std::mutex> lock(mu_);
    local_sim_->cancel(id);
    return;
  }
  bool cancel_follow{false};
  {
    std::lock_guard<std::mutex> lock(mu_);
    auto it = goals_.find(id);
    if (it == goals_.end()) {
      return;
    }
    cancel_follow = it->second.kind == bt_core::GoalKind::FollowLane;
  }
  if (cancel_follow) {
    follow_client_->async_cancel_all_goals();
  }
}

void RosNavBackend::clearCostmap() {
  if (local_sim_) {
    local_sim_->clearCostmap();
    return;
  }
  if (!clear_client_->wait_for_service(std::chrono::milliseconds(500))) {
    return;
  }
  auto req = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
  clear_client_->async_send_request(req);
}

bt_core::WorldSample RosNavBackend::snapshot() const {
  if (local_sim_) {
    return local_sim_->snapshot();
  }
  if (coupled_sim_) {
    return coupled_sim_->snapshot();
  }
  if (!sample_client_->wait_for_service(std::chrono::seconds(5))) {
    return last_remote_sample_;
  }
  auto req = std::make_shared<nav2_bt_mission::srv::GetWorldSample::Request>();
  auto fut = sample_client_->async_send_request(req);
  if (!waitWithSpin(fut, node_, spin_executor_, std::chrono::seconds(5))) {
    return last_remote_sample_;
  }
  auto res = fut.get();
  last_remote_sample_.x = res->x;
  last_remote_sample_.y = res->y;
  last_remote_sample_.battery = res->battery;
  last_remote_sample_.docked = res->docked;
  last_remote_sample_.brushes = res->brushes;
  last_remote_sample_.cov_pct = {res->cov_pct[0], res->cov_pct[1], res->cov_pct[2]};
  remote_tick_ = res->tick;
  has_remote_sample_ = true;
  return last_remote_sample_;
}

void RosNavBackend::step() {
  if (local_sim_) {
    local_sim_->step();
    return;
  }
  if (coupled_sim_) {
    if (auto mock = mock_peer_.lock()) {
      mock->stepSimPublic();
      return;
    }
  }
  if (!step_client_->wait_for_service(std::chrono::seconds(5))) {
    return;
  }
  auto req = std::make_shared<nav2_bt_mission::srv::SimStep::Request>();
  auto fut = step_client_->async_send_request(req);
  waitWithSpin(fut, node_, spin_executor_, std::chrono::seconds(5));
  spinBackend(node_, spin_executor_);
}

int RosNavBackend::tick() const {
  if (local_sim_) {
    return local_sim_->tick();
  }
  if (coupled_sim_) {
    return coupled_sim_->tick();
  }
  return remote_tick_;
}

double RosNavBackend::simTimeSec() const {
  if (local_sim_) {
    return local_sim_->simTimeSec();
  }
  if (coupled_sim_) {
    return coupled_sim_->simTimeSec();
  }
  return tick() * bt_core::MissionParams::DT_SEC;
}

void RosNavBackend::waitForRemoteReady(std::chrono::milliseconds timeout) {
  if (local_sim_) {
    return;
  }
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    const bool services =
        step_client_->wait_for_service(std::chrono::milliseconds(0)) &&
        sample_client_->wait_for_service(std::chrono::milliseconds(0));
    const bool actions =
        nav_client_->wait_for_action_server(std::chrono::milliseconds(0)) &&
        follow_client_->wait_for_action_server(std::chrono::milliseconds(0)) &&
        backup_client_->wait_for_action_server(std::chrono::milliseconds(0)) &&
        wait_client_->wait_for_action_server(std::chrono::milliseconds(0)) &&
        dock_client_->wait_for_action_server(std::chrono::milliseconds(0)) &&
        undock_client_->wait_for_action_server(std::chrono::milliseconds(0)) &&
        charge_client_->wait_for_action_server(std::chrono::milliseconds(0)) &&
        clear_client_->wait_for_service(std::chrono::milliseconds(0));
    if (services && actions) {
      return;
    }
    spinBackend(node_, spin_executor_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

}  // namespace nav2_bt_mission
