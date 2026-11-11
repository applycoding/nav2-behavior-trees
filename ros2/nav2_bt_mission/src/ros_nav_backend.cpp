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
bool spin_callbacks_on_node_thread_{false};

template <typename FutureT>
bool waitForFuture(FutureT& fut, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    if (fut.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return fut.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
}

template <typename FutureT>
bool waitWithSpin(FutureT& fut, const rclcpp::Node::SharedPtr& node, rclcpp::Executor* exec,
                  std::chrono::milliseconds timeout) {
  if (spin_callbacks_on_node_thread_) {
    (void)node;
    (void)exec;
    return waitForFuture(fut, timeout);
  }
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

template <typename ActionT>
bool acceptActionGoal(
    const rclcpp::Node::SharedPtr& node, rclcpp::Executor* exec,
    const typename rclcpp_action::Client<ActionT>::SharedPtr& client, const typename ActionT::Goal& goal,
    const typename rclcpp_action::Client<ActionT>::SendGoalOptions& opts,
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr& out) {
  (void)exec;
  (void)node;
  auto future = client->async_send_goal(goal, opts);
  if (!waitWithSpin(future, node, exec, std::chrono::seconds(30))) {
    return false;
  }
  out = future.get();
  return static_cast<bool>(out);
}

}  // namespace

void RosNavBackend::useNodeSpinThread(bool enabled) { spin_callbacks_on_node_thread_ = enabled; }

bool RosNavBackend::spinUsesNodeThread() { return spin_callbacks_on_node_thread_; }

void RosNavBackend::beginMissionTick() {
  if (!local_sim_) {
    tick_sample_valid_ = false;
  }
}

void RosNavBackend::setMockPeer(const std::shared_ptr<MockNav2>& mock) { mock_peer_ = mock; }

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
        follow_cancel_event_seq_ = ++action_event_seq_;
        dock_goal_after_follow_cancel_ = true;
      }
    } else {
      it->second.status = bt_core::GoalStatus::FAILED;
    }
  };

  switch (kind) {
    case bt_core::GoalKind::NavigateToPose: {
      if (args.nav_role == bt_core::NavPoseRole::Dock && dock_goal_after_follow_cancel_) {
        dock_nav_send_event_seq_ = ++action_event_seq_;
        ++follow_canceled_before_dock_;
        dock_goal_after_follow_cancel_ = false;
      }
      NavigateToPose::Goal goal;
      goal.pose = makePose(args.target_x, args.target_y);
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
      opts.result_callback = [result_cb](const auto& wrapped) { result_cb(wrapped.code); };
      rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr handle;
      if (!acceptActionGoal<NavigateToPose>(node_, spin_executor_, nav_client_, goal, opts, handle)) {
        mark_failed();
        return std::nullopt;
      }
      std::lock_guard<std::mutex> lock(mu_);
      goals_[id].nav_handle = handle;
      if (auto mock = mock_peer_.lock()) {
        goals_[id].sim_goal_id = mock->lookupGoalId(handle->get_goal_id());
      }
      break;
    }
    case bt_core::GoalKind::FollowLane: {
      FollowPath::Goal goal;
      goal.controller_id =
          "bt_lane:" + std::to_string(args.lane_zone) + ":" + std::to_string(args.lane_index);
      goal.goal_checker_id = "default";
      rclcpp_action::Client<FollowPath>::SendGoalOptions opts;
      opts.result_callback = [result_cb](const auto& wrapped) { result_cb(wrapped.code); };
      rclcpp_action::ClientGoalHandle<FollowPath>::SharedPtr handle;
      if (!acceptActionGoal<FollowPath>(node_, spin_executor_, follow_client_, goal, opts, handle)) {
        mark_failed();
        return std::nullopt;
      }
      std::lock_guard<std::mutex> lock(mu_);
      goals_[id].follow_handle = handle;
      if (auto mock = mock_peer_.lock()) {
        goals_[id].sim_goal_id = mock->lookupGoalId(handle->get_goal_id());
      }
      break;
    }
    case bt_core::GoalKind::BackUp: {
      BackUp::Goal goal;
      goal.speed = 0.2f;
      rclcpp_action::Client<BackUp>::SendGoalOptions opts;
      opts.result_callback = [result_cb](const auto& wrapped) { result_cb(wrapped.code); };
      rclcpp_action::ClientGoalHandle<BackUp>::SharedPtr handle;
      if (!acceptActionGoal<BackUp>(node_, spin_executor_, backup_client_, goal, opts, handle)) {
        mark_failed();
        return std::nullopt;
      }
      std::lock_guard<std::mutex> lock(mu_);
      goals_[id].backup_handle = handle;
      if (auto mock = mock_peer_.lock()) {
        goals_[id].sim_goal_id = mock->lookupGoalId(handle->get_goal_id());
      }
      break;
    }
    case bt_core::GoalKind::Wait: {
      Wait::Goal goal;
      goal.time = rclcpp::Duration::from_seconds(args.wait_sec);
      rclcpp_action::Client<Wait>::SendGoalOptions opts;
      opts.result_callback = [result_cb](const auto& wrapped) { result_cb(wrapped.code); };
      rclcpp_action::ClientGoalHandle<Wait>::SharedPtr handle;
      if (!acceptActionGoal<Wait>(node_, spin_executor_, wait_client_, goal, opts, handle)) {
        mark_failed();
        return std::nullopt;
      }
      std::lock_guard<std::mutex> lock(mu_);
      goals_[id].wait_handle = handle;
      if (auto mock = mock_peer_.lock()) {
        goals_[id].sim_goal_id = mock->lookupGoalId(handle->get_goal_id());
      }
      break;
    }
    case bt_core::GoalKind::Dock: {
      rclcpp_action::Client<Wait>::SendGoalOptions opts;
      opts.result_callback = [result_cb](const auto& wrapped) { result_cb(wrapped.code); };
      rclcpp_action::ClientGoalHandle<Wait>::SharedPtr handle;
      if (!acceptActionGoal<Wait>(node_, spin_executor_, dock_client_, Wait::Goal{}, opts, handle)) {
        mark_failed();
        return std::nullopt;
      }
      std::lock_guard<std::mutex> lock(mu_);
      goals_[id].wait_handle = handle;
      if (auto mock = mock_peer_.lock()) {
        goals_[id].sim_goal_id = mock->lookupGoalId(handle->get_goal_id());
      }
      break;
    }
    case bt_core::GoalKind::Undock: {
      rclcpp_action::Client<Wait>::SendGoalOptions opts;
      opts.result_callback = [result_cb](const auto& wrapped) { result_cb(wrapped.code); };
      rclcpp_action::ClientGoalHandle<Wait>::SharedPtr handle;
      if (!acceptActionGoal<Wait>(node_, spin_executor_, undock_client_, Wait::Goal{}, opts, handle)) {
        mark_failed();
        return std::nullopt;
      }
      std::lock_guard<std::mutex> lock(mu_);
      goals_[id].wait_handle = handle;
      if (auto mock = mock_peer_.lock()) {
        goals_[id].sim_goal_id = mock->lookupGoalId(handle->get_goal_id());
      }
      break;
    }
    case bt_core::GoalKind::Charge: {
      rclcpp_action::Client<Wait>::SendGoalOptions opts;
      opts.result_callback = [result_cb](const auto& wrapped) { result_cb(wrapped.code); };
      rclcpp_action::ClientGoalHandle<Wait>::SharedPtr handle;
      if (!acceptActionGoal<Wait>(node_, spin_executor_, charge_client_, Wait::Goal{}, opts, handle)) {
        mark_failed();
        return std::nullopt;
      }
      std::lock_guard<std::mutex> lock(mu_);
      goals_[id].wait_handle = handle;
      if (auto mock = mock_peer_.lock()) {
        goals_[id].sim_goal_id = mock->lookupGoalId(handle->get_goal_id());
      }
      break;
    }
    default: {
      std::lock_guard<std::mutex> lock(mu_);
      goals_.erase(id);
      return std::nullopt;
    }
  }

  return id;
}

bt_core::GoalState RosNavBackend::poll(bt_core::GoalId id) {
  if (!local_sim_ && !spin_callbacks_on_node_thread_) {
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
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr nav_h;
  rclcpp_action::ClientGoalHandle<FollowPath>::SharedPtr follow_h;
  rclcpp_action::ClientGoalHandle<BackUp>::SharedPtr backup_h;
  rclcpp_action::ClientGoalHandle<Wait>::SharedPtr wait_h;
  bool cancel_follow{false};
  {
    std::lock_guard<std::mutex> lock(mu_);
    auto it = goals_.find(id);
    if (it == goals_.end()) {
      return;
    }
    cancel_follow = it->second.kind == bt_core::GoalKind::FollowLane;
    nav_h = it->second.nav_handle;
    follow_h = it->second.follow_handle;
    backup_h = it->second.backup_handle;
    wait_h = it->second.wait_handle;
  }
  if (cancel_follow) {
    dock_goal_after_follow_cancel_ = true;
  }
  if (nav_h) {
    nav_client_->async_cancel_goal(nav_h);
  } else if (follow_h) {
    follow_client_->async_cancel_goal(follow_h);
  } else if (backup_h) {
    backup_client_->async_cancel_goal(backup_h);
  } else if (wait_h) {
    wait_client_->async_cancel_goal(wait_h);
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
    auto s = local_sim_->snapshot();
    s.collisions = local_sim_->collisions();
    s.goal_overlap = local_sim_->goalOverlap();
    s.min_battery = local_sim_->minBattery();
    return s;
  }
  if (tick_sample_valid_) {
    return tick_sample_;
  }
  if (!sample_client_->wait_for_service(std::chrono::milliseconds(500))) {
    return last_remote_sample_;
  }
  auto req = std::make_shared<nav2_bt_mission::srv::GetWorldSample::Request>();
  auto fut = sample_client_->async_send_request(req);
  if (!waitWithSpin(fut, node_, spin_executor_, std::chrono::seconds(30))) {
    return last_remote_sample_;
  }
  auto res = fut.get();
  last_remote_sample_.x = res->x;
  last_remote_sample_.y = res->y;
  last_remote_sample_.battery = res->battery;
  last_remote_sample_.docked = res->docked;
  last_remote_sample_.brushes = res->brushes;
  last_remote_sample_.cov_pct = {res->cov_pct[0], res->cov_pct[1], res->cov_pct[2]};
  last_remote_sample_.collisions = res->collisions;
  last_remote_sample_.goal_overlap = res->goal_overlap;
  last_remote_sample_.min_battery = res->min_battery;
  remote_tick_ = res->tick;
  has_remote_sample_ = true;
  tick_sample_ = last_remote_sample_;
  tick_sample_valid_ = true;
  return tick_sample_;
}

void RosNavBackend::step() {
  if (local_sim_) {
    local_sim_->step();
    return;
  }
  if (auto mock = mock_peer_.lock()) {
    mock->stepSimPublic();
    remote_tick_ = mock->sim()->tick();
    ++remote_step_count_;
    tick_sample_valid_ = false;
    return;
  }
  if (!step_client_->wait_for_service(std::chrono::milliseconds(500))) {
    return;
  }
  auto req = std::make_shared<nav2_bt_mission::srv::SimStep::Request>();
  auto fut = step_client_->async_send_request(req);
  if (!waitWithSpin(fut, node_, spin_executor_, std::chrono::seconds(30))) {
    return;
  }
  auto res = fut.get();
  if (res != nullptr && res->ok) {
    ++remote_tick_;
  }
  ++remote_step_count_;
  tick_sample_valid_ = false;
  if (!spin_callbacks_on_node_thread_) {
    spinBackend(node_, spin_executor_);
  }
}

int RosNavBackend::tick() const {
  if (local_sim_) {
    return local_sim_->tick();
  }
  return remote_tick_;
}

double RosNavBackend::simTimeSec() const {
  if (local_sim_) {
    return local_sim_->simTimeSec();
  }
  return tick() * bt_core::MissionParams::DT_SEC;
}

bool RosNavBackend::remoteActionServersReady() const {
  return nav_client_->wait_for_action_server(std::chrono::milliseconds(0)) &&
         follow_client_->wait_for_action_server(std::chrono::milliseconds(0)) &&
         backup_client_->wait_for_action_server(std::chrono::milliseconds(0)) &&
         wait_client_->wait_for_action_server(std::chrono::milliseconds(0)) &&
         dock_client_->wait_for_action_server(std::chrono::milliseconds(0)) &&
         undock_client_->wait_for_action_server(std::chrono::milliseconds(0)) &&
         charge_client_->wait_for_action_server(std::chrono::milliseconds(0));
}

bool RosNavBackend::isStuckCondition() const {
  if (local_sim_) {
    return local_sim_->isStuckCondition();
  }
  if (auto mock = mock_peer_.lock()) {
    return mock->sim()->isStuckCondition();
  }
  return false;
}

bool RosNavBackend::remoteActionServersReadyAfterSpin() {
  if (!spin_callbacks_on_node_thread_) {
    for (int i = 0; i < 32; ++i) {
      spinBackend(node_, spin_executor_);
    }
  }
  return remoteActionServersReady();
}

bool RosNavBackend::waitForRemoteReady(std::chrono::milliseconds timeout) {
  if (local_sim_) {
    return true;
  }
  const bool paired_mock = static_cast<bool>(mock_peer_.lock());
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    const bool services =
        paired_mock ||
        (step_client_->wait_for_service(std::chrono::milliseconds(0)) &&
         sample_client_->wait_for_service(std::chrono::milliseconds(0)));
    const bool clear_ready =
        paired_mock || clear_client_->wait_for_service(std::chrono::milliseconds(0));
    const bool actions = remoteActionServersReadyAfterSpin() && clear_ready && services;
    if (services && actions) {
      return true;
    }
    if (!spin_callbacks_on_node_thread_) {
      for (int i = 0; i < 32; ++i) {
        spinBackend(node_, spin_executor_);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  const bool services =
      paired_mock ||
      (step_client_->wait_for_service(std::chrono::milliseconds(0)) &&
       sample_client_->wait_for_service(std::chrono::milliseconds(0)));
  const bool clear_ready =
      paired_mock || clear_client_->wait_for_service(std::chrono::milliseconds(0));
  return services && remoteActionServersReady() && clear_ready;
}

bool RosNavBackend::followPathCanceledBeforeDockOrdered() const {
  if (!follow_cancel_event_seq_ || !dock_nav_send_event_seq_) {
    return false;
  }
  return *follow_cancel_event_seq_ < *dock_nav_send_event_seq_;
}

}  // namespace nav2_bt_mission
