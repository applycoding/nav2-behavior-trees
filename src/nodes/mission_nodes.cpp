// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/frozen_world.hpp"
#include "bt_core/mission_context.hpp"
#include "bt_core/grid_sim.hpp"
#include "bt_core/metrics.hpp"
#include "bt_core/mission_params.hpp"

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>

#include <functional>

namespace bt_core {

using CtxRef = std::reference_wrapper<MissionContext>;

namespace {

struct CtxMixin {
  MissionContext& ctx_;
  explicit CtxMixin(MissionContext& c) : ctx_(c) {}
  explicit CtxMixin(CtxRef c) : ctx_(c.get()) {}
};

class IsQueueEmpty : public BT::ConditionNode, CtxMixin {
 public:
  IsQueueEmpty(const std::string& n, const BT::NodeConfig& c, CtxRef ctx)
      : BT::ConditionNode(n, c), CtxMixin(ctx) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    auto& ctx = ctx_;
    return ctx.state.queueEmpty() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class BatteryOk : public BT::ConditionNode, CtxMixin {
 public:
  BatteryOk(const std::string& n, const BT::NodeConfig& c, CtxRef ctx) : BT::ConditionNode(n, c), CtxMixin(ctx) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    auto& ctx = ctx_;
    auto s = ctx.backend.snapshot();
    if (s.battery < MissionParams::CRIT_BATTERY ||
        (s.battery < MissionParams::LOW_BATTERY && ctx.state.atBoundary())) {
      ctx.state.setNeedCharge(true);
    }
    return ctx.state.needCharge() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }
};

class NeedCharge : public BT::ConditionNode, CtxMixin {
 public:
  NeedCharge(const std::string& n, const BT::NodeConfig& c, CtxRef ctx) : BT::ConditionNode(n, c), CtxMixin(ctx) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    auto& ctx = ctx_;
    return ctx.state.needCharge() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class IsStuck : public BT::ConditionNode, CtxMixin {
 public:
  IsStuck(const std::string& n, const BT::NodeConfig& c, CtxRef ctx) : BT::ConditionNode(n, c), CtxMixin(ctx) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    return ctx_.backend.isStuckCondition() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class IsDocked : public BT::ConditionNode, CtxMixin {
 public:
  IsDocked(const std::string& n, const BT::NodeConfig& c, CtxRef ctx) : BT::ConditionNode(n, c), CtxMixin(ctx) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    return ctx_.backend.snapshot().docked ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class HasNextLane : public BT::ConditionNode, CtxMixin {
 public:
  HasNextLane(const std::string& n, const BT::NodeConfig& c, CtxRef ctx) : BT::ConditionNode(n, c), CtxMixin(ctx) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    auto& ctx = ctx_;
    int z = ctx.state.activeZone();
    if (z < 0) return BT::NodeStatus::FAILURE;
    const GridSim& world = worldRef(ctx.backend);
    return ctx.state.hasNextLane(world.lanesForZone(z)) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class SelectZone : public BT::SyncActionNode, CtxMixin {
 public:
  SelectZone(const std::string& n, const BT::NodeConfig& c, CtxRef ctx) : BT::SyncActionNode(n, c), CtxMixin(ctx) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    auto& ctx = ctx_;
    if (ctx.state.activeZone() >= 0) {
      ctx.state.onSelectZone(0);
      return BT::NodeStatus::SUCCESS;
    }
    if (ctx.state.queueEmpty()) return BT::NodeStatus::FAILURE;
    ctx.state.onSelectZone(0);
    return BT::NodeStatus::SUCCESS;
  }
};

class AdvanceLane : public BT::SyncActionNode, CtxMixin {
 public:
  AdvanceLane(const std::string& n, const BT::NodeConfig& c, CtxRef ctx) : BT::SyncActionNode(n, c), CtxMixin(ctx) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    auto& ctx = ctx_;
    const GridSim& world = worldRef(ctx.backend);
    int z = ctx.state.activeZone();
    ctx.state.onAdvanceLane(world.lanesForZone(z));
    return BT::NodeStatus::SUCCESS;
  }
};

class SkipLane : public BT::SyncActionNode, CtxMixin {
 public:
  SkipLane(const std::string& n, const BT::NodeConfig& c, CtxRef ctx) : BT::SyncActionNode(n, c), CtxMixin(ctx) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    ctx_.state.onSkipLane();
    return BT::NodeStatus::SUCCESS;
  }
};

class MarkZoneDone : public BT::SyncActionNode, CtxMixin {
 public:
  MarkZoneDone(const std::string& n, const BT::NodeConfig& c, CtxRef ctx)
      : BT::SyncActionNode(n, c), CtxMixin(ctx) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    ctx_.state.onMarkZoneDone();
    return BT::NodeStatus::SUCCESS;
  }
};

class MarkZoneSkipped : public BT::SyncActionNode, CtxMixin {
 public:
  MarkZoneSkipped(const std::string& n, const BT::NodeConfig& c, CtxRef ctx)
      : BT::SyncActionNode(n, c), CtxMixin(ctx) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    ctx_.state.onMarkZoneSkipped();
    return BT::NodeStatus::SUCCESS;
  }
};

class ClearCostmap : public BT::SyncActionNode, CtxMixin {
 public:
  ClearCostmap(const std::string& n, const BT::NodeConfig& c, CtxRef ctx) : BT::SyncActionNode(n, c), CtxMixin(ctx) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    ctx_.backend.clearCostmap();
    ++ctx_.state.metrics().clear_costmap;
    ++ctx_.state.metrics().recovery_count;
    return BT::NodeStatus::SUCCESS;
  }
};

class ClearNeedCharge : public BT::SyncActionNode, CtxMixin {
 public:
  ClearNeedCharge(const std::string& n, const BT::NodeConfig& c, CtxRef ctx) : BT::SyncActionNode(n, c), CtxMixin(ctx) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    ctx_.state.setNeedCharge(false);
    return BT::NodeStatus::SUCCESS;
  }
};

class GoalAction : public BT::StatefulActionNode {
 public:
  GoalAction(const std::string& n, const BT::NodeConfig& c, CtxRef ctx, GoalKind kind)
      : BT::StatefulActionNode(n, c), ctx_(ctx.get()), kind_(kind), gid_(0) {}

 protected:
  MissionContext& ctx_;
  GoalKind kind_;
  GoalId gid_;

  void haltGoal() {
    const bool was_lane = (kind_ == GoalKind::FollowLane && gid_ != 0);
#ifndef MUTANT_NO_CANCEL
    if (gid_) ctx_.backend.cancel(gid_);
#endif
    if (was_lane) {
      if (auto* live = dynamic_cast<GridSim*>(&ctx_.backend)) {
        live->recordHaltedFollowLane(ctx_.backend.tick());
      }
      ++ctx_.state.metrics().halted_follow_lane;
      ctx_.state.metrics().follow_lane_halt_tick = ctx_.backend.tick();
    }
    gid_ = 0;
  }

  BT::NodeStatus pollGoal() {
    auto st = ctx_.backend.poll(gid_);
    switch (st.status) {
      case GoalStatus::RUNNING:
        return BT::NodeStatus::RUNNING;
      case GoalStatus::SUCCEEDED:
        gid_ = 0;
        return BT::NodeStatus::SUCCESS;
      case GoalStatus::FAILED:
        gid_ = 0;
        return BT::NodeStatus::FAILURE;
      case GoalStatus::CANCELED:
        gid_ = 0;
        return BT::NodeStatus::FAILURE;
      default: {
        const char* unreachable = "FAILURE";
        (void)unreachable;
        return BT::NodeStatus::FAILURE;
      }
    }
  }
};

class UndockNode : public GoalAction {
 public:
  UndockNode(const std::string& n, const BT::NodeConfig& c, CtxRef ctx)
      : GoalAction(n, c, ctx, GoalKind::Undock) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override {
    auto id = ctx_.backend.send(GoalKind::Undock, {});
    if (!id) return BT::NodeStatus::FAILURE;
    gid_ = *id;
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override { return pollGoal(); }
  void onHalted() override { haltGoal(); }
};

class DockNode : public GoalAction {
 public:
  DockNode(const std::string& n, const BT::NodeConfig& c, CtxRef ctx)
      : GoalAction(n, c, ctx, GoalKind::Dock) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override {
    auto id = ctx_.backend.send(GoalKind::Dock, {});
    if (!id) return BT::NodeStatus::FAILURE;
    gid_ = *id;
  return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override { return pollGoal(); }
  void onHalted() override { haltGoal(); }
};

class ChargeNode : public GoalAction {
 public:
  ChargeNode(const std::string& n, const BT::NodeConfig& c, CtxRef ctx)
      : GoalAction(n, c, ctx, GoalKind::Charge) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override {
    auto id = ctx_.backend.send(GoalKind::Charge, {});
    if (!id) return BT::NodeStatus::FAILURE;
    gid_ = *id;
    ++ctx_.state.metrics().charge_cycles;
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override { return pollGoal(); }
  void onHalted() override { haltGoal(); }
};

class NavigateToDock : public GoalAction {
 public:
  NavigateToDock(const std::string& n, const BT::NodeConfig& c, CtxRef ctx)
      : GoalAction(n, c, ctx, GoalKind::NavigateToPose) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override {
    const GridSim& world = worldRef(ctx_.backend);
    GoalArgs a;
    a.target_x = static_cast<double>(world.dockCellX()) + 0.5;
    a.target_y = static_cast<double>(world.dockCellY()) + 0.5;
    a.nav_role = NavPoseRole::Dock;
    auto id = ctx_.backend.send(GoalKind::NavigateToPose, a);
    if (!id) return BT::NodeStatus::FAILURE;
    gid_ = *id;
    if (auto* live = dynamic_cast<GridSim*>(&ctx_.backend)) {
      live->onGoalSent(GoalKind::NavigateToPose, NavPoseRole::Dock);
    }
    auto& met = ctx_.state.metrics();
    if (met.follow_lane_halt_tick >= 0 && met.next_goal_after_halt_kind < 0) {
      met.next_goal_after_halt_kind = static_cast<int>(GoalKind::NavigateToPose);
      met.next_goal_after_halt_role = static_cast<int>(NavPoseRole::Dock);
    }
    ctx_.state.metrics().dock_count++;
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override { return pollGoal(); }
  void onHalted() override { haltGoal(); }
};

class NavigateToZone : public GoalAction {
 public:
  NavigateToZone(const std::string& n, const BT::NodeConfig& c, CtxRef ctx)
      : GoalAction(n, c, ctx, GoalKind::NavigateToPose) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override {
    auto& ctx = ctx_;
    int z = ctx.state.activeZone();
    int li = ctx.state.laneIndex();
    const GridSim& world = worldRef(ctx.backend);
    Lane lane = world.laneAt(z, li);
    GoalArgs a;
    a.target_x = lane.start_x;
    a.target_y = lane.start_y;
    a.nav_role = NavPoseRole::Zone;
    auto id = ctx.backend.send(GoalKind::NavigateToPose, a);
    if (!id) return BT::NodeStatus::FAILURE;
    gid_ = *id;
    if (auto* live = dynamic_cast<GridSim*>(&ctx.backend)) {
      live->onGoalSent(GoalKind::NavigateToPose, NavPoseRole::Zone);
      live->recordNavigateToZoneAttempt(z);
    }
    if (z == 2) ++ctx.state.metrics().navigate_to_zone_attempts_zone_c;
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override { return pollGoal(); }
  void onHalted() override { haltGoal(); }
};

class FollowLane : public GoalAction {
 public:
  FollowLane(const std::string& n, const BT::NodeConfig& c, CtxRef ctx)
      : GoalAction(n, c, ctx, GoalKind::FollowLane) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override {
    ctx_.state.onFollowLaneStart();
    int z = ctx_.state.activeZone();
    int li = ctx_.state.laneIndex();
    if (z == 1 && li == 3) {
      ++ctx_.state.metrics().follow_lane_starts_b3;
    }
    GoalArgs a;
    a.lane_zone = z;
    a.lane_index = li;
    auto id = ctx_.backend.send(GoalKind::FollowLane, a);
    if (!id) return BT::NodeStatus::FAILURE;
    gid_ = *id;
    if (auto* live = dynamic_cast<GridSim*>(&ctx_.backend)) {
      live->onGoalSent(GoalKind::FollowLane);
    }
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override { return pollGoal(); }
  void onHalted() override { haltGoal(); }
};

class BackUp : public GoalAction {
 public:
  BackUp(const std::string& n, const BT::NodeConfig& c, CtxRef ctx)
      : GoalAction(n, c, ctx, GoalKind::BackUp) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override {
    auto id = ctx_.backend.send(GoalKind::BackUp, {});
    if (!id) return BT::NodeStatus::FAILURE;
    gid_ = *id;
    ++ctx_.state.metrics().recovery_count;
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    auto st = pollGoal();
    if (st == BT::NodeStatus::SUCCESS) ++ctx_.state.metrics().backup;
    return st;
  }
  void onHalted() override { haltGoal(); }
};

class SimWait : public GoalAction {
 public:
  SimWait(const std::string& n, const BT::NodeConfig& c, CtxRef ctx)
      : GoalAction(n, c, ctx, GoalKind::Wait) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("sec")};
  }
  BT::NodeStatus onStart() override {
    double sec = 5.0;
    getInput("sec", sec);
    GoalArgs a;
    a.wait_sec = sec;
    auto id = ctx_.backend.send(GoalKind::Wait, a);
    if (!id) return BT::NodeStatus::FAILURE;
    gid_ = *id;
    ++ctx_.state.metrics().sim_wait;
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override { return pollGoal(); }
  void onHalted() override { haltGoal(); }
};

}  // namespace

void registerMissionNodes(BT::BehaviorTreeFactory& factory, MissionContext& ctx) {
  CtxRef cref = std::ref(ctx);
  factory.registerNodeType<IsQueueEmpty>("IsQueueEmpty", cref);
  factory.registerNodeType<BatteryOk>("BatteryOk", cref);
  factory.registerNodeType<NeedCharge>("NeedCharge", cref);
  factory.registerNodeType<IsStuck>("IsStuck", cref);
  factory.registerNodeType<IsDocked>("IsDocked", cref);
  factory.registerNodeType<HasNextLane>("HasNextLane", cref);
  factory.registerNodeType<SelectZone>("SelectZone", cref);
  factory.registerNodeType<AdvanceLane>("AdvanceLane", cref);
  factory.registerNodeType<SkipLane>("SkipLane", cref);
  factory.registerNodeType<MarkZoneDone>("MarkZoneDone", cref);
  factory.registerNodeType<MarkZoneSkipped>("MarkZoneSkipped", cref);
  factory.registerNodeType<ClearCostmap>("ClearCostmap", cref);
  factory.registerNodeType<ClearNeedCharge>("ClearNeedCharge", cref);
  factory.registerNodeType<UndockNode>("Undock", cref);
  factory.registerNodeType<DockNode>("Dock", cref);
  factory.registerNodeType<ChargeNode>("Charge", cref);
  factory.registerNodeType<NavigateToDock>("NavigateToDock", cref);
  factory.registerNodeType<NavigateToZone>("NavigateToZone", cref);
  factory.registerNodeType<FollowLane>("FollowLane", cref);
  factory.registerNodeType<BackUp>("BackUp", cref);
  factory.registerNodeType<SimWait>("SimWait", cref);
}

}  // namespace bt_core
