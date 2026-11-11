// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/runner.hpp"

#include "bt_core/mission_context.hpp"
#include "bt_core/mission_params.hpp"
#include "bt_core/status_change_logger.hpp"
#include "bt_core/trace_writer.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

namespace bt_core {

RunResult runMission(ScenarioId scenario, const std::string& tree_xml_path,
                     const std::string& out_dir, bool write_trace) {
  RunResult result;
  result.metrics.scenario = scenarioName(scenario);

  auto cfg = configForScenario(scenario);
  GridSim sim = GridSim::makeWorld(cfg);

  MissionState state;
  missionStateBindMetrics(state, result.metrics);
  state.resetZones(cfg.zones);

  static MissionParams params;
  MissionContext mctx{sim, state, params};
  BT::BehaviorTreeFactory factory;
  registerMissionNodes(factory, mctx);

  std::ifstream xml_in(tree_xml_path);
  std::string xml_text((std::istreambuf_iterator<char>(xml_in)), std::istreambuf_iterator<char>());
  factory.registerBehaviorTreeFromText(xml_text);
  auto tree = factory.createTree("Mission");

  std::string tree_hash = sha256File(tree_xml_path);
  TraceWriter tracer(result.metrics.scenario, sim, tree, tree_hash);

  fs::create_directories(out_dir);
  std::ofstream trace_out;
  if (write_trace) {
    trace_out.open(out_dir + "/trace.jsonl");
    tracer.writeHeader(trace_out);
  }

  int status_changes_this_tick = 0;
  int tick_var = 0;

  auto logger = std::make_shared<MissionStatusChangeLogger>(
      tree.rootNode(), tick_var,
      [&](int tick, int seq, std::uint16_t uid, const std::string& from, const std::string& to) {
        ++status_changes_this_tick;
        TraceTransition tr{tick, seq, uid, from, to};
        if (write_trace && trace_out) tracer.writeTransition(trace_out, tr);
      });
  BT::NodeStatus root = BT::NodeStatus::RUNNING;
  while (root == BT::NodeStatus::RUNNING && sim.tick() < MissionParams::MAX_TICKS) {
    status_changes_this_tick = 0;
    tick_var = sim.tick();
    root = tree.tickExactlyOnce();
    sim.step();
    WorldSample sample = sim.snapshot();
    if (sample.battery <= 0) {
      result.metrics.battery_zero = true;
      break;
    }
    if (write_trace && trace_out) tracer.writeSample(trace_out, sim.tick(), sample);
    result.metrics.status_changes_max_per_tick =
        std::max(result.metrics.status_changes_max_per_tick, status_changes_this_tick);
    if (status_changes_this_tick > MissionParams::MAX_STATUS_CHANGES_PER_TICK) break;
  }

  WorldSample final_sample = sim.snapshot();
  result.metrics.ticks = sim.tick();
  result.metrics.sim_time_sec = sim.simTimeSec();
  result.metrics.success = (root == BT::NodeStatus::SUCCESS);
  result.metrics.collisions = sim.collisions();
  result.metrics.goal_overlap = sim.goalOverlap();
  result.metrics.min_battery = sim.minBattery();
  result.metrics.lanes_done = state.metrics().lanes_done;
  result.metrics.zones_skipped = state.metrics().zones_skipped;
  result.metrics.lanes_skipped = state.metrics().lanes_skipped;
  result.metrics.charge_cycles = state.metrics().charge_cycles;
  result.metrics.clear_costmap = state.metrics().clear_costmap;
  result.metrics.sim_wait = state.metrics().sim_wait;
  result.metrics.backup = state.metrics().backup;
  result.metrics.halted_follow_lane = state.metrics().halted_follow_lane;
  result.metrics.follow_lane_halt_tick = state.metrics().follow_lane_halt_tick;
  result.metrics.next_goal_after_halt_kind = state.metrics().next_goal_after_halt_kind;
  result.metrics.navigate_to_zone_attempts_zone_c = state.metrics().navigate_to_zone_attempts_zone_c;
  result.metrics.recovery_count = state.metrics().recovery_count;
  result.metrics.follow_lane_starts_b3 = state.metrics().follow_lane_starts_b3;
  result.metrics.dock_count = state.metrics().dock_count;
  result.metrics.critical_battery_event_tick = sim.criticalBatteryEventTick();
  if (result.metrics.follow_lane_halt_tick < 0) {
    result.metrics.follow_lane_halt_tick = sim.followLaneHaltTick();
  }
  if (result.metrics.next_goal_after_halt_kind < 0) {
    result.metrics.next_goal_after_halt_kind = sim.nextGoalAfterHaltKind();
  }
  if (result.metrics.next_goal_after_halt_role == 0) {
    result.metrics.next_goal_after_halt_role = sim.nextGoalAfterHaltRole();
  }
  result.metrics.ended_docked = final_sample.docked;
  result.metrics.cov_pct = final_sample.cov_pct;
  if (write_trace) tracer.writeMetrics(out_dir + "/metrics.json", result.metrics);
  return result;
}

}  // namespace bt_core
