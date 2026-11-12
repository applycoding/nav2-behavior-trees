// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/grid_sim.hpp"

#include "bt_core/mission_params.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace bt_core {

namespace {

constexpr std::uint8_t OCCUPIED = 1;

int idx(int w, int x, int y) { return y * w + x; }

double cellCenter(int c) { return static_cast<double>(c) + 0.5; }

}  // namespace

GridSim GridSim::makeWorld(const GridSimConfig& cfg) {
  GridSim g;
  g.cfg_ = cfg;
  g.width_ = cfg.width;
  g.height_ = cfg.height;
  g.battery_ = cfg.battery0;
  g.min_battery_ = cfg.battery0;
  g.real_.assign(static_cast<std::size_t>(g.width_ * g.height_), 0);
  g.cost_.assign(static_cast<std::size_t>(g.width_ * g.height_), 0);
  g.buildFrozenWorld();
  g.generateLanes();
  g.covered_.assign(3, std::vector<std::vector<bool>>(g.height_,
                                                      std::vector<bool>(g.width_, false)));
  g.x_ = cellCenter(g.dock_x_);
  g.y_ = cellCenter(g.dock_y_);
  g.docked_ = true;
  if (cfg.scenario == ScenarioId::unreachable_zone) {
    g.doors_ac_closed_ = true;
  }
  return g;
}

GridSim GridSim::makeTestGrid(int w, int h, const std::vector<std::uint8_t>& real,
                               const std::vector<std::uint8_t>& cost) {
  GridSim g;
  g.width_ = w;
  g.height_ = h;
  g.real_ = real;
  g.cost_ = cost.empty() ? real : cost;
  g.battery_ = 50;
  g.x_ = 0.5;
  g.y_ = 0.5;
  g.docked_ = false;
  ZoneDef z;
  z.id = 0;
  z.name = 'T';
  z.x0 = 0;
  z.y0 = 0;
  z.x1 = w - 1;
  z.y1 = h - 1;
  z.free_cells = 0;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (g.cellFreeReal(x, y)) ++z.free_cells;
    }
  }
  g.zones_.push_back(z);
  g.covered_.assign(1, std::vector<std::vector<bool>>(h, std::vector<bool>(w, false)));
  return g;
}

void GridSim::buildFrozenWorld() {
  auto setOcc = [&](int x, int y) {
    if (x < 0 || y < 0 || x >= width_ || y >= height_) return;
    real_[idx(width_, x, y)] = OCCUPIED;
    cost_[idx(width_, x, y)] = OCCUPIED;
  };

  for (int x = 0; x < width_; ++x) {
    setOcc(x, 0);
    setOcc(x, height_ - 1);
  }
  for (int y = 0; y < height_; ++y) {
    setOcc(0, y);
    setOcc(width_ - 1, y);
  }

  for (int y = 1; y < height_ - 1; ++y) {
  if (y < 10 || y > 12) {
      if (!(y >= 28 && y <= 30)) setOcc(30, y);
    }
  }

  for (int x = 1; x < width_ - 1; ++x) {
    if (x < 14 || x > 16) {
      if (!(x >= 44 && x <= 46)) setOcc(x, 20);
    }
  }

  for (int x = 12; x <= 15; ++x)
    for (int y = 8; y <= 10; ++y) setOcc(x, y);
  for (int x = 40; x <= 42; ++x)
    for (int y = 12; y <= 13; ++y) setOcc(x, y);

  zones_ = {
      {0, 'A', 4, 4, 24, 15, 0, 0},
      {1, 'B', 35, 4, 54, 15, 0, 0},
      {2, 'C', 4, 24, 24, 34, 0, 0},
  };

  for (auto& z : zones_) {
    int free = 0;
    for (int y = z.y0; y <= z.y1; ++y) {
      for (int x = z.x0; x <= z.x1; ++x) {
        if (cellFreeReal(x, y)) ++free;
      }
    }
    z.free_cells = free;
  }
}

void GridSim::generateLanes() {
  lanes_.clear();
  int lane_global = 0;
  for (const auto& z : zones_) {
    int lane_in_zone = 0;
    bool reverse_row = false;
    for (int y = z.y0 + 1; y <= z.y1; y += 2) {
      struct Seg {
        int x0, x1;
      };
      std::vector<Seg> segs;
      int x = z.x0;
      while (x <= z.x1) {
        while (x <= z.x1 && !cellFreeReal(x, y)) ++x;
        if (x > z.x1) break;
        int xstart = x;
        while (x <= z.x1 && cellFreeReal(x, y)) ++x;
        int len = x - xstart;
        if (len >= 2) segs.push_back({xstart, x - 1});
      }
      if (reverse_row) {
        std::reverse(segs.begin(), segs.end());
      }
      for (const auto& s : segs) {
        Lane lane;
        lane.zone = z.id;
        lane.index = lane_in_zone++;
        lane.row_y = y;
        if (!reverse_row) {
          lane.start_x = cellCenter(s.x0);
          lane.start_y = cellCenter(y);
          lane.end_x = cellCenter(s.x1);
          lane.end_y = cellCenter(y);
          lane.reverse = false;
        } else {
          lane.start_x = cellCenter(s.x1);
          lane.start_y = cellCenter(y);
          lane.end_x = cellCenter(s.x0);
          lane.end_y = cellCenter(y);
          lane.reverse = true;
        }
        lanes_.push_back(lane);
        ++lane_global;
      }
      reverse_row = !reverse_row;
    }
    zones_[z.id].lane_count = lane_in_zone;
  }
  (void)lane_global;
}

bool GridSim::cellFreeReal(int x, int y) const {
  if (x < 0 || y < 0 || x >= width_ || y >= height_) return false;
  return real_[idx(width_, x, y)] == 0;
}

bool GridSim::cellBlockedCost(int x, int y) const {
  if (x < 0 || y < 0 || x >= width_ || y >= height_) return true;
  return cost_[idx(width_, x, y)] != 0;
}

bool GridSim::cellWalkableCost(int x, int y) const {
  return !cellBlockedCost(x, y);
}

int GridSim::lanesForZone(int zone_id) const {
  return zones_.at(static_cast<std::size_t>(zone_id)).lane_count;
}

int GridSim::freeCellsInZone(int zone_id) const {
  return zones_.at(static_cast<std::size_t>(zone_id)).free_cells;
}

Lane GridSim::laneAt(int zone_id, int lane_idx) const {
  for (const auto& l : lanes_) {
    if (l.zone == zone_id && l.index == lane_idx) return l;
  }
  throw std::runtime_error("lane not found");
}

std::optional<GoalId> GridSim::send(GoalKind kind, GoalArgs args) {
  if (active_ && active_->status == GoalStatus::RUNNING) {
    ++goal_overlap_;
    return std::nullopt;
  }
  ActiveGoal g;
  g.id = next_id_++;
  g.kind = kind;
  g.args = args;
  g.status = GoalStatus::RUNNING;
  g.last_x = x_;
  g.last_y = y_;

  switch (kind) {
    case GoalKind::NavigateToPose: {
      if (!planAStar(x_, y_, args.target_x, args.target_y, g.path)) {
        g.status = GoalStatus::FAILED;
        g.reason = FailReason::NO_PATH;
      }
      break;
    }
    case GoalKind::FollowLane: {
      if (cfg_.scenario == ScenarioId::lane_blocked_perm && args.lane_zone == 1 && args.lane_index == 3) {
        lane_b_perm_ = true;
      }
      if (cfg_.scenario == ScenarioId::lane_obstacle && args.lane_zone == 1 && args.lane_index == 3) {
        lane_b_obstacle_ticks_ = 80;
      }
      Lane lane = laneAt(args.lane_zone, args.lane_index);
      if (!planAStar(x_, y_, lane.start_x, lane.start_y, g.path)) {
        g.status = GoalStatus::FAILED;
        g.reason = FailReason::NO_PATH;
      }
      g.lane_segment = 0;
      break;
    }
    case GoalKind::BackUp:
      g.wait_ticks_left = static_cast<int>(MissionParams::BACKUP_SEC / MissionParams::DT_SEC);
      break;
    case GoalKind::Wait:
      g.wait_ticks_left = static_cast<int>(std::ceil(args.wait_sec / MissionParams::DT_SEC));
      break;
    case GoalKind::Dock:
      g.wait_ticks_left = static_cast<int>(MissionParams::DOCK_SEC / MissionParams::DT_SEC);
      break;
    case GoalKind::Undock:
      g.wait_ticks_left = static_cast<int>(MissionParams::UNDOCK_SEC / MissionParams::DT_SEC);
      break;
    case GoalKind::Charge:
      charging_ = true;
      break;
    default:
      break;
  }

  last_sent_kind_ = kind;
  active_ = std::move(g);
  if (active_->status == GoalStatus::FAILED) {
    last_fail_reason_ = active_->reason;
    GoalId id = active_->id;
    GoalState st{active_->status, active_->reason};
    (void)st;
    return id;
  }
  return active_->id;
}

GoalState GridSim::poll(GoalId id) {
  if (!active_ || active_->id != id) {
    return {GoalStatus::CANCELED, FailReason::NONE};
  }
  GoalState st{active_->status, active_->reason};
  if (active_->status == GoalStatus::FAILED || active_->status == GoalStatus::SUCCEEDED) {
    active_.reset();
  }
  return st;
}

void GridSim::cancel(GoalId id) {
  if (!active_ || active_->id != id) return;
  if (active_->kind == GoalKind::Charge) charging_ = false;
  active_->status = GoalStatus::CANCELED;
  active_.reset();
}

void GridSim::clearCostmap() {
  for (std::size_t i = 0; i < cost_.size(); ++i) {
    if (real_[i] == 0) cost_[i] = 0;
  }
  corridor_phantom_ = false;
  phantom_costmap_cleared_ = true;
}

WorldSample GridSim::snapshot() const {
  WorldSample s;
  s.x = x_;
  s.y = y_;
  s.battery = battery_;
  s.docked = docked_;
  s.brushes = active_ && active_->brushes;
  s.cov_pct.resize(3);
  for (int zi = 0; zi < 3 && zi < static_cast<int>(zones_.size()); ++zi) {
    const auto& z = zones_[zi];
    int covered = 0;
    for (int y = z.y0; y <= z.y1; ++y) {
      for (int x = z.x0; x <= z.x1; ++x) {
        if (cellFreeReal(x, y) && covered_[zi][y][x]) ++covered;
      }
    }
    int denom = z.free_cells;
    s.cov_pct[zi] = denom > 0 ? 100.0 * covered / denom : 0;
  }
  return s;
}

bool GridSim::planAStar(double sx, double sy, double gx, double gy,
                        std::vector<std::pair<int, int>>& out) {
  int start_x = static_cast<int>(std::floor(sx));
  int start_y = static_cast<int>(std::floor(sy));
  int goal_x = static_cast<int>(std::floor(gx));
  int goal_y = static_cast<int>(std::floor(gy));

  auto heuristic = [](int x0, int y0, int x1, int y1) {
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int dmin = std::min(dx, dy);
    int dmax = std::max(dx, dy);
    return 14 * dmin + 10 * (dmax - dmin);
  };

  struct Node {
    int x, y;
    int g;
    int f;
    bool operator>(const Node& o) const { return f > o.f; }
  };

  const int W = width_;
  const int H = height_;
  std::vector<int> gscore(W * H, std::numeric_limits<int>::max());
  std::vector<std::pair<int, int>> parent(W * H, {-1, -1});

  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
  gscore[idx(W, start_x, start_y)] = 0;
  open.push({start_x, start_y, 0, heuristic(start_x, start_y, goal_x, goal_y)});

  while (!open.empty()) {
    Node cur = open.top();
    open.pop();
    if (cur.x == goal_x && cur.y == goal_y) {
      out.clear();
      int cx = goal_x, cy = goal_y;
      while (!(cx == start_x && cy == start_y)) {
        out.emplace_back(cx, cy);
        auto p = parent[idx(W, cx, cy)];
        cx = p.first;
        cy = p.second;
      }
      out.emplace_back(start_x, start_y);
      std::reverse(out.begin(), out.end());
      return true;
    }
    if (cur.g > gscore[idx(W, cur.x, cur.y)]) continue;

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;
        int nx = cur.x + dx;
        int ny = cur.y + dy;
        if (!cellWalkableCost(nx, ny)) continue;
        if (dx != 0 && dy != 0) {
          if (!cellWalkableCost(cur.x + dx, cur.y) || !cellWalkableCost(cur.x, cur.y + dy))
            continue;
        }
        int step_cost = (dx != 0 && dy != 0) ? 14 : 10;
        int ng = cur.g + step_cost;
        if (ng < gscore[idx(W, nx, ny)]) {
          gscore[idx(W, nx, ny)] = ng;
          parent[idx(W, nx, ny)] = {cur.x, cur.y};
          open.push({nx, ny, ng, ng + heuristic(nx, ny, goal_x, goal_y)});
        }
      }
    }
  }
  return false;
}

void GridSim::advanceMotion(ActiveGoal& g) {
  if (g.path.empty()) return;

  if (stuck_flag_ && g.kind != GoalKind::BackUp) {
    ++g.stuck_no_progress_ticks;
    if (g.stuck_no_progress_ticks >= 20) {
      g.status = GoalStatus::FAILED;
      g.reason = FailReason::STUCK;
      last_fail_reason_ = FailReason::STUCK;
    }
    return;
  }

  std::size_t target_i = std::min(g.path_i + 1, g.path.size() - 1);
  double tx = cellCenter(g.path[target_i].first);
  double ty = cellCenter(g.path[target_i].second);
  double dx = tx - x_;
  double dy = ty - y_;
  double dist = std::hypot(dx, dy);
  if (dist < 1e-9) {
    g.path_i = target_i;
    if (g.path_i >= g.path.size() - 1) {
      x_ = tx;
      y_ = ty;
    }
    return;
  }
  double step = MissionParams::CELLS_PER_TICK;
  if (step >= dist) {
    x_ = tx;
    y_ = ty;
    g.path_i = target_i;
    int cx = g.path[target_i].first;
    int cy = g.path[target_i].second;
    if (!cellFreeReal(cx, cy)) {
      g.status = GoalStatus::FAILED;
      g.reason = FailReason::BLOCKED;
      last_fail_reason_ = FailReason::BLOCKED;
      return;
    }
    markCoverageIfBrushing(cx, cy);
  } else {
    x_ += dx / dist * step;
    y_ += dy / dist * step;
  }

  if (std::hypot(x_ - g.last_x, y_ - g.last_y) < 0.01) {
    ++g.stuck_no_progress_ticks;
  } else {
    g.stuck_no_progress_ticks = 0;
    g.last_x = x_;
    g.last_y = y_;
  }

  if (stuck_flag_ && g.kind != GoalKind::BackUp && g.stuck_no_progress_ticks >= 20) {
    g.status = GoalStatus::FAILED;
    g.reason = FailReason::STUCK;
    last_fail_reason_ = FailReason::STUCK;
  }
}

void GridSim::markCoverageIfBrushing(int cx, int cy) {
  if (!active_ || !active_->brushes) return;
  for (int zi = 0; zi < static_cast<int>(zones_.size()); ++zi) {
    const auto& z = zones_[zi];
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        int nx = cx + dx;
        int ny = cy + dy;
        if (nx < z.x0 || nx > z.x1 || ny < z.y0 || ny > z.y1) continue;
        if (!cellFreeReal(nx, ny)) continue;
        covered_[zi][ny][nx] = true;
      }
    }
  }
}

void GridSim::applyBattery() {
  if (charging_) {
    battery_ = std::min(100.0, battery_ + MissionParams::CHARGE_PER_SEC * MissionParams::DT_SEC);
  } else {
    battery_ -= MissionParams::TIME_DRAIN_PER_SEC * MissionParams::DT_SEC;
  }
  min_battery_ = std::min(min_battery_, battery_);
}

void GridSim::applyScenarioHooks() {
  double t = simTimeSec();
  if (cfg_.scenario == ScenarioId::critical_battery) {
    if (tick_ == 1200) {
      critical_battery_pending_ = true;
    }
    if (critical_battery_pending_ && active_ && active_->kind == GoalKind::FollowLane) {
      battery_ = std::min(battery_, 8.0);
      min_battery_ = std::min(min_battery_, battery_);
      critical_battery_event_tick_ = tick_;
      critical_battery_pending_ = false;
    }
  }
  if (cfg_.scenario == ScenarioId::blocked_corridor) {
    for (int y = 21; y <= 38; ++y) {
      int i = idx(width_, 52, y);
      if (t < 12.0) {
        real_[i] = OCCUPIED;
        cost_[i] = OCCUPIED;
      } else {
        real_[i] = 0;
        cost_[i] = 0;
      }
    }
  }
  if (cfg_.scenario == ScenarioId::phantom_obstacle) {
    corridor_phantom_ = !phantom_costmap_cleared_ && (t < 12.0);
    for (int y = 21; y <= 38; ++y) {
      int i = idx(width_, 52, y);
      if (corridor_phantom_ && real_[i] == 0) {
        cost_[i] = OCCUPIED;
      } else if (!corridor_phantom_ && real_[i] == 0) {
        cost_[i] = 0;
      }
    }
  }
  if (cfg_.scenario == ScenarioId::stuck) {
    if (t >= 40.0 && !stuck_scenario_latched_) {
      stuck_scenario_latched_ = true;
      stuck_flag_ = true;
    }
  }
  if (doors_ac_closed_) {
    for (int x = 14; x <= 16; ++x) {
      real_[idx(width_, x, 20)] = OCCUPIED;
      cost_[idx(width_, x, 20)] = OCCUPIED;
    }
    for (int y = 28; y <= 30; ++y) {
      real_[idx(width_, 30, y)] = OCCUPIED;
      cost_[idx(width_, 30, y)] = OCCUPIED;
    }
  }

  if (cfg_.scenario == ScenarioId::lane_obstacle && lane_b3_obstacle_armed_) {
    Lane lane3 = laneAt(1, 3);
    int mx = static_cast<int>((lane3.start_x + lane3.end_x) * 0.5);
    int my = lane3.row_y;
    int cell = idx(width_, mx, my);
    if (lane_b_obstacle_ticks_ > 0) {
      real_[cell] = OCCUPIED;
      cost_[cell] = OCCUPIED;
      --lane_b_obstacle_ticks_;
    } else {
      if (real_[cell] == OCCUPIED) real_[cell] = 0;
      if (cost_[cell] == OCCUPIED) cost_[cell] = 0;
    }
  } else if (cfg_.scenario == ScenarioId::lane_blocked_perm) {
    const bool lane3_brushing = active_ && active_->kind == GoalKind::FollowLane &&
                                active_->args.lane_zone == 1 && active_->args.lane_index == 3 &&
                                active_->lane_segment == 1;
    if (lane3_brushing && lane_b_perm_) {
      Lane lane3 = laneAt(1, 3);
      int mx = static_cast<int>((lane3.start_x + lane3.end_x) * 0.5);
      int my = lane3.row_y;
      real_[idx(width_, mx, my)] = OCCUPIED;
      cost_[idx(width_, mx, my)] = OCCUPIED;
    }
  }
}

void GridSim::step() {
  applyScenarioHooks();

  if (active_ && active_->status == GoalStatus::RUNNING) {
    auto& g = *active_;
    switch (g.kind) {
      case GoalKind::NavigateToPose:
        advanceMotion(g);
        if (g.status == GoalStatus::RUNNING && g.path_i >= g.path.size() - 1 &&
            std::hypot(x_ - g.args.target_x, y_ - g.args.target_y) < 0.05) {
          g.status = GoalStatus::SUCCEEDED;
        }
        break;
      case GoalKind::FollowLane: {
        if (g.lane_segment == 0) {
          advanceMotion(g);
          Lane lane = laneAt(g.args.lane_zone, g.args.lane_index);
          if (g.status == GoalStatus::RUNNING && g.path_i >= g.path.size() - 1 &&
              std::hypot(x_ - lane.start_x, y_ - lane.start_y) < 0.05) {
            g.lane_segment = 1;
            int x0 = static_cast<int>(std::floor(std::min(lane.start_x, lane.end_x)));
            int x1 = static_cast<int>(std::floor(std::max(lane.start_x, lane.end_x)));
            g.path.clear();
            if (!lane.reverse) {
              for (int x = x0; x <= x1; ++x) g.path.emplace_back(x, lane.row_y);
            } else {
              for (int x = x1; x >= x0; --x) g.path.emplace_back(x, lane.row_y);
            }
            g.path_i = 0;
            g.brushes = true;
            x_ = lane.start_x;
            y_ = lane.start_y;
            if (cfg_.scenario == ScenarioId::lane_obstacle && !lane_b3_obstacle_armed_ &&
                g.args.lane_zone == 1 && g.args.lane_index == 3) {
              lane_b_obstacle_ticks_ = 80;
              lane_b3_obstacle_armed_ = true;
              lane_b_perm_ = false;
            } else if (cfg_.scenario == ScenarioId::lane_blocked_perm &&
                       g.args.lane_zone == 1 && g.args.lane_index == 3) {
              lane_b_perm_ = true;
            }
          }
        } else {
          advanceMotion(g);
          Lane lane = laneAt(g.args.lane_zone, g.args.lane_index);
          if (g.status == GoalStatus::RUNNING && g.path_i >= g.path.size() - 1) {
            x_ = lane.end_x;
            y_ = lane.end_y;
            g.brushes = false;
            g.status = GoalStatus::SUCCEEDED;
            ++lanes_completed_;
          }
        }
        break;
      }
      case GoalKind::BackUp:
        if (--g.wait_ticks_left <= 0) {
          stuck_flag_ = false;
          last_fail_reason_ = FailReason::NONE;
          g.status = GoalStatus::SUCCEEDED;
        }
        break;
      case GoalKind::Wait:
        if (--g.wait_ticks_left <= 0) g.status = GoalStatus::SUCCEEDED;
        break;
      case GoalKind::Dock:
        if (--g.wait_ticks_left <= 0) {
          docked_ = true;
          x_ = cellCenter(dock_x_);
          y_ = cellCenter(dock_y_);
          g.status = GoalStatus::SUCCEEDED;
        }
        break;
      case GoalKind::Undock:
        if (--g.wait_ticks_left <= 0) {
          docked_ = false;
          g.status = GoalStatus::SUCCEEDED;
        }
        break;
      case GoalKind::Charge:
        if (battery_ >= MissionParams::CHARGED_BATTERY) {
          charging_ = false;
          g.status = GoalStatus::SUCCEEDED;
          recordChargeCycleComplete();
        }
        break;
      default:
        break;
    }
    if (g.status == GoalStatus::FAILED) {
      last_fail_reason_ = g.reason;
    } else if (g.status == GoalStatus::SUCCEEDED) {
      /* cleared on poll */
    } else if (g.status != GoalStatus::RUNNING) {
      active_.reset();
    }
  }

  double motion_m = 0;
  if (active_ && active_->status == GoalStatus::RUNNING && !active_->path.empty()) {
    motion_m += MissionParams::CELLS_PER_TICK * MissionParams::CELL_SIZE_M;
  }
  if (!charging_) {
    battery_ -= motion_m * MissionParams::MOTION_DRAIN_PER_M;
  }
  applyBattery();
  if (active_ && active_->brushes) {
    markCoverageIfBrushing(static_cast<int>(std::floor(x_)), static_cast<int>(std::floor(y_)));
  }
  ++tick_;
}

void GridSim::onGoalSent(GoalKind k, NavPoseRole nav_role) {
  last_sent_kind_ = k;
  if (follow_lane_halt_tick_ >= 0 && next_goal_after_halt_kind_ < 0) {
    next_goal_after_halt_kind_ = static_cast<int>(k);
    if (k == GoalKind::NavigateToPose) {
      next_goal_after_halt_role_ = static_cast<int>(nav_role);
    }
  }
}

void GridSim::recordHaltedFollowLane(int at_tick) {
  ++halted_follow_lane_count_;
  follow_lane_halt_tick_ = at_tick;
}

void GridSim::recordLaneAdvanced() {}
void GridSim::recordZoneSkipped() {}
void GridSim::recordLaneSkipped() {}
void GridSim::recordChargeCycleComplete() {}
void GridSim::recordClearCostmap() {}
void GridSim::recordSimWait() {}
void GridSim::recordBackup() {}
void GridSim::recordNavigateToZoneAttempt(int) {}
void GridSim::recordRecovery() {}

void GridSim::testForceCollision() {
  int cx = static_cast<int>(std::floor(x_));
  int cy = static_cast<int>(std::floor(y_));
  if (!cellFreeReal(cx, cy)) ++collisions_;
}

}  // namespace bt_core
