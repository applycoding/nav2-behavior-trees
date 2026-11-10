// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#pragma once

#include "bt_core/grid_sim.hpp"
#include "bt_core/metrics.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <fstream>
#include <string>
#include <vector>

namespace bt_core {

struct TraceTransition {
  int tick{0};
  int seq{0};
  std::uint16_t uid{0};
  std::string from;
  std::string to;
};

class TraceWriter {
 public:
  TraceWriter(const std::string& scenario, const GridSim& sim, const BT::Tree& tree,
              const std::string& tree_sha256);
  void writeHeader(std::ostream& out) const;
  void writeSample(std::ostream& out, int tick, const WorldSample& s) const;
  void writeTransition(std::ostream& out, const TraceTransition& t) const;
  void writeMetrics(const std::string& path, const RunMetrics& m) const;

 private:
  std::string scenario_;
  const GridSim& sim_;
  const BT::Tree& tree_;
  std::string tree_sha256_;
};

std::string sha256File(const std::string& path);

}  // namespace bt_core
