// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/trace_writer.hpp"

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/basic_types.h>

#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <openssl/sha.h>

namespace bt_core {

namespace {

std::string fmt3(double v) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(3) << v;
  return os.str();
}

std::string fmt1(double v) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(1) << v;
  return os.str();
}

}  // namespace

std::string sha256File(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  SHA256_CTX ctx;
#if OPENSSL_VERSION_NUMBER >= 0x30000000L
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  SHA256_Init(&ctx);
  char buf[4096];
  while (in.read(buf, sizeof buf) || in.gcount()) {
    SHA256_Update(&ctx, buf, static_cast<std::size_t>(in.gcount()));
  }
  unsigned char hash[SHA256_DIGEST_LENGTH];
  SHA256_Final(hash, &ctx);
#if OPENSSL_VERSION_NUMBER >= 0x30000000L
#pragma GCC diagnostic pop
#endif
  std::ostringstream os;
  for (unsigned char c : hash) {
    os << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c);
  }
  return os.str();
}

TraceWriter::TraceWriter(const std::string& scenario, const GridSim& sim, const BT::Tree& tree,
                           const std::string& tree_sha256)
    : scenario_(scenario), sim_(sim), tree_(tree), tree_sha256_(tree_sha256) {}

void TraceWriter::writeHeader(std::ostream& out) const {
  out << "{\"schema_version\":1,\"scenario\":\"" << scenario_ << "\",\"dt_ms\":100,\"tree_sha256\":\""
      << tree_sha256_ << "\",\"nodes\":[";
  bool first = true;
  BT::applyRecursiveVisitor(tree_.rootNode(), [&](const BT::TreeNode* node) {
    if (!first) out << ',';
    first = false;
    out << "{\"uid\":" << node->UID() << ",\"path\":\"" << node->fullPath() << "\",\"type\":\""
        << node->registrationName() << "\"}";
  });
  out << "],\"map\":{\"dock\":[" << sim_.dockCellX() << ',' << sim_.dockCellY() << "]}}\n";
}

void TraceWriter::writeSample(std::ostream& out, int tick, const WorldSample& s) const {
  out << "{\"k\":\"s\",\"tick\":" << tick << ",\"pose\":[" << fmt3(s.x) << ',' << fmt3(s.y)
      << "],\"battery\":" << fmt3(s.battery) << ",\"docked\":" << (s.docked ? "true" : "false")
      << ",\"brushes\":" << (s.brushes ? "true" : "false") << ",\"cov\":[" << fmt1(s.cov_pct[0])
      << ',' << fmt1(s.cov_pct[1]) << ',' << fmt1(s.cov_pct[2]) << "]}\n";
}

void TraceWriter::writeTransition(std::ostream& out, const TraceTransition& t) const {
  out << "{\"k\":\"t\",\"tick\":" << t.tick << ",\"seq\":" << t.seq << ",\"uid\":" << t.uid
      << ",\"from\":\"" << t.from << "\",\"to\":\"" << t.to << "\"}\n";
}

void TraceWriter::writeMetrics(const std::string& path, const RunMetrics& m) const {
  std::ofstream out(path);
  out << "{\"scenario\":\"" << m.scenario << "\",\"success\":" << (m.success ? "true" : "false")
      << ",\"sim_time_sec\":" << fmt3(m.sim_time_sec) << ",\"lanes_done\":" << m.lanes_done
      << ",\"collisions\":" << m.collisions << ",\"goal_overlap\":" << m.goal_overlap << "}\n";
}

}  // namespace bt_core
