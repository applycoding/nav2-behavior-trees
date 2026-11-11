// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/grid_sim.hpp"
#include "bt_core/runner.hpp"
#include "bt_core/trace_writer.hpp"

#include <gtest/gtest.h>
#include <fstream>
#include <sstream>
#include <unordered_map>

namespace {

struct GoldenRow {
  std::string trace_hash;
  std::string metrics_hash;
};

std::unordered_map<std::string, GoldenRow> loadGoldenHashes() {
  std::unordered_map<std::string, GoldenRow> out;
  std::ifstream in(BT_SOURCE_DIR "/traces/golden.sha256");
  std::string hash;
  std::string path;
  while (in >> hash >> path) {
    if (hash.size() != 64) continue;
    const std::string marker = "traces/";
    auto pos = path.find(marker);
    if (pos == std::string::npos) continue;
    std::string rest = path.substr(pos + marker.size());
    auto slash = rest.find('/');
    if (slash == std::string::npos) continue;
    const std::string scenario = rest.substr(0, slash);
    const std::string leaf = rest.substr(slash + 1);
    if (leaf == "trace.jsonl") {
      out[scenario].trace_hash = hash;
    } else if (leaf == "metrics.json") {
      out[scenario].metrics_hash = hash;
    }
  }
  return out;
}

}  // namespace

TEST(Golden, TraceSha256MatchesCommitted) {
  auto expected = loadGoldenHashes();
  ASSERT_EQ(expected.size(), 9u) << "traces/golden.sha256 must list 9 scenarios";

  for (const auto& [scenario, want] : expected) {
    ASSERT_FALSE(want.trace_hash.empty()) << scenario << " trace hash";
    ASSERT_FALSE(want.metrics_hash.empty()) << scenario << " metrics hash";
    bt_core::ScenarioId id = bt_core::scenarioFromName(scenario);
    const std::string out = "/tmp/bt_golden_" + scenario;
    bt_core::runMission(id, BT_SOURCE_DIR "/trees/mission.xml", out, true);
    std::string trace_path = out + "/trace.jsonl";
    std::string got_trace = bt_core::sha256File(trace_path);
    EXPECT_EQ(got_trace, want.trace_hash) << scenario << " trace";
    std::string got_metrics = bt_core::sha256File(out + "/metrics.json");
    EXPECT_EQ(got_metrics, want.metrics_hash) << scenario << " metrics";
  }
}
