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

std::unordered_map<std::string, std::string> loadGoldenHashes() {
  std::unordered_map<std::string, std::string> out;
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
    if (rest.substr(slash + 1) != "trace.jsonl") continue;
    const std::string scenario = rest.substr(0, slash);
    out[scenario] = hash;
  }
  return out;
}

}  // namespace

TEST(Golden, TraceSha256MatchesCommitted) {
  auto expected = loadGoldenHashes();
  ASSERT_EQ(expected.size(), 9u) << "traces/golden.sha256 must list 9 scenarios";

  for (const auto& [scenario, want] : expected) {
    bt_core::ScenarioId id = bt_core::scenarioFromName(scenario);
    const std::string out = "/tmp/bt_golden_" + scenario;
    bt_core::runMission(id, BT_SOURCE_DIR "/trees/mission.xml", out, true);
    std::string trace_path = out + "/trace.jsonl";
    std::string got = bt_core::sha256File(trace_path);
    EXPECT_EQ(got, want) << scenario;
  }
}
