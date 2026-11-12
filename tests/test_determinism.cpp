// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/runner.hpp"
#include "bt_core/trace_writer.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <string>
#include <sys/wait.h>
#include <unistd.h>

#include <vector>

namespace {

constexpr const char* kScenarios[] = {
    "nominal",         "low_battery",      "critical_battery", "blocked_corridor",
    "phantom_obstacle", "stuck",           "unreachable_zone", "lane_obstacle",
    "lane_blocked_perm",
};

std::string missionBinary() {
  return std::string(BT_MISSION_EXECUTABLE);
}

int runMissionCli(const std::string& scenario, const std::string& out) {
  const std::string cmd = missionBinary() + " --scenario " + scenario + " --tree " + BT_SOURCE_DIR +
                          "/trees/mission.xml --out " + out + " >/dev/null 2>&1";
  return std::system(cmd.c_str());
}

std::string readFile(const std::string& path) {
  std::ifstream in(path);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

}  // namespace

TEST(Determinism, TraceBytesMatchInProcess) {
  const std::string out_a = "/tmp/bt_det_a";
  const std::string out_b = "/tmp/bt_det_b";
  bt_core::runMission(bt_core::ScenarioId::nominal, BT_SOURCE_DIR "/trees/mission.xml", out_a, true);
  bt_core::runMission(bt_core::ScenarioId::nominal, BT_SOURCE_DIR "/trees/mission.xml", out_b, true);
  EXPECT_EQ(readFile(out_a + "/trace.jsonl"), readFile(out_b + "/trace.jsonl"));
  EXPECT_EQ(readFile(out_a + "/metrics.json"), readFile(out_b + "/metrics.json"));
}

TEST(Determinism, TraceBytesMatchAcrossProcesses) {
  for (const char* scenario : kScenarios) {
    const std::string out_a = std::string("/tmp/bt_det_proc_a_") + scenario;
    const std::string out_b = std::string("/tmp/bt_det_proc_b_") + scenario;
    const pid_t child = fork();
    ASSERT_NE(child, -1);
    if (child == 0) {
      const int rc = runMissionCli(scenario, out_a);
      _exit(rc == 0 ? 0 : 1);
    }
    int status = 0;
    ASSERT_EQ(waitpid(child, &status, 0), child);
    ASSERT_TRUE(WIFEXITED(status));
    ASSERT_EQ(WEXITSTATUS(status), 0);
    ASSERT_EQ(runMissionCli(scenario, out_b), 0);
    EXPECT_EQ(readFile(out_a + "/trace.jsonl"), readFile(out_b + "/trace.jsonl")) << scenario;
    EXPECT_EQ(readFile(out_a + "/metrics.json"), readFile(out_b + "/metrics.json")) << scenario;
  }
}
