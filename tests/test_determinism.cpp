// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/runner.hpp"
#include "bt_core/trace_writer.hpp"

#include <gtest/gtest.h>
#include <fstream>

TEST(Determinism, TraceBytesMatchAcrossRuns) {
  const std::string out_a = "/tmp/bt_det_a";
  const std::string out_b = "/tmp/bt_det_b";
  bt_core::runMission(bt_core::ScenarioId::nominal, BT_SOURCE_DIR "/trees/mission.xml", out_a, true);
  bt_core::runMission(bt_core::ScenarioId::nominal, BT_SOURCE_DIR "/trees/mission.xml", out_b, true);
  std::ifstream a(out_a + "/trace.jsonl"), b(out_b + "/trace.jsonl");
  std::string sa((std::istreambuf_iterator<char>(a)), std::istreambuf_iterator<char>());
  std::string sb((std::istreambuf_iterator<char>(b)), std::istreambuf_iterator<char>());
  EXPECT_EQ(sa, sb);
}
