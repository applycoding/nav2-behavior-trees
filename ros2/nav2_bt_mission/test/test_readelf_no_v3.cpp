// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include <cstdlib>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

TEST(Readelf, MissionBinaryHasNoBehaviorTreeV3) {
  const char* bin = std::getenv("BT_MISSION_NODE");
  if (bin == nullptr) {
    bin = BT_MISSION_NODE_PATH;
  }
  ASSERT_NE(bin, nullptr) << "mission node path unset";

  std::string cmd = "readelf -d ";
  cmd += bin;
  cmd += " 2>/dev/null";

  FILE* pipe = popen(cmd.c_str(), "r");
  ASSERT_NE(pipe, nullptr);
  std::string output;
  char buf[256];
  while (fgets(buf, sizeof buf, pipe)) {
    output += buf;
  }
  pclose(pipe);

  EXPECT_EQ(output.find("behaviortree_cpp_v3"), std::string::npos)
      << "linked against behaviortree_cpp_v3";
  EXPECT_NE(output.find("btcpp-4.11"), std::string::npos)
      << "expected RUNPATH/RPATH to prefer /opt/btcpp-4.11";
}
