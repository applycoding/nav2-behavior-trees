// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

std::string readDynamicSection(const std::string& bin) {
  const std::string cmd = "readelf -d \"" + bin + "\" 2>&1";
  FILE* pipe = popen(cmd.c_str(), "r");
  if (pipe == nullptr) {
    ADD_FAILURE() << "popen failed for " << bin << ": errno=" << errno;
    return {};
  }
  std::string output;
  char buf[512];
  while (fgets(buf, sizeof buf, pipe) != nullptr) {
    output += buf;
  }
  const int status = pclose(pipe);
  EXPECT_EQ(status, 0) << "readelf must succeed for " << bin << " (status=" << status << ")";
  EXPECT_NE(output.find("Dynamic section"), std::string::npos)
      << "readelf output must contain Dynamic section for " << bin;
  return output;
}

std::vector<std::string> binariesToCheck() {
  std::vector<std::string> bins;
  auto add_env = [&bins](const char* var) {
    if (const char* p = std::getenv(var); p != nullptr && p[0] != '\0') {
      bins.emplace_back(p);
    }
  };
  add_env("BT_MISSION_NODE");
  add_env("BT_MISSION_STACK");
  add_env("MOCK_NAV2_SERVERS");
  if (bins.empty()) {
    bins.emplace_back(BT_MISSION_NODE_PATH);
#ifdef BT_MISSION_STACK_PATH
    bins.emplace_back(BT_MISSION_STACK_PATH);
#endif
#ifdef MOCK_NAV2_SERVERS_PATH
    bins.emplace_back(MOCK_NAV2_SERVERS_PATH);
#endif
  }
  return bins;
}

}  // namespace

TEST(Readelf, InstalledBinariesHaveNoBehaviorTreeV3) {
  for (const auto& bin : binariesToCheck()) {
    const std::string output = readDynamicSection(bin);
    if (output.empty()) {
      continue;
    }
    EXPECT_EQ(output.find("behaviortree_cpp_v3"), std::string::npos)
        << bin << " linked against behaviortree_cpp_v3";
    const bool static_bt = output.find("libbehaviortree_cpp.so") == std::string::npos;
    if (!static_bt) {
      EXPECT_NE(output.find("btcpp-4.11"), std::string::npos)
          << bin << " expected RUNPATH/RPATH to prefer /opt/btcpp-4.11 when using shared BT.CPP";
    }
  }
}
