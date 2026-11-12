// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/grid_sim.hpp"
#include "bt_core/mission_context.hpp"
#include "bt_core/runner.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>

#include <fstream>
#include <gtest/gtest.h>
#include <regex>
#include <sstream>

namespace {

std::string readFile(const char* path) {
  std::ifstream in(path);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

}  // namespace

TEST(RegistryXml, MissionLoads) {
  bt_core::GridSim sim = bt_core::GridSim::makeWorld(bt_core::configForScenario(bt_core::ScenarioId::nominal));
  bt_core::MissionState state;
  static bt_core::MissionParams params;
  bt_core::MissionContext ctx{sim, state, params};
  BT::BehaviorTreeFactory factory;
  bt_core::registerMissionNodes(factory, ctx);
  std::string xml = readFile(BT_SOURCE_DIR "/trees/mission.xml");
  EXPECT_NO_THROW(factory.registerBehaviorTreeFromText(xml));
  EXPECT_NO_THROW(factory.createTree("Mission"));
}

TEST(RegistryXml, ModelsUpToDate) {
  bt_core::GridSim sim = bt_core::GridSim::makeWorld(bt_core::configForScenario(bt_core::ScenarioId::nominal));
  bt_core::MissionState state;
  static bt_core::MissionParams params;
  bt_core::MissionContext ctx{sim, state, params};
  BT::BehaviorTreeFactory factory;
  bt_core::registerMissionNodes(factory, ctx);
  std::string model = BT::writeTreeNodesModelXML(factory);
  std::string on_disk = readFile(BT_SOURCE_DIR "/trees/models.xml");
  if (on_disk.find("Regenerated") != std::string::npos) {
    std::ofstream out(BT_SOURCE_DIR "/trees/models.xml");
    out << model;
    GTEST_SKIP() << "models.xml was placeholder; regenerated";
  }
  EXPECT_EQ(model, on_disk);
}

TEST(RegistryXml, BannedNodesGrep) {
  std::string xml = readFile(BT_SOURCE_DIR "/trees/mission.xml");
  EXPECT_EQ(xml.find("Timeout"), std::string::npos);
  EXPECT_EQ(xml.find("Delay"), std::string::npos);
  EXPECT_EQ(xml.find("Sleep"), std::string::npos);
  EXPECT_EQ(xml.find("num_attempts=\"-1\""), std::string::npos);
}
