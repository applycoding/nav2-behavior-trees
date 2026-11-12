// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#pragma once

#include <behaviortree_cpp/loggers/abstract_logger.h>

#include <functional>
#include <vector>

namespace bt_core {

struct TraceTransition;

class MissionStatusChangeLogger : public BT::StatusChangeLogger {
 public:
  using Callback = std::function<void(int tick, int seq, std::uint16_t uid,
                                      const std::string& from, const std::string& to)>;

  MissionStatusChangeLogger(BT::TreeNode* root, int& tick_ref, Callback cb);

  void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev,
                BT::NodeStatus status) override;
  void flush() override {}

 private:
  int& tick_;
  Callback cb_;
  int seq_{0};
};

}  // namespace bt_core
