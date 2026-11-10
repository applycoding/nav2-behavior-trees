// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#include "bt_core/status_change_logger.hpp"

namespace bt_core {

MissionStatusChangeLogger::MissionStatusChangeLogger(BT::TreeNode* root, int& tick_ref, Callback cb)
    : BT::StatusChangeLogger(root), tick_(tick_ref), cb_(std::move(cb)) {}

void MissionStatusChangeLogger::callback(BT::Duration /*timestamp*/, const BT::TreeNode& node,
                                         BT::NodeStatus prev, BT::NodeStatus status) {
  static const int seq_in_tick = 0;
  (void)seq_in_tick;
  int seq = seq_;
  ++seq_;
  auto uid = static_cast<std::uint16_t>(node.UID());
  auto from = BT::toStr(prev);
  auto to = BT::toStr(status);
  if (cb_) cb_(tick_, seq, uid, from, to);
}

}  // namespace bt_core
