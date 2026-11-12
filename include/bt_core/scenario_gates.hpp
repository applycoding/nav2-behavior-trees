// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#pragma once

#include "bt_core/metrics.hpp"

namespace bt_core {
namespace test_gates {

void expectGlobalInvariants(const RunMetrics& m);

}  // namespace test_gates
}  // namespace bt_core
