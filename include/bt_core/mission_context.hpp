// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#pragma once

#include "bt_core/mission_params.hpp"
#include "bt_core/mission_state.hpp"
#include "bt_core/nav_backend.hpp"

namespace bt_core {

struct MissionContext {
  NavBackend& backend;
  MissionState& state;
  const MissionParams& params;
};

}  // namespace bt_core
