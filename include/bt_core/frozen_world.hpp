// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Arpan Bom

#pragma once

#include "bt_core/grid_sim.hpp"
#include "bt_core/nav_backend.hpp"

namespace bt_core {

const GridSim& frozenWorld();

const GridSim& worldRef(NavBackend& backend);

}  // namespace bt_core
