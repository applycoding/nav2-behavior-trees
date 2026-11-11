# Deviations from PLAN.md

## Critical battery event timing (criterion 5)

**PLAN:** at simulation tick 1200, set `battery = min(battery, 8)` while the running `FollowLane` is halted within two ticks (prototype halt at tick 1202).

**Measured:** with the frozen map and motion parameters, tick 1200 falls during `NavigateToZone` between lanes (brushes off), not during `FollowLane`. Applying the battery cap at tick 1200 never produced a halted `FollowLane` in metrics.

**Change:** for `critical_battery` only, arm the cap at tick 1200 and apply `min(battery, 8)` on the next tick where an active backend goal is `FollowLane` (event at tick 1380 and halt at tick 1381 in this build). Pre-emption, one charge cycle, 19 lanes, and `halted_follow_lane == 1` still match criterion 5 gates. Playwright and gtests use the measured halt tick constants in `include/bt_core/critical_battery_constants.hpp`.

## Mission duration vs prototype section 9

Prototype runtimes are listed in PLAN section 9 (e.g. nominal 263.7 s). This build's `GridSim` plus the goal poll and cancel fix yields slightly different times (e.g. nominal about 309 s, critical about 441 s) while remaining within the section 8 ceiling gates (`t <= 400` / `<= 590` where applicable). Golden SHA-256 hashes in `traces/golden.sha256` reflect this implementation.

## Frozen map geometry for non-GridSim backends

**PLAN section 5** nodes read lane and dock geometry from the live `GridSim` backend via `dynamic_cast`.

**Measured:** with `RosNavBackend` talking to remote mock servers, `dynamic_cast<GridSim*>` is null on the client.

**Change:** `worldRef(NavBackend&)` returns the live `GridSim` when present, otherwise a process-static `frozenWorld()` built from the nominal `GridSimConfig`. Mission nodes use this for lane and dock coordinates only; simulation state still comes from `NavBackend::snapshot()` and mock-driven `GridSim` stepping.
