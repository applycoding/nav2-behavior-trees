# nav2-behavior-trees: plan

Date: 2026-11-10. Author: Arpan Bom. Licence: Apache-2.0. Status: decision-complete. An independent review has checked the plan, and a throwaway prototype has measured every threshold (see "Verification record").

## 1. Goal

This is a public portfolio demo of a **behavior-tree mission executive for a simulated cleaning robot**. A BehaviorTree.CPP v4 tree undocks and cleans three rooms with boustrophedon lanes. It handles a low battery by docking at a lane boundary. On a critical battery it pre-empts the current goal at once, charges, and then **resumes the interrupted lane**. It recovers from blocked corridors, phantom costmap obstacles, a stuck robot and lane obstacles. It skips unreachable zones and permanently blocked lanes, and it finishes docked.

Everything runs against a deterministic grid-world `NavBackend` in headless CI. An optional ROS 2 Humble layer drives the same tree through `rclcpp_action` clients against mock Nav2 action servers. A GitHub Pages viewer replays the CI traces so you can watch the tree tick.

The project is **simulation only**. It makes no claim about real robots, real cleaning, or production use.

It differs from the sibling repos:
- `ros2-nav2-slam-sim` is a Python YAML waypoint runner with a hand-written FSM.
- `nav2-coverage-planner-compare` is a planner bake-off.

The artefact here is the tree's composition, its pre-emption and resume semantics, and the evidence that each part is needed. That evidence comes from negative "mutant" trees that must fail.

Skills shown: C++17, BehaviorTree.CPP v4, reactive control flow, task sequencing, recovery design, ROS 2 actions, Nav2 interfaces, deterministic testing, TypeScript.

## 2. Scope

**In v1 (CI-gated):**
- `bt_core`: C++17, no ROS. Contains the nodes, `MissionState`, `GridSim`, the runner, metrics and the trace writer.
- `bt_mission` CLI.
- Nine frozen scenarios with numeric gates.
- Five mutant trees and builds that must fail named gates.
- Byte-identical deterministic traces.
- The ROS 2 Humble package `nav2_bt_mission`: an action-client backend, mock Nav2 servers, and a gtest plus a launch_test inside a `ros:humble` container.
- The Pages trace viewer, with Playwright producing `docs/demo.png`.

**Manual only (documented, not gated):** running the mission node against a real Nav2 Humble `tb3_simulation_launch.py` (Gazebo Classic).

**Out of scope:**
- Real robots, brushes and docking contacts.
- Groot/Groot2 and ZeroMQ.
- Nav2 `bt_navigator` plugins.
- opennav_docking, opennav_coverage and Fields2Cover.
- Coverage-planner comparison and SLAM.
- A WASM build of the tree.
- Multi-robot operation.
- Publishing to apt or ROS Index.

## 3. Pinned stack (verified 2026-11-10)

| Item | Pin | Licence (from the LICENSE file) |
| --- | --- | --- |
| BehaviorTree.CPP | tag `4.11.0`, commit `8dad25dce26c3fd0787ff5c8790714bc6159dabe` (released 2026-11-08) | MIT |
| GoogleTest | `v1.18.0`, commit `063de7e9578f82b369302001269680b4b1553359` | BSD-3-Clause |
| ROS 2 | Humble. Rosdistro lists `behaviortree_cpp` 4.11.0-1 (repo `behaviortree_cpp_v4`) and navigation2 1.1.20-1 | n/a |
| ROS image | `ros:humble-ros-base-jammy@sha256:d9bcf2e6f72df127fdee07048c2e87b1b4b9a59a74af0592f4489fc2844aa855` (Docker Hub, updated 2026-10-27) | n/a |
| Node.js | 26.13.0 LTS (2026-10-28), with npm from that release and a committed `package-lock.json` | MIT |
| TypeScript | 7.0.2 | Apache-2.0 |
| Vite | 8.3.7 | MIT |
| Vitest | 5.1.0 | MIT |
| @playwright/test | 1.65.0 (Chromium installed with `npx playwright install --with-deps chromium`) | Apache-2.0 |
| Python (schema test) | 3.12 with pytest 9.2.0, pinned in `python/requirements.txt` with hashes | MIT |
| Core runner | `ubuntu-24.04`, distro g++-13, CMake ≥ 3.22. The job prints the exact versions. | n/a |

GitHub Actions are pinned by full SHA, with the version in a comment:

| Action | Version | SHA |
| --- | --- | --- |
| `actions/checkout` | v7.0.1 | `3d3c42e5aac5ba805825da76410c181273ba90b1` |
| `actions/setup-node` | v7.1.0 | `949feb2413d6458794dcd2491c4babbbce0c15c1` |
| `actions/setup-python` | v7.1.0 | `6d4fd22b20465ccc0f7354476588e8df7c9ca0e0` |
| `actions/upload-artifact` | v7.0.2 | `cf430e030ddbb5b0abf93d22962f4752f3646cd9` |
| `actions/configure-pages` | v6.0.0 | `45bfe0192ca1faeb007ade9deae92b16b8254a0d` |
| `actions/upload-pages-artifact` | v5.0.0 | `fc324d3547104276b827a68afc52ff2a11cc49c9` |
| `actions/deploy-pages` | v5.0.1 | `368f82528645a54fb793d4d04e342629a3f51346` |

BehaviorTree.CPP is consumed with **FetchContent / add_subdirectory**, not as an installed static package. A static install plus `find_package` failed in the prototype, because the exported targets reference vendored targets that are not installed. Set these options before `FetchContent_MakeAvailable`:
- `BTCPP_GROOT_INTERFACE=OFF`
- `BTCPP_SQLITE_LOGGING=OFF`
- `BTCPP_EXAMPLES=OFF`
- `BTCPP_BUILD_TOOLS=OFF`
- `BTCPP_SHARED_LIBS=OFF`

Set the dependency's `BUILD_TESTING=OFF` in a scoped way, so the project's own CTest registration is kept. The prototype built 4.11.0 with these options in about 15 s.

In the ROS job, `bt_core` is built inside the container against apt `ros-humble-behaviortree-cpp`. The job records the full `dpkg-query -W` version and asserts that the upstream part starts with `4.11.0`. If the assertion fails, the job falls back to the same FetchContent commit and logs that it did. `ros-humble-behaviortree-cpp-v3` must not be a dependency of `nav2_bt_mission`. A test checks the mission binary with `ldd`/`readelf` and fails if any `behaviortree_cpp_v3` library appears.

**JSON:** there is no JSON library dependency. Traces and metrics are written by a small fixed-format writer in `trace_writer.cpp`, which includes no BehaviorTree.CPP headers. Scenarios are compiled-in C++ tables, so the run has no parser. This avoids mixing a second nlohmann/json with the copy BT.CPP vendors.

## 4. Architecture

```
        ┌──────────── bt_mission (one process, BT.CPP v4 only) ────────────┐
 XML ─▶ │ BehaviorTreeFactory ─ custom nodes ─▶ NavBackend (abstract)      │
        │        │                     ▲          ├─ GridSim (core, CI)    │
        │   MissionState (shared ctx)  │          └─ RosNavBackend (ROS)   │
        │   Runner: tickExactlyOnce → backend.step(1 tick) → sample        │
        │   TraceWriter: per-tick samples + status transitions             │
        └───────────────────────────────────────────────────────────────────┘
```

- **Separate process from Nav2.** Nav2 Humble 1.1.20 `nav2_bt_navigator` depends on `behaviortree_cpp_v3` (3.8.7). BT.CPP v4 also uses namespace `BT`, so a process must never link both. The mission node is a client of Nav2's actions.
- **Time.** Integer `tick` counter with `DT = 100 ms` (`sim_time_s = tick / 10.0`). No node reads the wall clock. Stock `Timeout`, `Delay` and `Sleep` use BT.CPP's real-time TimerQueue, so they are **banned** in mission XML; a test greps for them. Waiting uses the custom `SimWait` node, which counts sim ticks. The runner calls `tree.tickExactlyOnce()` and never `tickOnce()` or `tickWhileRunning()`. `tickOnce()` can re-tick on wake-up signals, which breaks the one-tick-one-step rule.
- **Context injection.** Nodes are registered with `factory.registerNodeType<T>(id, ctx)`, where `ctx` is a `MissionContext&` holding `NavBackend&`, `MissionState&` and `Params`. That extra constructor-argument overload exists in 4.11.0 `bt_factory.h`. Most nodes have no ports; only `SimWait` has `sec` (double). No blackboard pointers or globals are used.
- **Goal discipline.** `NavBackend` admits at most one active goal. `send()` returns `std::optional<GoalId>` and returns `nullopt` if a goal is already RUNNING. GridSim also counts this as an `overlap` metric that must stay 0. `cancel(id)` is idempotent, and its result is immediate in GridSim. Every async node is a `BT::StatefulActionNode`: `onStart` sends, `onRunning` polls, and `onHalted` cancels.

  Cancel happens **before** the replacement goal is sent, by construction. Pre-emption comes from `BatteryOk` (the first child of a `ReactiveSequence`) returning FAILURE. In 4.11.0, `ReactiveSequence` then calls `resetChildren()`, which halts the running branch and cancels its goal, and only then returns FAILURE to the parent. The parent `ReactiveFallback` starts `DockAndCharge` after that. The prototype measured `overlap = 0` in all nine scenarios. A mutant that skips the cancel reaches `overlap = 1` (see §9).
- **Backend API**, in `include/bt_core/nav_backend.hpp`:
  - `send(GoalKind, GoalArgs) -> optional<GoalId>`
  - `poll(GoalId) -> GoalState{RUNNING, SUCCEEDED, FAILED, CANCELED}` plus a `FailReason{NONE, NO_PATH, BLOCKED, STUCK}`
  - `cancel(GoalId)`
  - `clearCostmap()`
  - `snapshot() -> WorldSample` (pose, battery, docked, brushes)
  - `step()`, called only by the runner

  `GoalKind` is one of `NavigateToPose`, `FollowLane` (drive to the lane start, then a strict straight lane with brushes on), `BackUp`, `Wait`, `Dock`, `Undock` and `Charge`.
- **Resume lives in `MissionState`, not in the tree's memory.** `SelectZone` is idempotent. If `active_zone >= 0`, it returns SUCCESS without popping the queue or touching `lane_idx`. Only `MarkZoneDone` and `MarkZoneSkipped` clear `active_zone`. Only `AdvanceLane` and `SkipLane` move `lane_idx`. A halted `FollowLane` therefore re-runs the **same** lane from its start after charging. Lanes already finished are never repeated, and each lane is completed at most once. The resume unit is a lane, not a waypoint; the README states this.
- **Battery latch.** `BatteryOk` is a condition with one side effect: it latches `need_charge = true` when `battery < CRIT`, or when `battery < LOW` and `at_boundary`. `at_boundary` is set by `AdvanceLane` and `SkipLane` and cleared by `FollowLane::onStart`. It returns `!need_charge`. The latch is cleared **only** by `ClearNeedCharge`, which runs after `Undock` succeeds. So rising past a threshold while charging cannot interrupt `Dock`, `Charge` or `Undock`, and a critical reading during the docking cycle cannot restart it. The prototype hit both of these bugs before the latch was moved after `Undock`.

## 5. Frozen tree (`trees/mission.xml`)

This XML is exactly what the prototype ran, apart from the SPDX comment:

```xml
<root BTCPP_format="4" main_tree_to_execute="Mission">
  <BehaviorTree ID="Mission">
    <Sequence>
      <Undock/>
      <RetryUntilSuccessful num_attempts="50">
        <ReactiveFallback name="Step">
          <IsQueueEmpty/>
          <ForceFailure>
            <ReactiveSequence name="Work">
              <BatteryOk/>
              <SubTree ID="CleanZone"/>
            </ReactiveSequence>
          </ForceFailure>
          <ForceFailure>
            <Sequence><NeedCharge/><SubTree ID="DockAndCharge"/></Sequence>
          </ForceFailure>
        </ReactiveFallback>
      </RetryUntilSuccessful>
      <SubTree ID="ReturnHome"/>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="CleanZone">
    <Sequence>
      <SelectZone/>
      <Fallback name="ZoneOrSkip">
        <Sequence>
          <RetryUntilSuccessful num_attempts="4">
            <Fallback><NavigateToZone/><ForceFailure><SubTree ID="NavRecovery"/></ForceFailure></Fallback>
          </RetryUntilSuccessful>
          <ForceSuccess>
            <KeepRunningUntilFailure>
              <Sequence>
                <HasNextLane/>
                <Fallback name="LaneOrSkip">
                  <Sequence>
                  <RetryUntilSuccessful num_attempts="3">
                    <Fallback><FollowLane/><ForceFailure><SubTree ID="NavRecovery"/></ForceFailure></Fallback>
                  </RetryUntilSuccessful>
                  <AdvanceLane/>
                  </Sequence>
                  <SkipLane/>
                </Fallback>
              </Sequence>
            </KeepRunningUntilFailure>
          </ForceSuccess>
          <MarkZoneDone/>
        </Sequence>
        <MarkZoneSkipped/>
      </Fallback>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="NavRecovery">
    <Fallback>
      <Sequence><IsStuck/><BackUp/></Sequence>
      <Sequence><ClearCostmap/><SimWait sec="5"/></Sequence>
    </Fallback>
  </BehaviorTree>

  <BehaviorTree ID="DockAndCharge">
    <Sequence>
      <RetryUntilSuccessful num_attempts="6">
        <Fallback><NavigateToDock/><ForceFailure><SubTree ID="NavRecovery"/></ForceFailure></Fallback>
      </RetryUntilSuccessful>
      <Dock/>
      <Charge/>
      <Undock/>
      <ClearNeedCharge/>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="ReturnHome">
    <Fallback>
      <IsDocked/>
      <Sequence>
        <RetryUntilSuccessful num_attempts="6">
          <Fallback><NavigateToDock/><ForceFailure><SubTree ID="NavRecovery"/></ForceFailure></Fallback>
        </RetryUntilSuccessful>
        <Dock/>
      </Sequence>
    </Fallback>
  </BehaviorTree>
</root>
```

Semantics the tests pin down:
- **Zone loop:** `RetryUntilSuccessful(50)` around `ReactiveFallback "Step"`. Each finished or skipped zone, and each charge cycle, ends in `ForceFailure`, so Retry starts the next iteration. `IsQueueEmpty` (`active_zone < 0 && queue.empty()`) is the only SUCCESS path. The bound of 50 is a safety cap, since a real run uses at most 3 zones plus 2 docks. `num_attempts="-1"` is banned. Retry can loop synchronously on children that fail immediately, so the runner also asserts at most 2,000 node status changes per tick.
- **Lane loop:** `KeepRunningUntilFailure` yields RUNNING after each lane and stops when `HasNextLane` fails. `ForceSuccess` then lets `MarkZoneDone` run. A lane is either followed and then advanced (`Sequence[Retry(3){FollowLane | recovery}, AdvanceLane]`) or skipped (`SkipLane` returns SUCCESS).

  The obvious alternative, `Fallback[Retry(..), ForceFailure(SkipLane)]` followed by `AdvanceLane`, ends the whole zone after the first skipped lane. The prototype hit exactly that bug: zone B coverage was 69%. The `lane_blocked_perm` gate (§8) now catches it.
- **Recovery:** first `IsStuck` (the last goal failed with `STUCK`) → `BackUp`; otherwise `ClearCostmap` → `SimWait 5 s`. Recovery is wrapped in `ForceFailure`, so every recovery counts as one failed attempt. Nav to zone has 4 attempts, so a zone is skipped after 4 failed navigations. FollowLane has 3 attempts before the lane is skipped. Nav to dock has 6 attempts.
- **Undock first.** The robot starts docked, so the mission starts with `Undock`. `ReturnHome` ensures the mission ends docked.

Node registry (all names must match the XML exactly):
- **Conditions:** `IsQueueEmpty`, `BatteryOk`, `NeedCharge`, `IsStuck`, `IsDocked`, `HasNextLane`.
- **Synchronous actions:** `SelectZone`, `AdvanceLane`, `SkipLane`, `MarkZoneDone`, `MarkZoneSkipped`, `ClearCostmap`, `ClearNeedCharge`.
- **StatefulActionNodes:** `Undock`, `NavigateToZone` (goal = start of the current lane), `NavigateToDock`, `FollowLane`, `Dock`, `Charge`, `BackUp`, `SimWait`.

`BT::writeTreeNodesModelXML(factory)` (the namespace-level function in `xml_parsing.h`) is exported to `trees/models.xml`. A test checks that it is up to date.

## 6. World model (`GridSim`, frozen)

**Grid:** 60 × 40 cells at 0.2 m (12 × 8 m), with the origin at cell (0, 0).

**Walls:**
- The border.
- The column x = 30.
- The row y = 20.

**Doors** (wall cells opened):
- A-B: x = 30, y 10-12.
- B-hall: x 44-46, y = 20.
- A-C: x 14-16, y = 20.
- C-hall: x = 30, y 28-30.

**Furniture:**
- In A: x 12-15, y 8-10.
- In B: x 40-42, y 12-13.

**Zones** (inclusive cell rectangles), processed in this order:

| Zone | Cells | Free cells | Lanes |
| --- | --- | --- | --- |
| A | (4,4)-(24,15) | 240 | 7 |
| B | (35,4)-(54,15) | 234 | 7 |
| C | (4,24)-(24,34) | 231 | 5 |

That makes **19 lanes in total**.

**Dock:** cell (55, 30), in the hall (x > 30, y > 20). The robot starts docked with battery `battery0`.

**Lanes:** rows `y = y0+1, y0+3, …` up to `y1`. Each row is split into free segments, keeping those at least 2 cells long. Row direction alternates (boustrophedon); an odd row reverses the order of its segments and their direction.

**Motion:**
- 0.4 m/s, i.e. 0.2 cells per tick.
- 8-connected A* with an octile heuristic, no corner cutting, and a fixed neighbour order (dx then dy, from -1 to 1).
- A* runs on the *costmap* (real obstacles plus phantom cells). Motion is checked against the *real* map. Before entering each next cell, the sim checks whether it is really occupied; if so, the goal fails with `BLOCKED` without moving.
- `collisions` counts entries into a really occupied cell. It must be 0. It is a separate count from planner failures and fault aborts.
- Coverage: while brushes are on (the straight part of a lane only), each cell entered marks its 3 × 3 neighbourhood as covered.
- Coverage % = covered free cells in the zone / free cells in the zone. The target masks are frozen at load time and the denominators never change.

**Battery** (clamped to ≤ 100; the run fails if it reaches 0):

| Effect | Rate |
| --- | --- |
| Motion drain | 0.3 per metre (√2 cells per diagonal step) |
| Time drain | 0.02 per second, except while charging |
| Charging | 1.0 per second, up to `CHARGED` |

Thresholds: `LOW = 25` (applies at lane boundaries), `CRIT = 10` (applies every tick), `CHARGED = 95`.

**Timed actions:**

| Action | Duration |
| --- | --- |
| `Dock` | 3 s |
| `Undock` | 2 s |
| `BackUp` | 2 s (clears `stuck`) |
| `SimWait` | `sec` |

**Order within a tick:** the runner ticks the tree once. Then it calls `GridSim::step()`, which applies scheduled events, advances the active goal, applies drain or charge, updates the minimum battery and increments `tick`. Then it writes the tick sample.

Termination comes when:
- the root returns non-RUNNING;
- the battery reaches ≤ 0 (failure); or
- `tick` reaches 15,000 (failure).

## 7. Scenarios (frozen; compiled-in tables)

| ID | Change from nominal |
| --- | --- |
| `nominal` | battery0 = 100, zones A, B, C |
| `low_battery` | battery0 = 30 |
| `critical_battery` | at tick 1200 (t = 120 s), battery = min(battery, 8) |
| `blocked_corridor` | cells x = 52, y 21-38 are really occupied until t = 12 s |
| `phantom_obstacle` | the same cells are costmap-only (phantom) until `ClearCostmap` |
| `stuck` | at t = 40 s the robot becomes stuck: each motion goal fails with `STUCK` after 2 s without progress, until a `BackUp` completes |
| `unreachable_zone` | the A-C and C-hall doors stay closed, so C has no path |
| `lane_obstacle` | when lane 3 of zone B starts, its midpoint cell is occupied for 8 s |
| `lane_blocked_perm` | the same, but the obstacle is permanent |

## 8. Numbered success criteria

The gates sit about 1.5× beyond the values the prototype measured (in brackets). In every row, "cov" means each listed zone's coverage % as defined in §6.

1. **Builds clean.** `cmake --preset default && ctest` passes on ubuntu-24.04 with g++-13, using `-Wall -Wextra -Wpedantic -Werror` on project code and the pinned FetchContent commits.
2. **XML loads.** `trees/mission.xml` loads through `createTreeFromText` with the full registry. Unknown or misspelled nodes fail. `models.xml` is up to date. The mission XML contains no `Timeout`, `Delay`, `Sleep` or `num_attempts="-1"`.
3. **`nominal`:** SUCCESS; cov A/B/C ≥ 95 [100/100/100]; t ≤ 400 s [263.7]; docks = 0; min battery ≥ 50 [62.6]; lanes done = 19; no halted nodes.
4. **`low_battery`:** SUCCESS; exactly 1 charge cycle; the dock is triggered at a lane boundary (no `FollowLane` halted); lanes done = 19, with each lane completed exactly once; cov ≥ 95 [100]; min battery ≥ 15 [21.5]; t ≤ 560 s [372.3].
5. **`critical_battery`:** SUCCESS; the running `FollowLane` is halted within 2 ticks of the battery event [halt at tick 1202; event applied at the step after tick 1200]; exactly 1 halted `FollowLane`; the next goal sent is `NavigateToDock`; exactly 1 charge cycle; the interrupted lane is re-run and lanes done = 19; cov ≥ 95 [100]; min battery > 2 [5.8]; t ≤ 590 s [391.8].
6. **`blocked_corridor`:** SUCCESS; ≥ 1 `ClearCostmap` and ≥ 1 `SimWait` [2/2]; 0 skipped zones; cov ≥ 95 [100]; t ≤ 410 s [273.9].
7. **`phantom_obstacle`:** SUCCESS; exactly 1 `ClearCostmap` [1]; 0 skipped; cov ≥ 95 [100]; t ≤ 405 s [268.8].
8. **`stuck`:** SUCCESS; exactly 1 `BackUp` [1]; 0 skipped; cov ≥ 95 [100]; t ≤ 405 s [270.0].
9. **`unreachable_zone`:** SUCCESS; exactly 1 skipped zone (C), after exactly 4 `NavigateToZone` attempts for C and 4 recoveries [4]; cov A/B ≥ 95 [100/100]; zone C cov = 0 is reported, and global coverage is reported (not hidden); lanes done = 14; ends docked.
10. **`lane_obstacle`:** SUCCESS; 0 skipped lanes; ≥ 1 recovery wait [1]; cov ≥ 95 [100].
11. **`lane_blocked_perm`:** SUCCESS; exactly 1 skipped lane, after 3 `FollowLane` attempts; lanes done = 18; cov B ≥ 90 [96.2]; cov A and C ≥ 95; the zone continues after the skip (the remaining lanes of B are done).
12. **Global invariants**, for every scenario on the product tree:
    - collisions = 0
    - goal overlap = 0
    - battery > 0 throughout
    - the mission ends docked
    - status changes per tick ≤ 2,000
13. **Determinism.** Two runs of each scenario in separate processes produce byte-identical `trace.jsonl` and `metrics.json`. CI also compares their SHA-256 against committed golden hashes. A golden update is allowed only with a metrics diff in the PR.
14. **Mutant `no_battery_guard.xml`** (`BatteryOk` removed) fails `low_battery` and `critical_battery`: the battery reaches 0 [at 208.8 s and 177.4 s].
15. **Mutant `nonreactive.xml`** (`ReactiveSequence "Work"` changed to `Sequence`) fails `critical_battery`: the battery reaches 0 [177.6 s].
16. **Mutant `no_recovery.xml`** (each recovery subtree replaced by `AlwaysFailure`):
    - `blocked_corridor` and `phantom_obstacle` end with cov 0 on all zones and 3 skipped zones;
    - `stuck` fails, with 6 skipped lanes and 2 skipped zones.
17. **Mutant build `MUTANT_NO_CANCEL`** (`onHalted` does not cancel) violates goal overlap = 0 in `critical_battery` [overlap = 1].
18. **Mutant build `MUTANT_RESET_LANE`** (`SelectZone` resets `lane_idx` on re-entry) violates "each lane exactly once" in `low_battery` [lanes done 20 of 19].
19. **ROS layer.** In the pinned Humble container, `colcon test` passes for `nav2_bt_mission`:
    - an in-process gtest (single-threaded executor) runs `nominal`, `stuck` and `critical_battery` through `RosNavBackend` against mock `nav2_msgs` action servers and asserts the same semantic gates (criteria 3, 5 and 8). Byte equality is not required.
    - the critical case asserts that the `FollowPath` goal reaches terminal CANCELED before the `NavigateToPose` dock goal is sent.
    - a `launch_testing` test starts the mock-server node and the mission node as separate processes and checks that `nominal` exits 0.
    - the readelf check finds no v3 library.
20. **Trace schema.** pytest validates the trace schema: `schema_version`, `scenario`, `tick`, sample keys, the status enum, and monotonic ticks. It rejects a trace with a sample missing `battery`.
21. **Viewer.** The Pages viewer, built with base `/nav2-behavior-trees/`, loads the manifest and all 9 traces. Playwright, on the built site:
    - plays, pauses, steps one tick, and scrubs backwards;
    - checks the DOM readouts (tick, battery, active leaf, node colours) at known ticks of `critical_battery`, including the halt at tick 1202;
    - fails on console errors or failed requests;
    - writes `docs/demo.png`.
22. **Licensing and docs.**
    - `LICENSE` is byte-identical to the Apache-2.0 text (SHA-256 `cfc7749b96f63bd31c3c42b5c471bf756814053e847c10f3eb003417bc523d30`).
    - `NOTICE` begins `nav2-behavior-trees` / `Copyright 2026 Arpan Bom` and lists BT.CPP (MIT) and GoogleTest (BSD-3-Clause).
    - Authored source files carry an SPDX header (JSON files are exempt).
    - The README states simulation-only scope, says lane-level resume, contrasts the sibling repos, and links each criterion to its test.
23. **History.** Every commit's author and committer is `Arpan Bom <applycoding@gmail.com>`, with no `Co-authored-by` trailers. CI checks this with `git log --format='%an <%ae>|%cn <%ce>|%b'`.

## 9. Verification record

A throwaway C++ prototype (not shipped) implemented §5-§7 against BehaviorTree.CPP 4.11.0 built from source with g++ 14. Each scenario ran twice, and the transition-trace hashes matched in all runs.

| Scenario | Product tree | Notes |
| --- | --- | --- |
| nominal | ok, 263.7 s, 100/100/100, min 62.6, 0 docks | 0 halts |
| low_battery | ok, 372.3 s, 100/100/100, min 21.5, 1 dock | 19/19 lanes, 0 halts |
| critical_battery | ok, 391.8 s, 100/100/100, min 5.8, 1 dock | `FollowLane` halted at tick 1202 |
| blocked_corridor | ok, 273.9 s | 2 clears, 2 waits |
| phantom_obstacle | ok, 268.8 s | 1 clear |
| stuck | ok, 270.0 s | 1 backup |
| unreachable_zone | ok, 208.5 s, A/B 100, C 0 | 1 zone skipped after 4 attempts, 14 lanes |
| lane_obstacle | ok, 278.3 s | 1 wait, 0 skipped |
| lane_blocked_perm | ok, 298.0 s, B 96.2 | 1 lane skipped, 18 lanes |

Goal overlap was 0 and collisions were 0 in every product run.

The mutant results are recorded in criteria 14-18. `lane_obstacle` does **not** separate the no-recovery mutant (an 8 s obstacle clears on its own within 3 retries), so it is not used as a negative gate.

## 10. Trace format

**`traces/<scenario>/trace.jsonl`:**
- Line 1 is a header: `schema_version: 1`, `scenario`, `dt_ms: 100`, `tree_sha256`, `nodes` (uid, fullPath, type, parent uid in tree order), `map` (walls, zones, dock) and `lanes`.
- After that, the file has two kinds of lines:
  - `{"k":"s","tick":…,"pose":[x,y],"battery":…,"docked":…,"brushes":…,"cov":[a,b,c]}` once per tick, plus one final sample;
  - `{"k":"t","tick":…,"seq":…,"uid":…,"from":"IDLE","to":"RUNNING"}` for each status change, with `seq` giving the order within the tick.
- Numbers use fixed formatting: pose and battery with 3 decimal places, and coverage with 1, under the C locale. Keys are in a fixed order.

**`metrics.json`** holds the final counts and every gate value.

The viewer is a TypeScript and Canvas 2D app with no framework. It reconstructs node statuses at any tick by replaying transitions from the nearest checkpoint (every 100 ticks). It shows the map, robot, lanes, coverage, a battery gauge and the tree, coloured by status. The DOM readouts exist for tests.

## 11. ROS 2 layer (`ros2/nav2_bt_mission`)

`RosNavBackend` maps goal kinds to Humble `nav2_msgs` interfaces at their default names:

| Goal kind | Endpoint | Message |
| --- | --- | --- |
| `NavigateToZone` / `NavigateToDock` | `/navigate_to_pose` | `NavigateToPose`: `PoseStamped pose`, `string behavior_tree`; result is `std_msgs/Empty` |
| `FollowLane` | `/follow_path` | `FollowPath`: `nav_msgs/Path path`, `controller_id`, `goal_checker_id`. The path is A* to the lane start plus the lane. |
| `BackUp` | `/backup` | `BackUp`: `geometry_msgs/Point target`, `float32 speed`, `Duration time_allowance` |
| `SimWait` | `/wait` | `Wait`: `Duration time` |
| `ClearCostmap` | `/global_costmap/clear_entirely_global_costmap` | `ClearEntireCostmap`, with `std_msgs/Empty` request and response |

Success is the action's `ResultCode::SUCCEEDED`, not the Empty result. ABORTED maps to FAILED and CANCELED maps to CANCELED. A rejected goal maps to FAILED.

Async rules:
- A cancel requested before the goal is accepted is sent as soon as the goal handle arrives.
- A per-goal generation token drops stale callbacks.
- `ClearCostmap` is async, with a 2 s sim-time deadline (FAILED on timeout).

`Dock`, `Undock` and `Charge` have no Nav2 Humble equivalent. They are mock-only services (`/mock_dock/dock`, `/mock_dock/undock`, `/mock_dock/charge` actions using `nav2_msgs/action/Wait` as a carrier type). The README labels this as simulated.

The mock servers wrap one `GridSim` that a single owner advances. In the in-process gtest, a single-threaded executor serves both the client and the servers, and the runner steps the sim once per tick after `spin_some` reaches a quiescent state.

The manual Nav2 + Gazebo path (`launch/nav2_tb3_manual.launch.py`, documented and not tested) uses `nav2_bringup tb3_simulation_launch.py` and maps grid cells onto the TB3 world. Battery and dock stay simulated there.

## 12. Repository layout

```
CMakeLists.txt CMakePresets.json LICENSE NOTICE README.md REFERENCES.md
include/bt_core/ src/ (nodes, mission_state, grid_sim, runner, metrics, trace_writer)
apps/bt_mission.cpp
trees/mission.xml trees/models.xml trees/mutants/{no_battery_guard,nonreactive,no_recovery}.xml
tests/ (gtest: registry, xml, gridsim unit, scenarios, mutants, determinism, golden)
traces/golden.sha256
python/ (trace_schema test, requirements.txt)
web/ (Vite + TS viewer, Vitest, Playwright)
ros2/nav2_bt_mission/ (ament_cmake package, mock servers, gtest, launch_test, manual launch)
docs/demo.png
.github/workflows/{ci.yml,ros.yml,pages.yml}
```

## 13. CI

- **`ci.yml`** (push and PR): configure, build and run ctest, including the mutants built as separate targets with `-DMUTANT_*` compile definitions. Then generate the traces, check the golden hashes, run pytest, run Vitest and the type check (`tsc --noEmit`), and upload the traces and metrics.
- **`ros.yml`** (push and PR): the pinned container digest, then rosdep, colcon build and colcon test, with `colcon test-result --verbose` failing the job.
- **`pages.yml`** (push to `main`, plus `workflow_dispatch`): one workflow with dependent jobs. It builds the core and generates the traces in the same run, copies them to `web/public/traces` **before** `vite build`, runs Playwright against `vite preview` at the base path, and uploads and deploys `web/dist`.

Pages must be enabled with source "GitHub Actions".

## 14. Risks

| Risk | Handling |
| --- | --- |
| BT.CPP behaviour drift between versions | Pinned commit; the semantics tests in criteria 5, 11, 17 and 18 catch it. |
| Apt `behaviortree_cpp` not at 4.11.0 | Logged fallback to FetchContent. |
| ROS callback timing | Semantic gates only; byte goldens apply to `GridSim` only. |
| Manual Gazebo divergence | README says the CI gates apply to `GridSim` only. |
| Golden churn | Change goldens only alongside a metrics diff. |

## 15. Licence and provenance

- Outbound licence: Apache-2.0.
- No third-party source is copied.
- BehaviorTree.CPP and GoogleTest are dependencies, not vendored forks.
- Nav2 is mixed-licence (its root LICENSE lists LGPL-2.1-or-later, Apache-2.0 and BSD-3-Clause). Only `nav2_msgs` interfaces are used, and no Nav2 source is copied (in particular no LGPL `nav2_amcl` code).
- See REFERENCES.md.
