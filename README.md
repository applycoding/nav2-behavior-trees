# nav2-behavior-trees

Apache-2.0. Author: Arpan Bom.

**Simulation only.** This repository is an original implementation of a behavior-tree mission executive, inspired by the sources listed in `REFERENCES.md`. **BehaviorTree.CPP 4.11.0** is a third-party library; this project uses its public C++ API only (FetchContent at configure time). It demonstrates a reactive mission for a *cleaning robot in a deterministic grid world*. No real robots, no production cleaning, and no employment or certification claims. Resume after charging is **lane-level** (interrupted `FollowLane` goals restart the same lane from its start; finished lanes are not repeated).

Sibling repos: [`ros2-nav2-slam-sim`](https://github.com/applycoding/ros2-nav2-slam-sim) (Python waypoint FSM), [`nav2-coverage-planner-compare`](https://github.com/applycoding/nav2-coverage-planner-compare) (planner bake-off). This tree shows reactive pre-emption, docking, and negative mutant evidence.

**Live demo:** [GitHub Pages trace viewer](https://applycoding.github.io/nav2-behavior-trees/).

## Why a separate process from Nav2

Humble `nav2_bt_navigator` links **BehaviorTree.CPP v3** (`behaviortree_cpp_v3`). This project uses **v4** (same `BT` namespace). A single process must not link both; the mission node is a **BT.CPP v4 client** of Nav2 actions (`nav2_msgs` only). ROS CI always runs in-process gtests where `RosNavBackend` and `MockNav2` share one executor: every goal uses `async_send_goal`, `poll` on handles, and `async_cancel_goal` into mock servers on one `GridSim` (no coupled send shortcut). Those tests assert nominal, stuck, and critical_battery gates, cancel-before-dock ordering, and reject-all failure. `colcon test` also runs `test_launch_nominal.py` with separate `mock_nav2_servers` and `bt_mission_node` processes; see DEVIATIONS.md if that launch step cannot see peers in the Actions runner image.

## Mission tree (excerpt)

```xml
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
```

Full tree: `trees/mission.xml`. Models: `trees/models.xml`.

## Scenario results (measured in CI `GridSim`)

| Scenario | Success gates (summary) | Measured `t` (s) |
| --- | --- | ---: |
| `nominal` | cov ≥95% all zones, 19 lanes, 0 docks | 308.8 |
| `low_battery` | 1 charge, no halted `FollowLane`, 19 lanes | 434.1 |
| `critical_battery` | 1 halt, 1 charge, 19 lanes | 441.3 |
| `blocked_corridor` | ≥1 clear + wait, cov ≥95% | 319.0 |
| `phantom_obstacle` | exactly 1 `ClearCostmap` | 313.9 |
| `stuck` | exactly 1 `BackUp` | 318.2 |
| `unreachable_zone` | skip C, 14 lanes | 240.7 |
| `lane_obstacle` | 0 skipped lanes, ≥1 recovery wait | 322.8 |
| `lane_blocked_perm` | 1 skipped lane, 18 lanes, cov B ≥90% | 341.2 |

See `PLAN.md` §8 for full thresholds. Timing is above the throwaway prototype band; see `DEVIATIONS.md`. Golden trace and `metrics.json` hashes in `traces/golden.sha256` are recorded for **g++-13 / libstdc++ on ubuntu-24.04**. A **clang / libc++** build yields different trace bytes for all nine scenarios.

## Mutants (must fail named gates)

| Mutant | Proves |
| --- | --- |
| `no_battery_guard.xml` | Low/critical battery must not reach 0 on product path |
| `nonreactive.xml` | `ReactiveSequence` needed for critical pre-emption |
| `no_recovery.xml` | Recovery required for corridor/phantom/stuck |
| `MUTANT_NO_CANCEL` | `onHalted` must cancel goals (`overlap = 0`) |
| `MUTANT_RESET_LANE` | `SelectZone` must not reset `lane_idx` on re-entry |

## How this was verified

| Criterion | Tests / checks |
| --- | --- |
| 1 Build | `cmake --preset default && ctest` (`ci.yml`) |
| 2 XML / registry | `Registry.*`, banned-node grep |
| 3-11 Scenarios | `Scenarios.*` gtests |
| 12 Invariants | `expectGlobalInvariants` in scenario tests |
| 13 Determinism | `Determinism.*` (trace + metrics bytes) |
| 14-18 Mutants | `Mutants.*`, `MutantBuild.*` |
| 19 ROS | `ros.yml` → `colcon test` + readelf |
| 20 Schema | `python/test_trace_schema.py` |
| 21 Viewer | Vitest + Playwright (`web/`) → `docs/demo.png` |
| 22 Docs | This file, `LICENSE`, `NOTICE` |
| 23 History | Authorship job in `ci.yml` |

## Run locally

```bash
cmake --preset default
cmake --build build --parallel
ctest --test-dir build --output-on-failure
./build/bt_mission --scenario nominal --tree trees/mission.xml --out traces/out
```

**Viewer:** `cd web && npm ci && npm run copy-traces && npm test && npm run build`

**ROS 2 (optional):** build with `colcon` inside pinned `ros:humble-ros-base-jammy` (see `ros.yml`). Start `mock_nav2_servers`, then `bt_mission_node` or run `colcon test` for in-process action round trips and the nominal launch test. Manual Gazebo TB3: `ros2/nav2_bt_mission/launch/nav2_tb3_manual.launch.py` (documented, not CI).

Planning sources: `PLAN.md`, `TASKS.md`. Deviations: `DEVIATIONS.md`.
