# References

These are ideas and interfaces only. No third-party source is copied or translated. Licences were checked against each project's LICENSE file on 2026-11-10.

## Dependencies (linked, not vendored)

| Project | Version | Licence | Use |
| --- | --- | --- | --- |
| [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) | 4.11.0 (`8dad25dc…`) | MIT ([LICENSE](https://raw.githubusercontent.com/BehaviorTree/BehaviorTree.CPP/4.11.0/LICENSE)) | Tree engine, through its public API (`StatefulActionNode`, `ReactiveSequence`/`ReactiveFallback`, `RetryUntilSuccessful`, `KeepRunningUntilFailure`, `StatusChangeLogger`, `writeTreeNodesModelXML`). The docs are at https://www.behaviortree.dev/. |
| [GoogleTest](https://github.com/google/googletest) | v1.18.0 | BSD-3-Clause ([LICENSE](https://raw.githubusercontent.com/google/googletest/v1.18.0/LICENSE)) | Tests |
| [nav2_msgs (Nav2 Humble)](https://github.com/ros-navigation/navigation2/tree/humble/nav2_msgs) | navigation2 1.1.20 | Apache-2.0 (package.xml). The repo root [LICENSE](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/LICENSE) lists LGPL-2.1-or-later, Apache-2.0 and BSD-3-Clause, per package. | Action and service *interfaces* only: `NavigateToPose`, `FollowPath`, `BackUp`, `Wait`, `ClearEntireCostmap` |
| Node 26.13.0, TypeScript 7.0.2, Vite 8.3.7, Vitest 5.1.0, @playwright/test 1.65.0, pytest 9.2.0 | as listed | MIT / Apache-2.0 / MIT / MIT / Apache-2.0 / MIT | Viewer and tests |

## Idea sources (read for concepts, not copied)

| Project | Licence (LICENSE file) | What we take from it |
| --- | --- | --- |
| [Nav2 `nav2_behavior_tree` / `nav2_bt_navigator`](https://github.com/ros-navigation/navigation2) (Humble) | Apache-2.0 per package.xml. Nav2 is mixed-licence; `nav2_amcl` is LGPL-2.1-or-later and is **not** read or used. | The pattern of recovery around navigation (clear costmap, back up, wait). Humble `bt_navigator` depends on `behaviortree_cpp_v3`, which is why this project runs as a separate v4 process. |
| [BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2) | Apache-2.0 ([LICENSE](https://raw.githubusercontent.com/BehaviorTree/BehaviorTree.ROS2/humble/LICENSE)). Note that its `behaviortree_ros2/package.xml` on `humble` declares MIT. | Shape of a BT action-client wrapper. Not a dependency; our client is written from scratch on `rclcpp_action`. |
| [opennav_docking](https://github.com/open-navigation/opennav_docking) | Apache-2.0 ([LICENSE](https://raw.githubusercontent.com/open-navigation/opennav_docking/main/LICENSE)). Humble release 0.0.2 in rosdistro. | The undock, approach, charge and undock cycle as a concept. Not used; docking is simulated. |
| [opennav_coverage](https://github.com/open-navigation/opennav_coverage) | Apache-2.0 ([LICENSE](https://raw.githubusercontent.com/open-navigation/opennav_coverage/main/LICENSE)). It has a `humble` branch but is not released in the Humble rosdistro. | Boustrophedon coverage as a concept. Not used; lanes are a fixed row sweep. |
| [Fields2Cover](https://github.com/Fields2Cover/Fields2Cover) | BSD-3-Clause ([LICENSE](https://raw.githubusercontent.com/Fields2Cover/Fields2Cover/main/LICENSE)) | Background on coverage path planning. Not used. |
| Colledanchise and Ögren, *Behavior Trees in Robotics and AI: An Introduction* (CRC Press, 2018; arXiv:1709.00084) | Book/paper | Reactive sequences and fallbacks, and the priority/pre-emption semantics |

## Not used

- **Groot / Groot2.** Groot 1 targets BT.CPP 3.8. Groot2 is a proprietary freemium tool, and no LICENSE file is published in its repository. Neither is used, and `BTCPP_GROOT_INTERFACE` is OFF.
- GPL or LGPL robotics code is not read for implementation.

## Sibling portfolio repos (contrast)

- `ros2-nav2-slam-sim`: a Python YAML waypoint mission runner with an FSM, retries and a battery stub.
- `nav2-coverage-planner-compare`: compares Nav2 planners.

This repo is the behavior-tree executive that the waypoint runner is not.
