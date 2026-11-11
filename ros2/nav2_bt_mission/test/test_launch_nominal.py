# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Arpan Bom

import re
import unittest

import launch_testing
import launch_testing.markers
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable, Shutdown, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest


@launch_testing.markers.keep_alive
def generate_test_description():
    mock_proc = Node(
        package='nav2_bt_mission',
        executable='mock_nav2_servers',
        arguments=['--scenario', 'nominal'],
        output='screen',
    )
    mission_proc = Node(
        package='nav2_bt_mission',
        executable='bt_mission_node',
        arguments=['--scenario', 'nominal'],
        output='screen',
    )

    return (
        LaunchDescription([
            SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
            SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
            mock_proc,
            TimerAction(period=10.0, actions=[mission_proc]),
            RegisterEventHandler(
                OnProcessExit(target_action=mission_proc, on_exit=Shutdown()),
            ),
            ReadyToTest(),
        ]),
        {
            'mock_proc': mock_proc,
            'mission_proc': mission_proc,
        },
    )


class TestNominalLaunchWait(unittest.TestCase):

    def test_wait_for_mission_shutdown(self, proc_info, mission_proc):
        proc_info.assertWaitForStartup(process=mission_proc, timeout=60)
        proc_info.assertWaitForShutdown(process=mission_proc, timeout=1200)


@launch_testing.post_shutdown_test()
class TestNominalProcesses(unittest.TestCase):

    def test_mission_runs(self, proc_info, mission_proc):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=mission_proc, allowable_exit_codes=[0])
