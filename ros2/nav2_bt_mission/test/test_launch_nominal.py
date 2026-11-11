# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Arpan Bom

import os
import re
import unittest

import launch_testing
import launch_testing.markers
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    SetEnvironmentVariable,
)
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest


@launch_testing.markers.keep_alive
def generate_test_description():
    extra_env = []
    btcpp_lib = '/opt/btcpp-4.11/lib'
    if os.path.isdir(btcpp_lib):
        extra_env.append(AppendEnvironmentVariable('LD_LIBRARY_PATH', btcpp_lib, separator=':'))

    mission_proc = Node(
        package='nav2_bt_mission',
        executable='bt_mission_stack',
        arguments=['--scenario', 'nominal'],
        output='screen',
    )

    return (
        LaunchDescription([
            SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
            SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
            *extra_env,
            mission_proc,
            ReadyToTest(),
        ]),
        {
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

    def test_mission_drove_remote_sim(self, proc_output, mission_proc):
        io = proc_output[mission_proc]
        output = ''.join(io.stdout or []) + ''.join(io.stderr or [])
        self.assertRegex(
            output,
            r'success=1\b',
            msg='mission stdout must report success=1 (mock action path)',
        )
        steps = 0
        for line in output.splitlines():
            m = re.search(r'remote_steps=(\d+)', line)
            if m:
                steps = int(m.group(1))
        self.assertGreater(
            steps,
            1000,
            msg='mission must step the remote mock sim (remote_steps>1000)',
        )
        lanes = -1
        for line in output.splitlines():
            m = re.search(r'lanes=(\d+)', line)
            if m:
                lanes = int(m.group(1))
        self.assertEqual(lanes, 19, msg='nominal lanes_done must be 19')
