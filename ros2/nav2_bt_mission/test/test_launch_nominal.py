# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Arpan Bom

import os
import re
import unittest

import launch_testing
import launch_testing.markers
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch_testing.actions import ReadyToTest


def install_bin(name: str) -> str:
    prefix = get_package_prefix('nav2_bt_mission')
    return os.path.join(prefix, 'lib', 'nav2_bt_mission', name)


def rmw_env() -> dict:
    cyclone_xml = os.path.join(os.path.dirname(__file__), 'cyclonedds_ci.xml')
    env = {
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
        'ROS_LOCALHOST_ONLY': '1',
        'ROS_DOMAIN_ID': '0',
        'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
        'CYCLONEDDS_URI': f'file://{cyclone_xml}',
    }
    btcpp_lib = '/opt/btcpp-4.11/lib'
    if os.path.isdir(btcpp_lib):
        env['LD_LIBRARY_PATH'] = btcpp_lib + os.pathsep + os.environ.get('LD_LIBRARY_PATH', '')
    return env


@launch_testing.markers.keep_alive
def generate_test_description():
    proc_env = rmw_env()
    cyclone_xml = os.path.join(os.path.dirname(__file__), 'cyclonedds_ci.xml')

    mock_proc = ExecuteProcess(
        cmd=[install_bin('mock_nav2_servers')],
        output='screen',
        additional_env=proc_env,
        name='mock_nav2_servers',
    )
    mission_proc = ExecuteProcess(
        cmd=[install_bin('bt_mission_node'), '--scenario', 'nominal'],
        output='screen',
        additional_env=proc_env,
        name='bt_mission_node',
    )

    return (
        LaunchDescription([
            SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
            SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
            SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
            SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
            SetEnvironmentVariable('CYCLONEDDS_URI', f'file://{cyclone_xml}'),
            *([AppendEnvironmentVariable('LD_LIBRARY_PATH', '/opt/btcpp-4.11/lib', separator=':')]
              if os.path.isdir('/opt/btcpp-4.11/lib') else []),
            mock_proc,
            TimerAction(period=3.0, actions=[mission_proc]),
            RegisterEventHandler(OnProcessExit(target_action=mission_proc, on_exit=[Shutdown()])),
            ReadyToTest(),
        ]),
        {
            'mock_proc': mock_proc,
            'mission_proc': mission_proc,
        },
    )


class TestNominalLaunchWait(unittest.TestCase):

    def test_wait_for_processes(self, proc_info, mock_proc, mission_proc):
        proc_info.assertWaitForStartup(process=mock_proc, timeout=60)
        proc_info.assertWaitForStartup(process=mission_proc, timeout=60)
        proc_info.assertWaitForShutdown(process=mission_proc, timeout=1200)


@launch_testing.post_shutdown_test()
class TestNominalProcesses(unittest.TestCase):

    def test_process_exit_codes(self, proc_info, mock_proc, mission_proc):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=mission_proc, allowable_exit_codes=[0])
        launch_testing.asserts.assertExitCodes(
            proc_info, process=mock_proc, allowable_exit_codes=[0, -15, 15])

    def test_mission_drove_remote_sim(self, proc_output, mission_proc):
        chunks = proc_output[mission_proc]

        def as_text(part):
            raw = part.text if hasattr(part, 'text') else part
            if isinstance(raw, bytes):
                return raw.decode('utf-8', errors='replace')
            return str(raw)

        if isinstance(chunks, (list, tuple)):
            output = ''.join(as_text(p) for p in chunks)
        else:
            stdout = chunks.stdout or []
            stderr = chunks.stderr or []
            output = ''.join(as_text(p) for p in stdout) + ''.join(as_text(p) for p in stderr)
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
