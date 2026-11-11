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
    SetEnvironmentVariable,
)
from launch_testing.actions import ReadyToTest


def install_bin(name: str) -> str:
    prefix = get_package_prefix('nav2_bt_mission')
    return os.path.join(prefix, 'lib', 'nav2_bt_mission', name)


@launch_testing.markers.keep_alive
def generate_test_description():
    prefix = get_package_prefix('nav2_bt_mission')
    script = os.path.join(prefix, 'lib', 'nav2_bt_mission', 'run_nominal_cross.sh')
    fastdds_xml = os.path.join(os.path.dirname(__file__), 'fastdds_ci.xml')

    proc_env = {
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
        'ROS_LOCALHOST_ONLY': '1',
        'ROS_DOMAIN_ID': '0',
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'FASTRTPS_DEFAULT_PROFILES_FILE': fastdds_xml,
    }
    btcpp_lib = '/opt/btcpp-4.11/lib'
    if os.path.isdir(btcpp_lib):
        proc_env['LD_LIBRARY_PATH'] = btcpp_lib + os.pathsep + os.environ.get('LD_LIBRARY_PATH', '')

    cross_proc = ExecuteProcess(
        cmd=['bash', script, install_bin('mock_nav2_servers'), install_bin('bt_mission_node')],
        output='screen',
        additional_env=proc_env,
        name='nominal_cross_process',
    )

    return (
        LaunchDescription([
            SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
            SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
            SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
            SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
            SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_xml),
            *([AppendEnvironmentVariable('LD_LIBRARY_PATH', btcpp_lib, separator=':')]
              if os.path.isdir(btcpp_lib) else []),
            cross_proc,
            ReadyToTest(),
        ]),
        {
            'cross_proc': cross_proc,
        },
    )


class TestNominalLaunchWait(unittest.TestCase):

    def test_wait_for_cross_process_run(self, proc_info, cross_proc):
        proc_info.assertWaitForStartup(process=cross_proc, timeout=60)
        proc_info.assertWaitForShutdown(process=cross_proc, timeout=1200)


@launch_testing.post_shutdown_test()
class TestNominalProcesses(unittest.TestCase):

    def test_cross_process_exit_code(self, proc_info, cross_proc):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=cross_proc, allowable_exit_codes=[0])

    def test_mission_drove_remote_sim(self, proc_output, cross_proc):
        chunks = proc_output[cross_proc]

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
