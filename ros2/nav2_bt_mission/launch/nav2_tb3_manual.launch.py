# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Arpan Bom

"""Manual TB3 + Nav2 path (not CI-gated).

Bring up Gazebo Classic via nav2_bringup, then run bt_mission_node against live action servers.
Grid battery and dock behaviour stay simulated in RosNavBackend; map alignment is approximate.
See PLAN.md section 2 (manual only).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_share = get_package_share_directory('nav2_bringup')
    tb3_launch = os.path.join(bringup_share, 'launch', 'tb3_simulation_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='nominal'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(tb3_launch)),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'nav2_bt_mission', 'bt_mission_node',
                '--scenario', LaunchConfiguration('scenario'),
            ],
            output='screen',
        ),
    ])
