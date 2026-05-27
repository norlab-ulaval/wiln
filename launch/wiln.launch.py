"""
wiln.launch.py -- Single entry-point launch for the full WILN teach-and-repeat stack.

Usage:
    ros2 launch wiln wiln.launch.py
    ros2 launch wiln wiln.launch.py platform:=mtt
    ros2 launch wiln wiln.launch.py platform:=mtt launch_safety_monitor:=false

Nodes launched:
    wiln_obstacle_node   -- lidar aggregation + filtering  (C++)
    wiln_teach_node      -- recording                      (C++)
    wiln_route_node      -- save/load .ltr files           (C++)
    wiln_replay_node     -- hot-path 10 Hz elastic band    (C++)
    wiln_path_follower   -- 20 Hz curvature controller     (C++)
    wiln_safety_monitor  -- passive watchdog               (Python, optional)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('wiln')

    # ----- Launch arguments -----
    platform_arg = DeclareLaunchArgument(
        'platform',
        default_value='generic',
        description='Platform name. Loads config/platforms/<platform>.yaml as an overlay.',
    )
    launch_safety_arg = DeclareLaunchArgument(
        'launch_safety_monitor',
        default_value='true',
        description='Set false to skip wiln_safety_monitor (useful for CI/testing).',
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='ROS log level for all wiln nodes.',
    )

    platform = LaunchConfiguration('platform')
    launch_safety = LaunchConfiguration('launch_safety_monitor')
    log_level = LaunchConfiguration('log_level')

    # ----- Config files -----
    base_params = os.path.join(pkg_share, 'config', 'wiln_params.yaml')

    # Platform overlay is loaded conditionally via OpaqueFunction
    def make_nodes(context):
        platform_str = context.launch_configurations.get('platform', 'generic')
        platform_params_path = os.path.join(
            pkg_share, 'config', 'platforms', f'{platform_str}.yaml')

        # Build params list: base always, platform overlay if it exists
        params_files = [base_params]
        if os.path.isfile(platform_params_path):
            params_files.append(platform_params_path)
        else:
            import warnings
            warnings.warn(
                f'[wiln] No platform config found at {platform_params_path} — '
                'using generic defaults.')

        log_lvl = context.launch_configurations.get('log_level', 'info')
        launch_sfty = context.launch_configurations.get('launch_safety_monitor', 'true')

        nodes = [
            # ----------------------------------------------------------------
            # wiln_obstacle_node
            # ----------------------------------------------------------------
            Node(
                package='wiln',
                executable='wiln_obstacle_node',
                name='wiln_obstacle_node',
                parameters=params_files,
                arguments=['--ros-args', '--log-level', log_lvl],
                output='screen',
            ),

            # ----------------------------------------------------------------
            # wiln_teach_node
            # ----------------------------------------------------------------
            Node(
                package='wiln',
                executable='wiln_teach_node',
                name='wiln_teach_node',
                parameters=params_files,
                arguments=['--ros-args', '--log-level', log_lvl],
                output='screen',
            ),

            # ----------------------------------------------------------------
            # wiln_route_node
            # ----------------------------------------------------------------
            Node(
                package='wiln',
                executable='wiln_route_node',
                name='wiln_route_node',
                parameters=params_files,
                arguments=['--ros-args', '--log-level', log_lvl],
                output='screen',
            ),

            # ----------------------------------------------------------------
            # wiln_replay_node
            # ----------------------------------------------------------------
            Node(
                package='wiln',
                executable='wiln_replay_node',
                name='wiln_replay_node',
                parameters=params_files,
                arguments=['--ros-args', '--log-level', log_lvl],
                output='screen',
            ),

            # ----------------------------------------------------------------
            # wiln_path_follower
            # ----------------------------------------------------------------
            Node(
                package='wiln',
                executable='wiln_path_follower',
                name='wiln_path_follower',
                parameters=params_files,
                arguments=['--ros-args', '--log-level', log_lvl],
                output='screen',
            ),

            # ----------------------------------------------------------------
            # wiln_safety_monitor (Python, optional)
            # ----------------------------------------------------------------
            Node(
                package='wiln',
                executable='wiln_safety_monitor.py',
                name='wiln_safety_monitor',
                parameters=params_files,
                arguments=['--ros-args', '--log-level', log_lvl],
                output='screen',
                condition=IfCondition(launch_sfty),
            ),
        ]
        return nodes

    return LaunchDescription([
        platform_arg,
        launch_safety_arg,
        log_level_arg,
        OpaqueFunction(function=make_nodes),
    ])
