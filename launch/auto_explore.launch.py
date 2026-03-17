#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('auto_explore_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    rviz_config = LaunchConfiguration('rviz_config')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    explorer_params_file = LaunchConfiguration('explorer_params_file')

    default_rviz_config = os.path.join(pkg_dir, 'rviz', 'nav2_tuning.rviz')
    if not os.path.isfile(default_rviz_config):
        default_rviz_config = os.path.join(pkg_dir, 'rviz', 'explore.rviz')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'nav2_tuning.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
            'headless': headless,
            'nav2_params_file': params_file,
            'slam_params_file': slam_params_file,
            'rviz_config': rviz_config,
        }.items(),
    )

    frontier_explorer = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='auto_explore_sim',
                executable='frontier_explorer',
                name='frontier_explorer',
                output='screen',
                parameters=[explorer_params_file, {'use_sim_time': use_sim_time}],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to start RViz',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo headless (no GUI)',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='RViz config file',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
            description='Nav2 parameters file',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml'),
            description='SLAM Toolbox parameters file',
        ),
        DeclareLaunchArgument(
            'explorer_params_file',
            default_value=os.path.join(pkg_dir, 'config', 'frontier_explorer_params.yaml'),
            description='Frontier explorer parameters file',
        ),
        bringup_launch,
        frontier_explorer,
    ])
