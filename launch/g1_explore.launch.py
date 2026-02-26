"""
All-in-one launch for autonomous exploration with Unitree G1 humanoid:
  Gazebo (office world) + G1 Robot + SLAM Toolbox + Nav2 + Frontier Explorer + RViz

Based on auto_explore.launch.py but uses the Unitree G1 humanoid robot
instead of TurtleBot3 Waffle. The G1 uses a hidden diff-drive base with
the humanoid visual mesh on top.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription

from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Package directories
    pkg_dir = get_package_share_directory('auto_explore_sim')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to start RViz')

    declare_headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo headless (no GUI)')

    # File paths
    world_file = os.path.join(pkg_dir, 'worlds', 'office.sdf')
    nav2_params_file = os.path.join(pkg_dir, 'config', 'g1_nav2_params.yaml')
    slam_params_file = os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml')
    explore_params_file = os.path.join(pkg_dir, 'config', 'frontier_explorer_params.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'explore.rviz')

    # G1 model paths
    g1_model_dir = os.path.join(pkg_dir, 'models', 'g1_description')
    g1_sdf_file = os.path.join(g1_model_dir, 'model.sdf')
    g1_urdf_file = os.path.join(g1_model_dir, 'g1.urdf')

    # Nav2 params with substitutions
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params_file,
            root_key='',
            param_rewrites={'autostart': 'true'},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Robot URDF for robot_state_publisher
    with open(g1_urdf_file, 'r') as f:
        robot_description = f.read()

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Set GZ_SIM_RESOURCE_PATH so Gazebo can find G1 meshes
    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_dir, 'models', 'g1_description')
        + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''))

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # ===== 1. Gazebo Server =====
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_file],
        output='screen',
    )

    # ===== 2. Gazebo Client (GUI) =====
    gazebo_client = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
        condition=UnlessCondition(headless),
    )

    # ===== 3. Spawn G1 in Gazebo =====
    # Read the SDF file content for spawning
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'unitree_g1',
            '-file', g1_sdf_file,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
            '-Y', '0.0',
        ],
        output='screen',
    )

    # ===== 4. ROS-Gazebo Bridge =====
    # Bridge Gazebo topics to ROS 2 using YAML config (same pattern as TB3)
    bridge_config_file = os.path.join(pkg_dir, 'config', 'g1_bridge.yaml')
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config_file}],
        output='screen',
    )

    # ===== 5. Robot State Publisher =====
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    # ===== 6. SLAM Toolbox (online async) =====
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
    )

    # ===== 7. Nav2 — only the nodes needed for exploration =====
    nav2_lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
    ]

    nav2_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'autostart': True,
                    'node_names': nav2_lifecycle_nodes,
                }],
            ),
        ],
    )

    # ===== 8. Frontier Explorer (delayed to let Nav2 + SLAM start first) =====
    frontier_explorer = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='auto_explore_sim',
                executable='frontier_explorer.py',
                name='frontier_explorer',
                output='screen',
                parameters=[explore_params_file, {'use_sim_time': use_sim_time}],
            ),
        ],
    )

    # ===== 9. RViz =====
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    # Build launch description
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(gz_resource_path)

    # Declare arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_headless)

    # Start simulation
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(spawn_robot)

    # Start bridge & robot description
    ld.add_action(gz_bridge)
    ld.add_action(robot_state_publisher)

    # Start SLAM
    ld.add_action(slam_toolbox)

    # Start Nav2 (only needed nodes)
    ld.add_action(nav2_nodes)

    # Start exploration (delayed)
    ld.add_action(frontier_explorer)

    # Start visualization
    ld.add_action(rviz_node)

    return ld
