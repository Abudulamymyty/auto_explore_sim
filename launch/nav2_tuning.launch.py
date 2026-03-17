"""
Nav2 tuning launch file:
  Gazebo (office world) + TB3 Waffle + SLAM Toolbox + Nav2 + RViz

NO explore_lite — you set goals manually in RViz ("2D Goal Pose") to test
and fine-tune Nav2 parameters (controller, planner, costmaps, etc.).

Usage:
  ros2 launch auto_explore_sim nav2_tuning.launch.py
  ros2 launch auto_explore_sim nav2_tuning.launch.py headless:=true
  ros2 launch auto_explore_sim nav2_tuning.launch.py nav2_params_file:=/absolute/path/to/custom_params.yaml
  ros2 launch auto_explore_sim nav2_tuning.launch.py rviz_config:=/absolute/path/to/custom.rviz
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # ── Package directories ──────────────────────────────────────────────
    pkg_dir = get_package_share_directory('auto_explore_sim')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')

    # ── Launch configuration variables ───────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config = LaunchConfiguration('rviz_config')

    # ── Declare launch arguments ─────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to start RViz')

    declare_headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo headless (no GUI)')

    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 params file to load (swap for tuning)')

    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml'),
        description='Full path to the SLAM Toolbox params file')

    # ── File paths ───────────────────────────────────────────────────────
    world_file = os.path.join(pkg_dir, 'worlds', 'office.sdf')
    default_rviz_config = os.path.join(pkg_dir, 'rviz', 'nav2_tuning.rviz')

    # Fallback to explore.rviz if the tuning rviz config doesn't exist yet
    if not os.path.isfile(default_rviz_config):
        default_rviz_config = os.path.join(pkg_dir, 'rviz', 'explore.rviz')

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='RViz config file')

    # ── Nav2 params with substitutions ───────────────────────────────────
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params_file,
            root_key='',
            param_rewrites={'autostart': 'true'},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # ── Robot URDF / SDF ─────────────────────────────────────────────────
    urdf_file = os.path.join(sim_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    robot_sdf = os.path.join(sim_dir, 'urdf', 'gz_waffle.sdf.xacro')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

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
        # Show Gazebo GUI whenever RViz is requested, even if headless:=true
        condition=IfCondition(
            PythonExpression([
                '"', use_rviz, '" == "true" or "',
                headless, '" == "false"'
            ])
        ),
    )

    # ===== 3. Spawn TB3 in Gazebo =====
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, 'launch', 'spawn_tb3.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot_name': 'turtlebot3_waffle',
            'robot_sdf': robot_sdf,
            'x_pose': '0.0',
            'y_pose': '0.0',
            'z_pose': '0.01',
            'roll': '0.0',
            'pitch': '0.0',
            'yaw': '0.0',
        }.items(),
    )

    # ===== 4. Robot State Publisher =====
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

    # ===== 5. SLAM Toolbox (online async) =====
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
    )

    # ===== 6. Nav2 — all navigation nodes (NO explore_lite) =====
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

    # ===== 7. RViz =====
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    # ── Build launch description ─────────────────────────────────────────
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    # Declare arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_headless)
    ld.add_action(declare_nav2_params)
    ld.add_action(declare_slam_params)
    ld.add_action(declare_rviz_config)

    # Start simulation
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(spawn_robot)

    # Start robot description
    ld.add_action(robot_state_publisher)

    # Delay SLAM/Nav2 bringup until the Gazebo bridges have had time to publish
    # /tf and /odom; otherwise Nav2 activates before the map/odom frames exist.
    ld.add_action(
        TimerAction(
            period=5.0,
            actions=[
                slam_toolbox,
                nav2_nodes,
            ],
        )
    )

    # Start visualization
    ld.add_action(rviz_node)

    return ld
