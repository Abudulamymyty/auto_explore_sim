"""
g1_explore_orbslam3.launch.py
==============================
All-in-one launch for autonomous exploration with Unitree G1 + ORB-SLAM3.

  Gazebo (office world) + G1 Robot + ORB-SLAM3 (RGBD) + Nav2 + Frontier Explorer + RViz

Replaces slam_toolbox with ORB-SLAM3 visual SLAM using the G1's head-mounted
RGBD camera.  Explorer and Nav2 stack are unchanged from g1_explore.launch.py.

Data-flow
---------
  Gazebo rgbd_camera  →  /camera/{image, depth_image, camera_info}
    relay nodes       →  /camera/rgb   /camera/depth
                                 │
                        ros2 run orbslam3 rgbd <vocab> <cfg>   (positional args)
                          publishes /orb_slam3/pose (PoseStamped, camera in ORB world)
                                 │
                   orb_slam3_tf_bridge.py
                     converts optical→ROS axes and broadcasts map→odom TF
                                 │
                   orb_slam3_map_publisher.py
                     accumulates /scan hits via TF → publishes /map (OccupancyGrid)
                                 │
                               Nav2 ────► Frontier Explorer

Pre-requisites
--------------
  1. Extract the ORB vocabulary (one-time):
       cd ~/ros2_ws/src/orbslam3_ros2/vocabulary
       tar -xf ORBvoc.txt.tar.gz

  2. Build both packages:
       cd ~/ros2_ws
       colcon build --packages-select orbslam3 auto_explore_sim

  3. Launch (vocabulary path auto-detected from workspace):
       ros2 launch auto_explore_sim g1_explore_orbslam3.launch.py
     Or override:
       ros2 launch auto_explore_sim g1_explore_orbslam3.launch.py \\
         orb_vocabulary:=/absolute/path/to/ORBvoc.txt
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # ── Package directories ──────────────────────────────────────────────────
    pkg_dir = get_package_share_directory('auto_explore_sim')

    # Default vocabulary location alongside the orbslam3_ros2 source tree
    _vocab_default = os.path.expanduser(
        '~/ros2_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt')

    # ── Launch configuration variables ──────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz     = LaunchConfiguration('use_rviz')
    headless     = LaunchConfiguration('headless')
    orb_vocab    = LaunchConfiguration('orb_vocabulary')

    # ── Declare launch arguments ─────────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to start RViz')

    declare_headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo headless (no GUI)')

    declare_orb_vocab = DeclareLaunchArgument(
        'orb_vocabulary',
        default_value=_vocab_default,
        description='Absolute path to ORBvoc.txt '
                    '(extract from orbslam3_ros2/vocabulary/ORBvoc.txt.tar.gz first)')

    # ── File paths ────────────────────────────────────────────────────────────
    world_file          = os.path.join(pkg_dir, 'worlds', 'office.sdf')
    nav2_params_file    = os.path.join(pkg_dir, 'config', 'g1_nav2_params.yaml')
    orbslam3_cam_file   = os.path.join(pkg_dir, 'config', 'orbslam3_camera.yaml')
    explore_params_file = os.path.join(pkg_dir, 'config', 'frontier_explorer_params.yaml')
    rviz_config_file    = os.path.join(pkg_dir, 'rviz', 'explore.rviz')

    g1_model_dir = os.path.join(pkg_dir, 'models', 'g1_description')
    g1_sdf_file  = os.path.join(g1_model_dir, 'model.sdf')
    g1_urdf_file = os.path.join(g1_model_dir, 'g1.urdf')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params_file,
            root_key='',
            param_rewrites={'autostart': 'true'},
            convert_types=True,
        ),
        allow_substs=True,
    )

    with open(g1_urdf_file, 'r') as f:
        robot_description = f.read()

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # ── Environment ───────────────────────────────────────────────────────────
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_dir, 'models', 'g1_description')
        + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''))

    # =========================================================================
    # 1. Gazebo Server
    # =========================================================================
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_file],
        output='screen',
    )

    # =========================================================================
    # 2. Gazebo Client (GUI)
    # =========================================================================
    gazebo_client = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
        condition=UnlessCondition(headless),
    )

    # =========================================================================
    # 3. Spawn G1 in Gazebo
    # =========================================================================
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'unitree_g1',
            '-file', g1_sdf_file,
            '-x', '0.0', '-y', '0.0', '-z', '0.01', '-Y', '0.0',
        ],
        output='screen',
    )

    # =========================================================================
    # 4. ROS-Gazebo Bridge
    #    g1_bridge.yaml includes camera topics:
    #      /camera/image        (RGB, gz.msgs.Image)
    #      /camera/depth_image  (depth float32, gz.msgs.Image)
    #      /camera/camera_info  (gz.msgs.CameraInfo)
    #      /camera/points       (gz.msgs.PointCloudPacked)
    # =========================================================================
    bridge_config_file = os.path.join(pkg_dir, 'config', 'g1_bridge.yaml')
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config_file}],
        output='screen',
    )

    # =========================================================================
    # 5. Robot State Publisher
    #    Publishes base_link → camera_link static TF (from g1.urdf).
    # =========================================================================
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

    # =========================================================================
    # 6a. Camera topic relay
    #     The Gazebo bridge publishes at /camera/image and /camera/depth_image.
    #     The orbslam3 rgbd node subscribes to relative topics camera/rgb and
    #     camera/depth (which resolve to /camera/rgb and /camera/depth).
    # =========================================================================
    relay_rgb = Node(
        package='topic_tools',
        executable='relay',
        name='relay_rgb',
        arguments=['/camera/image', '/camera/rgb'],
        output='screen',
    )

    relay_depth = Node(
        package='topic_tools',
        executable='relay',
        name='relay_depth',
        arguments=['/camera/depth_image', '/camera/depth'],
        output='screen',
    )

    # =========================================================================
    # 6b. ORB-SLAM3 RGBD
    #     Package:    orbslam3   (ros2 run orbslam3 rgbd)
    #     Args:       <vocabulary>  <camera_settings_yaml>   (positional)
    #     Subscribes: camera/rgb   camera/depth
    #     Publishes:  /orb_slam3/pose  (PoseStamped — camera in ORB world)
    #
    #     NOTE: vocabulary and settings are positional CLI args, not ROS params.
    # =========================================================================
    orb_slam3 = ExecuteProcess(
        cmd=['ros2', 'run', 'orbslam3', 'rgbd', orb_vocab, orbslam3_cam_file],
        output='screen',
    )

    # =========================================================================
    # 6c. ORB-SLAM3 TF Bridge  (delayed 20 s for ORB-SLAM3 to initialise)
    #     Converts /orb_slam3/pose to map→odom TF with coordinate correction:
    #       ORB world (Z fwd / optical) → ROS map (X fwd / REP-103)
    # =========================================================================
    orb_slam3_tf_bridge = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='auto_explore_sim',
                executable='orb_slam3_tf_bridge.py',
                name='orb_slam3_tf_bridge',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ],
    )

    # =========================================================================
    # 6d. LiDAR-based map builder  (uses ORB-SLAM3 TF, reads /scan)
    #     Accumulates LiDAR hits projected into the map frame → /map.
    #     Replaces slam_toolbox's map output without projection ambiguity.
    # =========================================================================
    orb_slam3_map_pub = Node(
        package='auto_explore_sim',
        executable='orb_slam3_map_publisher.py',
        name='orb_slam3_map_publisher',
        output='screen',
        parameters=[{
            'use_sim_time':   use_sim_time,
            'map_frame':      'map',
            'scan_topic':     '/scan',
            'map_resolution': 0.05,
            'map_size':       40.0,
            'publish_rate':   1.0,
        }],
    )

    # =========================================================================
    # 7. Nav2 — identical to g1_explore.launch.py
    # =========================================================================
    nav2_lifecycle_nodes = [
        'controller_server', 'smoother_server', 'planner_server',
        'behavior_server', 'velocity_smoother', 'collision_monitor',
        'bt_navigator',
    ]

    nav2_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(package='nav2_controller',  executable='controller_server',
                 output='screen', parameters=[configured_params],
                 remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(package='nav2_smoother',    executable='smoother_server',
                 name='smoother_server', output='screen',
                 parameters=[configured_params], remappings=remappings),
            Node(package='nav2_planner',     executable='planner_server',
                 name='planner_server', output='screen',
                 parameters=[configured_params], remappings=remappings),
            Node(package='nav2_behaviors',   executable='behavior_server',
                 name='behavior_server', output='screen',
                 parameters=[configured_params],
                 remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(package='nav2_bt_navigator', executable='bt_navigator',
                 name='bt_navigator', output='screen',
                 parameters=[configured_params], remappings=remappings),
            Node(package='nav2_velocity_smoother', executable='velocity_smoother',
                 name='velocity_smoother', output='screen',
                 parameters=[configured_params],
                 remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(package='nav2_collision_monitor', executable='collision_monitor',
                 name='collision_monitor', output='screen',
                 parameters=[configured_params], remappings=remappings),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', output='screen',
                 parameters=[{'autostart': True, 'node_names': nav2_lifecycle_nodes}]),
        ],
    )

    # =========================================================================
    # 8. Frontier Explorer  (delayed 30 s — extra time for ORB-SLAM3 init)
    # =========================================================================
    frontier_explorer = TimerAction(
        period=30.0,
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

    # =========================================================================
    # 9. RViz
    # =========================================================================
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen', arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    # =========================================================================
    # Assemble
    # =========================================================================
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(gz_resource_path)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_headless)
    ld.add_action(declare_orb_vocab)

    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(spawn_robot)
    ld.add_action(gz_bridge)
    ld.add_action(robot_state_publisher)

    ld.add_action(relay_rgb)
    ld.add_action(relay_depth)

    ld.add_action(orb_slam3)
    ld.add_action(orb_slam3_tf_bridge)
    ld.add_action(orb_slam3_map_pub)

    ld.add_action(nav2_nodes)
    ld.add_action(frontier_explorer)
    ld.add_action(rviz_node)

    return ld
