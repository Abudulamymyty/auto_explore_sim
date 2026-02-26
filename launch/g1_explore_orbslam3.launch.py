"""
g1_explore_orbslam3.launch.py
==============================
All-in-one launch for autonomous exploration with Unitree G1 + ORB-SLAM3.

  Gazebo (office world) + G1 Robot + ORB-SLAM3 (RGBD) + Nav2 + Frontier Explorer + RViz

Replaces slam_toolbox with ORB-SLAM3 (visual SLAM using the G1's head-mounted
RGBD camera).  The explorer and Nav2 stack are kept identical to
g1_explore.launch.py.

Architecture
------------
  Gazebo rgbd_camera  ─►  /camera/{image, depth_image, camera_info}
                                │
                          orb_slam3_ros2_wrapper (rgbd node)
                                │
              ┌─────────────────┼───────────────────┐
              │                 │                   │
     /orb_slam3/pose   /orb_slam3/all_map_points   /orb_slam3/tracking_image
              │                 │
      tf_bridge.py     map_publisher.py
     (map→odom TF)     (/map OccupancyGrid)
              │                 │
              └────────── Nav2 ──┘ ──► Frontier Explorer

Pre-requisites
--------------
  1. Build ORB-SLAM3 and its ROS 2 wrapper in your workspace:
       cd ~/ros2_ws/src
       git clone https://github.com/zang09/ORB-SLAM3-ROS2.git orbslam3_ros2
       # Follow its README to build ORB-SLAM3 first, then colcon build

  2. Point the ORB-SLAM3 vocabulary and this package's camera config:
       vocabulary: /path/to/ORB_SLAM3/Vocabulary/ORBvoc.txt
       settings:   <auto_explore_sim_share>/config/orbslam3_camera.yaml

  3. Launch:
       ros2 launch auto_explore_sim g1_explore_orbslam3.launch.py \\
         orb_vocabulary:=/path/to/ORBvoc.txt
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

    # ── Launch configuration variables ──────────────────────────────────────
    use_sim_time  = LaunchConfiguration('use_sim_time')
    use_rviz      = LaunchConfiguration('use_rviz')
    headless      = LaunchConfiguration('headless')
    orb_vocab     = LaunchConfiguration('orb_vocabulary')

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
        default_value=os.path.expanduser('~/ORB_SLAM3/Vocabulary/ORBvoc.txt'),
        description='Absolute path to the ORB-SLAM3 vocabulary file (ORBvoc.txt)')

    # ── File paths ────────────────────────────────────────────────────────────
    world_file         = os.path.join(pkg_dir, 'worlds', 'office.sdf')
    nav2_params_file   = os.path.join(pkg_dir, 'config', 'g1_nav2_params.yaml')
    orbslam3_cam_file  = os.path.join(pkg_dir, 'config', 'orbslam3_camera.yaml')
    explore_params_file= os.path.join(pkg_dir, 'config', 'frontier_explorer_params.yaml')
    rviz_config_file   = os.path.join(pkg_dir, 'rviz', 'explore.rviz')

    # G1 model paths
    g1_model_dir = os.path.join(pkg_dir, 'models', 'g1_description')
    g1_sdf_file  = os.path.join(g1_model_dir, 'model.sdf')
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

    # Robot URDF for robot_state_publisher (also publishes camera_link TF)
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
    #    Bridges /scan, /odom, /cmd_vel, /imu, /tf, /joint_states, and
    #    camera topics (/camera/image, /camera/depth_image, /camera/camera_info,
    #    /camera/points) — all defined in g1_bridge.yaml.
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
    #    Publishes static TFs from the URDF, including base_link → camera_link.
    #    NOTE: the URDF (g1.urdf) must include the camera_link joint that
    #    matches the SDF camera_link placement (x=0.10, z=1.05 from base_link).
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
    # 6a. Camera topic remapping relay
    #     The Gazebo bridge publishes at:
    #       /camera/image        (RGB)
    #       /camera/depth_image  (depth)
    #       /camera/camera_info
    #     orb_slam3_ros2_wrapper expects:
    #       /camera/image_raw
    #       /camera/depth/image_raw
    #       /camera/camera_info   (same — no remap needed)
    #     We use ros2 topic relay via simple topic_tools nodes.
    # =========================================================================
    relay_rgb = Node(
        package='topic_tools',
        executable='relay',
        name='relay_rgb',
        arguments=['/camera/image', '/camera/image_raw'],
        output='screen',
    )

    relay_depth = Node(
        package='topic_tools',
        executable='relay',
        name='relay_depth',
        arguments=['/camera/depth_image', '/camera/depth/image_raw'],
        output='screen',
    )

    # =========================================================================
    # 6b. ORB-SLAM3 (RGBD mode)
    #     Package: orb_slam3_ros2_wrapper  (must be built in the workspace)
    #     Executable: rgbd
    #     Subscribes: /camera/image_raw, /camera/depth/image_raw,
    #                 /camera/camera_info
    #     Publishes:  /orb_slam3/pose         (PoseStamped — camera in map)
    #                 /orb_slam3/all_map_points (PointCloud2)
    #                 /orb_slam3/tracking_image (Image)
    #     Does NOT broadcast TF — TF is handled by orb_slam3_tf_bridge below.
    # =========================================================================
    orb_slam3 = Node(
        package='orb_slam3_ros2_wrapper',
        executable='rgbd',
        name='orb_slam3',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'vocabulary':   orb_vocab,
            'settings':     orbslam3_cam_file,
            'publish_tf':   False,   # TF is handled by orb_slam3_tf_bridge
        }],
        remappings=[
            ('/camera/image_raw',       '/camera/image_raw'),
            ('/camera/depth/image_raw', '/camera/depth/image_raw'),
            ('/camera/camera_info',     '/camera/camera_info'),
        ],
    )

    # =========================================================================
    # 6c. ORB-SLAM3 TF Bridge
    #     Converts /orb_slam3/pose (map→camera) into the map→odom TF
    #     that Nav2 requires.  Runs after ORB-SLAM3 starts (20 s delay to
    #     allow ORB-SLAM3 to initialise and start publishing poses).
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
    # 6d. ORB-SLAM3 Map Publisher
    #     Projects ORB-SLAM3 3D map points → 2-D occupancy grid → /map
    #     Nav2's static_layer reads /map — this replaces slam_toolbox's map.
    # =========================================================================
    orb_slam3_map_pub = Node(
        package='auto_explore_sim',
        executable='orb_slam3_map_publisher.py',
        name='orb_slam3_map_publisher',
        output='screen',
        parameters=[{
            'use_sim_time':  use_sim_time,
            'map_frame':     'map',
            'map_resolution': 0.05,
            'map_size':       40.0,
            'publish_rate':   1.0,
            'min_height':     0.10,
            'max_height':     1.80,
            'inflation_cells': 1,
        }],
    )

    # =========================================================================
    # 7. Nav2 — same nodes as g1_explore.launch.py (no changes)
    # =========================================================================
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

    # =========================================================================
    # 8. Frontier Explorer (delayed — waits for Nav2 + ORB-SLAM3 to be ready)
    # =========================================================================
    frontier_explorer = TimerAction(
        period=30.0,   # longer than g1_explore.launch.py to allow ORB-SLAM3 init
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
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    # =========================================================================
    # Build launch description
    # =========================================================================
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(gz_resource_path)

    # Declare arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_headless)
    ld.add_action(declare_orb_vocab)

    # Simulation
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(spawn_robot)

    # Bridge & robot description
    ld.add_action(gz_bridge)
    ld.add_action(robot_state_publisher)

    # Camera topic relays (bridge → ORB-SLAM3 expected names)
    ld.add_action(relay_rgb)
    ld.add_action(relay_depth)

    # ORB-SLAM3 visual SLAM stack (replaces slam_toolbox)
    ld.add_action(orb_slam3)
    ld.add_action(orb_slam3_tf_bridge)
    ld.add_action(orb_slam3_map_pub)

    # Nav2
    ld.add_action(nav2_nodes)

    # Exploration (delayed)
    ld.add_action(frontier_explorer)

    # Visualisation
    ld.add_action(rviz_node)

    return ld
