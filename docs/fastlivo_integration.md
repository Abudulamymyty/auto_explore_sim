# FastLiVO → Nav2 + Frontier Explorer Integration

## Replacing SLAM Toolbox with FAST-LIVO2

This document specifies the exact topics, message types, frame IDs, and field
contracts that FastLiVO must satisfy for Nav2 and the Frontier Explorer to work
correctly. It then documents every gap between what FastLiVO natively publishes
and what the downstream consumers require, and describes the bridge nodes needed
to fill those gaps.

---

## 1. The Interface Contract

Everything downstream (Nav2 stack + Frontier Explorer) is a **consumer** of the
SLAM system. The SLAM system is a **producer** that must supply the following
outputs with specific topics, types, and frame IDs regardless of which algorithm
runs underneath.

### 1.1 Mandatory Outputs

| # | Topic / TF | Message Type | `frame_id` | `child_frame_id` | Who consumes |
|---|-----------|-------------|-----------|----------------|-------------|
| **M1** | `/map` | `nav_msgs/OccupancyGrid` | `map` | — | Frontier Explorer, Nav2 global costmap (`static_layer`) |
| **M2** | TF `map → odom` | continuous broadcast | `map` | `odom` | **Every** Nav2 node, Frontier Explorer (robot pose lookup) |
| **M3** | `/odom` | `nav_msgs/Odometry` | `odom` | `base_link` | `bt_navigator`, `velocity_smoother`, `collision_monitor` |
| **M4** | `/scan` | `sensor_msgs/LaserScan` | `laser_frame` (any) | — | Nav2 local costmap, global costmap, `collision_monitor` |

All four are **non-negotiable**: omitting or renaming any of them breaks the
pipeline silently (nodes start but produce no useful output).

---

## 2. What FastLiVO Publishes Natively

Source: `/home/zxj/ros2_ws/src/FAST-LIVO2_ROS2-dev/src/LIVMapper.cpp`

### 2.1 Native Topics

| Topic | Type | `frame_id` | Frequency | Content |
|-------|------|-----------|-----------|---------|
| `/aft_mapped_to_init` | `nav_msgs/Odometry` | `camera_init` | ~10–50 Hz | Robot pose estimate in FastLiVO world frame |
| `/LIVO2/imu_propagate` | `nav_msgs/Odometry` | `camera_init` | ~200 Hz | High-rate IMU-propagated pose |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | `camera_init` | ~10 Hz | 3D registered map points |
| `/Laser_map` | `sensor_msgs/PointCloud2` | `camera_init` | low | Accumulated laser map |
| `/path` | `nav_msgs/Path` | `camera_init` | ~10 Hz | Robot trajectory history |

### 2.2 Native TF

FastLiVO broadcasts **one** transform:

```
camera_init  →  aft_mapped        (FastLiVO world frame → FastLiVO body frame)
```

This is published as a `tf::StampedTransform` via `tf::TransformBroadcaster`
using the **ROS1 TF API**. In the ROS2 context this goes to `/tf`.

### 2.3 What FastLiVO Does NOT Publish

| Missing output | Why it is needed |
|---------------|-----------------|
| `nav_msgs/OccupancyGrid` on `/map` | Frontier Explorer and Nav2 global costmap consume this directly |
| TF `map → odom` | All Nav2 nodes use this to resolve robot position in the global frame |
| `sensor_msgs/LaserScan` on `/scan` | Nav2 costmap obstacle layers and collision_monitor are configured for 2D scan input |

---

## 3. Gap Analysis

```
FastLiVO native output          Gap                     What Nav2+Explorer need
────────────────────────────    ──────────────────────   ────────────────────────
/aft_mapped_to_init             frame rename             TF: map → odom
(nav_msgs/Odometry,             + odom decomposition
 frame: camera_init)

TF: camera_init → aft_mapped   frame rename only         TF: map → odom
                                (camera_init = map
                                 aft_mapped ≠ odom)

/cloud_registered               3D → 2D projection       /map (OccupancyGrid)
(PointCloud2, 3D)               + height filter           frame: map, res 0.05m
                                + log-odds grid build

/livox/lidar                    3D → 2D scan slice        /scan (LaserScan)
(PointCloud2, raw input)        height-filtered            for Nav2 costmaps
```

---

## 4. Topic & Message Format Specifications

### 4.1 `/map` — `nav_msgs/OccupancyGrid`

This is the single most critical output for Nav2 + Frontier Explorer.

```
nav_msgs/OccupancyGrid
├── header
│   ├── stamp      (builtin_interfaces/Time)  ← current ROS time, updated every publish
│   └── frame_id   (string)                  ← MUST be "map" (exact string match)
├── info
│   ├── map_load_time  (builtin_interfaces/Time)
│   ├── resolution     (float32)             ← MUST be 0.05  (matches Nav2 costmap resolution)
│   ├── width          (uint32)              ← number of cells in X
│   ├── height         (uint32)              ← number of cells in Y
│   └── origin         (geometry_msgs/Pose)  ← world coords of cell [0,0] lower-left corner
│       ├── position.x  (float64)            ← e.g. -20.0 for a 40m-wide map centred at origin
│       ├── position.y  (float64)
│       ├── position.z  (float64)            ← 0.0
│       └── orientation (quaternion)         ← identity (w=1, x=y=z=0)
└── data            (int8[])                 ← row-major, width × height entries
    ├──  0          ← FREE cell
    ├── 100         ← OCCUPIED cell
    └──  -1         ← UNKNOWN (not yet observed)
```

**Cell indexing** (must match): cell `(col, row)` → `data[row * width + col]`

**Resolution**: 0.05 m/cell is hard-coded as the common resolution across
`slam_toolbox_params.yaml` (resolution: 0.05), `g1_nav2_params.yaml`
(global_costmap resolution: 0.05, local_costmap resolution: 0.05), and the
Frontier Explorer (which reads the map grid directly). **Do not change this.**

**Update rate**: Nav2 global costmap polls `/map` on the `static_layer` at its
`update_frequency` (1.0 Hz in `g1_nav2_params.yaml`). The Frontier Explorer
processes every message it receives (callback-driven). Publish at **≥ 1 Hz**;
3–5 Hz is comfortable. Publishing faster than 10 Hz wastes CPU with no benefit.

**Transient local QoS**: The Nav2 `static_layer` subscribes with
`map_subscribe_transient_local: True`. You must publish `/map` with
`TRANSIENT_LOCAL` durability so late-joining subscribers receive the last map
immediately. In rclpy:

```python
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

map_qos = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
```

---

### 4.2 TF `map → odom`

This is the localization correction transform — the output of any SLAM system.

```
geometry_msgs/TransformStamped   (inside tf2_msgs/TFMessage on /tf)
├── header
│   ├── stamp     ← timestamp of the pose estimate (FastLiVO odometry timestamp)
│   └── frame_id  ← "map"        (MUST match global_frame in all Nav2 params)
├── child_frame_id ← "odom"      (MUST match odom_frame in all Nav2 params)
└── transform
    ├── translation.x, .y, .z    ← T(map → odom) position
    └── rotation.x, .y, .z, .w   ← T(map → odom) orientation as quaternion
```

**Broadcast rate**: SLAM Toolbox broadcasts at 50 Hz (`transform_publish_period: 0.02`).
Nav2's `transform_tolerance` is set to 0.5–1.0 s in the param files, so 10 Hz
is the functional minimum, but **20–50 Hz** is strongly recommended for MPPI
trajectory sampling accuracy.

**Derivation from FastLiVO output**:

FastLiVO gives you `T(camera_init → aft_mapped)` = `T(map → robot_body_fastlivo)`.
Gazebo/wheel encoders give you `T(odom → base_link)`.

The required transform is:

```
T(map → odom) = T(map → base_link) × inv(T(odom → base_link))
              = T(fastlivo_pose) × T_body_to_base × inv(T(odom → base_link))
```

where `T_body_to_base` is the static extrinsic offset between the FastLiVO body
frame (`aft_mapped`, which tracks the IMU/LiDAR) and the robot's `base_link`.

In practice, if the IMU and base_link are co-located (or the offset is small),
`T_body_to_base ≈ identity`.

**Critical**: Never publish `map → base_link` directly. Nav2 expects the
**two-hop** chain: `map → odom → base_link`. The `odom → base_link` hop comes
from wheel encoders (already published by Gazebo). The `map → odom` hop is what
SLAM/FastLiVO must provide.

---

### 4.3 `/odom` — `nav_msgs/Odometry`

This is already provided by Gazebo via `ros_gz_bridge` (see `g1_bridge.yaml`).
FastLiVO does **not** need to replace this.

```
nav_msgs/Odometry  (from Gazebo wheel encoders)
├── header.frame_id   = "odom"
├── child_frame_id    = "base_link"
├── pose.pose         ← encoder-integrated position (drifts over time)
└── twist.twist       ← current linear/angular velocity
```

The `bt_navigator` (`odom_topic: /odom`), `velocity_smoother` (`odom_topic: odom`),
and `collision_monitor` (`odom_frame_id: odom`) all read this. No changes needed.

---

### 4.4 `/scan` — `sensor_msgs/LaserScan`

Nav2's local and global costmap obstacle layers are configured for LaserScan input:

```yaml
# g1_nav2_params.yaml (both local_costmap and global_costmap)
scan:
  topic: /scan
  data_type: "LaserScan"
```

FastLiVO's G1 has a Livox Mid-360 3D LiDAR publishing `PointCloud2` on
`/livox/lidar`. This must be converted to a 2D `LaserScan` slice.

**Required LaserScan format**:

```
sensor_msgs/LaserScan
├── header
│   ├── stamp     ← timestamp matching point cloud acquisition time
│   └── frame_id  ← frame of the LiDAR (e.g. "laser_frame" or "livox_frame")
│                    Must have a TF path to base_link (static transform)
├── angle_min     ← -π  (full 360° sweep)
├── angle_max     ← +π
├── angle_increment ← angular resolution (e.g. 2π / 360 for 1°)
├── time_increment  ← 0.0 (if not per-beam timestamps)
├── scan_time       ← 1/scan_frequency (e.g. 0.1 for 10 Hz)
├── range_min       ← 0.1  (metres, minimum valid range)
├── range_max       ← 20.0 (metres, maximum valid range)
└── ranges[]        ← float32 array, NaN or inf for invalid readings
```

**Height filter**: Only include 3D points that fall within a height band that
corresponds to obstacles at robot navigation level. Recommended:

```
z_min = 0.10 m  (above floor, ignore ground returns)
z_max = 1.50 m  (below head level, focus on room-level obstacles)
```

These are in the LiDAR's own frame. Adjust based on LiDAR mounting height on G1.

---

## 5. Frame Alignment

### 5.1 TF Tree Required by Nav2

```
map                         ← global planning frame
 └─ odom                    ← local odometry frame
     └─ base_footprint       ← ground-projected robot center (Nav2 collision_monitor)
         └─ base_link         ← robot body origin
             ├─ laser_frame   ← LiDAR origin (static, from URDF)
             ├─ imu_link      ← IMU (static, from URDF)
             └─ camera_link   ← camera (static, from URDF)
```

**Who publishes what**:
| Transform | Publisher |
|-----------|-----------|
| `map → odom` | **FastLiVO TF bridge** (must implement) |
| `odom → base_footprint` | `robot_state_publisher` + Gazebo encoder |
| `base_footprint → base_link` | `robot_state_publisher` (from URDF) |
| `base_link → laser_frame` | `robot_state_publisher` (static, from URDF) |

### 5.2 FastLiVO Frame Mapping

| FastLiVO frame | ROS standard frame | How to map |
|---------------|-------------------|-----------|
| `camera_init` | `map` | Publish static TF: `map → camera_init` with identity transform OR relabel in bridge node |
| `aft_mapped` | ~ `base_link` (IMU/LiDAR frame) | Use as intermediate; do not publish as `odom` or `base_link` |

**The cleanest approach** is to relabel inside the bridge node rather than
adding extra TF hops:

```
FastLiVO pose output  →  bridge node  →  publish T(map → odom) directly
```

Do **not** chain: `camera_init → aft_mapped → base_link → ...` and hope
Nav2 resolves it. The `odom` frame must be the same `odom` that `base_link` is
referenced from (i.e., Gazebo's wheel encoder frame).

---

## 6. Required Bridge Nodes

Two bridge nodes are needed, analogous to `orb_slam3_tf_bridge.py` and
`orb_slam3_map_publisher.py` already in the codebase.

---

### 6.1 `fastlivo_tf_bridge.py` — Pose → TF `map → odom`

**Purpose**: Subscribe to FastLiVO's odometry (pose of robot in `camera_init`
frame) and derive + publish the `map → odom` TF that Nav2 needs.

**Subscribes**:
| Topic | Type | Notes |
|-------|------|-------|
| `/aft_mapped_to_init` | `nav_msgs/Odometry` | FastLiVO pose estimate at ~10–50 Hz |
| TF `odom → base_link` | (via tf2_ros.Buffer) | Wheel encoder pose, already published by Gazebo |

**Publishes**:
| Topic/TF | Type | Notes |
|---------|------|-------|
| `/tf` | TF `map → odom` | Derived transform at same rate as FastLiVO input |

**Algorithm**:

```python
def fastlivo_odom_callback(msg):
    # msg: nav_msgs/Odometry
    # msg.header.frame_id = "camera_init"  (= map)
    # msg.child_frame_id  = "aft_mapped"   (= FastLiVO body / IMU frame)

    # Step 1: Extract T(map → fastlivo_body) from FastLiVO odometry
    T_map_to_flbody = pose_msg_to_matrix(msg.pose.pose)
    # Apply static extrinsic: T(fastlivo_body → base_link)
    # This is the offset between aft_mapped (IMU/LiDAR frame) and base_link
    # From extrin_calib in fastlivo_params.yaml: extrinsic_T, extrinsic_R
    T_map_to_base = T_map_to_flbody @ T_flbody_to_base  # static, from config

    # Step 2: Look up T(odom → base_link) from wheel encoders
    try:
        t = tf_buffer.lookup_transform('odom', 'base_link', Time())
        T_odom_to_base = transform_to_matrix(t.transform)
    except:
        return  # TF not yet available

    # Step 3: Compute T(map → odom)
    # T(map → base) = T(map → odom) × T(odom → base)
    # → T(map → odom) = T(map → base) × inv(T(odom → base))
    T_map_to_odom = T_map_to_base @ np.linalg.inv(T_odom_to_base)

    # Step 4: Publish as TF
    t_out = TransformStamped()
    t_out.header.stamp    = msg.header.stamp
    t_out.header.frame_id = 'map'
    t_out.child_frame_id  = 'odom'
    t_out.transform       = matrix_to_transform(T_map_to_odom)
    tf_broadcaster.sendTransform(t_out)
```

**Extrinsic calibration**: The offset `T(fastlivo_body → base_link)` must
match the physical sensor mounting on the G1. In `fastlivo_params.yaml`:

```yaml
extrin_calib:
  extrinsic_T: [0.0, 0.0, 0.3]   # LiDAR at 0.3m above IMU
  extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]  # identity rotation
```

Verify this matches the `<pose>` of the LiDAR in `models/g1_description/model.sdf`
relative to `base_link`.

---

### 6.2 `fastlivo_map_publisher.py` — PointCloud2 → OccupancyGrid

**Purpose**: Convert FastLiVO's registered 3D point cloud into the 2D
`OccupancyGrid` that Frontier Explorer and Nav2 need.

**Subscribes**:
| Topic | Type | Notes |
|-------|------|-------|
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 3D map in `camera_init` frame, ~10 Hz |
| TF `map → base_link` | (via tf2_ros.Buffer) | To determine robot position for map updates |

**Publishes**:
| Topic | Type | QoS | Notes |
|-------|------|-----|-------|
| `/map` | `nav_msgs/OccupancyGrid` | RELIABLE + TRANSIENT_LOCAL | 2D grid, frame_id="map", res=0.05m |

**Algorithm**:

```python
# Grid parameters — must match Nav2 costmap and Frontier Explorer
RESOLUTION = 0.05       # metres/cell
MAP_SIZE_M = 40.0       # map covers 40m × 40m centred at origin
N = int(MAP_SIZE_M / RESOLUTION)  # = 800 cells each side

log_odds = np.zeros((N, N), dtype=np.float32)
LOG_ODDS_OCCUPIED =  0.85   # ln(P_occ / P_free) for a hit
LOG_ODDS_FREE     = -0.40   # for a miss (along ray)
LOG_ODDS_MAX      =  3.5    # saturation
LOG_ODDS_MIN      = -3.5

def cloud_callback(msg):
    # msg: sensor_msgs/PointCloud2, frame_id = "camera_init" = "map"
    # Points are already in map frame — no TF needed

    # Look up robot position in map frame (for ray casting)
    try:
        robot_tf = tf_buffer.lookup_transform('map', 'base_link', Time())
        robot_x, robot_y = robot_tf.transform.translation.x, \
                           robot_tf.transform.translation.y
    except:
        return

    robot_col = world_to_cell(robot_x)
    robot_row = world_to_cell(robot_y)

    for x, y, z in read_xyz(msg):
        # Height filter: only take obstacles at torso/wall level
        if z < 0.10 or z > 1.50:
            continue

        # Convert world coordinates to grid cell
        col = world_to_cell(x)
        row = world_to_cell(y)
        if not in_bounds(col, row):
            continue

        # Mark endpoint as occupied
        log_odds[row, col] = min(LOG_ODDS_MAX,
                                 log_odds[row, col] + LOG_ODDS_OCCUPIED)

        # Ray-cast: mark cells along ray as free
        for rc, rr in bresenham(robot_col, robot_row, col, row)[:-1]:
            if in_bounds(rc, rr):
                log_odds[rr, rc] = max(LOG_ODDS_MIN,
                                       log_odds[rr, rc] + LOG_ODDS_FREE)

    publish_map()

def publish_map():
    grid = np.full((N, N), -1, dtype=np.int8)  # start unknown
    grid[log_odds > 0.5]  = 100   # occupied
    grid[log_odds < -0.5] = 0     # free

    msg = OccupancyGrid()
    msg.header.stamp    = now()
    msg.header.frame_id = 'map'
    msg.info.resolution = RESOLUTION
    msg.info.width      = N
    msg.info.height     = N
    msg.info.origin.position.x = -MAP_SIZE_M / 2   # lower-left corner
    msg.info.origin.position.y = -MAP_SIZE_M / 2
    msg.info.origin.orientation.w = 1.0
    msg.data = grid.flatten().tolist()
    map_pub.publish(msg)
```

**Performance note**: `read_xyz()` should use `sensor_msgs_py.point_cloud2`
or `numpy` structured array parsing. Do not iterate over PointCloud2 with a
Python `for` loop per-field — this is prohibitively slow on 50k+ point clouds.

---

### 6.3 `pointcloud_to_laserscan` — PointCloud2 → LaserScan

This is a standard ROS2 package (`ros-humble-pointcloud-to-laserscan` or built
from source). It provides a Nav2-compatible `/scan` from the Livox 3D cloud.

**Node**: `pointcloud_to_laserscan/pointcloud_to_laserscan_node`

**Configuration**:

```yaml
# config/pointcloud_to_laserscan.yaml
pointcloud_to_laserscan_node:
  ros__parameters:
    # Input
    target_frame: laser_frame      # output scan frame (must have TF to base_link)
    transform_tolerance: 0.01

    # 3D → 2D height filter
    min_height: 0.10               # metres in target_frame; ignore floor returns
    max_height: 1.50               # metres; ignore ceiling returns

    # Output scan geometry (Livox Mid-360 specs)
    angle_min: -3.14159            # -π  (full circle)
    angle_max:  3.14159            # +π
    angle_increment: 0.00872       # ~0.5° = π/360 rad

    # Range limits (match SLAM Toolbox's max_laser_range)
    range_min: 0.10
    range_max: 20.0

    # Concurrency
    use_inf: true                  # publish inf for no-return (instead of range_max+1)
    inf_epsilon: 1.0
```

**Remapping** in launch file:

```python
Node(
    package='pointcloud_to_laserscan',
    executable='pointcloud_to_laserscan_node',
    name='livox_to_scan',
    remappings=[
        ('cloud_in', '/cloud_registered'),   # FastLiVO registered cloud (map frame)
        # OR use '/livox/lidar' for raw Gazebo cloud (laser_frame)
        ('scan', '/scan'),                   # output consumed by Nav2
    ],
    parameters=['config/pointcloud_to_laserscan.yaml'],
)
```

**Frame choice**: Prefer `/livox/lidar` (raw Gazebo input, in LiDAR frame) over
`/cloud_registered` (FastLiVO output, in map frame) for the LaserScan source.
The raw cloud arrives faster and avoids a coordinate transformation back into the
sensor frame. The `pointcloud_to_laserscan` node will project it into `target_frame`
using TF automatically.

---

## 7. Nav2 Parameter Changes Required

### 7.1 Changes to `g1_nav2_params.yaml`

If sticking with LaserScan-based costmaps (recommended, no changes to source):

```yaml
# No changes needed — /scan will be provided by pointcloud_to_laserscan node
# Verify these match:
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom          # ← must remain "odom"
      robot_base_frame: base_link # ← must remain "base_link"
      obstacle_layer:
        scan:
          topic: /scan            # ← provided by pointcloud_to_laserscan
          data_type: "LaserScan"

global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map           # ← must remain "map"
      robot_base_frame: base_link
      static_layer:
        map_subscribe_transient_local: True  # ← must remain True for TRANSIENT_LOCAL /map
      obstacle_layer:
        scan:
          topic: /scan
          data_type: "LaserScan"
```

**If you want to use PointCloud2 directly** (avoids the `pointcloud_to_laserscan`
node but requires changing both costmap configs):

```yaml
obstacle_layer:
  observation_sources: livox
  livox:
    topic: /cloud_registered
    data_type: "PointCloud"          # ← change from "LaserScan"
    max_obstacle_height: 1.5
    min_obstacle_height: 0.1
    clearing: True
    marking: True
    obstacle_max_range: 5.0
    raytrace_max_range: 6.0
```

---

## 8. Launch File Changes

Replace the `slam_toolbox` section in `g1_explore.launch.py` with three new nodes:

```python
# Remove this:
# slam_toolbox = IncludeLaunchDescription(...)

# Add these instead:

fastlivo = Node(
    package='fast_livo',
    executable='fastlivo_mapping',
    name='fastlivo_mapping',
    output='screen',
    parameters=[
        os.path.join(pkg_dir, 'config', 'fastlivo_params.yaml'),
        {'use_sim_time': use_sim_time},
    ],
)

fastlivo_tf_bridge = Node(
    package='auto_explore_sim',
    executable='fastlivo_tf_bridge.py',
    name='fastlivo_tf_bridge',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}],
)

fastlivo_map_pub = Node(
    package='auto_explore_sim',
    executable='fastlivo_map_publisher.py',
    name='fastlivo_map_publisher',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}],
)

livox_to_scan = Node(
    package='pointcloud_to_laserscan',
    executable='pointcloud_to_laserscan_node',
    name='livox_to_scan',
    remappings=[
        ('cloud_in', '/livox/lidar'),
        ('scan', '/scan'),
    ],
    parameters=[
        os.path.join(pkg_dir, 'config', 'pointcloud_to_laserscan.yaml'),
        {'use_sim_time': use_sim_time},
    ],
)
```

**Startup sequence** (replace SLAM Toolbox section in launch description):

```
t=0s    Gazebo starts
t=2s    Robot spawned
t=3s    Robot State Publisher, Gazebo bridge
t=4s    FastLiVO starts (needs /livox/lidar, /imu, /camera/image)
        fastlivo_tf_bridge starts (waits for /aft_mapped_to_init)
        fastlivo_map_publisher starts (waits for /cloud_registered)
        livox_to_scan starts (waits for /livox/lidar)
t=5s    Nav2 lifecycle nodes start
t=15s   Frontier Explorer starts
```

No change to the 15s Frontier Explorer delay — FastLiVO typically needs a few
seconds to initialise its voxel map before producing reliable output.

---

## 9. Integration Checklist

Use this as a validation checklist after wiring up the bridge nodes.

### 9.1 TF Tree

```bash
# Should print the full chain without errors:
ros2 run tf2_tools view_frames

# Verify map→base_link resolves:
ros2 run tf2_ros tf2_echo map base_link
```

Expected output: live-updating translation/rotation (not "waiting for transform").

### 9.2 `/map` Topic

```bash
ros2 topic echo /map --once | grep -E "frame_id|resolution|width|height"
# Expected:
#   frame_id: map
#   resolution: 0.05
#   width: 800
#   height: 800

ros2 topic hz /map
# Expected: ≥ 1.0 Hz
```

### 9.3 `/scan` Topic

```bash
ros2 topic echo /scan --once | grep -E "frame_id|angle_min|angle_max|range_max"
ros2 topic hz /scan
# Expected: ≥ 5.0 Hz
```

### 9.4 Nav2 Ready

```bash
ros2 action list
# Expected: /navigate_to_pose and /navigate_through_poses appear

ros2 service call /navigate_to_pose/_action/get_result \
    nav2_msgs/action/NavigateToPose_GetResult_Request
# Should not error on "no such service" — service existence = Nav2 active
```

### 9.5 Frontier Explorer

```bash
ros2 topic echo /explore/frontiers | head -20
# Should show MarkerArray with frontier positions

ros2 topic echo /explore/gvd | head -5
# Should show GVD skeleton markers (cyan lines in RViz)
```

### 9.6 QoS Mismatch Check

If `/map` is published but Nav2 global costmap ignores it:

```bash
ros2 topic info /map --verbose
# Check: Publisher QoS durability = TRANSIENT_LOCAL
#        Subscriber QoS durability = TRANSIENT_LOCAL (Nav2 static_layer)
# A mismatch (e.g. publisher is VOLATILE) causes silent data loss.
```

---

## 10. Summary: Complete Topic Mapping

```
Sensor Hardware / Gazebo Bridge
  /livox/lidar    (PointCloud2)  ──┬──► FastLiVO (fastlivo_mapping)
  /imu            (Imu)          ──┤
  /camera/image   (Image)        ──┘
  /odom           (Odometry)     ──────────────────────────────────────► Nav2 bt_navigator
                                                                          Nav2 velocity_smoother

FastLiVO output
  /aft_mapped_to_init (Odometry, camera_init→aft_mapped)
                                 ──► fastlivo_tf_bridge.py
                                         └── /tf: map→odom  ──────────► Nav2 (all nodes)
                                                                          Frontier Explorer (robot pose)

  /cloud_registered (PointCloud2, camera_init)
                                 ──► fastlivo_map_publisher.py
                                         └── /map (OccupancyGrid, map)  ► Frontier Explorer
                                                                          Nav2 global costmap

Scan conversion
  /livox/lidar    (PointCloud2)  ──► pointcloud_to_laserscan_node
                                         └── /scan (LaserScan)  ────────► Nav2 local costmap
                                                                           Nav2 global costmap
                                                                           collision_monitor
```
