# auto_explore_sim

Autonomous robotic exploration in simulation using **SLAM Toolbox**, **Nav2**, and a custom **GVD-based Frontier Explorer**. Supports TurtleBot3 Waffle and Unitree G1 humanoid robots, with both LiDAR SLAM and ORB-SLAM3 visual SLAM backends.

---

## Table of Contents

1. [System Architecture](#1-system-architecture)
2. [Component Overview](#2-component-overview)
3. [SLAM Toolbox — LiDAR SLAM Backend](#3-slam-toolbox--lidar-slam-backend)
4. [Nav2 Navigation Stack](#4-nav2-navigation-stack)
5. [Frontier Explorer — Mechanism Under the Hood](#5-frontier-explorer--mechanism-under-the-hood)
6. [GVD Path Planning](#6-gvd-path-planning)
7. [Full Data Flow](#7-full-data-flow)
8. [ORB-SLAM3 Visual Backend (Alternative)](#8-orb-slam3-visual-backend-alternative)
9. [Configuration & Tuning](#9-configuration--tuning)
10. [Launch Guide](#10-launch-guide)

---

## 1. System Architecture

The system integrates three major subsystems into a closed-loop autonomous exploration pipeline:

```
┌──────────────────────────────────────────────────────────────────────┐
│                        Gazebo Harmonic Simulation                    │
│   Robot (TB3 / G1) + Office World + Sensors (LiDAR / RGBD Camera)   │
└───────────┬──────────────────────────┬───────────────────────────────┘
            │ /scan (LaserScan)         │ /cmd_vel (velocity commands)
            ▼                           ▲
┌───────────────────────┐   ┌──────────┴──────────────────────────────┐
│     SLAM Toolbox      │   │              Nav2 Stack                  │
│  (LiDAR-based SLAM)   │   │                                         │
│                       │   │  ┌─────────────────────────────────┐    │
│  • Scan matching      │   │  │  bt_navigator                   │    │
│  • Pose graph         │   │  │  (NavigateToPose /              │    │
│  • Loop closure       │   │  │   NavigateThroughPoses actions) │    │
│  • Ceres solver       │   │  └───────────┬────────────┬────────┘    │
│                       │   │              │            │             │
│  Publishes:           │   │  ┌───────────▼──┐  ┌──────▼──────────┐ │
│  /map (OccupancyGrid) │   │  │planner_server│  │controller_server│ │
│  map→odom TF          │   │  │(SmacPlanner  │  │(MPPI — 1000     │ │
└───────────┬───────────┘   │  │ 2D, A*)      │  │ trajectory      │ │
            │ /map           │  └──────────────┘  │ samples/cycle)  │ │
            ▼               │                     └─────────────────┘ │
┌───────────────────────┐   │  + behavior_server (spin/backup/wait)   │
│   Frontier Explorer   │   │  + velocity_smoother                    │
│  (custom ROS2 node)   │   │  + collision_monitor                    │
│                       ├───┘                                         │
│  1. BFS frontier      │  NavigateToPose /                           │
│     detection         │  NavigateThroughPoses                       │
│  2. GVD extraction    │  (action client→server)                     │
│  3. Cost ranking      │                                             │
│  4. GVD goal snap     │                                             │
│  5. A* on skeleton    │                                             │
│  6. Waypoint publish  │                                             │
└───────────────────────┘
```

### Node Graph Summary

| Node | Role | Key I/O |
|------|------|---------|
| `slam_toolbox` | LiDAR SLAM | In: `/scan` → Out: `/map`, `map→odom TF` |
| `bt_navigator` | Navigation orchestration | In: `navigate_to_pose` action → Out: nav commands |
| `planner_server` | Global path planning (A*) | In: goal pose → Out: path |
| `controller_server` | Local trajectory control (MPPI) | In: path → Out: `/cmd_vel` |
| `frontier_explorer` | Autonomous goal selection | In: `/map` → Out: `navigate_to_pose` / `navigate_through_poses` |

---

## 2. Component Overview

### Repository Layout

```
auto_explore_sim/
├── launch/
│   ├── auto_explore.launch.py          # TurtleBot3 + SLAM Toolbox + Nav2 + Explorer
│   ├── g1_explore.launch.py            # Unitree G1 + SLAM Toolbox + Nav2 + Explorer
│   └── g1_explore_orbslam3.launch.py   # Unitree G1 + ORB-SLAM3 + Nav2 + Explorer
├── scripts/
│   ├── frontier_explorer.py            # Main exploration logic (~1260 lines)
│   ├── orb_slam3_tf_bridge.py          # ORB-SLAM3 pose → map→odom TF
│   └── orb_slam3_map_publisher.py      # LiDAR scan → OccupancyGrid via ORB-SLAM3 TF
├── config/
│   ├── frontier_explorer_params.yaml   # Explorer tuning
│   ├── nav2_params.yaml                # Nav2 (TB3)
│   ├── g1_nav2_params.yaml             # Nav2 (G1, tuned for heavier robot)
│   ├── slam_toolbox_params.yaml        # SLAM Toolbox settings
│   ├── orbslam3_camera.yaml            # ORB-SLAM3 camera intrinsics
│   └── g1_bridge.yaml                  # Gazebo↔ROS2 topic bridge
├── models/g1_description/              # Unitree G1 SDF, URDF, and meshes
└── worlds/office.sdf                   # Office simulation environment
```

### Launch Startup Sequence

```
t=0s   Gazebo starts, office world loads
t=2s   Robot spawned into simulation
t=3s   Robot State Publisher (URDF TF tree)
t=4s   SLAM Toolbox starts, begins building map
t=5s   Nav2 lifecycle nodes start
t=15s  Frontier Explorer starts (waits for Nav2 to be ready)
```

The 15-second delay ensures Nav2's lifecycle nodes (planner, controller, bt_navigator) are fully active before the explorer sends its first goal.

---

## 3. SLAM Toolbox — LiDAR SLAM Backend

SLAM Toolbox implements **graph-based SLAM** using pose graph optimization with the Ceres solver.

### How It Works

```
LiDAR scan (10 Hz)
    │
    ▼
Scan Matching ──────────────────────────────────────────────────────┐
    │  Correlate current scan with previous scan using a 2D          │
    │  cross-correlation on a 0.5m search window, 0.01m step         │
    │  → refined robot pose estimate (corrects wheel odometry drift) │
    ▼                                                                │
Pose Graph Node Addition                                            │
    │  New node added if robot moved >0.3m OR rotated >17°           │
    │  Each node stores: scan + refined pose estimate                │
    ▼                                                                │
Ceres Graph Optimization (SPARSE_NORMAL_CHOLESKY)                   │
    │  Minimizes sum of squared pose errors across all edges         │
    │  Edges: odometry constraints + scan match constraints           │
    ▼                                                                │
Loop Closure Detection                                              │
    │  Search for revisited areas within 3.0m radius                │
    │  If found: add loop edge → re-optimize full graph              │
    ▼                                                                │
Map Publication (every 3s) + TF Publication (50 Hz)                │
    │  /map: OccupancyGrid, 0.05 m/cell resolution                  │
    │  map→odom TF: localization anchor for all Nav2 nodes           └─◄
```

### Key Parameters (`slam_toolbox_params.yaml`)

| Parameter | Value | Effect |
|-----------|-------|--------|
| `resolution` | 0.05 m | Grid cell size — matches Nav2 costmap and frontier explorer |
| `mode` | mapping | Build new map (vs. localization-only) |
| `minimum_travel_distance` | 0.3 m | Min movement to add new pose graph node |
| `minimum_travel_heading` | 0.3 rad | Min rotation to add new pose graph node |
| `loop_search_distance` | 3.0 m | Radius to search for loop closure candidates |
| `transform_publish_period` | 0.02 s | 50 Hz map→odom TF broadcast |
| `map_update_interval` | 3.0 s | `/map` topic update rate |
| `solver_plugin` | Ceres | Sparse Cholesky pose graph solver |

### Why Resolution Matters

All three systems (SLAM, Nav2 costmap, and Frontier Explorer) share **0.05 m/cell resolution**. This alignment is intentional — the frontier explorer reads the SLAM map directly as a NumPy array and operates in the same grid coordinate system, avoiding any re-projection or interpolation artifacts.

---

## 4. Nav2 Navigation Stack

Nav2 provides the navigation infrastructure: global path planning, local trajectory control, and failure recovery.

### Behavior Tree Navigator

The `bt_navigator` uses `navigate_w_replanning_and_recovery.xml` as the behavior tree. This tree:

1. **Compute path** → `planner_server`
2. **Follow path** → `controller_server`
3. **On failure** → try recovery behaviors in order: `spin → backup → wait → clear_costmap`
4. **Replan** after recovery — the robot does not abort immediately

This means Nav2 is resilient: a temporarily blocked path triggers recovery rather than goal cancellation.

### Global Planner — SmacPlanner2D (A*)

`planner_server` runs **SmacPlanner2D**, a 2D grid A* search on the Nav2 costmap.

- **Costmap**: built from SLAM's `/map` topic + inflation layer
- **Inflation**: obstacle cells expanded by robot radius to enforce clearance
- **Output**: a `nav_msgs/Path` (sequence of poses) from current robot pose to goal

### Local Controller — MPPI

`controller_server` runs **Model Predictive Path Integral (MPPI)** control, a sampling-based trajectory optimizer.

```
Every control cycle (0.05 s):
    │
    ├─ Sample 1000 random trajectories over 2.8s horizon (56 steps × 0.05s)
    │   Each trajectory: perturb previous best controls with Gaussian noise
    │
    ├─ Evaluate each trajectory:
    │   Cost = Σ (path_following_error + obstacle_proximity + velocity_deviation)
    │
    ├─ Weighted average: lower-cost trajectories get exponentially higher weight
    │
    └─ Apply first control step → /cmd_vel_nav → velocity_smoother → /cmd_vel
```

**Robot Limits:**

| Robot | Max Linear | Max Angular | Max Linear Accel |
|-------|-----------|-------------|-----------------|
| TurtleBot3 Waffle | 1.0 m/s | 1.9 rad/s | 2.5 m/s² |
| Unitree G1 | 0.8 m/s | 1.5 rad/s | 1.5 m/s² |

### Goal Tolerance (Deliberately Relaxed)

```yaml
xy_goal_tolerance: 0.5    # robot stops anywhere within 0.5m of goal
yaw_goal_tolerance: 3.14  # any final heading accepted
```

Frontier goals are approximate by nature (centroids of unexplored clusters), so tight goal tolerance would cause Nav2 to fail at goals that are valid exploration targets. The Frontier Explorer handles the "did we actually explore this area?" question through its own **blacklisting mechanism** (see Section 5).

### Progress Checker

```yaml
movement_time_allowance: 20.0  # 20s to move 0.15m before Nav2 reports stuck
```

Very permissive — allows the robot to push slowly through tight spaces without aborting. The Frontier Explorer adds its own shorter-term progress check on top of this.

---

## 5. Frontier Explorer — Mechanism Under the Hood

The frontier explorer (`scripts/frontier_explorer.py`) is the "brain" that decides **where to go next**. It runs on a timer at `planner_frequency` (default 0.5 Hz) and also replans immediately whenever a navigation goal completes.

### What Is a Frontier?

A **frontier** is a boundary between known free space and unknown space in the occupancy grid:

```
  Map cell states:
    0   = FREE (known empty)
   -1   = UNKNOWN (not yet observed)
  100   = OCCUPIED (obstacle)

  Frontier cell: occupancy == -1 AND has at least one 4-connected FREE neighbor
```

A frontier cell means: "if the robot goes here, it will observe new space." Exploring frontiers drives the robot to cover the entire map.

### Step 1 — BFS Frontier Detection

```python
def _search_from(robot_grid_position):
    # Phase 1: BFS through FREE space from robot position
    queue = [robot_grid_position]
    visited = set()

    while queue:
        cell = queue.pop()
        if cell in visited: continue
        visited.add(cell)

        for neighbor in 8_connected_neighbors(cell):
            if is_free(neighbor):
                queue.append(neighbor)         # expand through free space
            elif is_frontier(neighbor):
                build_frontier_cluster(neighbor)  # found frontier edge
```

**Frontier Cluster Building** (Phase 2):
When a frontier cell is found, a second BFS collects all 8-connected frontier cells into one cluster:

```python
def _build_frontier(seed_cell):
    cluster = []
    queue = [seed_cell]
    while queue:
        cell = queue.pop()
        cluster.append(cell)
        for neighbor in 8_connected_neighbors(cell):
            if is_frontier(neighbor) and neighbor not in cluster:
                queue.append(neighbor)

    return Frontier(
        size      = len(cluster),
        centroid  = mean_position(cluster),    # average x,y of all cells
        min_dist  = min(distance(robot, cell) for cell in cluster)
    )
```

### Step 2 — Cost Function

Each frontier is ranked by a composite cost:

```
cost = min_distance - clearance_scale × clearance_at_centroid
```

- **`min_distance`** (metres): distance from robot to the nearest frontier cell — biases toward nearby unexplored areas (nearest-first strategy)
- **`clearance_at_centroid`** (metres): distance to the nearest obstacle at the frontier centroid — how wide is the corridor here?
- **`clearance_scale = 0.2`**: in a tie, 1m extra clearance reduces cost by 0.2m (prefer open areas over narrow gaps)

The clearance bonus is small enough that distance dominates, but breaks ties in favor of safer routes.

### Step 3 — Generalized Voronoi Diagram (GVD) Extraction

**Problem**: Frontier centroids can land near walls or in geometrically awkward positions, causing Nav2 to fail (path through obstacles) or the robot to wedge into corners.

**Solution**: Compute the **Generalized Voronoi Diagram** — the skeleton of free space equidistant from all obstacles — and snap navigation goals onto it.

The GVD is computed in two phases each time the map updates:

#### Phase A — Obstacle Connected-Component Labeling

```
Each occupied cell (occupancy > 50) is assigned a unique label.
8-connected BFS flood-fill groups contiguous obstacle cells
into distinct "obstacle regions".

  Example:
  ████████   ←─ obstacle region 0
  ···  ···
  ·········
  ███  ███   ←─ region 1 (left wall)  region 2 (right wall)
```

Unknown cells (`-1`) are **excluded** — they are the exploration frontier, not solid walls.

#### Phase B — Multi-Source BFS Distance Transform

Starting from all obstacle cells simultaneously:

```python
# Multi-source BFS: every obstacle cell seeds the queue
for each occupied cell:
    queue.append((cell, distance=0, label=cell_label))

while queue:
    cell, dist, label = queue.pop()
    if cell already visited: continue
    dist_map[cell] = dist
    label_map[cell] = label     # which obstacle is nearest?
    for neighbor in 4_connected(cell):
        if not visited:
            queue.append((neighbor, dist+1, label))
```

Result: every free cell knows **how far it is from the nearest obstacle** (`dist_map`) and **which obstacle that is** (`label_map`).

#### GVD Point Extraction (Vectorized NumPy)

```python
# A cell is a GVD skeleton point if:
#   1. It is FREE (not occupied or unknown)
#   2. At least one 4-connected neighbor has a DIFFERENT obstacle label
#      → cell is equidistant from 2+ distinct obstacles
#   3. Its distance to obstacle >= gvd_min_clearance (default 5 cells = 0.25m)

for each free cell (y, x):
    neighbors = [label_map[y-1,x], label_map[y+1,x],
                 label_map[y,x-1], label_map[y,x+1]]
    if any(n != label_map[y,x] for n in neighbors) \
       and dist_map[y,x] >= gvd_min_clearance:
        gvd_mask[y,x] = True
```

The resulting `gvd_mask` is the Voronoi skeleton: a 1-cell-wide ridge running through the center of every corridor and open area, maximally distant from all walls.

```
Narrow corridor example:
    ████████████
    ···○·○·○·○··   ← GVD skeleton points (○) equidistant from both walls
    ····○·○·○···
    ████████████
```

### Step 4 — Frontier Goal Snapping

Before sending a goal to Nav2, the frontier centroid is **snapped** to the nearest GVD point:

```python
def _snap_to_gvd(frontier_centroid, robot_position):
    # BFS outward from frontier centroid
    # Find nearest GVD cell that:
    #   1. Is within gvd_snap_radius (2.0m)
    #   2. Lies between robot and frontier (not behind robot)
    #      dot(gvd_point - robot, frontier - robot) > 0
    ...
    return snapped_goal if found else frontier_centroid
```

The directional constraint prevents the snapped goal from placing the robot in the opposite direction from the frontier.

### Step 5 — Blacklisting & Progress Monitoring

The explorer tracks two kinds of failure:

**Progress timeout** (robot stuck):
```
Every 1s: check if robot moved > 0.3m since last check
If not moved for progress_timeout (15s):
    → cancel current Nav2 goal
    → blacklist frontier centroid
    → immediately replan
```

**Arrive-and-loop bug** (Nav2 "succeeds" within 0.5m tolerance, frontier still visible):
```
On Nav2 SUCCEEDED result:
    → blacklist frontier centroid + all frontiers within blacklist_radius (0.5m)
On Nav2 ABORTED result:
    → blacklist goal location
```

Blacklist entries expire after `blacklist_timeout` (120s), allowing retry if the area becomes accessible later.

**Blacklist override**: If blacklisting removes all candidate frontiers, the blacklist is cleared — exploration continues rather than halting.

### Step 6 — Navigation Goal Dispatch

Two modes depending on whether a GVD path was found:

```
GVD path found (A* succeeded):
    → _navigate_through_poses(waypoints)
       Sends NavigateThroughPoses action to bt_navigator
       Nav2 advances through waypoints as robot passes each one

GVD path not found (fallback):
    → _navigate_to(snapped_goal)
       Sends NavigateToPose action to bt_navigator
       Nav2 plans direct path to single goal
```

### Step 7 — Immediate Replanning

Unlike explore_lite (which waits for a periodic timer), this explorer replans **immediately** on goal completion:

```python
def _navigation_result_cb(future):
    result = future.result().status
    # Called when NavigateToPose or NavigateThroughPoses finishes
    # (SUCCESS, ABORTED, or CANCELED)
    self.navigating = False
    self._make_plan()   # ← immediate, not waiting for timer
```

This keeps the robot in constant motion — as soon as one goal is resolved, the next is computed.

---

## 6. GVD Path Planning

When the GVD skeleton is available, the explorer plans a path along it using **A\* graph search**.

### A\* Cost Function

The graph is the full occupancy grid. Edge costs are:

| Cell type | Straight cost | Diagonal cost |
|-----------|:---:|:---:|
| GVD skeleton | 1.0 | √2 ≈ 1.41 |
| Free (non-GVD) | 5.0 | 5√2 ≈ 7.07 |
| Occupied / Unknown | ∞ | — |

The **5× penalty** on non-GVD cells strongly encourages the path to stay on the skeleton, while still allowing it to cross open areas if the skeleton is disconnected.

### Nearest GVD Cell Search

Before running A\*, the robot position and goal position are each snapped to their nearest GVD cells:

```python
def _find_nearest_gvd(position, max_radius=1.5m):
    # BFS outward from position
    # Return first GVD cell found within max_radius
    # Fallback: None → use single-goal navigation
```

If either the robot or the goal has no GVD cell within 1.5m (e.g., robot is in open space far from any wall), the GVD path planning is skipped and `NavigateToPose` is used directly.

### Waypoint Sampling

The raw A\* path contains one point per grid cell (0.05m spacing). This is down-sampled for Nav2:

```python
waypoints = [path[0]]
for i, point in enumerate(path):
    if distance(point, waypoints[-1]) >= 0.5m:
        waypoints.append(point)
waypoints.append(path[-1])
```

0.5m spacing avoids overwhelming Nav2's `NavigateThroughPoses` action with hundreds of waypoints while preserving path shape.

### Visualization

Three RViz topics provide real-time debugging:

| Topic | Color | Content |
|-------|-------|---------|
| `/explore/frontiers` | Blue (candidate) / Green (chosen) | Frontier centroid spheres |
| `/explore/gvd` | Cyan | GVD skeleton (LINE_LIST) |
| `/explore/gvd_path` | Magenta | Current navigation path (LINE_STRIP + waypoint spheres) |

---

## 7. Full Data Flow

### LiDAR SLAM Mode (default)

```
Gazebo Harmonic
    ├─ /scan (LaserScan, 10 Hz)  ──────────────────────► SLAM Toolbox
    │                                                        │
    │                                                   ┌────┴────────────┐
    │                                                   │ Scan matching   │
    │                                                   │ Graph optimize  │
    │                                                   │ Loop closure    │
    │                                                   └────┬────────────┘
    │                                                        │
    │                                           /map ◄───────┤ (3 Hz)
    │                                        map→odom TF ◄───┘ (50 Hz)
    │
    └─ robot physics ◄──── /cmd_vel ◄──────────────────── controller_server
                                                                ▲
                                                         /cmd_vel_nav
                                                                ▲
                                                        MPPI controller
                                                          (1000 samples)
                                                                ▲
                                                           global path
                                                                ▲
                                                        SmacPlanner2D (A*)
                                                                ▲
                                                         goal pose
                                                                ▲
                                                        bt_navigator
                                                          (BT + recovery)
                                                                ▲
                                               NavigateToPose / NavigateThroughPoses
                                                                ▲
                                                      Frontier Explorer
                                                          │         ▲
                                                       /map         │ nav result cb
                                                          │         │
                                              1. BFS frontier detection
                                              2. GVD extraction (BFS distance transform)
                                              3. Cost ranking (distance + clearance)
                                              4. Goal snap to GVD
                                              5. A* on skeleton → waypoints
```

### Key TF Tree

```
map
 └─ odom          ← published by SLAM Toolbox (50 Hz)
     └─ base_link ← published by robot odometry (wheel encoders)
         ├─ laser_frame   (LiDAR)
         ├─ camera_link   (RGBD camera)
         └─ [wheel frames, etc.]
```

SLAM Toolbox owns the `map→odom` edge. This edge corrects accumulated odometry drift. The Frontier Explorer queries `map→base_link` to know the robot's position in the global map frame.

---

## 8. ORB-SLAM3 Visual Backend (Alternative)

The `g1_explore_orbslam3.launch.py` configuration replaces SLAM Toolbox with **ORB-SLAM3** (visual SLAM from RGBD camera).

### Why ORB-SLAM3?

| | SLAM Toolbox | ORB-SLAM3 |
|---|---|---|
| **Sensor** | 2D LiDAR | RGBD Camera |
| **Features** | Range rings | Visual keypoints + depth |
| **Loop closure** | Scan correlation | BoW descriptor matching |
| **Robustness** | Dark environments | Texture-rich environments |
| **Output** | `/map` directly | Pose only → needs bridge nodes |

### Coordinate Frame Problem

ORB-SLAM3 outputs poses in **camera optical frame** convention:
- Z = forward (into scene)
- X = right
- Y = down

ROS2 / Nav2 expects **REP-103 body frame**:
- X = forward
- Y = left
- Z = up

Two bridge nodes handle this:

**`orb_slam3_tf_bridge.py`** — Pose → TF:
```python
# Two rotation matrices:
R_ORB_TO_ROS = [[0, 0, 1],   # ORB Z (forward) → ROS X
                [-1, 0, 0],  # ORB X (right)   → ROS -Y
                [0, -1, 0]]  # ORB Y (down)    → ROS -Z

R_OPT_TO_BODY = [[0, 0, 1],  # opt Z → body X
                 [-1, 0, 0], # opt -X → body Y
                 [0, -1, 0]] # opt -Y → body Z

# Final: T(map→odom) derived from ORB-SLAM3 camera pose
# using known camera_link→base_link static transform
```

**`orb_slam3_map_publisher.py`** — LiDAR → OccupancyGrid:
ORB-SLAM3 provides localization but no map. This node uses ORB-SLAM3's `map→odom` TF to project each incoming LiDAR scan into the map frame and maintains an occupancy grid using **log-odds updates**:

```
Obstacle cell (ray endpoint): log_odds += 0.85  (log(P(occ)/P(free)))
Free cell (along ray):         log_odds -= 0.40
Publish at 1 Hz → /map (for Frontier Explorer and Nav2)
```

The rest of the pipeline (Frontier Explorer, Nav2) is identical to the SLAM Toolbox mode.

---

## 9. Configuration & Tuning

### Frontier Explorer (`config/frontier_explorer_params.yaml`)

| Parameter | Default | Effect |
|-----------|---------|--------|
| `planner_frequency` | 0.5 Hz | How often frontier search runs (not including immediate replans) |
| `min_frontier_size` | 40 cells | Minimum cluster size — filters noise, walls, and tiny gaps |
| `blacklist_radius` | 0.5 m | Radius around a blacklisted point where nearby frontiers are also excluded |
| `blacklist_timeout` | 120.0 s | How long before a blacklisted frontier can be retried |
| `progress_timeout` | 15.0 s | Seconds without 0.3m movement before canceling goal |
| `clearance_scale` | 0.2 | Weight of GVD clearance bonus in cost function |
| `gvd_min_clearance` | 5 cells | Minimum distance to obstacle for a valid GVD skeleton point (0.25m at 0.05m/cell) |
| `gvd_snap_radius` | 2.0 m | Maximum search radius when snapping goal to GVD |
| `return_to_init` | false | Navigate back to start when no frontiers remain |

### Nav2 Key Parameters

| Parameter | Location | Default | Notes |
|-----------|----------|---------|-------|
| `xy_goal_tolerance` | nav2_params.yaml | 0.5 m | Relaxed — frontier goals are approximate |
| `movement_time_allowance` | nav2_params.yaml | 20.0 s | Progress checker patience |
| `vx_max` | nav2_params.yaml | 1.0 / 0.8 m/s | TB3 / G1 |
| `wz_max` | nav2_params.yaml | 1.9 / 1.5 rad/s | TB3 / G1 |

### SLAM Toolbox Key Parameters

| Parameter | Default | Notes |
|-----------|---------|-------|
| `resolution` | 0.05 m | Must match Nav2 costmap resolution |
| `minimum_travel_distance` | 0.3 m | Increase for slow robots to reduce graph size |
| `loop_search_distance` | 3.0 m | Increase for larger environments |
| `map_update_interval` | 3.0 s | Decrease for more responsive frontier detection |

---

## 10. Launch Guide

### TurtleBot3 Waffle (LiDAR SLAM)

```bash
ros2 launch auto_explore_sim auto_explore.launch.py
```

Requires: `nav2_minimal_tb3_sim` package, Gazebo Harmonic.

### Unitree G1 Humanoid (LiDAR SLAM)

```bash
ros2 launch auto_explore_sim g1_explore.launch.py
```

Requires: G1 SDF (`models/g1_description/`), `ros_gz_bridge`, Gazebo Harmonic.

### Unitree G1 Humanoid (ORB-SLAM3 Visual SLAM)

```bash
# 1. Extract ORB vocabulary (one-time, ~1 min)
cd /path/to/ORB_SLAM3/Vocabulary
tar -xf ORBvoc.txt.tar.gz

# 2. Launch
ros2 launch auto_explore_sim g1_explore_orbslam3.launch.py
```

Requires: ORB-SLAM3 compiled and sourced, vocabulary file at expected path.

### Pause/Resume Exploration

```bash
# Pause
ros2 topic pub /explore/resume std_msgs/Bool "data: false" --once

# Resume
ros2 topic pub /explore/resume std_msgs/Bool "data: true" --once
```

### Monitor in RViz

Open `rviz/explore.rviz`. Key displays:
- **Map** (`/map`) — SLAM occupancy grid
- **Frontiers** (`/explore/frontiers`) — blue/green spheres
- **GVD** (`/explore/gvd`) — cyan skeleton overlay
- **GVD Path** (`/explore/gvd_path`) — current magenta waypoint path
- **Robot Model** — from URDF via `robot_state_publisher`

---

## Design Notes

**Why GVD instead of pure nearest-first?**
Pure nearest-first frontier selection sends the robot into narrow gaps and near-wall positions, causing frequent Nav2 failures. The GVD skeleton represents the safest navigable path — maximally distant from all obstacles — so snapping goals and paths onto it dramatically improves navigation reliability.

**Why immediate replanning instead of periodic?**
explore_lite uses a fixed timer (typically 0.1 Hz = 10s latency). Replanning immediately on goal completion keeps the robot moving continuously, which is critical in small environments where goals are reached quickly.

**Why relaxed goal tolerance?**
Frontier centroids are statistical averages of unknown cell positions. They often land in areas the robot cannot physically reach (inside unmapped space). A 0.5m tolerance allows Nav2 to "succeed" at the boundary of the frontier, after which the explorer blacklists the centroid and moves on — correctly modeling exploration progress.
