#!/usr/bin/env python3
"""
Frontier-based autonomous explorer node.

Inspired by m-explore (https://github.com/MusLead/m_explorer_ROS2_husarion).

Key features:
  - BFS frontier search outward from robot position (not full-map scan)
  - min_distance per frontier (closest cell to robot, not centroid)
  - Nearest-first cost with GVD clearance bonus for tiebreaking
  - Same-goal detection (skip re-sending identical goal)
  - Stop/Resume via explore/resume topic (std_msgs/Bool)
  - Return-to-init option when exploration completes
  - Immediate replan on goal completion (no waiting for next timer tick)
  - Blacklisting of unreachable frontiers with timeout
  - RViz frontier marker visualisation
"""

import math
import heapq
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros


# ---------------------------------------------------------------------------
# Frontier data container
# ---------------------------------------------------------------------------
class Frontier:
    """One frontier cluster discovered by BFS."""
    __slots__ = ('size', 'min_distance', 'cost', 'centroid', 'middle',
                 'initial', 'points', 'cells', 'goal')

    def __init__(self):
        self.size = 0               # number of cells
        self.min_distance = float('inf')
        self.cost = 0.0
        self.centroid = (0.0, 0.0)  # average of all points (world coords)
        self.middle = (0.0, 0.0)    # point at size//2 (world coords)
        self.initial = (0.0, 0.0)   # first point found (world coords)
        self.points = []            # all points as (wx, wy)
        self.cells = []             # all frontier cells as (mx, my)
        self.goal = None            # filtered navigation goal (wx, wy)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class FrontierExplorerNode(Node):
    """Autonomous frontier exploration using Nav2."""

    def __init__(self):
        super().__init__('frontier_explorer')
        self.get_logger().info('Frontier Explorer Node starting...')

        # ── Declare parameters ──────────────────────────────────────────
        self.declare_parameter('planner_frequency', 0.5)        # Hz
        self.declare_parameter('min_frontier_size', 5)           # cells
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('transform_tolerance', 2.0)
        self.declare_parameter('blacklist_radius', 0.5)          # metres
        self.declare_parameter('blacklist_timeout', 60.0)        # seconds
        self.declare_parameter('progress_timeout', 30.0)
        self.declare_parameter('min_goal_distance', 0.5)         # metres
        self.declare_parameter('same_goal_cooldown', 4.0)        # seconds
        self.declare_parameter('stuck_escape_enabled', True)
        self.declare_parameter('near_wall_clearance', 0.35)       # metres
        self.declare_parameter('stuck_escape_search_radius', 2.5)  # metres
        self.declare_parameter('stuck_escape_min_goal_distance', 0.6)  # metres
        self.declare_parameter('stuck_escape_min_clearance', 0.5)  # metres
        self.declare_parameter('stuck_escape_distance_weight', 0.2)
        self.declare_parameter('visualize', True)
        self.declare_parameter('clearance_scale', 0.3)       # GVD clearance tiebreaker
        self.declare_parameter('gvd_goal_on_skeleton', True)
        self.declare_parameter('gvd_projection_radius', 0.0)   # metres (<=0 means unlimited)
        self.declare_parameter('gvd_min_clearance', 0.1)      # metres
        self.declare_parameter('gvd_visualize', True)
        self.declare_parameter('gvd_marker_stride', 2)
        self.declare_parameter('gvd_marker_scale', 0.03)       # metres
        self.declare_parameter('orientation_scale', 0.5)     # heading alignment bonus
        self.declare_parameter('orientation_window_deg', 60.0)  # forward cone half-angle
        self.declare_parameter('return_to_init', False)
        self.declare_parameter('global_costmap_topic', 'global_costmap/costmap_raw')
        self.declare_parameter('local_costmap_topic', 'local_costmap/costmap_raw')
        self.declare_parameter('costmap_obstacle_threshold', 253)
        self.declare_parameter('treat_unknown_as_obstacle', True)

        # ── Read parameters ─────────────────────────────────────────────
        self.planner_freq = self.get_parameter('planner_frequency').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.tf_tolerance = self.get_parameter('transform_tolerance').value
        self.blacklist_radius = self.get_parameter('blacklist_radius').value
        self.blacklist_timeout = self.get_parameter('blacklist_timeout').value
        self.progress_timeout = self.get_parameter('progress_timeout').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value
        self.same_goal_cooldown = self.get_parameter('same_goal_cooldown').value
        self.stuck_escape_enabled = self.get_parameter('stuck_escape_enabled').value
        self.near_wall_clearance = self.get_parameter('near_wall_clearance').value
        self.stuck_escape_search_radius = self.get_parameter('stuck_escape_search_radius').value
        self.stuck_escape_min_goal_distance = self.get_parameter('stuck_escape_min_goal_distance').value
        self.stuck_escape_min_clearance = self.get_parameter('stuck_escape_min_clearance').value
        self.stuck_escape_distance_weight = self.get_parameter('stuck_escape_distance_weight').value
        self.visualize = self.get_parameter('visualize').value
        self.clearance_scale = self.get_parameter('clearance_scale').value
        self.gvd_goal_on_skeleton = self.get_parameter('gvd_goal_on_skeleton').value
        self.gvd_projection_radius = self.get_parameter('gvd_projection_radius').value
        self.gvd_min_clearance = self.get_parameter('gvd_min_clearance').value
        self.gvd_visualize = self.get_parameter('gvd_visualize').value
        self.gvd_marker_stride = self.get_parameter('gvd_marker_stride').value
        self.gvd_marker_scale = self.get_parameter('gvd_marker_scale').value
        self.orientation_scale = self.get_parameter('orientation_scale').value
        self.orientation_window_deg = self.get_parameter('orientation_window_deg').value
        self.return_to_init = self.get_parameter('return_to_init').value
        self.global_costmap_topic = self.get_parameter('global_costmap_topic').value
        self.local_costmap_topic = self.get_parameter('local_costmap_topic').value
        self.costmap_obstacle_threshold = self.get_parameter('costmap_obstacle_threshold').value
        self.treat_unknown_as_obstacle = self.get_parameter('treat_unknown_as_obstacle').value

        # ── TF listener ─────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscribers ─────────────────────────────────────────────────
        self.map_data = None
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, 10)
        self.global_costmap_data = None
        self.local_costmap_data = None
        self.global_costmap_sub = self.create_subscription(
            Costmap, self.global_costmap_topic, self._global_costmap_callback, 10)
        self.local_costmap_sub = self.create_subscription(
            Costmap, self.local_costmap_topic, self._local_costmap_callback, 10)

        # Stop / resume subscription
        self.exploring = True
        self.resume_sub = self.create_subscription(
            Bool, 'explore/resume', self._resume_callback, 10)

        # ── Nav2 action client ──────────────────────────────────────────
        self._action_cb_group = MutuallyExclusiveCallbackGroup()
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._action_cb_group)

        # ── Visualisation publisher ─────────────────────────────────────
        if self.visualize:
            self.marker_pub = self.create_publisher(
                MarkerArray, 'explore/frontiers', 10)

        # ── State ───────────────────────────────────────────────────────
        self.navigating = False
        self.current_goal = None            # (x, y) world coords
        self.current_goal_kind = None       # 'frontier' | 'recovery' | 'return'
        self.prev_goal = None               # last goal sent to Nav2
        self.prev_goal_time = None          # time when prev_goal was sent
        self.goal_handle = None
        self.blacklisted = []               # list of (x, y, stamp)
        self._goal_seq = 0                  # incremented each _navigate_to call
        self.last_progress_time = None
        self.last_robot_pos = None
        self.initial_pose = None            # stored for return_to_init
        self._cached_dist_map = None        # GVD distance transform cache
        self._cached_map_token = None
        self._cached_voronoi_labels = None
        self._cached_gvd_mask = None
        self._cached_gvd_project_dist = None
        self._cached_gvd_project_x = None
        self._cached_gvd_project_y = None

        # ── Timer ───────────────────────────────────────────────────────
        period = 1.0 / max(self.planner_freq, 0.01)
        self.timer = self.create_timer(period, self._explore_tick)

        self.get_logger().info(
            f'Frontier Explorer ready  (freq={self.planner_freq} Hz, '
            f'min_frontier={self.min_frontier_size} cells, '
            f'clearance_scale={self.clearance_scale}, '
            f'gvd_goal_on_skeleton={self.gvd_goal_on_skeleton}, '
            f'same_goal_cooldown={self.same_goal_cooldown}s, '
            f'orientation_scale={self.orientation_scale}, '
            f'orientation_window_deg={self.orientation_window_deg}, '
            f'return_to_init={self.return_to_init}, '
            f'stuck_escape_enabled={self.stuck_escape_enabled})')

    # ================================================================
    # Callbacks
    # ================================================================

    def _map_callback(self, msg: OccupancyGrid):
        self.map_data = msg
        # Invalidate cached GVD whenever SLAM publishes a new map message.
        self._cached_map_token = None

    def _global_costmap_callback(self, msg: Costmap):
        self.global_costmap_data = msg

    def _local_costmap_callback(self, msg: Costmap):
        self.local_costmap_data = msg

    def _resume_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Exploration RESUMED')
            self.exploring = True
        else:
            self.get_logger().info('Exploration STOPPED')
            self.exploring = False
            self._cancel_current_goal()
            self.navigating = False

    # ================================================================
    # Main exploration loop
    # ================================================================

    def _explore_tick(self):
        if not self.exploring:
            return

        self._make_plan()

    def _make_plan(self):
        """Core planning: find frontiers from robot, pick best, navigate."""
        if self.map_data is None:
            self.get_logger().info('Waiting for map...', throttle_duration_sec=5.0)
            return

        # 1. Get robot pose in map frame
        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            return
        robot_xy = (robot_pose[0], robot_pose[1])
        robot_yaw = robot_pose[2]

        # Store initial pose for return_to_init
        if self.initial_pose is None:
            self.initial_pose = robot_xy
            self.get_logger().info(
                f'Initial pose stored: ({robot_xy[0]:.2f}, {robot_xy[1]:.2f})')

        # 2. Check navigation progress
        self._check_progress(robot_xy)

        # If already navigating and making progress, do nothing
        if self.navigating:
            return

        # 3. BFS frontier search from robot position
        info = self.map_data.info
        map_array = np.array(self.map_data.data, dtype=np.int8).reshape(
            (info.height, info.width))

        frontiers = self._search_from(robot_xy, robot_yaw, map_array, info)

        if not frontiers:
            self.get_logger().info(
                'No frontiers found — exploration may be complete!',
                throttle_duration_sec=10.0)
            if self.return_to_init and self.initial_pose is not None:
                self._return_to_initial_pose()
            return

        # 4. Filter blacklisted
        now = self.get_clock().now()
        self.blacklisted = [
            (bx, by, t) for bx, by, t in self.blacklisted
            if (now - t).nanoseconds / 1e9 < self.blacklist_timeout
        ]
        valid = [
            f for f in frontiers
            if not self._is_blacklisted(*(f.goal if f.goal is not None else f.centroid))
        ]

        if not valid:
            self.get_logger().warn(
                'All frontiers blacklisted — clearing blacklist')
            self.blacklisted.clear()
            valid = frontiers

        # 5. Pick best (already sorted by cost, pick first non-blacklisted)
        best = valid[0]

        goal_x, goal_y = best.goal if best.goal is not None else best.centroid
        goal_projected = False
        if self.gvd_goal_on_skeleton:
            goal_mx, goal_my = self._world_to_grid_cell(goal_x, goal_y, info)
            if goal_mx is not None:
                projected_cell = self._project_to_gvd_cell(
                    goal_mx, goal_my, map_array, info.resolution)
                if projected_cell is not None:
                    ox = info.origin.position.x
                    oy = info.origin.position.y
                    pgx = float(projected_cell[0]) * info.resolution + ox
                    pgy = float(projected_cell[1]) * info.resolution + oy
                    if self._is_world_point_safe(pgx, pgy):
                        goal_x, goal_y = pgx, pgy
                        best.goal = (goal_x, goal_y)
                        goal_projected = True

        goal_on_gvd = False
        goal_mx, goal_my = self._world_to_grid_cell(goal_x, goal_y, info)
        if goal_mx is not None:
            gvd_mask = self._get_gvd_mask(map_array)
            goal_on_gvd = bool(gvd_mask[goal_my, goal_mx])

        heading_bonus = self._frontier_orientation_bonus(
            robot_xy, robot_yaw, goal_x, goal_y)

        self.get_logger().info(
            f'Best frontier: goal=({goal_x:.2f}, {goal_y:.2f})  '
            f'centroid=({best.centroid[0]:.2f}, {best.centroid[1]:.2f})  '
            f'min_dist={best.min_distance:.2f}m  size={best.size}  '
            f'heading_bonus={heading_bonus:.2f}  '
            f'on_gvd={goal_on_gvd}  projected={goal_projected}  '
            f'cost={best.cost:.1f}')

        # 6. Visualise
        if self.visualize:
            self._publish_markers(valid, best, map_array, info)

        # 7. Same-goal detection — skip if goal hasn't changed
        gx, gy = goal_x, goal_y
        if self.prev_goal is not None:
            dx = gx - self.prev_goal[0]
            dy = gy - self.prev_goal[1]
            if math.sqrt(dx * dx + dy * dy) < 0.01:
                if self.prev_goal_time is not None:
                    elapsed = (self.get_clock().now() - self.prev_goal_time).nanoseconds / 1e9
                else:
                    elapsed = float('inf')

                if elapsed < self.same_goal_cooldown:
                    self.get_logger().debug(
                        f'Same goal as before — waiting cooldown '
                        f'({elapsed:.1f}/{self.same_goal_cooldown:.1f}s)')
                    return
                self.get_logger().info(
                    f'Same goal persisted for {elapsed:.1f}s — resending')

        # 8. Navigate
        self._navigate_to(gx, gy)

    # ================================================================
    # BFS frontier search from robot position
    # ================================================================

    def _search_from(self, robot_xy, robot_yaw, map_array, info):
        """
        BFS outward from robot position to find frontiers.
        Mirrors FrontierSearch::searchFrom from m-explore.
        Returns list of Frontier objects sorted by cost.
        """
        resolution = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        w = info.width
        h = info.height

        # Robot position to map cell
        mx = int((robot_xy[0] - ox) / resolution)
        my = int((robot_xy[1] - oy) / resolution)

        if mx < 0 or mx >= w or my < 0 or my >= h:
            self.get_logger().warning('Robot position outside map bounds')
            return []

        # If robot cell is not free, find nearest free cell
        if map_array[my, mx] != 0:
            found = self._nearest_free_cell(map_array, mx, my, w, h)
            if found is None:
                self.get_logger().warning('Cannot find free cell near robot')
                return []
            mx, my = found

        # State flags for each cell
        # 0 = unvisited, 1 = in map-BFS queue, 2 = in map-BFS visited,
        # 3 = in frontier-BFS queue, 4 = frontier-BFS done
        MAP_OPEN = 1
        MAP_CLOSED = 2
        FRONTIER_OPEN = 3
        FRONTIER_CLOSED = 4

        state = np.zeros((h, w), dtype=np.uint8)

        # BFS queue for the main map traversal
        bfs_queue = deque()
        bfs_queue.append((mx, my))
        state[my, mx] = MAP_OPEN

        frontiers = []
        path_dist_map = self._compute_path_distance_map(
            map_array, mx, my, resolution)

        # 8-connected neighbours
        nbrs = [(-1, -1), (-1, 0), (-1, 1),
                (0, -1),           (0, 1),
                (1, -1),  (1, 0),  (1, 1)]

        while bfs_queue:
            cx, cy = bfs_queue.popleft()
            if state[cy, cx] == MAP_CLOSED:
                continue
            state[cy, cx] = MAP_CLOSED

            # Check all 8 neighbours
            for dx, dy in nbrs:
                nx, ny = cx + dx, cy + dy
                if nx < 0 or nx >= w or ny < 0 or ny >= h:
                    continue

                # Is this neighbour a new frontier cell?
                if state[ny, nx] not in (FRONTIER_OPEN, FRONTIER_CLOSED):
                    if self._is_frontier_cell(map_array, nx, ny, w, h, nbrs):
                        # Build a new frontier starting from this cell
                        frontier = self._build_frontier(
                            map_array, state, nx, ny, w, h,
                            robot_xy, ox, oy, resolution, nbrs,
                            FRONTIER_OPEN, FRONTIER_CLOSED)
                        if frontier.size >= self.min_frontier_size:
                            frontiers.append(frontier)

                # Enqueue free-space neighbours for continued map BFS
                val = map_array[ny, nx]
                if val == 0 and state[ny, nx] not in (MAP_OPEN, MAP_CLOSED):
                    # Only expand through free space that neighbours
                    # at least one unknown cell (to stay near boundaries)
                    # OR free space (to traverse open areas)
                    state[ny, nx] = MAP_OPEN
                    bfs_queue.append((nx, ny))

        # Compute distance transform for GVD clearance bonus
        dist_map = self._get_distance_transform(map_array)

        # Filter each frontier to a safe free-space goal and score it.
        # Cost = min_distance - clearance_scale * clearance_at_goal
        #                      - orientation_scale * heading_bonus
        # Primary: nearest first.
        # Secondary: prefer open corridors and forward-aligned goals.
        filtered = []
        rejected_no_goal = 0
        rejected_too_close = 0
        for f in frontiers:
            goal, goal_dist = self._select_frontier_goal(
                f, map_array, info, robot_xy, nbrs, path_dist_map)
            if goal is None:
                rejected_no_goal += 1
                continue
            if goal_dist < self.min_goal_distance:
                rejected_too_close += 1
                continue
            f.goal = goal
            f.min_distance = goal_dist

            cx, cy = self._world_to_grid_cell(goal[0], goal[1], info)
            if cx is None:
                continue
            clearance = float(dist_map[cy, cx]) * resolution  # metres
            heading_bonus = self._frontier_orientation_bonus(
                robot_xy, robot_yaw, goal[0], goal[1])
            f.cost = (
                f.min_distance
                - self.clearance_scale * clearance
                - self.orientation_scale * heading_bonus
            )
            filtered.append(f)

        if frontiers and not filtered:
            self.get_logger().warning(
                'All frontier goals rejected '
                f'(no_goal={rejected_no_goal}, '
                f'too_close={rejected_too_close}, '
                f'min_goal_distance={self.min_goal_distance:.2f}m)',
                throttle_duration_sec=5.0)

        frontiers = filtered
        frontiers.sort(key=lambda f: f.cost)
        return frontiers

    def _compute_path_distance_map(self, map_array, sx, sy, resolution):
        """
        Shortest-path distance from robot cell to all free cells (in metres).
        Uses 8-connected Dijkstra on free space so scoring reflects traversable
        path distance rather than straight-line Euclidean distance.
        """
        h, w = map_array.shape
        dist = np.full((h, w), np.inf, dtype=np.float32)
        if sx < 0 or sx >= w or sy < 0 or sy >= h:
            return dist
        if map_array[sy, sx] != 0:
            return dist

        moves = (
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)),
        )

        pq = []
        dist[sy, sx] = 0.0
        heapq.heappush(pq, (0.0, sx, sy))

        while pq:
            cd, cx, cy = heapq.heappop(pq)
            if cd > float(dist[cy, cx]):
                continue

            for dx, dy, step in moves:
                nx = cx + dx
                ny = cy + dy
                if nx < 0 or nx >= w or ny < 0 or ny >= h:
                    continue
                if map_array[ny, nx] != 0:
                    continue
                nd = cd + step * resolution
                if nd < float(dist[ny, nx]):
                    dist[ny, nx] = nd
                    heapq.heappush(pq, (nd, nx, ny))

        return dist

    def _is_frontier_cell(self, map_array, x, y, w, h, nbrs):
        """A frontier cell is unknown (-1) with at least one free (0) neighbour."""
        if map_array[y, x] != -1:
            return False
        for dx, dy in nbrs:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h:
                if map_array[ny, nx] == 0:
                    return True
        return False

    def _build_frontier(self, map_array, state, sx, sy, w, h,
                        robot_xy, ox, oy, resolution, nbrs,
                        FRONTIER_OPEN, FRONTIER_CLOSED):
        """
        BFS to collect all connected frontier cells from (sx, sy).
        Computes centroid, min_distance, size etc.
        """
        frontier = Frontier()
        fqueue = deque()
        fqueue.append((sx, sy))
        state[sy, sx] = FRONTIER_OPEN

        sum_x = 0.0
        sum_y = 0.0

        while fqueue:
            cx, cy = fqueue.popleft()
            if state[cy, cx] == FRONTIER_CLOSED:
                continue
            state[cy, cx] = FRONTIER_CLOSED

            # Convert to world coords
            wx = cx * resolution + ox
            wy = cy * resolution + oy

            # Track min_distance — closest cell to robot
            d = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
            if d < frontier.min_distance:
                frontier.min_distance = d

            sum_x += wx
            sum_y += wy
            frontier.points.append((wx, wy))
            frontier.cells.append((cx, cy))

            if frontier.size == 0:
                frontier.initial = (wx, wy)

            frontier.size += 1

            # Expand to neighbouring frontier cells (8-connected)
            for dx, dy in nbrs:
                nx, ny = cx + dx, cy + dy
                if nx < 0 or nx >= w or ny < 0 or ny >= h:
                    continue
                if state[ny, nx] not in (FRONTIER_OPEN, FRONTIER_CLOSED):
                    if self._is_frontier_cell(map_array, nx, ny, w, h, nbrs):
                        state[ny, nx] = FRONTIER_OPEN
                        fqueue.append((nx, ny))

        if frontier.size > 0:
            frontier.centroid = (sum_x / frontier.size, sum_y / frontier.size)
            mid_idx = frontier.size // 2
            frontier.middle = frontier.points[mid_idx]

        return frontier

    def _nearest_free_cell(self, map_array, sx, sy, w, h, max_radius=50):
        """Find nearest free cell to (sx, sy) via expanding square search."""
        for r in range(1, max_radius):
            for dx in range(-r, r + 1):
                for dy in (-r, r):
                    nx, ny = sx + dx, sy + dy
                    if 0 <= nx < w and 0 <= ny < h and map_array[ny, nx] == 0:
                        return (nx, ny)
            for dy in range(-r + 1, r):
                for dx in (-r, r):
                    nx, ny = sx + dx, sy + dy
                    if 0 <= nx < w and 0 <= ny < h and map_array[ny, nx] == 0:
                        return (nx, ny)
        return None

    # ================================================================
    # GVD distance transform
    # ================================================================

    def _get_distance_transform(self, map_array):
        """Return cached obstacle distance transform (cells)."""
        dist_map, _, _, _, _, _ = self._get_gvd_data(map_array)
        return dist_map

    def _get_gvd_mask(self, map_array):
        """Return cached generalized Voronoi skeleton mask."""
        _, _, gvd_mask, _, _, _ = self._get_gvd_data(map_array)
        return gvd_mask

    def _get_gvd_data(self, map_array):
        """
        Build and cache:
          - obstacle distance transform
          - nearest-obstacle labels
          - GVD skeleton mask
          - nearest-skeleton projection fields
        """
        token = id(self.map_data) if self.map_data is not None else None
        cache_valid = (
            self._cached_dist_map is not None
            and self._cached_voronoi_labels is not None
            and self._cached_gvd_mask is not None
            and self._cached_gvd_project_dist is not None
            and self._cached_gvd_project_x is not None
            and self._cached_gvd_project_y is not None
            and self._cached_map_token == token
            and self._cached_dist_map.shape == map_array.shape
        )
        if cache_valid:
            return (
                self._cached_dist_map,
                self._cached_voronoi_labels,
                self._cached_gvd_mask,
                self._cached_gvd_project_dist,
                self._cached_gvd_project_x,
                self._cached_gvd_project_y,
            )

        dist_map, labels = self._compute_distance_and_labels(map_array)
        gvd_mask = self._compute_gvd_mask(map_array, dist_map, labels)
        project_dist, project_x, project_y = self._compute_gvd_projection(
            map_array, gvd_mask)

        self._cached_dist_map = dist_map
        self._cached_voronoi_labels = labels
        self._cached_gvd_mask = gvd_mask
        self._cached_gvd_project_dist = project_dist
        self._cached_gvd_project_x = project_x
        self._cached_gvd_project_y = project_y
        self._cached_map_token = token
        return dist_map, labels, gvd_mask, project_dist, project_x, project_y

    def _compute_distance_and_labels(self, map_array):
        """
        Multi-source BFS from obstacle cells.
        Returns:
          - dist[y,x]: Manhattan distance in cells to nearest obstacle
          - label[y,x]: ID of nearest obstacle seed (for Voronoi regions)
        """
        h, w = map_array.shape
        obstacle = (map_array > 50) | (map_array == -1)

        dist = np.full((h, w), -1, dtype=np.int32)
        labels = np.full((h, w), -1, dtype=np.int32)
        queue = deque()

        obs_y, obs_x = np.where(obstacle)
        for i in range(len(obs_y)):
            y = int(obs_y[i])
            x = int(obs_x[i])
            dist[y, x] = 0
            labels[y, x] = y * w + x
            queue.append((x, y))

        if not queue:
            dist.fill(0)
            return dist, labels

        while queue:
            cx, cy = queue.popleft()
            nd = dist[cy, cx] + 1
            cl = labels[cy, cx]
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx = cx + dx
                ny = cy + dy
                if nx < 0 or nx >= w or ny < 0 or ny >= h:
                    continue
                if dist[ny, nx] == -1:
                    dist[ny, nx] = nd
                    labels[ny, nx] = cl
                    queue.append((nx, ny))
                elif dist[ny, nx] == nd and labels[ny, nx] != cl:
                    labels[ny, nx] = min(labels[ny, nx], cl)

        dist[dist < 0] = 0
        return dist, labels

    def _compute_gvd_mask(self, map_array, dist_map, labels):
        """
        Approximate GVD skeleton:
        free cells that border at least two different Voronoi regions and
        satisfy a minimum clearance threshold.
        """
        h, w = map_array.shape
        free = (map_array == 0)
        ridge = np.zeros((h, w), dtype=np.bool_)

        # 8-neighbour region-label discontinuities mark Voronoi ridges.
        for dy, dx in ((1, 0), (0, 1), (1, 1), (1, -1)):
            if dy >= 0:
                y1 = slice(dy, h)
                y2 = slice(0, h - dy)
            else:
                y1 = slice(0, h + dy)
                y2 = slice(-dy, h)
            if dx >= 0:
                x1 = slice(dx, w)
                x2 = slice(0, w - dx)
            else:
                x1 = slice(0, w + dx)
                x2 = slice(-dx, w)

            f1 = free[y1, x1]
            f2 = free[y2, x2]
            l1 = labels[y1, x1]
            l2 = labels[y2, x2]
            cond = f1 & f2 & (l1 >= 0) & (l2 >= 0) & (l1 != l2)
            ridge[y1, x1] |= cond
            ridge[y2, x2] |= cond

        clearance_m = dist_map.astype(np.float32) * float(self.map_data.info.resolution)
        clear_enough = clearance_m >= float(self.gvd_min_clearance)
        return free & ridge & clear_enough

    def _compute_gvd_projection(self, map_array, gvd_mask):
        """
        For each free cell, compute nearest GVD skeleton cell by 4-connected BFS.
        Returns:
          - proj_dist (cells)
          - proj_x, proj_y (nearest GVD cell indices)
        """
        h, w = map_array.shape
        free = (map_array == 0)

        proj_dist = np.full((h, w), -1, dtype=np.int32)
        proj_x = np.full((h, w), -1, dtype=np.int32)
        proj_y = np.full((h, w), -1, dtype=np.int32)
        queue = deque()

        ys, xs = np.where(gvd_mask)
        for i in range(len(ys)):
            y = int(ys[i])
            x = int(xs[i])
            proj_dist[y, x] = 0
            proj_x[y, x] = x
            proj_y[y, x] = y
            queue.append((x, y))

        if not queue:
            return proj_dist, proj_x, proj_y

        while queue:
            cx, cy = queue.popleft()
            nd = proj_dist[cy, cx] + 1
            nx0 = proj_x[cy, cx]
            ny0 = proj_y[cy, cx]
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx = cx + dx
                ny = cy + dy
                if nx < 0 or nx >= w or ny < 0 or ny >= h:
                    continue
                if not free[ny, nx]:
                    continue
                if proj_dist[ny, nx] != -1:
                    continue
                proj_dist[ny, nx] = nd
                proj_x[ny, nx] = nx0
                proj_y[ny, nx] = ny0
                queue.append((nx, ny))

        return proj_dist, proj_x, proj_y

    def _select_frontier_goal(self, frontier, map_array, info, robot_xy, nbrs, path_dist_map):
        """
        Pick a navigable goal cell adjacent to a frontier cluster.
        The goal must be free in /map and not obstacle/high-cost in costmaps.
        """
        resolution = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        w = info.width
        h = info.height

        candidates = set()
        for fx, fy in frontier.cells:
            for dx, dy in nbrs:
                nx, ny = fx + dx, fy + dy
                if nx < 0 or nx >= w or ny < 0 or ny >= h:
                    continue
                if map_array[ny, nx] == 0:
                    candidates.add((nx, ny))

        best_goal = None
        best_dist = float('inf')

        for gx, gy in candidates:
            wx = gx * resolution + ox
            wy = gy * resolution + oy
            if not self._is_world_point_safe(wx, wy):
                continue

            d = float(path_dist_map[gy, gx])
            if not np.isfinite(d):
                continue
            if d < best_dist:
                best_dist = d
                best_goal = (wx, wy)

        return best_goal, best_dist

    def _project_to_gvd_cell(self, mx, my, map_array, resolution):
        """
        Project a free cell to nearest GVD skeleton cell within projection radius.
        gvd_projection_radius <= 0 disables the radius cap (global nearest edge).
        Returns projected (mx, my) or None.
        """
        _, _, gvd_mask, proj_dist, proj_x, proj_y = self._get_gvd_data(map_array)
        if gvd_mask[my, mx]:
            return (mx, my)

        max_radius_m = float(self.gvd_projection_radius)
        max_cells = None
        if max_radius_m > 0.0:
            max_cells = max(0, int(max_radius_m / max(resolution, 1e-6)))
        d = int(proj_dist[my, mx])
        if d < 0:
            return None
        if max_cells is not None and d > max_cells:
            return None

        gx = int(proj_x[my, mx])
        gy = int(proj_y[my, mx])
        if gx < 0 or gy < 0:
            return None
        if not gvd_mask[gy, gx]:
            return None
        return (gx, gy)

    def _world_to_grid_cell(self, wx, wy, info):
        """Convert world coordinates to map cell index (mx, my)."""
        resolution = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        mx = int(math.floor((wx - ox) / resolution))
        my = int(math.floor((wy - oy) / resolution))
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None, None
        return mx, my

    def _frontier_orientation_bonus(self, robot_xy, robot_yaw, goal_x, goal_y):
        """
        Return heading alignment bonus in [0, 1].
        1 means goal straight ahead, 0 means outside forward cone.
        """
        dx = goal_x - robot_xy[0]
        dy = goal_y - robot_xy[1]
        if dx == 0.0 and dy == 0.0:
            return 0.0

        goal_heading = math.atan2(dy, dx)
        heading_error = abs(self._normalize_angle(goal_heading - robot_yaw))
        cone = math.radians(max(1.0, float(self.orientation_window_deg)))
        if heading_error > cone:
            return 0.0
        return 1.0 - (heading_error / cone)

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def _is_world_point_safe(self, wx, wy):
        """Reject points on obstacle/inscribed/unknown cells in costmaps."""
        global_cost = self._costmap_cost_at_world(self.global_costmap_data, wx, wy)
        if global_cost is not None and self._is_obstacle_cost(global_cost):
            return False

        local_cost = self._costmap_cost_at_world(self.local_costmap_data, wx, wy)
        if local_cost is not None and self._is_obstacle_cost(local_cost):
            return False

        return True

    def _is_obstacle_cost(self, cost):
        if cost == 255 and self.treat_unknown_as_obstacle:
            return True
        return cost >= self.costmap_obstacle_threshold

    def _costmap_cost_at_world(self, costmap, wx, wy):
        """
        Return costmap value at world location.
        For rolling local costmap, out-of-bounds means "not applicable".
        """
        if costmap is None:
            return None

        meta = costmap.metadata
        resolution = meta.resolution
        ox = meta.origin.position.x
        oy = meta.origin.position.y
        w = int(meta.size_x)
        h = int(meta.size_y)

        mx = int(math.floor((wx - ox) / resolution))
        my = int(math.floor((wy - oy) / resolution))
        if mx < 0 or my < 0 or mx >= w or my >= h:
            return None

        idx = my * w + mx
        if idx < 0 or idx >= len(costmap.data):
            return None

        return int(costmap.data[idx])

    # ================================================================
    # Navigation
    # ================================================================

    def _navigate_to(self, x: float, y: float, goal_kind='frontier'):
        """Send a NavigateToPose goal to Nav2."""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            return

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = self.global_frame
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        self._goal_seq += 1
        seq = self._goal_seq
        self.get_logger().info(
            f'Sending {goal_kind} goal: ({x:.2f}, {y:.2f})')

        send_future = self.nav_client.send_goal_async(nav_goal)
        send_future.add_done_callback(
            lambda f, s=seq: self._goal_response_cb(f, s))

        self.current_goal = (x, y)
        self.current_goal_kind = goal_kind
        self.prev_goal = (x, y)
        self.prev_goal_time = self.get_clock().now()
        self.navigating = True
        self.last_progress_time = self.get_clock().now()
        self.last_robot_pos = self._get_robot_position()

    def _goal_response_cb(self, future, seq):
        # Ignore stale callback from a superseded goal
        if seq != self._goal_seq:
            return

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected by Nav2')
            self._blacklist_current_goal()
            self.navigating = False
            self.current_goal_kind = None
            self.prev_goal = None
            self.prev_goal_time = None
            # Immediate replan
            self._make_plan()
            return

        self.get_logger().info('Goal accepted')
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, s=seq: self._navigation_result_cb(f, s))

    def _navigation_result_cb(self, future, seq):
        """Handle navigation result — then immediately replan."""
        # Ignore stale callback from a superseded goal
        if seq != self._goal_seq:
            self.get_logger().debug(
                f'Ignoring stale result callback (seq {seq}, current {self._goal_seq})')
            return

        status = None
        try:
            status = future.result().status
            # 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
            if status == 4:
                self.get_logger().info('Navigation succeeded')
            elif status == 6:
                self.get_logger().warning('Navigation aborted — blacklisting goal')
                self._blacklist_current_goal()
            elif status == 5:
                self.get_logger().info('Navigation cancelled')
            else:
                self.get_logger().warning(f'Navigation ended with status {status}')
        except Exception as e:
            self.get_logger().error(f'Navigation result error: {e}')
            self._blacklist_current_goal()

        goal_kind = self.current_goal_kind
        self.navigating = False
        self.goal_handle = None
        self.current_goal_kind = None
        # Keep prev_goal on successful frontier goals to avoid immediate
        # re-sending the exact same frontier before map/frontiers update.
        if goal_kind != 'frontier' or status != 4:
            self.prev_goal = None
            self.prev_goal_time = None

        if goal_kind == 'recovery' and status == 4:
            self.get_logger().info(
                'Recovery waypoint reached — replanning frontier goal')

        # Immediate replan (like reachedGoal -> makePlan in m-explore)
        if self.exploring:
            self._make_plan()

    def _return_to_initial_pose(self):
        """Navigate back to the pose where the robot started."""
        if self.initial_pose is None or self.navigating:
            return
        self.get_logger().info(
            f'Exploration complete — returning to initial pose '
            f'({self.initial_pose[0]:.2f}, {self.initial_pose[1]:.2f})')
        # Disable further exploration so we don't replan after arrival
        self.exploring = False
        self._navigate_to(
            self.initial_pose[0], self.initial_pose[1], goal_kind='return')

    # ================================================================
    # Progress checking
    # ================================================================

    def _check_progress(self, robot_xy):
        """Cancel goal if robot hasn't moved for progress_timeout seconds."""
        if not self.navigating or self.last_robot_pos is None:
            return

        dist_moved = math.hypot(
            robot_xy[0] - self.last_robot_pos[0],
            robot_xy[1] - self.last_robot_pos[1])

        if dist_moved > 0.3:
            self.last_progress_time = self.get_clock().now()
            self.last_robot_pos = robot_xy
            return

        elapsed = (self.get_clock().now() - self.last_progress_time).nanoseconds / 1e9
        if elapsed > self.progress_timeout:
            self.get_logger().warning(
                f'No progress for {elapsed:.0f}s — cancelling goal')
            self._cancel_current_goal()
            self._blacklist_current_goal()
            self.navigating = False
            self.current_goal_kind = None

            if self.stuck_escape_enabled:
                escape_goal = self._find_spacious_escape_goal(robot_xy)
                if escape_goal is not None:
                    gx, gy, dist_m, clearance = escape_goal
                    self.get_logger().info(
                        f'Stuck near wall — recovery to spacious point '
                        f'({gx:.2f}, {gy:.2f}), dist={dist_m:.2f}m, '
                        f'clearance={clearance:.2f}m')
                    self._navigate_to(gx, gy, goal_kind='recovery')

    def _cancel_current_goal(self):
        if self.goal_handle is not None:
            self.get_logger().info('Cancelling current goal...')
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    # ================================================================
    # Blacklisting
    # ================================================================

    def _blacklist_current_goal(self):
        if self.current_goal_kind != 'frontier':
            return
        if self.current_goal is not None:
            self.get_logger().info(
                f'Blacklisting ({self.current_goal[0]:.2f}, '
                f'{self.current_goal[1]:.2f})')
            self.blacklisted.append(
                (self.current_goal[0], self.current_goal[1],
                 self.get_clock().now()))

    def _is_blacklisted(self, x, y):
        for bx, by, _ in self.blacklisted:
            if math.hypot(x - bx, y - by) < self.blacklist_radius:
                return True
        return False

    def _find_spacious_escape_goal(self, robot_xy):
        """
        If robot is stuck and close to walls, pick a nearby open-space waypoint.
        Returns (wx, wy, distance_from_robot_m, clearance_m) or None.
        """
        if self.map_data is None:
            return None

        info = self.map_data.info
        h = info.height
        w = info.width
        resolution = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        map_array = np.array(self.map_data.data, dtype=np.int8).reshape((h, w))
        dist_map = self._get_distance_transform(map_array)

        mx = int((robot_xy[0] - ox) / resolution)
        my = int((robot_xy[1] - oy) / resolution)
        if mx < 0 or mx >= w or my < 0 or my >= h:
            return None

        if map_array[my, mx] != 0:
            found = self._nearest_free_cell(map_array, mx, my, w, h)
            if found is None:
                return None
            mx, my = found

        robot_clearance = float(dist_map[my, mx]) * resolution
        if robot_clearance > self.near_wall_clearance:
            return None

        max_radius_cells = max(
            1, int(self.stuck_escape_search_radius / max(resolution, 1e-6)))
        min_goal_dist = max(0.0, self.stuck_escape_min_goal_distance)
        min_clear = max(0.0, self.stuck_escape_min_clearance)
        dist_weight = max(0.0, self.stuck_escape_distance_weight)

        visited = np.zeros((h, w), dtype=np.uint8)
        queue = deque()
        queue.append((mx, my))
        visited[my, mx] = 1

        best_strict = None   # (score, wx, wy, d_m, clear_m)
        best_relaxed = None  # fallback if strict min_clear is unavailable

        while queue:
            cx, cy = queue.popleft()
            dx = cx - mx
            dy = cy - my
            cell_dist = math.hypot(dx, dy)
            if cell_dist > max_radius_cells:
                continue

            d_m = cell_dist * resolution
            if d_m >= min_goal_dist:
                clear_m = float(dist_map[cy, cx]) * resolution
                wx = cx * resolution + ox
                wy = cy * resolution + oy
                if self._is_world_point_safe(wx, wy):
                    score = clear_m - dist_weight * d_m
                    cand = (score, wx, wy, d_m, clear_m)
                    if best_relaxed is None or cand[0] > best_relaxed[0]:
                        best_relaxed = cand
                    if clear_m >= min_clear:
                        if best_strict is None or cand[0] > best_strict[0]:
                            best_strict = cand

            for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                if nx < 0 or nx >= w or ny < 0 or ny >= h:
                    continue
                if visited[ny, nx]:
                    continue
                if map_array[ny, nx] != 0:
                    continue
                visited[ny, nx] = 1
                queue.append((nx, ny))

        best = best_strict if best_strict is not None else best_relaxed
        if best is None:
            self.get_logger().warning(
                'Stuck near wall but no suitable spacious recovery point found',
                throttle_duration_sec=5.0)
            return None

        return best[1], best[2], best[3], best[4]

    # ================================================================
    # TF helper
    # ================================================================

    def _get_robot_pose(self):
        """Get robot (x, y, yaw) in map frame via TF."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_tolerance))
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (t.transform.translation.x, t.transform.translation.y, yaw)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warning(
                f'TF lookup failed: {e}', throttle_duration_sec=5.0)
            return None

    def _get_robot_position(self):
        """Get robot (x, y) in map frame via TF."""
        pose = self._get_robot_pose()
        if pose is None:
            return None
        return (pose[0], pose[1])

    # ================================================================
    # Visualisation
    # ================================================================

    def _publish_markers(self, frontiers, chosen, map_array, info):
        """Publish frontier centroids as RViz markers."""
        ma = MarkerArray()

        # Delete old markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        ma.markers.append(delete_marker)

        for i, f in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontiers'
            m.id = i + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            px, py = f.goal if f.goal is not None else f.centroid
            m.pose.position.x = px
            m.pose.position.y = py
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0

            # Scale by cluster size
            scale = max(0.15, min(0.6, f.size * 0.005))
            m.scale.x = scale
            m.scale.y = scale
            m.scale.z = scale

            is_chosen = (f is chosen)
            if is_chosen:
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 0.0
                m.color.a = 1.0
            else:
                m.color.r = 0.2
                m.color.g = 0.4
                m.color.b = 1.0
                m.color.a = 0.8

            m.lifetime.sec = 10
            ma.markers.append(m)

        if self.gvd_visualize:
            gvd_mask = self._get_gvd_mask(map_array)
            ys, xs = np.where(gvd_mask)
            if len(xs) > 0:
                gm = Marker()
                gm.header.frame_id = self.global_frame
                gm.header.stamp = self.get_clock().now().to_msg()
                gm.ns = 'gvd_skeleton'
                gm.id = 100000
                gm.type = Marker.POINTS
                gm.action = Marker.ADD
                gm.scale.x = float(max(0.005, self.gvd_marker_scale))
                gm.scale.y = float(max(0.005, self.gvd_marker_scale))
                gm.color.r = 1.0
                gm.color.g = 0.85
                gm.color.b = 0.15
                gm.color.a = 0.9
                gm.pose.orientation.w = 1.0
                gm.lifetime.sec = 10

                stride = max(1, int(self.gvd_marker_stride))
                ox = info.origin.position.x
                oy = info.origin.position.y
                resolution = info.resolution
                for i in range(0, len(xs), stride):
                    px = float(xs[i]) * resolution + ox
                    py = float(ys[i]) * resolution + oy
                    p = Point()
                    p.x = px
                    p.y = py
                    p.z = 0.03
                    gm.points.append(p)
                ma.markers.append(gm)

        self.marker_pub.publish(ma)


# ====================================================================
# Entry point
# ====================================================================

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorerNode()
    try:
        node.get_logger().info('Starting frontier exploration...')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Exploration stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
