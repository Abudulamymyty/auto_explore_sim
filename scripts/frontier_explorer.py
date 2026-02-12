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
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros


# ---------------------------------------------------------------------------
# Frontier data container
# ---------------------------------------------------------------------------
class Frontier:
    """One frontier cluster discovered by BFS."""
    __slots__ = ('size', 'min_distance', 'cost', 'centroid', 'middle',
                 'initial', 'points')

    def __init__(self):
        self.size = 0               # number of cells
        self.min_distance = float('inf')
        self.cost = 0.0
        self.centroid = (0.0, 0.0)  # average of all points (world coords)
        self.middle = (0.0, 0.0)    # point at size//2 (world coords)
        self.initial = (0.0, 0.0)   # first point found (world coords)
        self.points = []            # all points as (wx, wy)


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
        # arrival_radius removed — we now blacklist frontier on every success
        self.declare_parameter('progress_timeout', 30.0)
        self.declare_parameter('visualize', True)
        self.declare_parameter('clearance_scale', 0.3)       # GVD clearance tiebreaker
        self.declare_parameter('return_to_init', False)
        self.declare_parameter('gvd_min_clearance', 3)     # min obstacle dist (cells)
        self.declare_parameter('gvd_snap_radius', 2.0)     # max snap search (metres)
        self.declare_parameter('visualize_gvd', True)

        # ── Read parameters ─────────────────────────────────────────────
        self.planner_freq = self.get_parameter('planner_frequency').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.tf_tolerance = self.get_parameter('transform_tolerance').value
        self.blacklist_radius = self.get_parameter('blacklist_radius').value
        self.blacklist_timeout = self.get_parameter('blacklist_timeout').value
        # arrival_radius removed
        self.progress_timeout = self.get_parameter('progress_timeout').value
        self.visualize = self.get_parameter('visualize').value
        self.clearance_scale = self.get_parameter('clearance_scale').value
        self.return_to_init = self.get_parameter('return_to_init').value
        self.gvd_min_clearance = self.get_parameter('gvd_min_clearance').value
        self.gvd_snap_radius = self.get_parameter('gvd_snap_radius').value
        self.visualize_gvd = self.get_parameter('visualize_gvd').value

        # ── TF listener ─────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscribers ─────────────────────────────────────────────────
        self.map_data = None
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, 10)

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
        if self.visualize_gvd:
            self.gvd_marker_pub = self.create_publisher(
                MarkerArray, 'explore/gvd', 10)

        # ── State ───────────────────────────────────────────────────────
        self.navigating = False
        self.current_goal = None            # (x, y) snapped goal sent to Nav2
        self.current_frontier = None        # (x, y) original frontier centroid
        self.prev_goal = None               # last goal sent to Nav2
        self.goal_handle = None
        self.blacklisted = []               # list of (x, y, stamp)
        self._goal_seq = 0                  # incremented each _navigate_to call
        self.last_progress_time = None
        self.last_robot_pos = None
        self.initial_pose = None            # stored for return_to_init
        self._cached_dist_map = None        # GVD distance transform cache
        self._cached_map_stamp = None
        self._cached_label_map = None       # obstacle label map for GVD
        self._cached_gvd_mask = None        # boolean GVD point mask

        # ── Timer ───────────────────────────────────────────────────────
        period = 1.0 / max(self.planner_freq, 0.01)
        self.timer = self.create_timer(period, self._explore_tick)

        self.get_logger().info(
            f'Frontier Explorer ready  (freq={self.planner_freq} Hz, '
            f'min_frontier={self.min_frontier_size} cells, '
            f'clearance_scale={self.clearance_scale}, '
            f'gvd_snap_radius={self.gvd_snap_radius}m, '
            f'return_to_init={self.return_to_init})')

    # ================================================================
    # Callbacks
    # ================================================================

    def _map_callback(self, msg: OccupancyGrid):
        self.map_data = msg

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

        # 1. Get robot position in map frame
        robot_xy = self._get_robot_position()
        if robot_xy is None:
            return

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

        frontiers = self._search_from(robot_xy, map_array, info)

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
        valid = [f for f in frontiers if not self._is_blacklisted(*f.centroid)]

        if not valid:
            self.get_logger().warn(
                'All frontiers blacklisted — clearing blacklist')
            self.blacklisted.clear()
            valid = frontiers

        # 5. Pick best (already sorted by cost, pick first non-blacklisted)
        best = valid[0]

        self.get_logger().info(
            f'Best frontier: ({best.centroid[0]:.2f}, {best.centroid[1]:.2f})  '
            f'min_dist={best.min_distance:.2f}m  size={best.size}  '
            f'cost={best.cost:.1f}')

        # 6. Visualise frontiers and GVD skeleton
        if self.visualize:
            self._publish_markers(valid, best)
        if self.visualize_gvd:
            self._publish_gvd_markers(map_array, info)

        # 7. Snap frontier goal to nearest GVD point
        gx, gy = self._snap_to_gvd(best.centroid, robot_xy, map_array, info)

        # 8. Skip if goal is too close to robot (would cause instant-success loop)
        dist_to_goal = math.hypot(gx - robot_xy[0], gy - robot_xy[1])
        if dist_to_goal < 0.3:
            self.get_logger().warning(
                f'Goal ({gx:.2f}, {gy:.2f}) only {dist_to_goal:.2f}m away '
                f'— blacklisting frontier')
            self._blacklist_point(best.centroid[0], best.centroid[1])
            return

        # 9. Same-goal detection — skip if goal hasn't changed
        if self.prev_goal is not None:
            dx = gx - self.prev_goal[0]
            dy = gy - self.prev_goal[1]
            if math.sqrt(dx * dx + dy * dy) < 0.01:
                self.get_logger().debug('Same goal as before — skipping')
                return

        # 10. Navigate
        self.current_frontier = best.centroid
        self._navigate_to(gx, gy)

    # ================================================================
    # BFS frontier search from robot position
    # ================================================================

    def _search_from(self, robot_xy, map_array, info):
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

        # Cost = min_distance - clearance_scale * clearance_at_centroid
        # Primary: nearest first.  Secondary: prefer open-corridor frontiers.
        for f in frontiers:
            cx = int((f.centroid[0] - ox) / resolution)
            cy = int((f.centroid[1] - oy) / resolution)
            cx = max(0, min(w - 1, cx))
            cy = max(0, min(h - 1, cy))
            clearance = float(dist_map[cy, cx]) * resolution  # metres
            f.cost = f.min_distance - self.clearance_scale * clearance

        frontiers.sort(key=lambda f: f.cost)
        return frontiers

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
    # GVD distance transform & Voronoi skeleton
    # ================================================================

    def _get_distance_transform(self, map_array):
        """Return cached distance transform, recomputing only when map changes."""
        stamp = self.map_data.header.stamp if self.map_data else None
        if (self._cached_dist_map is not None
                and self._cached_map_stamp == stamp
                and self._cached_dist_map.shape == map_array.shape):
            return self._cached_dist_map

        self._cached_dist_map, self._cached_label_map = \
            self._compute_distance_and_labels(map_array)
        self._cached_gvd_mask = self._extract_gvd_mask(
            self._cached_dist_map, self._cached_label_map, map_array)
        self._cached_map_stamp = stamp
        return self._cached_dist_map

    def _get_gvd_mask(self, map_array):
        """Return cached GVD boolean mask, recomputing if needed."""
        # Ensure distance transform (and GVD) is up to date
        self._get_distance_transform(map_array)
        return self._cached_gvd_mask

    def _compute_distance_and_labels(self, map_array):
        """
        Two-phase computation:
        1. Connected-component labeling of real obstacle regions (8-connected)
           Unknown cells (-1) are excluded — they are the exploration frontier,
           not permanent obstacles, so they must not generate GVD lines.
        2. BFS distance transform propagating region labels into free space

        A GVD cell is then one whose neighbors were reached from different
        obstacle regions — i.e. equidistant from 2+ distinct walls.

        Returns (dist_map, label_map) where label_map holds the region ID
        of the nearest obstacle region for every cell.
        """
        h, w = map_array.shape
        obstacle = map_array > 50  # only real obstacles, NOT unknown (-1)

        # --- Phase 1: connected-component labeling of obstacle regions ---
        region_id = np.full((h, w), -1, dtype=np.int32)
        current_label = 0

        for sy in range(h):
            for sx in range(w):
                if obstacle[sy, sx] and region_id[sy, sx] == -1:
                    # BFS flood-fill this obstacle region (8-connected)
                    cc_queue = deque()
                    cc_queue.append((sx, sy))
                    region_id[sy, sx] = current_label
                    while cc_queue:
                        cx, cy = cc_queue.popleft()
                        for ddx in (-1, 0, 1):
                            for ddy in (-1, 0, 1):
                                if ddx == 0 and ddy == 0:
                                    continue
                                nx, ny = cx + ddx, cy + ddy
                                if (0 <= nx < w and 0 <= ny < h
                                        and obstacle[ny, nx]
                                        and region_id[ny, nx] == -1):
                                    region_id[ny, nx] = current_label
                                    cc_queue.append((nx, ny))
                    current_label += 1

        self.get_logger().debug(
            f'GVD: found {current_label} obstacle regions',
            throttle_duration_sec=10.0)

        # --- Phase 2: BFS distance transform with region labels ---
        dist = np.full((h, w), -1, dtype=np.int32)
        label = np.full((h, w), -1, dtype=np.int32)
        queue = deque()

        # Seed all obstacle cells with their region label
        obs_y, obs_x = np.where(obstacle)
        for i in range(len(obs_y)):
            y, x = int(obs_y[i]), int(obs_x[i])
            dist[y, x] = 0
            label[y, x] = region_id[y, x]
            queue.append((x, y))

        # 4-connected BFS — propagate distance and region label
        while queue:
            cx, cy = queue.popleft()
            nd = dist[cy, cx] + 1
            lbl = label[cy, cx]
            for ddx, ddy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + ddx, cy + ddy
                if 0 <= nx < w and 0 <= ny < h and dist[ny, nx] == -1:
                    dist[ny, nx] = nd
                    label[ny, nx] = lbl
                    queue.append((nx, ny))

        # Cells still -1 (unreachable) get 0
        dist[dist < 0] = 0
        return dist, label

    def _extract_gvd_mask(self, dist_map, label_map, map_array):
        """
        Extract GVD (Voronoi skeleton) points.
        A cell is a GVD point if:
          1. It is a free cell (occupancy == 0), NOT unknown or obstacle
          2. Its distance to the nearest obstacle >= gvd_min_clearance
          3. At least one 4-connected neighbor has a different obstacle label
        """
        h, w = dist_map.shape
        min_c = self.gvd_min_clearance
        gvd = np.zeros((h, w), dtype=bool)

        # Vectorised: check if any neighbor has a different label
        for ddx, ddy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            if ddx == 1:
                src = label_map[:, :-1]
                nbr = label_map[:, 1:]
                dst = gvd[:, :-1]
            elif ddx == -1:
                src = label_map[:, 1:]
                nbr = label_map[:, :-1]
                dst = gvd[:, 1:]
            elif ddy == 1:
                src = label_map[:-1, :]
                nbr = label_map[1:, :]
                dst = gvd[:-1, :]
            else:  # ddy == -1
                src = label_map[1:, :]
                nbr = label_map[:-1, :]
                dst = gvd[1:, :]
            diff = (src != nbr) & (src >= 0) & (nbr >= 0)
            dst |= diff

        # Only keep GVD points in FREE cells (== 0), not unknown (-1)
        gvd &= (map_array == 0)

        # Apply clearance threshold
        gvd &= (dist_map >= min_c)

        return gvd

    def _snap_to_gvd(self, point, robot_xy, map_array, info):
        """
        Snap a frontier centroid to the nearest GVD cell that lies
        *between* the robot and the frontier (not behind the robot).
        Returns (x, y) in world coordinates.
        Falls back to the original point if no suitable GVD cell is found.
        """
        gvd_mask = self._get_gvd_mask(map_array)
        if gvd_mask is None:
            return point

        resolution = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        w = info.width
        h = info.height

        # Convert point to map cell
        px = int((point[0] - ox) / resolution)
        py = int((point[1] - oy) / resolution)
        px = max(0, min(w - 1, px))
        py = max(0, min(h - 1, py))

        # If already on GVD, return as-is
        if gvd_mask[py, px]:
            return point

        # Distance from robot to frontier (for filtering candidates)
        robot_to_frontier = math.hypot(
            point[0] - robot_xy[0], point[1] - robot_xy[1])

        # BFS outward from frontier centroid to find nearest GVD cell
        max_cells = int(self.gvd_snap_radius / resolution)
        visited = set()
        queue = deque()
        queue.append((px, py))
        visited.add((px, py))

        best_gvd = None
        best_dist = float('inf')

        while queue:
            cx, cy = queue.popleft()
            if gvd_mask[cy, cx]:
                wx = cx * resolution + ox
                wy = cy * resolution + oy
                # Reject candidates that are farther from the frontier
                # than the robot is (i.e. behind the robot)
                cand_to_frontier = math.hypot(wx - point[0], wy - point[1])
                if cand_to_frontier < best_dist:
                    # Ensure candidate is not behind the robot:
                    # candidate-to-frontier must be less than robot-to-frontier
                    cand_to_robot = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
                    if cand_to_robot < robot_to_frontier:
                        best_gvd = (wx, wy)
                        best_dist = cand_to_frontier

            for ddx, ddy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + ddx, cy + ddy
                if (0 <= nx < w and 0 <= ny < h
                        and (nx, ny) not in visited
                        and abs(nx - px) <= max_cells
                        and abs(ny - py) <= max_cells):
                    visited.add((nx, ny))
                    queue.append((nx, ny))

        if best_gvd is not None:
            self.get_logger().info(
                f'Frontier snapped to GVD: '
                f'({point[0]:.2f}, {point[1]:.2f}) -> '
                f'({best_gvd[0]:.2f}, {best_gvd[1]:.2f})')
            return best_gvd

        self.get_logger().debug(
            f'No valid GVD point within {self.gvd_snap_radius}m '
            f'— using original centroid')
        return point

    # ================================================================
    # Navigation
    # ================================================================

    def _navigate_to(self, x: float, y: float):
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
        self.get_logger().info(f'Sending goal: ({x:.2f}, {y:.2f})')

        send_future = self.nav_client.send_goal_async(nav_goal)
        send_future.add_done_callback(
            lambda f, s=seq: self._goal_response_cb(f, s))

        self.current_goal = (x, y)
        self.prev_goal = (x, y)
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
            self.prev_goal = None
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

        try:
            status = future.result().status
            # 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
            if status == 4:
                self.get_logger().info('Navigation succeeded')
                # Always blacklist the frontier centroid after success.
                # If the frontier was truly explored, it vanishes from
                # BFS naturally and the blacklist entry is harmless.
                # If Nav2 "succeeded" via goal tolerance without actually
                # reaching it, the blacklist prevents an infinite loop.
                if self.current_frontier is not None:
                    self._blacklist_point(
                        self.current_frontier[0], self.current_frontier[1])
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

        self.navigating = False
        self.goal_handle = None
        self.prev_goal = None   # allow re-selecting same frontier if it persists

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
        self._navigate_to(self.initial_pose[0], self.initial_pose[1])

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

    def _cancel_current_goal(self):
        if self.goal_handle is not None:
            self.get_logger().info('Cancelling current goal...')
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    # ================================================================
    # Blacklisting
    # ================================================================

    def _blacklist_current_goal(self):
        if self.current_goal is not None:
            self._blacklist_point(self.current_goal[0], self.current_goal[1])

    def _blacklist_point(self, x, y):
        self.get_logger().info(f'Blacklisting ({x:.2f}, {y:.2f})')
        self.blacklisted.append((x, y, self.get_clock().now()))

    def _is_blacklisted(self, x, y):
        for bx, by, _ in self.blacklisted:
            if math.hypot(x - bx, y - by) < self.blacklist_radius:
                return True
        return False

    # ================================================================
    # TF helper
    # ================================================================

    def _get_robot_position(self):
        """Get robot (x, y) in map frame via TF."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_tolerance))
            return (t.transform.translation.x, t.transform.translation.y)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warning(
                f'TF lookup failed: {e}', throttle_duration_sec=5.0)
            return None

    # ================================================================
    # Visualisation
    # ================================================================

    def _publish_gvd_markers(self, map_array, info):
        """Publish GVD skeleton as connected LINE_LIST markers."""
        gvd_mask = self._get_gvd_mask(map_array)
        if gvd_mask is None:
            return

        resolution = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        h, w = gvd_mask.shape

        ma = MarkerArray()

        # Delete old GVD markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.ns = 'gvd'
        ma.markers.append(delete_marker)

        # Build LINE_LIST: for each GVD cell, connect to GVD neighbors
        line_marker = Marker()
        line_marker.header.frame_id = self.global_frame
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = 'gvd'
        line_marker.id = 1
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.02  # line width
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 1.0
        line_marker.color.a = 0.8
        line_marker.lifetime.sec = 10
        line_marker.pose.orientation.w = 1.0

        # 4-connected: only check right and down to avoid duplicate edges
        gvd_ys, gvd_xs = np.where(gvd_mask)
        gvd_set = set(zip(gvd_xs.tolist(), gvd_ys.tolist()))

        for x, y in gvd_set:
            wx = x * resolution + ox
            wy = y * resolution + oy
            p1 = Point(x=wx, y=wy, z=0.05)
            # Check right neighbor
            if (x + 1, y) in gvd_set:
                p2 = Point(x=(x + 1) * resolution + ox, y=wy, z=0.05)
                line_marker.points.append(p1)
                line_marker.points.append(p2)
            # Check down neighbor
            if (x, y + 1) in gvd_set:
                p2 = Point(x=wx, y=(y + 1) * resolution + oy, z=0.05)
                line_marker.points.append(p1)
                line_marker.points.append(p2)
            # Check diagonal right-down
            if (x + 1, y + 1) in gvd_set:
                p2 = Point(x=(x + 1) * resolution + ox,
                           y=(y + 1) * resolution + oy, z=0.05)
                line_marker.points.append(p1)
                line_marker.points.append(p2)
            # Check diagonal right-up
            if (x + 1, y - 1) in gvd_set:
                p2 = Point(x=(x + 1) * resolution + ox,
                           y=(y - 1) * resolution + oy, z=0.05)
                line_marker.points.append(p1)
                line_marker.points.append(p2)

        if line_marker.points:
            ma.markers.append(line_marker)

        self.gvd_marker_pub.publish(ma)

    def _publish_markers(self, frontiers, chosen):
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
            m.pose.position.x = f.centroid[0]
            m.pose.position.y = f.centroid[1]
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
