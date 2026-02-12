#!/usr/bin/env python3
"""
Frontier-based autonomous explorer node.

Based on: https://github.com/AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2
Enhanced with:
  - TF-based robot position tracking (instead of hardcoded origin)
  - Frontier clustering to group nearby frontier cells into regions
  - Configurable parameters via ROS 2 parameter server
  - Navigation state tracking to avoid spamming goals
  - Blacklisting of unreachable frontiers
  - Proper use_sim_time support

Subscribes to /map, detects frontiers (free cells adjacent to unknown cells),
clusters them, picks the closest cluster centroid, and sends it as a
NavigateToPose goal via the Nav2 action server.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros


class FrontierExplorerNode(Node):
    """Autonomous frontier exploration using Nav2."""

    def __init__(self):
        super().__init__('frontier_explorer')
        self.get_logger().info('Frontier Explorer Node starting...')

        # ── Declare parameters ──────────────────────────────────────────
        self.declare_parameter('explore_frequency', 0.2)        # Hz
        self.declare_parameter('min_frontier_size', 5)          # min cells in a cluster
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('transform_tolerance', 2.0)      # seconds
        self.declare_parameter('goal_distance_hysteresis', 1.0) # metres
        self.declare_parameter('blacklist_radius', 0.5)         # metres
        self.declare_parameter('blacklist_timeout', 60.0)       # seconds before re-trying
        self.declare_parameter('progress_timeout', 30.0)        # seconds
        self.declare_parameter('visualize', True)
        self.declare_parameter('potential_scale', 3.0)          # distance weight
        self.declare_parameter('gain_scale', 1.0)               # size weight
        self.declare_parameter('min_goal_distance', 0.6)        # metres – skip goals closer than this

        # ── Read parameters ─────────────────────────────────────────────
        self.explore_freq = self.get_parameter('explore_frequency').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.tf_tolerance = self.get_parameter('transform_tolerance').value
        self.goal_hysteresis = self.get_parameter('goal_distance_hysteresis').value
        self.blacklist_radius = self.get_parameter('blacklist_radius').value
        self.blacklist_timeout = self.get_parameter('blacklist_timeout').value
        self.progress_timeout = self.get_parameter('progress_timeout').value
        self.visualize = self.get_parameter('visualize').value
        self.potential_scale = self.get_parameter('potential_scale').value
        self.gain_scale = self.get_parameter('gain_scale').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value

        # ── TF listener ─────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscribers ─────────────────────────────────────────────────
        self.map_data = None
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, 10)

        # ── Nav2 action client ──────────────────────────────────────────
        self._action_cb_group = MutuallyExclusiveCallbackGroup()
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._action_cb_group)

        # ── Visualisation publisher ─────────────────────────────────────
        if self.visualize:
            self.marker_pub = self.create_publisher(
                MarkerArray, 'frontier_markers', 10)

        # ── State ───────────────────────────────────────────────────────
        self.navigating = False
        self.current_goal = None            # (x, y) world coords
        self.goal_handle = None
        self.blacklisted = []               # list of (x, y, stamp)
        self.last_progress_time = None
        self.last_robot_pos = None

        # ── Timer ───────────────────────────────────────────────────────
        period = 1.0 / max(self.explore_freq, 0.01)
        self.timer = self.create_timer(period, self._explore_tick)

        self.get_logger().info(
            f'Frontier Explorer ready  (freq={self.explore_freq} Hz, '
            f'min_cluster={self.min_frontier_size} cells)')

    # ================================================================
    # Callbacks
    # ================================================================

    def _map_callback(self, msg: OccupancyGrid):
        self.map_data = msg

    # ================================================================
    # Main exploration loop
    # ================================================================

    def _explore_tick(self):
        if self.map_data is None:
            self.get_logger().info('Waiting for map...', throttle_duration_sec=5.0)
            return

        # 1. Get robot position in map frame
        robot_xy = self._get_robot_position()
        if robot_xy is None:
            return

        # 2. Check navigation progress
        self._check_progress(robot_xy)

        # If already navigating and making progress, do nothing
        if self.navigating:
            return

        # 3. Build numpy map
        info = self.map_data.info
        map_array = np.array(self.map_data.data, dtype=np.int8).reshape(
            (info.height, info.width))

        # 4. Detect & cluster frontiers
        frontiers = self._find_frontiers(map_array)
        clusters = self._cluster_frontiers(frontiers)
        # Filter small clusters
        clusters = [c for c in clusters if len(c) >= self.min_frontier_size]

        if not clusters:
            self.get_logger().info(
                'No frontiers found — exploration may be complete!',
                throttle_duration_sec=10.0)
            return

        # 5. Convert cluster centroids to world coordinates
        #    Size stored in metres (cell_count * resolution) so it's
        #    comparable to distance in the cost function.
        cell_area = info.resolution  # metres per cell edge
        centroids = []
        for cluster in clusters:
            cr = np.mean([p[0] for p in cluster])
            cc = np.mean([p[1] for p in cluster])
            wx = cc * info.resolution + info.origin.position.x
            wy = cr * info.resolution + info.origin.position.y
            size_m = len(cluster) * cell_area  # frontier length in metres
            centroids.append((wx, wy, size_m))

        # 5b. Filter out frontiers whose centroids are too close to the
        #     robot.  These are the edges of the current scan footprint —
        #     the robot is already there and Nav2 will instantly declare
        #     "goal reached" without moving.
        centroids = [
            (cx, cy, sz) for cx, cy, sz in centroids
            if math.hypot(cx - robot_xy[0], cy - robot_xy[1])
               >= self.min_goal_distance
        ]

        if not centroids:
            self.get_logger().info(
                'All frontier centroids are within min_goal_distance '
                f'({self.min_goal_distance:.1f} m) — nothing to explore',
                throttle_duration_sec=10.0)
            return

        # 6. Filter blacklisted
        now = self.get_clock().now()
        # Expire old blacklist entries
        self.blacklisted = [
            (bx, by, t) for bx, by, t in self.blacklisted
            if (now - t).nanoseconds / 1e9 < self.blacklist_timeout
        ]
        valid = []
        for cx, cy, sz in centroids:
            if not self._is_blacklisted(cx, cy):
                valid.append((cx, cy, sz))

        if not valid:
            self.get_logger().warn(
                'All frontiers blacklisted — clearing blacklist')
            self.blacklisted.clear()
            valid = centroids

        # 7. Score & choose best frontier
        best = self._choose_frontier(valid, robot_xy)

        if best is None:
            return

        # 8. Hysteresis: don't switch if new goal is very close to old
        if self.current_goal is not None:
            dist = math.hypot(best[0] - self.current_goal[0],
                              best[1] - self.current_goal[1])
            if dist < self.goal_hysteresis:
                # Resend the same goal
                best = self.current_goal

        # 9. Visualise
        if self.visualize:
            self._publish_markers(valid, best)

        # 10. Navigate
        self._navigate_to(best[0], best[1])

    # ================================================================
    # Frontier detection
    # ================================================================

    def _find_frontiers(self, map_array: np.ndarray):
        """
        Detect frontier cells: free cells (0) adjacent to unknown cells (-1).
        Uses vectorised operations for speed.
        """
        rows, cols = map_array.shape
        free_mask = (map_array == 0)
        unknown_mask = (map_array == -1)

        # For each free cell check 4-connected neighbours for unknown
        frontier_mask = np.zeros_like(free_mask)
        frontier_mask[1:, :] |= free_mask[1:, :] & unknown_mask[:-1, :]
        frontier_mask[:-1, :] |= free_mask[:-1, :] & unknown_mask[1:, :]
        frontier_mask[:, 1:] |= free_mask[:, 1:] & unknown_mask[:, :-1]
        frontier_mask[:, :-1] |= free_mask[:, :-1] & unknown_mask[:, 1:]

        # Also check diagonal neighbours
        frontier_mask[1:, 1:] |= free_mask[1:, 1:] & unknown_mask[:-1, :-1]
        frontier_mask[1:, :-1] |= free_mask[1:, :-1] & unknown_mask[:-1, 1:]
        frontier_mask[:-1, 1:] |= free_mask[:-1, 1:] & unknown_mask[1:, :-1]
        frontier_mask[:-1, :-1] |= free_mask[:-1, :-1] & unknown_mask[1:, 1:]

        frontier_cells = list(zip(*np.where(frontier_mask)))
        self.get_logger().debug(f'Found {len(frontier_cells)} frontier cells')
        return frontier_cells

    def _cluster_frontiers(self, frontiers, connectivity=2):
        """
        Simple flood-fill clustering of frontier cells.
        Groups adjacent frontier cells into clusters.
        """
        if not frontiers:
            return []

        frontier_set = set(frontiers)
        visited = set()
        clusters = []

        for cell in frontiers:
            if cell in visited:
                continue
            # BFS from this cell
            cluster = []
            queue = [cell]
            while queue:
                current = queue.pop(0)
                if current in visited:
                    continue
                if current not in frontier_set:
                    continue
                visited.add(current)
                cluster.append(current)
                r, c = current
                for dr in range(-connectivity, connectivity + 1):
                    for dc in range(-connectivity, connectivity + 1):
                        if dr == 0 and dc == 0:
                            continue
                        nb = (r + dr, c + dc)
                        if nb in frontier_set and nb not in visited:
                            queue.append(nb)
            if cluster:
                clusters.append(cluster)

        self.get_logger().debug(f'Clustered into {len(clusters)} frontier groups')
        return clusters

    # ================================================================
    # Frontier selection
    # ================================================================

    def _choose_frontier(self, centroids, robot_xy):
        """
        Score frontiers by:  cost = potential_scale * distance - gain_scale * size
        Lower cost is better.  Both distance and size are in metres.
        """
        rx, ry = robot_xy
        best = None
        best_cost = float('inf')
        best_dist = 0.0
        best_sz = 0.0

        for cx, cy, sz in centroids:
            dist = math.hypot(cx - rx, cy - ry)
            cost = self.potential_scale * dist - self.gain_scale * sz
            if cost < best_cost:
                best_cost = cost
                best = (cx, cy)
                best_dist = dist
                best_sz = sz

        if best is not None:
            self.get_logger().info(
                f'Best frontier: ({best[0]:.2f}, {best[1]:.2f})  '
                f'dist={best_dist:.2f}m  size={best_sz:.1f}m  cost={best_cost:.1f}')
        else:
            self.get_logger().warning('No valid frontier to choose')
        return best

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

        self.get_logger().info(f'Sending goal: ({x:.2f}, {y:.2f})')

        send_future = self.nav_client.send_goal_async(nav_goal)
        send_future.add_done_callback(self._goal_response_cb)

        self.current_goal = (x, y)
        self.navigating = True
        self.last_progress_time = self.get_clock().now()
        self.last_robot_pos = self._get_robot_position()

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected by Nav2')
            self._blacklist_current_goal()
            self.navigating = False
            return

        self.get_logger().info('Goal accepted')
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._navigation_result_cb)

    def _navigation_result_cb(self, future):
        try:
            status = future.result().status
            # 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
            if status == 4:
                self.get_logger().info('Navigation succeeded ✓')
            elif status == 6:
                self.get_logger().warning('Navigation aborted — blacklisting goal')
                self._blacklist_current_goal()
            else:
                self.get_logger().warning(f'Navigation ended with status {status}')
        except Exception as e:
            self.get_logger().error(f'Navigation result error: {e}')
            self._blacklist_current_goal()

        self.navigating = False
        self.goal_handle = None

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
            # Making progress — reset timer
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

    def _publish_markers(self, centroids, chosen):
        """Publish frontier centroids as RViz markers."""
        ma = MarkerArray()

        # Delete old markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        ma.markers.append(delete_marker)

        for i, (cx, cy, sz) in enumerate(centroids):
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontiers'
            m.id = i + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0

            # Scale by cluster size
            scale = max(0.15, min(0.6, sz * 0.005))
            m.scale.x = scale
            m.scale.y = scale
            m.scale.z = scale

            if chosen and abs(cx - chosen[0]) < 0.01 and abs(cy - chosen[1]) < 0.01:
                # Current goal — green
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 0.0
                m.color.a = 1.0
            else:
                # Other frontiers — blue
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
