#!/usr/bin/python3
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

import heapq
import math
import time
from collections import deque

import numpy as np
import rclpy
from scipy import ndimage
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray


# ---------------------------------------------------------------------------
# Frontier data container
# ---------------------------------------------------------------------------
class Frontier:
    """One frontier cluster discovered by BFS."""
    __slots__ = ('size', 'min_distance', 'cost', 'centroid')

    def __init__(self):
        self.size = 0               # number of cells
        self.min_distance = float('inf')
        self.cost = 0.0
        self.centroid = (0.0, 0.0)  # average of all points (world coords)


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
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('odom_topic', '/odom')
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
        self.declare_parameter('gvd_marker_publish_every', 1)
        self.declare_parameter('gvd_marker_stride', 1)
        self.declare_parameter('profile_timing', False)
        self.declare_parameter('profile_log_interval', 15.0)
        self.declare_parameter('profile_callbacks', False)
        self.declare_parameter('profile_callback_log_interval', 10.0)

        # ── Read parameters ─────────────────────────────────────────────
        self.planner_freq = self.get_parameter('planner_frequency').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.odom_topic = self.get_parameter('odom_topic').value
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
        self.gvd_marker_publish_every = max(
            1, int(self.get_parameter('gvd_marker_publish_every').value))
        self.gvd_marker_stride = max(
            1, int(self.get_parameter('gvd_marker_stride').value))
        self.profile_timing = self.get_parameter('profile_timing').value
        self.profile_log_interval = self.get_parameter('profile_log_interval').value
        self.profile_callbacks = self.get_parameter('profile_callbacks').value
        self.profile_callback_log_interval = self.get_parameter(
            'profile_callback_log_interval').value

        # ── Subscribers ─────────────────────────────────────────────────
        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.map_data = None
        self.map_array = None
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, 20)
        self.tf_sub = self.create_subscription(
            TFMessage, '/tf', self._tf_callback, tf_qos)

        # Stop / resume subscription
        self.exploring = True
        self.resume_sub = self.create_subscription(
            Bool, 'explore/resume', self._resume_callback, 10)

        # ── Nav2 action client ──────────────────────────────────────────
        self._action_cb_group = MutuallyExclusiveCallbackGroup()
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._action_cb_group)
        self.nav_through_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses',
            callback_group=self._action_cb_group)

        # ── Visualisation publisher ─────────────────────────────────────
        if self.visualize:
            self.marker_pub = self.create_publisher(
                MarkerArray, 'explore/frontiers', 10)
        if self.visualize_gvd:
            self.gvd_marker_pub = self.create_publisher(
                MarkerArray, 'explore/gvd', 10)
        self.path_marker_pub = self.create_publisher(
            MarkerArray, 'explore/gvd_path', 10)

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
        self._cached_frontier_mask = None   # boolean frontier candidate mask
        self._cached_frontier_stamp = None
        self._cached_gvd_marker_array = None
        self._cached_gvd_marker_stamp = None
        self._gvd_marker_publish_counter = 0
        self._timing_totals = {}
        self._timing_counts = {}
        self._timing_max = {}
        self._timing_last_log = time.perf_counter()
        self._timing_plan_counter = 0
        self._callback_totals = {}
        self._callback_counts = {}
        self._callback_max = {}
        self._callback_last_log = time.perf_counter()
        self._odom_pose_xy = None
        self._odom_stamp = None
        self._map_to_odom_xy = None
        self._map_to_odom_yaw = None
        self._map_to_odom_stamp = None
        self._robot_pose_xy = None
        self._robot_pose_stamp = None
        self._using_nav_feedback_pose = False

        # ── Timer ───────────────────────────────────────────────────────
        period = 1.0 / max(self.planner_freq, 0.01)
        self.timer = self.create_timer(period, self._explore_tick)

        self.get_logger().info(
            f'Frontier Explorer ready  (freq={self.planner_freq} Hz, '
            f'min_frontier={self.min_frontier_size} cells, '
            f'clearance_scale={self.clearance_scale}, '
            f'gvd_snap_radius={self.gvd_snap_radius}m, '
            f'gvd_marker_every={self.gvd_marker_publish_every}, '
            f'gvd_marker_stride={self.gvd_marker_stride}, '
            f'profile_callbacks={self.profile_callbacks}, '
            f'return_to_init={self.return_to_init})')

    # ================================================================
    # Callbacks
    # ================================================================

    def _map_callback(self, msg: OccupancyGrid):
        started = time.perf_counter()
        try:
            self.map_data = msg
            info = msg.info
            self.map_array = np.asarray(msg.data, dtype=np.int8).reshape(
                (info.height, info.width))
        finally:
            self._record_callback_timing(
                'map_callback', time.perf_counter() - started)

    def _odom_callback(self, msg: Odometry):
        """Cache robot pose in the odom frame."""
        started = time.perf_counter()
        try:
            frame_id = self._normalize_frame_id(msg.header.frame_id)
            if frame_id and frame_id != self.odom_frame:
                self.get_logger().warning(
                    f'Ignoring odom pose in unexpected frame {frame_id!r}; '
                    f'expected {self.odom_frame!r}',
                    throttle_duration_sec=5.0)
                return

            prev_odom_xy = self._odom_pose_xy
            new_odom_xy = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
            )
            self._odom_pose_xy = new_odom_xy
            self._odom_stamp = msg.header.stamp

            # After initial map pose bootstrap, integrate odom deltas directly
            # instead of processing the full TF stream continuously.
            if (self.tf_sub is None
                    and prev_odom_xy is not None
                    and self._robot_pose_xy is not None
                    and self._map_to_odom_yaw is not None):
                dx = new_odom_xy[0] - prev_odom_xy[0]
                dy = new_odom_xy[1] - prev_odom_xy[1]
                cos_y = math.cos(self._map_to_odom_yaw)
                sin_y = math.sin(self._map_to_odom_yaw)
                self._robot_pose_xy = (
                    self._robot_pose_xy[0] + cos_y * dx - sin_y * dy,
                    self._robot_pose_xy[1] + sin_y * dx + cos_y * dy,
                )
                return

            self._update_robot_pose_cache()
        finally:
            self._record_callback_timing(
                'odom_callback', time.perf_counter() - started)

    def _nav_feedback_cb(self, feedback_msg):
        """Update cached robot pose from Nav2 action feedback."""
        started = time.perf_counter()
        try:
            current_pose = feedback_msg.feedback.current_pose
            frame_id = self._normalize_frame_id(current_pose.header.frame_id)
            if frame_id and frame_id != self.global_frame:
                self.get_logger().warning(
                    f'Ignoring Nav2 feedback pose in unexpected frame {frame_id!r}; '
                    f'expected {self.global_frame!r}',
                    throttle_duration_sec=5.0)
                return

            self._robot_pose_xy = (
                float(current_pose.pose.position.x),
                float(current_pose.pose.position.y),
            )
            self._robot_pose_stamp = current_pose.header.stamp

            if not self._using_nav_feedback_pose:
                self._using_nav_feedback_pose = True
                if self.odom_sub is not None:
                    self.get_logger().info(
                        'Switching from /odom tracking to Nav2 feedback pose tracking')
                    self.destroy_subscription(self.odom_sub)
                    self.odom_sub = None
        finally:
            self._record_callback_timing(
                'nav_feedback_cb', time.perf_counter() - started)

    def _tf_callback(self, msg: TFMessage):
        """Cache only the map->odom transform we need for pose composition."""
        started = time.perf_counter()
        try:
            updated = False

            for transform in msg.transforms:
                parent = self._normalize_frame_id(transform.header.frame_id)
                child = self._normalize_frame_id(transform.child_frame_id)

                if parent == self.global_frame and child == self.odom_frame:
                    self._store_map_to_odom(
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        self._yaw_from_quaternion(transform.transform.rotation),
                        transform.header.stamp)
                    updated = True
                elif parent == self.odom_frame and child == self.global_frame:
                    tx = float(transform.transform.translation.x)
                    ty = float(transform.transform.translation.y)
                    yaw = self._yaw_from_quaternion(transform.transform.rotation)
                    cos_y = math.cos(yaw)
                    sin_y = math.sin(yaw)
                    self._store_map_to_odom(
                        -(cos_y * tx + sin_y * ty),
                        sin_y * tx - cos_y * ty,
                        -yaw,
                        transform.header.stamp)
                    updated = True

            if updated:
                self._update_robot_pose_cache()
        finally:
            self._record_callback_timing(
                'tf_callback', time.perf_counter() - started)

    def _resume_callback(self, msg: Bool):
        started = time.perf_counter()
        try:
            if msg.data:
                self.get_logger().info('Exploration RESUMED')
                self.exploring = True
            else:
                self.get_logger().info('Exploration STOPPED')
                self.exploring = False
                self._cancel_current_goal()
                self.navigating = False
        finally:
            self._record_callback_timing(
                'resume_callback', time.perf_counter() - started)

    def _record_timing(self, stage, duration_s):
        """Accumulate stage timing stats for later logging."""
        if not self.profile_timing:
            return

        self._timing_totals[stage] = self._timing_totals.get(stage, 0.0) + duration_s
        self._timing_counts[stage] = self._timing_counts.get(stage, 0) + 1
        self._timing_max[stage] = max(self._timing_max.get(stage, 0.0), duration_s)

    def _maybe_log_timing(self):
        """Periodically log average stage timings for recent planning cycles."""
        if not self.profile_timing or not self._timing_totals:
            return

        now = time.perf_counter()
        if now - self._timing_last_log < self.profile_log_interval:
            return

        total_time = self._timing_totals.get('plan_total', 0.0)
        lines = [
            f'Frontier timing over {self._timing_plan_counter} plans '
            f'({self.profile_log_interval:.1f}s window):'
        ]

        for stage, total in sorted(
                self._timing_totals.items(), key=lambda item: item[1], reverse=True):
            count = self._timing_counts.get(stage, 1)
            avg_ms = 1000.0 * total / max(count, 1)
            max_ms = 1000.0 * self._timing_max.get(stage, 0.0)
            share = (100.0 * total / total_time) if total_time > 0.0 else 0.0
            lines.append(
                f'  {stage}: avg={avg_ms:.1f}ms  max={max_ms:.1f}ms  share={share:.1f}%')

        self.get_logger().info('\n'.join(lines))
        self._timing_totals.clear()
        self._timing_counts.clear()
        self._timing_max.clear()
        self._timing_last_log = now
        self._timing_plan_counter = 0

    def _record_callback_timing(self, callback_name, duration_s):
        """Accumulate callback timing stats for later logging."""
        if not self.profile_callbacks:
            return

        self._callback_totals[callback_name] = (
            self._callback_totals.get(callback_name, 0.0) + duration_s)
        self._callback_counts[callback_name] = (
            self._callback_counts.get(callback_name, 0) + 1)
        self._callback_max[callback_name] = max(
            self._callback_max.get(callback_name, 0.0), duration_s)
        self._maybe_log_callback_timing()

    def _maybe_log_callback_timing(self):
        """Periodically log per-callback rate and timing statistics."""
        if not self.profile_callbacks or not self._callback_totals:
            return

        now = time.perf_counter()
        window_s = now - self._callback_last_log
        if window_s < self.profile_callback_log_interval:
            return

        total_callback_time = sum(self._callback_totals.values())
        lines = [f'Frontier callback timing ({window_s:.1f}s window):']

        for callback_name, total in sorted(
                self._callback_totals.items(), key=lambda item: item[1], reverse=True):
            count = self._callback_counts.get(callback_name, 1)
            avg_ms = 1000.0 * total / max(count, 1)
            max_ms = 1000.0 * self._callback_max.get(callback_name, 0.0)
            rate_hz = count / max(window_s, 1e-6)
            share = (100.0 * total / total_callback_time) if total_callback_time > 0.0 else 0.0
            wall = 100.0 * total / max(window_s, 1e-6)
            lines.append(
                f'  {callback_name}: rate={rate_hz:.1f}Hz  avg={avg_ms:.3f}ms  '
                f'max={max_ms:.3f}ms  share={share:.1f}%  wall={wall:.1f}%')

        lines.append(
            f'  callback_busy_total: {100.0 * total_callback_time / max(window_s, 1e-6):.1f}%')
        self.get_logger().info('\n'.join(lines))
        self._callback_totals.clear()
        self._callback_counts.clear()
        self._callback_max.clear()
        self._callback_last_log = now

    @staticmethod
    def _normalize_frame_id(frame_id):
        """Normalize ROS frame IDs for direct string comparison."""
        return frame_id.lstrip('/') if frame_id else ''

    @staticmethod
    def _yaw_from_quaternion(quat):
        """Return planar yaw from a quaternion."""
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _store_map_to_odom(self, tx, ty, yaw, stamp):
        """Store the latest map->odom transform components."""
        self._map_to_odom_xy = (float(tx), float(ty))
        self._map_to_odom_yaw = float(yaw)
        self._map_to_odom_stamp = stamp

    def _update_robot_pose_cache(self):
        """Compose cached map->odom and odom->base data into a map pose."""
        if self._map_to_odom_xy is None or self._odom_pose_xy is None:
            return

        tx, ty = self._map_to_odom_xy
        ox, oy = self._odom_pose_xy
        cos_y = math.cos(self._map_to_odom_yaw)
        sin_y = math.sin(self._map_to_odom_yaw)

        self._robot_pose_xy = (
            tx + cos_y * ox - sin_y * oy,
            ty + sin_y * ox + cos_y * oy,
        )
        self._robot_pose_stamp = self._odom_stamp

        if self.tf_sub is not None:
            self.get_logger().info(
                'Initial map pose cached; switching to odom-delta tracking')
            self.destroy_subscription(self.tf_sub)
            self.tf_sub = None

    def _stamp_age_seconds(self, stamp):
        """Return the age of a ROS stamp using the node's active clock."""
        if stamp is None:
            return float('inf')

        now = self.get_clock().now()
        if now.nanoseconds <= 0:
            return 0.0

        age_ns = (now - Time.from_msg(stamp)).nanoseconds
        return max(0.0, age_ns / 1e9)

    # ================================================================
    # Main exploration loop
    # ================================================================

    def _explore_tick(self):
        started = time.perf_counter()
        try:
            if not self.exploring:
                return

            self._make_plan()
        finally:
            self._record_callback_timing(
                'explore_tick', time.perf_counter() - started)

    def _make_plan(self):
        """Core planning: find frontiers from robot, pick best, navigate."""
        if self.map_data is None:
            self.get_logger().info('Waiting for map...', throttle_duration_sec=5.0)
            return

        plan_started = time.perf_counter()

        # 1. Get robot position in map frame
        stage_started = time.perf_counter()
        robot_xy = self._get_robot_position()
        self._record_timing('tf_lookup', time.perf_counter() - stage_started)
        if robot_xy is None:
            return

        # Store initial pose for return_to_init
        if self.initial_pose is None:
            self.initial_pose = robot_xy
            self.get_logger().info(
                f'Initial pose stored: ({robot_xy[0]:.2f}, {robot_xy[1]:.2f})')

        # 2. Check navigation progress
        stage_started = time.perf_counter()
        self._check_progress(robot_xy)
        self._record_timing('progress_check', time.perf_counter() - stage_started)

        # If already navigating and making progress, do nothing
        if self.navigating:
            return

        # 3. BFS frontier search from robot position
        info = self.map_data.info
        map_array = self.map_array
        if map_array is None:
            self.get_logger().warning('Map array cache not ready yet')
            return

        stage_started = time.perf_counter()
        dist_map = self._get_distance_transform(map_array)
        self._record_timing('gvd_build', time.perf_counter() - stage_started)

        stage_started = time.perf_counter()
        frontiers = self._search_from(robot_xy, map_array, info, dist_map=dist_map)
        self._record_timing('frontier_search', time.perf_counter() - stage_started)

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
            stage_started = time.perf_counter()
            self._publish_markers(valid, best)
            self._record_timing('frontier_markers', time.perf_counter() - stage_started)
        if self.visualize_gvd:
            stage_started = time.perf_counter()
            self._publish_gvd_markers(map_array, info)
            self._record_timing('gvd_markers', time.perf_counter() - stage_started)

        # 7. Try to find a path along the GVD skeleton
        stage_started = time.perf_counter()
        gvd_path = self._find_gvd_path(
            robot_xy, best.centroid, map_array, info)
        self._record_timing('gvd_path_search', time.perf_counter() - stage_started)

        if gvd_path and len(gvd_path) >= 2:
            # GVD path found — sample waypoints and navigate through them
            stage_started = time.perf_counter()
            waypoints = self._sample_waypoints(gvd_path, spacing=0.5)
            self._record_timing('waypoint_sampling', time.perf_counter() - stage_started)
            gx, gy = waypoints[-1]
            self.get_logger().info(
                f'GVD path found: {len(gvd_path)} cells → '
                f'{len(waypoints)} waypoints')
        else:
            # Fallback: snap frontier goal to nearest GVD point
            stage_started = time.perf_counter()
            gx, gy = self._snap_to_gvd(
                best.centroid, robot_xy, map_array, info)
            self._record_timing('gvd_snap_fallback', time.perf_counter() - stage_started)
            waypoints = None
            self.get_logger().info('No GVD path — using single goal fallback')

        # 8. Skip if goal is too close to robot (instant-success loop)
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
        if waypoints and len(waypoints) >= 2:
            stage_started = time.perf_counter()
            self._publish_path_markers(waypoints)
            self._record_timing('path_markers', time.perf_counter() - stage_started)
            stage_started = time.perf_counter()
            self._navigate_through_poses(waypoints)
            self._record_timing('send_goal', time.perf_counter() - stage_started)
        else:
            stage_started = time.perf_counter()
            self._navigate_to(gx, gy)
            self._record_timing('send_goal', time.perf_counter() - stage_started)

        self._record_timing('plan_total', time.perf_counter() - plan_started)
        self._timing_plan_counter += 1
        self._maybe_log_timing()

    # ================================================================
    # BFS frontier search from robot position
    # ================================================================

    def _search_from(self, robot_xy, map_array, info, dist_map=None):
        """
        Find reachable frontiers from the robot position.

        This keeps the original frontier semantics but replaces the Python
        free-space / frontier BFS passes with connected-component operations
        in ndimage, which are much cheaper on large grids.
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

        frontier_mask = self._get_frontier_mask(map_array)
        if not np.any(frontier_mask):
            return []

        structure8 = np.ones((3, 3), dtype=np.uint8)

        # Reachability is determined from the robot's connected free-space
        # component, then we keep frontier clusters touching that region.
        free = (map_array == 0)
        free_labels, _ = ndimage.label(free, structure=structure8)
        reachable_label = int(free_labels[my, mx])
        if reachable_label <= 0:
            self.get_logger().warning('Robot free-space component not found')
            return []

        reachable_free = (free_labels == reachable_label)
        frontier_labels, _ = ndimage.label(frontier_mask, structure=structure8)

        touching_frontier = frontier_labels[
            ndimage.binary_dilation(reachable_free, structure=structure8) & frontier_mask
        ]
        frontier_ids = np.unique(touching_frontier)
        frontier_ids = frontier_ids[frontier_ids > 0]
        if frontier_ids.size == 0:
            return []

        frontier_ys, frontier_xs = np.nonzero(frontier_labels)
        all_labels = frontier_labels[frontier_ys, frontier_xs]
        keep = np.isin(all_labels, frontier_ids)
        if not np.any(keep):
            return []

        xs = frontier_xs[keep].astype(np.int32)
        ys = frontier_ys[keep].astype(np.int32)
        labels = all_labels[keep].astype(np.int32)

        wx = xs * resolution + ox
        wy = ys * resolution + oy
        dists = np.hypot(wx - robot_xy[0], wy - robot_xy[1])

        counts = np.bincount(labels)
        sum_x = np.bincount(labels, weights=wx, minlength=counts.size)
        sum_y = np.bincount(labels, weights=wy, minlength=counts.size)
        min_dist = np.full(counts.size, np.inf, dtype=np.float64)
        np.minimum.at(min_dist, labels, dists)

        # Distance transform is used only for the clearance bonus term.
        if dist_map is None:
            dist_map = self._get_distance_transform(map_array)
        frontiers = []

        # Cost = min_distance - clearance_scale * clearance_at_centroid.
        for label_id in frontier_ids.tolist():
            size = int(counts[label_id])
            if size < self.min_frontier_size:
                continue

            centroid_x = float(sum_x[label_id] / size)
            centroid_y = float(sum_y[label_id] / size)
            cx = int((centroid_x - ox) / resolution)
            cy = int((centroid_y - oy) / resolution)
            cx = max(0, min(w - 1, cx))
            cy = max(0, min(h - 1, cy))
            clearance = float(dist_map[cy, cx]) * resolution  # metres

            frontier = Frontier()
            frontier.size = size
            frontier.min_distance = float(min_dist[label_id])
            frontier.centroid = (centroid_x, centroid_y)
            frontier.cost = frontier.min_distance - self.clearance_scale * clearance
            frontiers.append(frontier)

        frontiers.sort(key=lambda f: f.cost)
        return frontiers

    def _get_frontier_mask(self, map_array):
        """Return cached frontier-candidate mask for the current map."""
        stamp = self.map_data.header.stamp if self.map_data else None
        if (self._cached_frontier_mask is not None
                and self._cached_frontier_stamp == stamp
                and self._cached_frontier_mask.shape == map_array.shape):
            return self._cached_frontier_mask

        free = (map_array == 0)
        unknown = (map_array == -1)
        h, w = map_array.shape
        adjacent_free = np.zeros((h, w), dtype=bool)

        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue

                if dy >= 0:
                    src_y = slice(0, h - dy)
                    dst_y = slice(dy, h)
                else:
                    src_y = slice(-dy, h)
                    dst_y = slice(0, h + dy)

                if dx >= 0:
                    src_x = slice(0, w - dx)
                    dst_x = slice(dx, w)
                else:
                    src_x = slice(-dx, w)
                    dst_x = slice(0, w + dx)

                adjacent_free[dst_y, dst_x] |= free[src_y, src_x]

        self._cached_frontier_mask = unknown & adjacent_free
        self._cached_frontier_stamp = stamp
        return self._cached_frontier_mask

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
        self._cached_gvd_marker_array = None
        self._cached_gvd_marker_stamp = None
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
        if not np.any(obstacle):
            return (np.zeros((h, w), dtype=np.float32),
                    np.full((h, w), -1, dtype=np.int32))

        region_id, num_regions = ndimage.label(
            obstacle, structure=np.ones((3, 3), dtype=np.uint8))
        self.get_logger().debug(
            f'GVD: found {num_regions} obstacle regions',
            throttle_duration_sec=10.0)

        dist, nearest_idx = ndimage.distance_transform_edt(
            ~obstacle, return_indices=True)
        nearest_y, nearest_x = nearest_idx

        label = region_id[nearest_y, nearest_x].astype(np.int32) - 1
        return dist.astype(np.float32), label

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

        max_cells = int(self.gvd_snap_radius / resolution)
        xs, ys = self._window_true_coords(gvd_mask, px, py, max_cells)
        if xs.size == 0:
            self.get_logger().debug(
                f'No valid GVD point within {self.gvd_snap_radius}m '
                f'— using original centroid')
            return point

        wx = xs * resolution + ox
        wy = ys * resolution + oy
        cand_to_robot = np.hypot(wx - robot_xy[0], wy - robot_xy[1])
        cand_to_frontier = np.hypot(wx - point[0], wy - point[1])
        valid = cand_to_robot < robot_to_frontier

        if np.any(valid):
            valid_idx = np.flatnonzero(valid)
            best_idx = valid_idx[np.argmin(cand_to_frontier[valid])]
            best_gvd = (float(wx[best_idx]), float(wy[best_idx]))
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
    # GVD path planning (A* on skeleton)
    # ================================================================

    def _window_true_coords(self, mask, px, py, max_cells):
        """Return absolute coordinates of True cells in a square window."""
        h, w = mask.shape
        x0 = max(0, px - max_cells)
        x1 = min(w, px + max_cells + 1)
        y0 = max(0, py - max_cells)
        y1 = min(h, py + max_cells + 1)

        ys, xs = np.nonzero(mask[y0:y1, x0:x1])
        if xs.size == 0:
            return np.empty(0, dtype=np.int32), np.empty(0, dtype=np.int32)
        return xs.astype(np.int32) + x0, ys.astype(np.int32) + y0

    def _nearest_gvd_cell(self, wx, wy, map_array, info, max_radius_m=1.5):
        """Find nearest GVD cell to a world-coordinate point in a local window."""
        gvd_mask = self._get_gvd_mask(map_array)
        if gvd_mask is None:
            return None
        resolution = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        w, h = info.width, info.height
        px = max(0, min(w - 1, int((wx - ox) / resolution)))
        py = max(0, min(h - 1, int((wy - oy) / resolution)))
        if gvd_mask[py, px]:
            return (px, py)
        max_cells = int(max_radius_m / resolution)
        xs, ys = self._window_true_coords(gvd_mask, px, py, max_cells)
        if xs.size == 0:
            return None
        dist2 = (xs - px) ** 2 + (ys - py) ** 2
        best = int(np.argmin(dist2))
        return (int(xs[best]), int(ys[best]))

    def _iter_gvd_neighbors(self, x, y, gvd_mask):
        """Yield 8-connected neighbors that remain on the GVD skeleton."""
        h, w = gvd_mask.shape
        sqrt2 = math.sqrt(2)
        for ddx, ddy in ((-1, -1), (-1, 0), (-1, 1),
                         (0, -1),           (0, 1),
                         (1, -1),  (1, 0),  (1, 1)):
            nx, ny = x + ddx, y + ddy
            if not (0 <= nx < w and 0 <= ny < h):
                continue
            if not gvd_mask[ny, nx]:
                continue
            yield nx, ny, (sqrt2 if (ddx != 0 and ddy != 0) else 1.0)

    def _find_gvd_path(self, robot_xy, goal_xy, map_array, info):
        """
        A* search from robot to goal on the GVD skeleton only.
        Returns list of (wx, wy) world-coordinate waypoints, or None.
        """
        gvd_mask = self._get_gvd_mask(map_array)
        if gvd_mask is None:
            return None

        # Find nearest GVD cells to robot and goal
        start = self._nearest_gvd_cell(
            robot_xy[0], robot_xy[1], map_array, info)
        end = self._nearest_gvd_cell(
            goal_xy[0], goal_xy[1], map_array, info)
        if start is None or end is None:
            self.get_logger().info(
                f'GVD path: no GVD cell near '
                f'{"robot" if start is None else "goal"}')
            return None
        if start == end:
            return None

        resolution = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y

        counter = 0
        open_set = []
        heapq.heappush(open_set, (0.0, counter, start))
        came_from = {}
        g_score = {start: 0.0}

        ex, ey = end

        while open_set:
            _, _, current = heapq.heappop(open_set)
            cx, cy = current

            if current == end:
                # Reconstruct path
                path = []
                node = end
                while node in came_from:
                    path.append(node)
                    node = came_from[node]
                path.append(start)
                path.reverse()
                # Convert to world coordinates
                world_path = [(x * resolution + ox, y * resolution + oy)
                              for x, y in path]
                gvd_count = sum(1 for x, y in path if gvd_mask[y, x])
                self.get_logger().info(
                    f'GVD path: {len(path)} cells, '
                    f'{gvd_count} on skeleton '
                    f'({100*gvd_count//len(path)}%)')
                return world_path

            for nx, ny, step_cost in self._iter_gvd_neighbors(cx, cy, gvd_mask):
                tentative_g = g_score[current] + step_cost
                neighbor = (nx, ny)

                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    heur = math.hypot(nx - ex, ny - ey)
                    counter += 1
                    heapq.heappush(open_set,
                                   (tentative_g + heur, counter, neighbor))

        self.get_logger().info(
            f'GVD path: A* failed (explored {len(g_score)} cells)')
        return None

    def _sample_waypoints(self, path, spacing=0.5):
        """
        Down-sample a dense path to waypoints spaced ~spacing metres apart.
        Always includes the first and last point.
        """
        if len(path) <= 2:
            return list(path)

        sampled = [path[0]]
        accum = 0.0

        for i in range(1, len(path)):
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
            accum += math.hypot(dx, dy)
            if accum >= spacing:
                sampled.append(path[i])
                accum = 0.0

        # Always include the last point
        if sampled[-1] != path[-1]:
            sampled.append(path[-1])

        return sampled

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

        send_future = self.nav_client.send_goal_async(
            nav_goal, feedback_callback=self._nav_feedback_cb)
        send_future.add_done_callback(
            lambda f, s=seq: self._goal_response_cb(f, s))

        self.current_goal = (x, y)
        self.prev_goal = (x, y)
        self.navigating = True
        self.last_progress_time = self.get_clock().now()
        self.last_robot_pos = self._get_robot_position()

    def _navigate_through_poses(self, waypoints):
        """Send a NavigateThroughPoses goal with GVD waypoints."""
        if not self.nav_through_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'NavigateThroughPoses action server not available!')
            return

        stamp = self.get_clock().now().to_msg()
        poses = []
        for wx, wy in waypoints:
            p = PoseStamped()
            p.header.frame_id = self.global_frame
            p.header.stamp = stamp
            p.pose.position.x = wx
            p.pose.position.y = wy
            p.pose.orientation.w = 1.0
            poses.append(p)

        nav_goal = NavigateThroughPoses.Goal()
        nav_goal.poses = poses

        self._goal_seq += 1
        seq = self._goal_seq
        last = waypoints[-1]
        self.get_logger().info(
            f'Navigating through {len(waypoints)} GVD waypoints '
            f'→ ({last[0]:.2f}, {last[1]:.2f})')

        send_future = self.nav_through_client.send_goal_async(
            nav_goal, feedback_callback=self._nav_feedback_cb)
        send_future.add_done_callback(
            lambda f, s=seq: self._goal_response_cb(f, s))

        self.current_goal = last
        self.prev_goal = last
        self.navigating = True
        self.last_progress_time = self.get_clock().now()
        self.last_robot_pos = self._get_robot_position()

    def _goal_response_cb(self, future, seq):
        started = time.perf_counter()
        try:
            # Ignore stale callback from a superseded goal
            if seq != self._goal_seq:
                return

            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warning(
                    'Goal rejected by Nav2; keeping frontier and waiting for next planner tick')
                self.navigating = False
                self.prev_goal = None
                return

            self.get_logger().info('Goal accepted')
            self.goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda f, s=seq: self._navigation_result_cb(f, s))
        finally:
            self._record_callback_timing(
                'goal_response_cb', time.perf_counter() - started)

    def _navigation_result_cb(self, future, seq):
        """Handle navigation result and return control to the timer loop."""
        started = time.perf_counter()
        try:
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
                    self.get_logger().warning(
                        'Navigation aborted by Nav2; keeping frontier and waiting for next planner tick')
                elif status == 5:
                    self.get_logger().info('Navigation cancelled')
                else:
                    self.get_logger().warning(f'Navigation ended with status {status}')
            except Exception as e:
                self.get_logger().error(f'Navigation result error: {e}')

            self.navigating = False
            self.goal_handle = None
            self.prev_goal = None   # allow re-selecting same frontier if it persists
        finally:
            self._record_callback_timing(
                'navigation_result_cb', time.perf_counter() - started)

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
        """Get robot (x, y) in map frame from cached odom and map->odom."""
        if self._robot_pose_xy is None:
            self.get_logger().warning(
                'Robot pose cache not ready yet',
                throttle_duration_sec=5.0)
            return None

        if self._using_nav_feedback_pose:
            pose_age = self._stamp_age_seconds(self._robot_pose_stamp)
            if self.navigating and pose_age > self.tf_tolerance:
                self.get_logger().warning(
                    f'Nav2 feedback pose is stale ({pose_age:.2f}s old)',
                    throttle_duration_sec=5.0)
            return self._robot_pose_xy

        odom_age = self._stamp_age_seconds(self._odom_stamp)
        if odom_age > self.tf_tolerance:
            self.get_logger().warning(
                f'Odom pose is stale ({odom_age:.2f}s old)',
                throttle_duration_sec=5.0)
            return None

        return self._robot_pose_xy

    # ================================================================
    # Visualisation
    # ================================================================

    def _publish_path_markers(self, waypoints):
        """Publish the current GVD navigation path as a LINE_STRIP."""
        ma = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.ns = 'gvd_path'
        ma.markers.append(delete_marker)

        if len(waypoints) >= 2:
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'gvd_path'
            m.id = 1
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.05  # line width
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 1.0
            m.color.a = 1.0
            m.lifetime.sec = 30
            m.pose.orientation.w = 1.0

            for wx, wy in waypoints:
                m.points.append(Point(x=wx, y=wy, z=0.08))
            ma.markers.append(m)

            # Add sphere markers at each waypoint
            for i, (wx, wy) in enumerate(waypoints):
                s = Marker()
                s.header.frame_id = self.global_frame
                s.header.stamp = m.header.stamp
                s.ns = 'gvd_path'
                s.id = i + 10
                s.type = Marker.SPHERE
                s.action = Marker.ADD
                s.pose.position.x = wx
                s.pose.position.y = wy
                s.pose.position.z = 0.08
                s.pose.orientation.w = 1.0
                s.scale.x = 0.1
                s.scale.y = 0.1
                s.scale.z = 0.1
                s.color.r = 1.0
                s.color.g = 0.0
                s.color.b = 1.0
                s.color.a = 1.0
                s.lifetime.sec = 30
                ma.markers.append(s)

        self.path_marker_pub.publish(ma)

    def _publish_gvd_markers(self, map_array, info):
        """Publish GVD skeleton as connected LINE_LIST markers."""
        self._gvd_marker_publish_counter += 1
        if ((self._gvd_marker_publish_counter - 1) % self.gvd_marker_publish_every) != 0:
            return

        stamp = self.map_data.header.stamp if self.map_data else None
        if (self._cached_gvd_marker_array is not None
                and self._cached_gvd_marker_stamp == stamp):
            self.gvd_marker_pub.publish(self._cached_gvd_marker_array)
            return

        gvd_mask = self._get_gvd_mask(map_array)
        if gvd_mask is None:
            return

        resolution = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

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
        line_marker.lifetime.sec = 0
        line_marker.pose.orientation.w = 1.0

        edge_masks = (
            (gvd_mask[:, :-1] & gvd_mask[:, 1:], 0, 0, 1, 0),
            (gvd_mask[:-1, :] & gvd_mask[1:, :], 0, 0, 0, 1),
            (gvd_mask[:-1, :-1] & gvd_mask[1:, 1:], 0, 0, 1, 1),
            (gvd_mask[1:, :-1] & gvd_mask[:-1, 1:], 0, 1, 1, -1),
        )

        for edge_mask, x_off, y_off, dx, dy in edge_masks:
            ys, xs = np.nonzero(edge_mask)
            if self.gvd_marker_stride > 1:
                xs = xs[::self.gvd_marker_stride]
                ys = ys[::self.gvd_marker_stride]
            for x, y in zip(xs.tolist(), ys.tolist()):
                x0 = x + x_off
                y0 = y + y_off
                p1 = Point(
                    x=x0 * resolution + ox,
                    y=y0 * resolution + oy,
                    z=0.05)
                p2 = Point(
                    x=(x0 + dx) * resolution + ox,
                    y=(y0 + dy) * resolution + oy,
                    z=0.05)
                line_marker.points.append(p1)
                line_marker.points.append(p2)

        if line_marker.points:
            ma.markers.append(line_marker)

        self._cached_gvd_marker_array = ma
        self._cached_gvd_marker_stamp = stamp
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
