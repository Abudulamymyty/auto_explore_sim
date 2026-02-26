#!/usr/bin/env python3
"""
orb_slam3_map_publisher.py
==========================
LiDAR-based occupancy-grid builder driven by the ORB-SLAM3 TF chain.

ORB-SLAM3 (via orb_slam3_tf_bridge) broadcasts the map→odom TF.  This node
subscribes to /scan (LaserScan from the G1's LiDAR) and, for every scan,
looks up the TF that maps the sensor frame into the map frame.  Hit endpoints
are accumulated in an OccupancyGrid and published on /map for Nav2.

This approach avoids the 3-D→2-D projection ambiguity of projecting ORB-SLAM3
point clouds and handles the ORB-SLAM3 coordinate-frame conventions cleanly —
all of that complexity is already resolved inside orb_slam3_tf_bridge.py.

Parameters (ROS 2 parameters)
------------------------------
map_frame       (str,   default 'map')     : TF frame of the output map.
scan_topic      (str,   default '/scan')   : LaserScan topic to subscribe to.
map_resolution  (float, default 0.05)     : Cell size in metres.
map_size        (float, default 40.0)     : Map full-width / height in m.
publish_rate    (float, default 1.0)      : /map publish frequency (Hz).

Subscribed topics
-----------------
/scan   sensor_msgs/LaserScan

Published topics
----------------
/map    nav_msgs/OccupancyGrid
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class OrbSlam3MapPublisher(Node):

    def __init__(self):
        super().__init__('orb_slam3_map_publisher')

        # --- Parameters ---
        self.declare_parameter('map_frame',      'map')
        self.declare_parameter('scan_topic',     '/scan')
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_size',       40.0)
        self.declare_parameter('publish_rate',   1.0)

        self._map_frame  = self.get_parameter('map_frame').value
        self._scan_topic = self.get_parameter('scan_topic').value
        self._resolution = self.get_parameter('map_resolution').value
        self._map_size   = self.get_parameter('map_size').value

        # Grid dimensions
        self._n = int(math.ceil(self._map_size / self._resolution))
        self._origin_x = -(self._map_size / 2.0)
        self._origin_y = -(self._map_size / 2.0)

        # Accumulated log-odds grid (float32 for numerical stability)
        # 0 = unknown, positive = occupied, negative = free
        self._log_odds = np.zeros((self._n, self._n), dtype=np.float32)

        self._L_OCC  =  0.85   # log-odds increment for occupied
        self._L_FREE = -0.40   # log-odds increment for free ray

        # --- TF ---
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_timeout  = Duration(seconds=0.1)

        # --- Publisher ---
        self._map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        # --- Subscriber ---
        self.create_subscription(LaserScan, self._scan_topic,
                                 self._scan_cb, 10)

        # --- Publish timer ---
        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self._publish_map)

        self.get_logger().info(
            f'orb_slam3_map_publisher ready — '
            f'{self._n}×{self._n} cells @ {self._resolution} m, '
            f'scan: {self._scan_topic}, '
            f'publishing /map at {rate:.1f} Hz')

    # ------------------------------------------------------------------
    def _scan_cb(self, msg: LaserScan):
        """Transform scan hit-points into map frame and accumulate."""
        # Look up transform: map ← scan frame
        scan_frame = msg.header.frame_id
        stamp      = msg.header.stamp
        try:
            ts = self._tf_buffer.lookup_transform(
                self._map_frame, scan_frame,
                RclpyTime.from_msg(stamp), self._tf_timeout)
        except Exception:
            # TF not yet available (ORB-SLAM3 still initialising)
            return

        # Sensor origin in map frame
        tx = ts.transform.translation.x
        ty = ts.transform.translation.y
        r  = ts.transform.rotation
        yaw = math.atan2(
            2.0 * (r.w * r.z + r.x * r.y),
            1.0 - 2.0 * (r.y * r.y + r.z * r.z))

        # Compute hit-point angles and ranges
        n      = len(msg.ranges)
        angles = np.array([msg.angle_min + i * msg.angle_increment
                           for i in range(n)], dtype=np.float64)
        ranges = np.array(msg.ranges, dtype=np.float64)

        # Valid hits: finite, within [range_min, range_max]
        valid = (np.isfinite(ranges) &
                 (ranges >= msg.range_min) &
                 (ranges <= msg.range_max))

        if not valid.any():
            return

        # Hit endpoints in map frame
        global_angles = angles[valid] + yaw
        hx = tx + ranges[valid] * np.cos(global_angles)
        hy = ty + ranges[valid] * np.sin(global_angles)

        # Grid indices for hits
        gx = ((hx - self._origin_x) / self._resolution).astype(int)
        gy = ((hy - self._origin_y) / self._resolution).astype(int)
        in_b = (gx >= 0) & (gx < self._n) & (gy >= 0) & (gy < self._n)
        gx, gy = gx[in_b], gy[in_b]

        # Mark hits as occupied
        self._log_odds[gy, gx] += self._L_OCC

        # Ray-cast free cells (Bresenham) for a subset to keep CPU load low
        ox = int((tx - self._origin_x) / self._resolution)
        oy = int((ty - self._origin_y) / self._resolution)
        if 0 <= ox < self._n and 0 <= oy < self._n:
            step = max(1, len(gx) // 50)   # at most ~50 rays per scan
            for i in range(0, len(gx), step):
                self._bresenham_free(ox, oy, gx[i], gy[i])

        # Clamp log-odds to avoid saturation
        np.clip(self._log_odds, -10.0, 10.0, out=self._log_odds)

    # ------------------------------------------------------------------
    def _bresenham_free(self, x0, y0, x1, y1):
        """Mark cells along a ray as free (stopping one cell before the hit)."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        n = self._n
        while True:
            if x0 == x1 and y0 == y1:
                break
            if not (0 <= x0 < n and 0 <= y0 < n):
                break
            self._log_odds[y0, x0] += self._L_FREE
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    # ------------------------------------------------------------------
    def _publish_map(self):
        """Convert log-odds grid to OccupancyGrid and publish."""
        now = self.get_clock().now().to_msg()

        # Convert log-odds to {-1, 0, 100}
        grid = np.full((self._n, self._n), -1, dtype=np.int8)
        grid[self._log_odds > 0.0]  = 100
        grid[self._log_odds < 0.0]  = 0

        msg = OccupancyGrid()
        msg.header.stamp    = now
        msg.header.frame_id = self._map_frame
        msg.info.resolution = self._resolution
        msg.info.width      = self._n
        msg.info.height     = self._n
        msg.info.map_load_time = now
        msg.info.origin.position.x = self._origin_x
        msg.info.origin.position.y = self._origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()

        self._map_pub.publish(msg)


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = OrbSlam3MapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
