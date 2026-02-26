#!/usr/bin/env python3
"""
orb_slam3_map_publisher.py
==========================
Converts ORB-SLAM3 3D map points into a 2-D occupancy grid for Nav2.

ORB-SLAM3 (via orb_slam3_ros2_wrapper) publishes a PointCloud2 of *all* map
points (in the map frame) on /orb_slam3/all_map_points.  This node:

  1. Accumulates those points over time (the SLAM map grows incrementally).
  2. Filters points by height to select obstacles relevant to ground navigation
     (e.g. between 0.1 m and 1.8 m above the floor).
  3. Projects surviving points onto a 2-D grid and marks occupied cells.
  4. Publishes a nav_msgs/OccupancyGrid on /map at a configurable rate.

Nav2's static_layer subscribes to /map, so this node acts as the SLAM map
backend, equivalent to what slam_toolbox produced — but driven by ORB-SLAM3
instead of LiDAR.

Parameters (all ROS 2 parameters)
----------------------------------
map_frame          (str,   default 'map')     : TF frame of the output map.
map_resolution     (float, default 0.05)      : Cell size in metres.
map_size           (float, default 40.0)      : Map full-width / height in m.
publish_rate       (float, default 1.0)       : /map publish frequency (Hz).
min_height         (float, default 0.10)      : Min obstacle height (m).
max_height         (float, default 1.80)      : Max obstacle height (m).
inflation_cells    (int,   default 1)         : Extra cells to inflate occupied.

Subscribed topics
-----------------
/orb_slam3/all_map_points   sensor_msgs/PointCloud2

Published topics
----------------
/map   nav_msgs/OccupancyGrid
"""

import math
import struct
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from builtin_interfaces.msg import Time as TimeMsg


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract (N, 3) float32 array of XYZ from a PointCloud2 message."""
    # Locate x, y, z field offsets
    offsets = {}
    for field in msg.fields:
        if field.name in ('x', 'y', 'z'):
            offsets[field.name] = field.offset

    if not {'x', 'y', 'z'} <= offsets.keys():
        return np.empty((0, 3), dtype=np.float32)

    point_step = msg.point_step
    data = msg.data
    n_points = msg.width * msg.height

    xyz = np.empty((n_points, 3), dtype=np.float32)
    ox = offsets['x']
    oy = offsets['y']
    oz = offsets['z']

    for i in range(n_points):
        base = i * point_step
        xyz[i, 0] = struct.unpack_from('<f', data, base + ox)[0]
        xyz[i, 1] = struct.unpack_from('<f', data, base + oy)[0]
        xyz[i, 2] = struct.unpack_from('<f', data, base + oz)[0]

    # Filter NaN/Inf
    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid]


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class OrbSlam3MapPublisher(Node):

    def __init__(self):
        super().__init__('orb_slam3_map_publisher')

        # --- Parameters ---
        self.declare_parameter('map_frame',       'map')
        self.declare_parameter('map_resolution',  0.05)
        self.declare_parameter('map_size',        40.0)
        self.declare_parameter('publish_rate',    1.0)
        self.declare_parameter('min_height',      0.10)
        self.declare_parameter('max_height',      1.80)
        self.declare_parameter('inflation_cells', 1)

        self.map_frame      = self.get_parameter('map_frame').value
        self.resolution     = self.get_parameter('map_resolution').value
        self.map_size       = self.get_parameter('map_size').value
        self.min_h          = self.get_parameter('min_height').value
        self.max_h          = self.get_parameter('max_height').value
        self.inflate        = self.get_parameter('inflation_cells').value

        # Derived grid dimensions
        self.n_cells = int(math.ceil(self.map_size / self.resolution))
        # origin: centre of the grid at (0, 0) in map frame
        self.origin_x = -(self.map_size / 2.0)
        self.origin_y = -(self.map_size / 2.0)

        # Accumulated occupancy grid: -1=unknown, 0=free, 100=occupied
        self._grid = np.full((self.n_cells, self.n_cells), -1, dtype=np.int8)

        # --- Publisher ---
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        # --- Subscriber ---
        self.create_subscription(
            PointCloud2,
            '/orb_slam3/all_map_points',
            self._map_points_callback,
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value,
        )

        # --- Publish timer ---
        period = 1.0 / self.get_parameter('publish_rate').value
        self.create_timer(period, self._publish_map)

        self.get_logger().info(
            f'orb_slam3_map_publisher ready — '
            f'{self.n_cells}×{self.n_cells} cells @ {self.resolution} m/cell, '
            f'publishing /map at {1.0/period:.1f} Hz'
        )

    # ------------------------------------------------------------------
    def _map_points_callback(self, msg: PointCloud2):
        """Accumulate new map points into the grid."""
        if msg.width * msg.height == 0:
            return

        xyz = _pointcloud2_to_xyz(msg)
        if xyz.shape[0] == 0:
            return

        # Height filter
        mask = (xyz[:, 2] >= self.min_h) & (xyz[:, 2] <= self.max_h)
        xyz = xyz[mask]
        if xyz.shape[0] == 0:
            return

        # World → grid index
        ix = ((xyz[:, 0] - self.origin_x) / self.resolution).astype(int)
        iy = ((xyz[:, 1] - self.origin_y) / self.resolution).astype(int)

        # Clamp to grid bounds
        in_bounds = (ix >= 0) & (ix < self.n_cells) & \
                    (iy >= 0) & (iy < self.n_cells)
        ix = ix[in_bounds]
        iy = iy[in_bounds]

        # Mark occupied
        self._grid[iy, ix] = 100

        # Simple inflation
        if self.inflate > 0:
            for di in range(-self.inflate, self.inflate + 1):
                for dj in range(-self.inflate, self.inflate + 1):
                    ni = np.clip(iy + di, 0, self.n_cells - 1)
                    nj = np.clip(ix + dj, 0, self.n_cells - 1)
                    self._grid[ni, nj] = np.maximum(self._grid[ni, nj], 50)

    # ------------------------------------------------------------------
    def _publish_map(self):
        """Publish the accumulated occupancy grid."""
        now = self.get_clock().now().to_msg()

        msg = OccupancyGrid()
        msg.header.stamp = now
        msg.header.frame_id = self.map_frame

        msg.info.resolution     = self.resolution
        msg.info.width          = self.n_cells
        msg.info.height         = self.n_cells
        msg.info.map_load_time  = now
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Row-major, C order: data[row * width + col] = grid[row, col]
        msg.data = self._grid.flatten().tolist()

        self.map_pub.publish(msg)


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
