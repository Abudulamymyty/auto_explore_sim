#!/usr/bin/env python3
"""
orb_slam3_tf_bridge.py
======================
Converts ORB-SLAM3 camera pose to the Nav2-compatible map→odom TF.

Coordinate conventions
----------------------
ORB-SLAM3 uses **camera optical** convention (OpenCV / COLMAP):
    X  →  right    Y  ↓  down    Z  →  forward (optical axis)

ROS REP-103 convention (what Nav2 / the map frame expects):
    X  →  forward  Y  ←  left    Z  ↑  up

Two corrections are therefore applied inside this node:

  1. ORB-SLAM3 world → ROS map frame rotation  (R_ORB_TO_ROS):
       columns = ORB-world basis vectors expressed in ROS-map coordinates
       ORB Z (fwd)   →  ROS X  :  column [1, 0, 0]
       ORB X (right) →  ROS -Y :  column [0,-1, 0]
       ORB Y (down)  →  ROS -Z :  column [0, 0,-1]
       →  R_ORB_TO_ROS = [[0, 0, 1],
                          [-1, 0, 0],
                          [0,-1, 0]]

  2. Camera optical frame → camera_link (body) frame rotation  (R_OPT_TO_BODY):
       body X (fwd)   →  optical Z  :  column [0, 0, 1]
       body Y (left)  →  optical -X :  column [-1, 0, 0]
       body Z (up)    →  optical -Y :  column [0,-1, 0]
       →  R_OPT_TO_BODY = [[0,-1, 0],
                            [0, 0,-1],
                            [1, 0, 0]]

The combined transform applied to Twc (camera optical in ORB world):

    T(rosmap → camera_link) = R_ORB_TO_ROS  ×  Twc  ×  R_OPT_TO_BODY

From this, map→odom is computed as:

    T(map→odom) = T(map→camera_link) × T(camera_link→base_link)
                  × inv(T(odom→base_link))

Subscribed topics
-----------------
/orb_slam3/pose   geometry_msgs/PoseStamped
    Camera pose in ORB-SLAM3 world (optical convention) from the modified
    orbslam3 rgbd node.

TF published
------------
map → odom   (at the incoming pose rate, ~15 Hz)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf_transformations import quaternion_matrix, quaternion_from_matrix


# ---------------------------------------------------------------------------
# Constant coordinate-correction matrices (4×4 homogeneous, rotation-only)
# ---------------------------------------------------------------------------

# Converts ORB-SLAM3 world frame (Z fwd, X right, Y down)
# to ROS map frame (X fwd, Y left, Z up).
# Each column = one ORB-world basis in ROS-map coordinates.
_R_ORB_TO_ROS = np.array([
    [0,  0,  1, 0],   # ORB Z → ROS X
    [-1, 0,  0, 0],   # ORB X → ROS -Y
    [0, -1,  0, 0],   # ORB Y → ROS -Z
    [0,  0,  0, 1],
], dtype=float)

# Converts camera optical frame (Z fwd, X right, Y down)
# to camera body/link frame (X fwd, Y left, Z up).
# Each column = one body-frame basis in optical coordinates.
_R_OPT_TO_BODY = np.array([
    [0, -1,  0, 0],   # body X → optical Z  (column 0 of inverse)
    [0,  0, -1, 0],   # body Y → optical -X
    [1,  0,  0, 0],   # body Z → optical -Y
    [0,  0,  0, 1],
], dtype=float)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _pose_to_matrix(pose):
    """geometry_msgs/Pose → 4×4 numpy homogeneous transform."""
    q = pose.orientation
    m = quaternion_matrix([q.x, q.y, q.z, q.w])
    m[0, 3] = pose.position.x
    m[1, 3] = pose.position.y
    m[2, 3] = pose.position.z
    return m


def _transform_to_matrix(ts: TransformStamped):
    """TransformStamped → 4×4 numpy homogeneous transform."""
    t = ts.transform.translation
    r = ts.transform.rotation
    m = quaternion_matrix([r.x, r.y, r.z, r.w])
    m[0, 3] = t.x
    m[1, 3] = t.y
    m[2, 3] = t.z
    return m


def _matrix_to_transform_stamped(mat, stamp, frame_id, child_frame_id):
    """4×4 numpy matrix → TransformStamped."""
    ts = TransformStamped()
    ts.header.stamp     = stamp
    ts.header.frame_id  = frame_id
    ts.child_frame_id   = child_frame_id
    ts.transform.translation.x = float(mat[0, 3])
    ts.transform.translation.y = float(mat[1, 3])
    ts.transform.translation.z = float(mat[2, 3])
    q = quaternion_from_matrix(mat)
    ts.transform.rotation.x = float(q[0])
    ts.transform.rotation.y = float(q[1])
    ts.transform.rotation.z = float(q[2])
    ts.transform.rotation.w = float(q[3])
    return ts


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class OrbSlam3TfBridge(Node):
    """
    Subscribes to /orb_slam3/pose, applies coordinate corrections, and
    broadcasts the map→odom TF that Nav2 requires.
    """

    def __init__(self):
        super().__init__('orb_slam3_tf_bridge')

        self.declare_parameter('camera_frame',      'camera_link')
        self.declare_parameter('base_frame',        'base_link')
        self.declare_parameter('odom_frame',        'odom')
        self.declare_parameter('map_frame',         'map')
        self.declare_parameter('tf_lookup_timeout', 0.1)

        self._cam_frame  = self.get_parameter('camera_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._map_frame  = self.get_parameter('map_frame').value
        self._tf_timeout = Duration(
            seconds=self.get_parameter('tf_lookup_timeout').value)

        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_pub      = TransformBroadcaster(self)

        self.create_subscription(
            PoseStamped, '/orb_slam3/pose', self._pose_cb, 10)

        self.get_logger().info(
            f'orb_slam3_tf_bridge ready — '
            f'listening /orb_slam3/pose → broadcasting '
            f'{self._map_frame}→{self._odom_frame}')

    # ------------------------------------------------------------------
    def _pose_cb(self, msg: PoseStamped):
        """
        Receive Twc (camera optical in ORB-SLAM3 world) and publish map→odom.

        Steps
        -----
        1. Apply _R_ORB_TO_ROS × Twc × _R_OPT_TO_BODY
           → T(rosmap → camera_link)
        2. Look up T(camera_link → base_link)  [static, from URDF]
        3. Look up T(odom → base_link)          [from diff-drive odometry]
        4. T(map→odom) = T(map→cam_link) × T(cam_link→base) × inv(T(odom→base))
        """
        stamp = msg.header.stamp

        # ── 1. Twc in ORB-SLAM3 world, then corrected to rosmap/camera_link ──
        Twc = _pose_to_matrix(msg.pose)

        # Combined: T_rosmap_camlink = R_ORB_TO_ROS × Twc × R_OPT_TO_BODY
        T_map_cam = _R_ORB_TO_ROS @ Twc @ _R_OPT_TO_BODY

        # ── 2. T(camera_link → base_link) — static from robot_state_publisher ──
        try:
            ts_cam_base = self._tf_buffer.lookup_transform(
                self._cam_frame, self._base_frame,
                RclpyTime(), self._tf_timeout)
        except Exception as e:
            self.get_logger().warn(
                f'TF {self._cam_frame}→{self._base_frame} unavailable: {e}',
                throttle_duration_sec=5.0)
            return
        T_cam_base = _transform_to_matrix(ts_cam_base)

        # ── 3. T(odom → base_link) — from diff-drive odometry ──
        try:
            ts_odom_base = self._tf_buffer.lookup_transform(
                self._odom_frame, self._base_frame,
                RclpyTime(), self._tf_timeout)
        except Exception as e:
            self.get_logger().warn(
                f'TF {self._odom_frame}→{self._base_frame} unavailable: {e}',
                throttle_duration_sec=5.0)
            return
        T_odom_base = _transform_to_matrix(ts_odom_base)

        # ── 4. Compose map→odom ──
        # T(map→odom) = T(map→cam) × T(cam→base) × T(base→odom)
        T_map_odom = T_map_cam @ T_cam_base @ np.linalg.inv(T_odom_base)

        ts_out = _matrix_to_transform_stamped(
            T_map_odom, stamp, self._map_frame, self._odom_frame)
        self._tf_pub.sendTransform(ts_out)


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = OrbSlam3TfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
