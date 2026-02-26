#!/usr/bin/env python3
"""
orb_slam3_tf_bridge.py
======================
Bridges ORB-SLAM3 camera pose output into the Nav2-compatible TF chain.

ORB-SLAM3 (via orb_slam3_ros2_wrapper) publishes the camera pose in the
*map* frame as a geometry_msgs/PoseStamped on /orb_slam3/pose.  Nav2 expects:

    map  →  odom  →  base_footprint / base_link  →  sensor frames

The diff-drive plugin already publishes odom → base_link.  The robot model
(robot_state_publisher) provides base_link → camera_link as a static TF.

This node computes and broadcasts:

    T(map → odom) = T(map → camera_link) × T(camera_link → base_link)
                     × inv( T(odom → base_link) )

i.e. it converts ORB-SLAM3's camera-centric pose into the standard SLAM
odometry correction transform that Nav2 reads.

Subscribed topics
-----------------
/orb_slam3/pose   geometry_msgs/PoseStamped
    Camera pose in the map frame published by orb_slam3_ros2_wrapper.

TF published
------------
map → odom   (at the same rate as the incoming pose messages)
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from rclpy.duration import Duration

import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf_transformations import (
    quaternion_matrix,
    quaternion_from_matrix,
    quaternion_inverse,
    quaternion_multiply,
)


def pose_to_matrix(pose):
    """Convert geometry_msgs/Pose to 4×4 numpy homogeneous transform."""
    q = pose.orientation
    mat = quaternion_matrix([q.x, q.y, q.z, q.w])
    mat[0, 3] = pose.position.x
    mat[1, 3] = pose.position.y
    mat[2, 3] = pose.position.z
    return mat


def matrix_to_transform_stamped(mat, stamp, frame_id, child_frame_id):
    """Convert 4×4 numpy matrix to a TransformStamped."""
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = frame_id
    t.child_frame_id = child_frame_id

    t.transform.translation.x = float(mat[0, 3])
    t.transform.translation.y = float(mat[1, 3])
    t.transform.translation.z = float(mat[2, 3])

    q = quaternion_from_matrix(mat)
    t.transform.rotation.x = float(q[0])
    t.transform.rotation.y = float(q[1])
    t.transform.rotation.z = float(q[2])
    t.transform.rotation.w = float(q[3])
    return t


def transform_stamped_to_matrix(ts):
    """Convert a TransformStamped to a 4×4 numpy homogeneous transform."""
    tr = ts.transform.translation
    rot = ts.transform.rotation
    mat = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
    mat[0, 3] = tr.x
    mat[1, 3] = tr.y
    mat[2, 3] = tr.z
    return mat


class OrbSlam3TfBridge(Node):
    """Converts ORB-SLAM3 camera pose → map→odom TF broadcast."""

    def __init__(self):
        super().__init__('orb_slam3_tf_bridge')

        # --- Parameters ---
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('tf_lookup_timeout', 0.1)

        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.tf_timeout = Duration(
            seconds=self.get_parameter('tf_lookup_timeout').value)

        # --- TF infrastructure ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Subscriber ---
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/orb_slam3/pose',
            self._pose_callback,
            10,
        )

        self.get_logger().info(
            f'orb_slam3_tf_bridge ready — '
            f'listening on /orb_slam3/pose, broadcasting {self.map_frame} → {self.odom_frame}'
        )

    # ------------------------------------------------------------------
    def _pose_callback(self, msg: PoseStamped):
        """
        Receive camera pose (map→camera) and broadcast map→odom.

        T(map→odom) = T(map→camera) × T(camera→base) × T(base→odom)
                    = T(map→camera) × T(camera→base) × inv(T(odom→base))
        """
        stamp = msg.header.stamp

        # 1. T(map → camera_link) — from ORB-SLAM3 pose message
        T_map_cam = pose_to_matrix(msg.pose)

        # 2. T(camera_link → base_link) — static, from robot_state_publisher
        try:
            ts_cam_base = self.tf_buffer.lookup_transform(
                self.camera_frame, self.base_frame, RclpyTime(), self.tf_timeout)
        except Exception as e:
            self.get_logger().warn(
                f'Cannot lookup {self.camera_frame}→{self.base_frame}: {e}',
                throttle_duration_sec=5.0)
            return
        T_cam_base = transform_stamped_to_matrix(ts_cam_base)

        # 3. T(odom → base_link) — from diff-drive odometry TF
        try:
            ts_odom_base = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, RclpyTime(), self.tf_timeout)
        except Exception as e:
            self.get_logger().warn(
                f'Cannot lookup {self.odom_frame}→{self.base_frame}: {e}',
                throttle_duration_sec=5.0)
            return
        T_odom_base = transform_stamped_to_matrix(ts_odom_base)

        # 4. Compose: T(map→odom)
        #    = T(map→cam) @ T(cam→base) @ inv(T(odom→base))
        T_base_odom = np.linalg.inv(T_odom_base)   # inv(T(odom→base)) = T(base→odom)
        T_map_odom = T_map_cam @ T_cam_base @ T_base_odom

        # 5. Broadcast
        ts_out = matrix_to_transform_stamped(
            T_map_odom, stamp, self.map_frame, self.odom_frame)
        self.tf_broadcaster.sendTransform(ts_out)


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
