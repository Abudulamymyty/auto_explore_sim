#!/usr/bin/env python3
"""
Moves the 'moving_obstacle' model back and forth along the corridor.

Publishes geometry_msgs/Twist on the ROS 2 topic
  /model/moving_obstacle/cmd_vel
which is bridged to the Gazebo VelocityControl plugin via ros_gz_bridge.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MovingObstacleController(Node):
    def __init__(self):
        super().__init__('moving_obstacle_controller')

        # ---------- parameters ----------
        self.declare_parameter('speed', 0.4)          # peak linear speed (m/s)
        self.declare_parameter('period', 20.0)         # full back-and-forth cycle (s)
        self.declare_parameter('update_rate', 10.0)    # Hz

        self.speed = self.get_parameter('speed').value
        self.period = self.get_parameter('period').value
        rate = self.get_parameter('update_rate').value

        # ---------- publisher ----------
        self.pub = self.create_publisher(
            Twist, '/model/moving_obstacle/cmd_vel', 10)

        # ---------- timer ----------
        self.dt = 1.0 / rate
        self.elapsed = 0.0
        self.timer = self.create_timer(self.dt, self._timer_cb)

        self.get_logger().info(
            f'Moving obstacle controller started  '
            f'(speed={self.speed:.2f} m/s, period={self.period:.1f} s)')

    # ------------------------------------------------------------------
    def _timer_cb(self):
        self.elapsed += self.dt

        # Sinusoidal velocity → smooth back-and-forth patrol
        vx = self.speed * math.sin(2.0 * math.pi * self.elapsed / self.period)

        msg = Twist()
        msg.linear.x = vx
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MovingObstacleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the obstacle before shutting down
        stop = Twist()
        node.pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
