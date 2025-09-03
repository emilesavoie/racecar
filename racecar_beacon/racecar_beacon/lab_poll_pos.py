#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf_transformations import euler_from_quaternion

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

from racecar_beacon.utils import yaw_from_quaternion


class PositionPoller(Node):
    def __init__(self):
        super().__init__("position_poller")
        self.subscriptions = self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self.odom_callback,
            10,
        )
        self.get_logger().info(f"{self.get_name()} started.")

    def odom_callback(self, msg: Odometry) -> None:
        self.get_logger().info(f"Received odometry data: {msg}")

def main(args=None):
    try:
        rclpy.init(args=args)
        node = PositionPoller()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()
