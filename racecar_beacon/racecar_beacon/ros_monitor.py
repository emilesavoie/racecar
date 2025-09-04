#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import socket
import threading
from struct import pack

from racecar_beacon.utils import yaw_from_quaternion


class ROSMonitor(Node):
    def __init__(self):
        super().__init__("ros_monitor")

        # Robot state
        self.id = int(0xFFFF)
        self.position = tuple([float(0), float(0), float(0)])
        self.obstacle_detected = bool(False)

        # Socket parameters
        self.host = self.declare_parameter("host", "127.0.0.1").value
        self.remote_request_port = self.declare_parameter(
            "remote_request_port", 65432
        ).value
        self.broadcast = self.declare_parameter("broadcast", "127.0.0.255").value
        self.position_broad_port = self.declare_parameter(
            "pos_broadcast_port", 65431
        ).value

        self.remote_request_t = threading.Thread(target=self.remote_request_loop)

        # TODO: Add your subscription(s) here. DONE
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        self.remote_request_t.start()

        self.get_logger().info(f"{self.get_name()} started.")

        self.PositionBroadcast()

    def odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.position = (x, y, yaw)

    def scan_callback(self, msg: LaserScan) -> None:
        pass

    def remote_request_loop(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            s.bind((self.host, self.remote_request_port))
            s.listen(1)
            self.srv_sock  = socket
        except:

            return
        while rclpy.ok():
            pass

    # TODO: Implement the PositionBroadcast service here.
    # NOTE: It is recommended to initializae your socket locally.
    def PositionBroadcast(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind((self.broadcast, self.position_broad_port))
        self.create_timer(1.0, self.send_position(s))



        
    def send_position(self, socket):
        data = pack("Ifff", self.id, *self.position)
        socket.sendto(data, (self.broadcast, self.position_broad_port))
        socket.close()
        
    def shutdown(self):
        """Gracefully shutdown the threads BEFORE terminating the node."""
        self.remote_request_t.join()


def main(args=None):
    try:
        rclpy.init(args=args)
        node = ROSMonitor()
        rclpy.get_default_context().on_shutdown(node.shutdown())
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
