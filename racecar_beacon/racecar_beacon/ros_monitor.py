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

        self.id = int(17220104 & 0xFFFFFFFF)
        self.position = tuple([float(0), float(0), float(0)])
        self.obstacle_detected = bool(False)

        # Socket parameters
        self.host = self.declare_parameter("host", "172.20.10.4").value
        self.remote_request_port = self.declare_parameter(
            "remote_request_port", 65432
        ).value
        self.broadcast = self.declare_parameter("broadcast", "172.20.10.15").value
        self.position_broad_port = self.declare_parameter(
            "pos_broadcast_port", 65431
        ).value

        self.broadcast_socket = None
        self.setup_broadcast_socket()

        # self.remote_request_t = threading.Thread(target=self.remote_request_loop)

        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        self.timer = self.create_timer(1.0, self.PositionBroadcast)

        # self.remote_request_t.start()

        self.get_logger().info(f"{self.get_name()} started.")

    def setup_broadcast_socket(self):
        """Initialize the broadcast socket"""
        try:
            self.broadcast_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Enable broadcast
            self.broadcast_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.get_logger().info("Broadcast socket initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize broadcast socket: {e}")
            self.broadcast_socket = None

    def odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        self.get_logger().info(f"Odometry received: x={x}, y={y}, yaw={yaw}")

        self.position = (x, y, yaw)

    def scan_callback(self, msg: LaserScan) -> None:
        pass

    # def remote_request_loop(self):
    #     s= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #     try:
    #         s.bind((self.host, self.remote_request_port))
    #         s.listen(1)
    #         self.srv_sock  = s
    #     except:
    #         return
    #     while rclpy.ok():
    #         pass

    def PositionBroadcast(self):
        if self.broadcast_socket is None:
            self.get_logger().warn("Broadcast socket not available")
            return
        try:
            data = pack("fffI", *self.position, self.id)
            self.broadcast_socket.sendto(data, (self.broadcast, self.position_broad_port))
            self.get_logger().debug(f"Broadcasted position: {self.position}")
        except Exception as e:
            self.get_logger().error(f"Failed to broadcast position: {e}")

    def shutdown(self):
        """Gracefully shutdown the threads and sockets BEFORE terminating the node."""
        if self.broadcast_socket:
            self.broadcast_socket.close()
        
        # Uncomment when you implement remote_request_t
        # if hasattr(self, 'remote_request_t'):
        #     self.remote_request_t.join()

def main(args=None):
    try:
        rclpy.init(args=args)
        node = ROSMonitor()
        # Fix: on_shutdown expects a callable, not the result of calling shutdown()
        rclpy.get_default_context().on_shutdown(node.shutdown)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()