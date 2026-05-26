#!/usr/bin/env python3
"""Fake laser scan publisher for Gazebo Harmonic on WSL/llvmpipe.
Publishes synthetic LaserScan data when GPU lidar can't render."""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FakeLaser(Node):
    def __init__(self):
        super().__init__('fake_laser')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('range_min', 0.3)
        self.declare_parameter('angle_min', -3.14)
        self.declare_parameter('angle_max', 3.14)
        self.declare_parameter('samples', 360)
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('frame_id', 'base_scan')

        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        period = 1.0 / self.get_parameter('rate').value
        self.timer = self.create_timer(period, self.publish_scan)
        self.get_logger().info('Fake laser publisher started')

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.angle_min = self.get_parameter('angle_min').value
        msg.angle_max = self.get_parameter('angle_max').value
        msg.angle_increment = (msg.angle_max - msg.angle_min) / self.get_parameter('samples').value
        msg.range_min = self.get_parameter('range_min').value
        msg.range_max = self.get_parameter('range_max').value
        msg.time_increment = 0.0
        msg.scan_time = 0.0

        # All ranges at max (no obstacles detected) — SLAM will map open space
        samples = self.get_parameter('samples').value
        msg.ranges = [msg.range_max] * samples
        msg.intensities = [0.0] * samples

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeLaser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
