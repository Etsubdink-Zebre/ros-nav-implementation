#!/usr/bin/env python3
"""Fake laser scan publisher for Gazebo Harmonic on WSL/llvmpipe.

Publishes synthetic LaserScan data that simulates walls around the robot.
This gives SLAM Toolbox enough features to initialize the map->odom
transform and allows Nav2 to start properly.

Without wall features, SLAM can't do scan matching and never publishes
the map frame, which prevents Nav2 from starting entirely.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# Simple hospital corridor walls relative to spawn point (-2, 3).
# These approximate the walls visible from the reception area.
# Format: (x1, y1, x2, y2) — line segments in the odom frame.
WALLS = [
    # Long corridor walls (north-south)
    (-4.0, -2.0, -4.0, 10.0),
    ( 0.0, -2.0,  0.0, 10.0),
    # Cross walls
    (-4.0, -2.0,  0.0, -2.0),
    (-4.0, 10.0,  0.0, 10.0),
    # Interior walls / doorframes
    (-4.0,  5.0, -2.5,  5.0),
    (-1.5,  5.0,  0.0,  5.0),
    # Right side rooms
    ( 0.0,  0.0,  6.0,  0.0),
    ( 6.0,  0.0,  6.0,  8.0),
    ( 0.0,  8.0,  6.0,  8.0),
]


def ray_segment_intersect(ox, oy, angle, x1, y1, x2, y2):
    """Return distance from (ox,oy) along ray at `angle` to line segment, or None."""
    dx = math.cos(angle)
    dy = math.sin(angle)
    sx = x2 - x1
    sy = y2 - y1

    denom = dx * sy - dy * sx
    if abs(denom) < 1e-9:
        return None

    t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom
    u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom

    if t > 0.01 and 0.0 <= u <= 1.0:
        return t
    return None


class FakeLaser(Node):
    def __init__(self):
        super().__init__('fake_laser')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('range_min', 0.12)
        self.declare_parameter('angle_min', -3.14159)
        self.declare_parameter('angle_max', 3.14159)
        self.declare_parameter('samples', 360)
        self.declare_parameter('rate', 5.0)
        self.declare_parameter('frame_id', 'base_scan')

        self.robot_x = -2.0  # spawn x
        self.robot_y = 3.0   # spawn y
        self.robot_yaw = 0.0

        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)

        period = 1.0 / self.get_parameter('rate').value
        self.timer = self.create_timer(period, self.publish_scan)
        self.get_logger().info(
            'Fake laser publisher started (with simulated walls)')

    def odom_cb(self, msg):
        """Track robot pose from odometry for raycasting."""
        self.robot_x = msg.pose.pose.position.x + (-2.0)  # offset by spawn
        self.robot_y = msg.pose.pose.position.y + 3.0
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def publish_scan(self):
        samples = self.get_parameter('samples').value
        range_min = self.get_parameter('range_min').value
        range_max = self.get_parameter('range_max').value
        angle_min = self.get_parameter('angle_min').value
        angle_max = self.get_parameter('angle_max').value

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.angle_min = angle_min
        msg.angle_max = angle_max
        msg.angle_increment = (angle_max - angle_min) / samples
        msg.range_min = range_min
        msg.range_max = range_max
        msg.time_increment = 0.0
        msg.scan_time = 0.0

        ranges = []
        for i in range(samples):
            angle = angle_min + i * msg.angle_increment + self.robot_yaw
            best = range_max
            for (x1, y1, x2, y2) in WALLS:
                d = ray_segment_intersect(
                    self.robot_x, self.robot_y, angle, x1, y1, x2, y2)
                if d is not None and d < best:
                    best = d
            if best < range_min:
                best = range_min
            ranges.append(best)

        msg.ranges = ranges
        msg.intensities = [100.0 if r < range_max else 0.0 for r in ranges]
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
