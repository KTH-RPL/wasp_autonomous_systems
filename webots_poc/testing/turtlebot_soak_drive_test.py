#!/usr/bin/env python3
"""Bounded, scriptable version of the collision test: drive forward for a
fixed duration, watch /imu for a clear collision spike, report PASS/FAIL and
health stats, then exit cleanly (no interactive kill needed)."""
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class DriveTest(Node):
    def __init__(self, duration, collision_threshold):
        super().__init__('soak_drive_test')
        self.create_subscription(Imu, '/imu', self.on_imu, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.x = 0.0
        self.max_acc = 0.0
        self.collision_x = None
        self.imu_count = 0
        self.last_imu_wall_time = None
        self.max_imu_gap = 0.0
        self.duration = duration
        self.collision_threshold = collision_threshold
        self.start = time.time()
        self.done = False
        self.timer = self.create_timer(1.0 / 20.0, self.tick)

    def on_odom(self, msg):
        self.x = msg.pose.pose.position.x

    def on_imu(self, msg):
        self.imu_count += 1
        now = time.time()
        if self.last_imu_wall_time is not None:
            gap = now - self.last_imu_wall_time
            self.max_imu_gap = max(self.max_imu_gap, gap)
        self.last_imu_wall_time = now

        a = msg.linear_acceleration
        mag = (a.x**2 + a.y**2 + (a.z - 9.81)**2) ** 0.5
        if mag > self.max_acc:
            self.max_acc = mag
        if mag > self.collision_threshold and self.collision_x is None:
            self.collision_x = self.x

    def tick(self):
        if time.time() - self.start > self.duration:
            self.done = True
            return
        t = Twist()
        t.linear.x = 0.3
        self.pub.publish(t)


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
    threshold = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0

    rclpy.init()
    node = DriveTest(duration, threshold)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)

    # stop
    stop = Twist()
    for _ in range(5):
        node.pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.05)

    result = "PASS" if node.collision_x is not None else "FAIL"
    node.get_logger().info(
        f'SOAK_DRIVE_RESULT={result} collision_x={node.collision_x} max_acc={node.max_acc:.2f} '
        f'final_x={node.x:.3f} imu_count={node.imu_count} max_imu_gap={node.max_imu_gap:.3f}s')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
