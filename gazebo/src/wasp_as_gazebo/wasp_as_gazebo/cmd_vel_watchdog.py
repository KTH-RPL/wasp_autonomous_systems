#!/usr/bin/env python
"""Stops the robot when /cmd_vel goes stale, the way ros2_control does.

The Webots path drives the Turtlebot through ros2_control's
diff_drive_controller, which has a `cmd_vel_timeout` (0.5 s by default):
if no fresh command arrives inside that window it halts the wheels. Gazebo's
DiffDrive system has no equivalent - checked against the parameters in
libgz-sim8-diff-drive-system, which has max_linear_acceleration,
max_linear_velocity, odom_topic and topic, and nothing resembling a timeout
- so it keeps applying the last command forever.

That difference is visible immediately in teleop: `ass_1_1_teleop` runs
teleop_twist_keyboard with key_timeout/repeat_rate at 0, so one keypress
sends exactly one message. Under Webots the robot creeps and stops; under
Gazebo it drives away and never comes back.

Fixing it here rather than in the teleop task is deliberate. The behaviour
that differs is the *controller's*, not the keyboard's, so anything that
publishes a single /cmd_vel - including a node a student writes later -
should see the same thing in both simulators.

The watchdog counts its own zero commands as traffic, so once the robot is
stopped it re-sends a zero every timeout rather than spinning. A real
command always wins: it resets the timer and the robot has a full timeout
to move before the next zero.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped


class CmdVelWatchdog(Node):

    def __init__(self):
        super().__init__('cmd_vel_watchdog')

        # 0.5 s is diff_drive_controller's own default for cmd_vel_timeout,
        # which is what the Webots path runs with (ros2control.yaml does not
        # override it).
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        timeout = self.get_parameter(
            'cmd_vel_timeout').get_parameter_value().double_value
        topic = self.get_parameter(
            'cmd_vel_topic').get_parameter_value().string_value

        self._pub = self.create_publisher(TwistStamped, topic, 10)
        self.create_subscription(TwistStamped, topic, self.on_cmd_vel, 10)

        self._timeout = timeout
        self._last = self.get_clock().now()
        # Checked several times per timeout so the stop lands close to the
        # deadline rather than up to a whole timeout late.
        self.create_timer(timeout / 5.0, self.check)

    def on_cmd_vel(self, msg: TwistStamped):
        self._last = self.get_clock().now()

    def check(self):
        elapsed = (self.get_clock().now() - self._last).nanoseconds / 1e9
        if elapsed < self._timeout:
            return
        stop = TwistStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(stop)
        self._last = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
