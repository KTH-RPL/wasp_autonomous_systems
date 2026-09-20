#!/usr/bin/env python
"""Gazebo counterpart of webots_ros2_driver's Ros2MavicController plugin.

Assignment 4's exercise files (altitude_manual.py, altitude_pid.py) are
shared by both simulators and must not be edited for either one, so this
node presents them the exact same interface the Webots plugin does:

  in   /thrust   std_msgs/Float64MultiArray   [thrust] or [thrust, o1..o4]
  out  /<model>/command/motor_speed  actuator_msgs/Actuators

Everything between those two is a direct port of
src/webots_ros2_driver/src/plugins/dynamic/Ros2MavicController.cpp - the
same roll/pitch/yaw stabilization constants, the same thrust/offset mixing
into four motors, and the same "battery weakening" thrust disturbance. The
quadrotor model in worlds/course_quadrotor_world.sdf is tuned to the same
hover point as the Webots Mavic (~68.5), so a feedforward/gain set found
in one simulator transfers to the other and the assignment text needs no
per-simulator numbers.

Two things differ, unavoidably:
  - Attitude and body rates come from the bridged /imu topic rather than
    from Webots' noise-free inertial unit and gyro. The quadrotor's IMU is
    declared without noise in the world file precisely so this stays a fair
    substitute.
  - Webots calls step() on every physics tick; here the IMU messages are
    the clock. The world publishes the IMU at 125 Hz, matching Webots'
    basicTimeStep of 8 ms, so the 100-step drift interval below means the
    same 0.8 s in both.
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from actuator_msgs.msg import Actuators
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray

# Same constants as Ros2MavicController.cpp (which took them from the stock
# upstream mavic_driver.py, minus the cmd_vel terms this course doesn't use).
K_ROLL_P = 50.0
K_PITCH_P = 30.0
K_YAW_P = 2.0

DRIFT_STEP_INTERVAL = 100


def clamp(value, lo, hi):
    return min(max(value, lo), hi)


def roll_pitch_from_quaternion(q):
    """Roll and pitch only - yaw is never used, the yaw loop is rate-only."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(clamp(sinp, -1.0, 1.0))

    return roll, pitch


class MavicController(Node):

    def __init__(self):
        super().__init__('mavic_controller')

        self.declare_parameter('motor_speed_topic', '/quadrotor/command/motor_speed')
        topic = self.get_parameter(
            'motor_speed_topic').get_parameter_value().string_value

        self._motor_pub = self.create_publisher(Actuators, topic, 10)

        self.create_subscription(
            Float64MultiArray, 'thrust', self.thrust_callback, 10)
        self.create_subscription(
            Imu, '/imu', self.imu_callback, qos_profile_sensor_data)

        self._thrust = 0.0
        self._offsets = [0.0, 0.0, 0.0, 0.0]

        self._rng = random.Random()
        self._thrust_offset = self._rng.uniform(-0.5, 0.5)
        self._step_count = 0

    def thrust_callback(self, msg: Float64MultiArray):
        if len(msg.data) == 1:
            self._thrust = msg.data[0]
            self._offsets = [0.0, 0.0, 0.0, 0.0]
        elif len(msg.data) == 5:
            self._thrust = msg.data[0]
            self._offsets = list(msg.data[1:5])
        # any other size: ignore, keep the previous command

    def imu_callback(self, msg: Imu):
        roll, pitch = roll_pitch_from_quaternion(msg.orientation)
        roll_velocity = msg.angular_velocity.x
        pitch_velocity = msg.angular_velocity.y
        yaw_velocity = msg.angular_velocity.z

        roll_input = K_ROLL_P * clamp(roll, -1.0, 1.0) + roll_velocity
        pitch_input = K_PITCH_P * clamp(pitch, -1.0, 1.0) + pitch_velocity
        yaw_input = K_YAW_P * (0.0 - yaw_velocity)

        self._step_count += 1
        if self._step_count % DRIFT_STEP_INTERVAL == 0:
            self._thrust_offset += self._rng.uniform(-0.01, 0.0)
        effective_thrust = self._thrust + self._thrust_offset

        m1 = effective_thrust + self._offsets[0] + yaw_input + pitch_input + roll_input
        m2 = effective_thrust + self._offsets[1] - yaw_input + pitch_input - roll_input
        m3 = effective_thrust + self._offsets[2] - yaw_input - pitch_input + roll_input
        m4 = effective_thrust + self._offsets[3] + yaw_input - pitch_input - roll_input

        # Webots takes signed motor velocities and gets the spin direction
        # from the sign (-m1, m2, m3, -m4 for FR/FL/RR/RL). Gazebo's
        # MulticopterMotorModel takes unsigned rotor speeds and gets the
        # direction from each rotor's own <turningDirection>, set in the
        # world file to match that grouping (FR/RL one way, FL/RR the
        # other). So the only translation needed is dropping the sign, and
        # a negative command is a rotor that should simply stop.
        u = Actuators()
        u.header.stamp = msg.header.stamp
        u.velocity = [max(m, 0.0) for m in (m1, m2, m3, m4)]
        self._motor_pub.publish(u)


def main(args=None):
    rclpy.init(args=args)
    node = MavicController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Not guarded by rclpy.ok(): altitude_manual/altitude_pid do their own
    # braking landing on Ctrl+C and need this node alive to relay it, so
    # they are the ones that cut the thrust - this node just stops.
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
