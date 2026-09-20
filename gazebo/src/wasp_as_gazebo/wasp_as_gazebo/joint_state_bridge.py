#!/usr/bin/env python
"""Republishes /joint_states as /dynamic_joint_states.

wasp_as's encoders node reads wheel positions from
control_msgs/DynamicJointState on /dynamic_joint_states, which is what
ros2_control's joint_state_broadcaster publishes on the Webots path.
Gazebo has no ros2_control layer here: its JointStatePublisher system emits
a plain sensor_msgs/JointState on /joint_states instead. This converts one
into the other so encoders runs unmodified, the same way gps_bridge.py and
cmd_vel_watchdog.py cover the other gaps between the two simulators.

The joint order matters. encoders indexes interface_values positionally,
taking [1] as left and [0] as right, so they are published in that order
rather than in whatever order Gazebo happens to send them.
"""

import rclpy
from rclpy.node import Node

from control_msgs.msg import DynamicJointState, InterfaceValue
from sensor_msgs.msg import JointState

LEFT = 'wheel_left_joint'
RIGHT = 'wheel_right_joint'


class JointStateBridge(Node):

    def __init__(self):
        super().__init__('joint_state_bridge')
        self._pub = self.create_publisher(
            DynamicJointState, '/dynamic_joint_states', 10)
        self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self._warned = False

    def joint_state_callback(self, msg: JointState):
        try:
            left = msg.name.index(LEFT)
            right = msg.name.index(RIGHT)
        except ValueError:
            if not self._warned:
                self._warned = True
                self.get_logger().warn(
                    f'/joint_states has no {LEFT}/{RIGHT} (saw {list(msg.name)}); '
                    'not publishing /dynamic_joint_states.')
            return

        out = DynamicJointState()
        out.header = msg.header
        # Right first, then left - see the note above on positional indexing.
        out.joint_names = [RIGHT, LEFT]
        for index in (right, left):
            value = InterfaceValue()
            value.interface_names = ['position']
            value.values = [msg.position[index]]
            out.interface_values.append(value)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
