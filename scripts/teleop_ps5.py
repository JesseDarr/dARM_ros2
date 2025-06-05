#!/usr/bin/env python3
import math, rclpy, xml.etree.ElementTree as ET
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

def _pretty_limits(lim_dict):
    """Return a multi-line string with one joint per line."""
    lines = []
    for j, (lo, hi) in lim_dict.items():
        lines.append(f'  {j:<20} : {lo:8.3f}  .. {hi:8.3f}')
    return '\n'.join(lines)

def get_urdf_limits(node: Node, joint_names):
    """
    Return a dict {joint_name: (lower, upper)} for the requested joints.
    """
    client = AsyncParameterClient(node, 'robot_state_publisher')
    future = client.get_parameters(['robot_description'])
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)

    if not future.done() or not future.result():
        node.get_logger().error('Could not fetch robot_description')
        return {j: (-math.inf, math.inf) for j in joint_names}

    urdf_xml = future.result().values[0].string_value
    if not urdf_xml:
        node.get_logger().error('robot_description is empty')
        return {j: (-math.inf, math.inf) for j in joint_names}

    limits = {}
    for joint in ET.fromstring(urdf_xml).iter('joint'):
        name = joint.attrib.get('name')
        if name in joint_names:
            lim = joint.find('limit')
            if lim is not None:
                lo = float(lim.attrib.get('lower', '-inf'))
                hi = float(lim.attrib.get('upper',  'inf'))
                limits[name] = (lo, hi)

    # fallback for any missing joints
    for j in joint_names:
        limits.setdefault(j, (-math.inf, math.inf))

    node.get_logger().debug('\nParsed limits:\n' + _pretty_limits(limits))
    return limits

class PS5Teleop(Node):
    def __init__(self):
        super().__init__('ps5_teleop')

        # Joint list (order matters – must match controller)
        self.joint_names = [
            'joint_1_joint', 'joint_2_joint', 'joint_3_joint', 'forearm_joint',
            'differential_joint', 'gripper_joint', 'finger_1_joint', 'finger_2_joint'
        ]

        # Pull the limits from the live URDF
        self.limits = get_urdf_limits(self, self.joint_names)

        # Runtime state
        self.targets     = [0.0] * len(self.joint_names)
        self.step        = 0.1            # radians or metres per tick
        self.deadman_btn = 4              # L1 on DualSense

        # I/O
        self.joy_sub = self.create_subscription(Joy, '/joy', self.handle_joy_input, 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)

        self.get_logger().info('PS5 Tele-op ready')

    def clamp(self, idx, value):
        lo, hi = self.limits[self.joint_names[idx]]
        return min(max(value, lo), hi)

    def handle_joy_input(self, msg: Joy):
        # Dead-man switch
        if not msg.buttons[self.deadman_btn]:
            return

        # Map left stick to joints 0 & 1 (demo)
        #self.targets[0] = self.clamp(0, self.targets[0] + msg.axes[1] * self.step)
        #self.targets[1] = self.clamp(1, self.targets[1] + msg.axes[0] * self.step)
        #self.targets[2] = self.clamp(2, self.targets[2] + msg.axes[4] * self.step)
        self.targets[3] = self.clamp(3, self.targets[3] + msg.axes[3] * self.step)

        self.cmd_pub.publish(Float64MultiArray(data=self.targets))

def main(args=None):
    rclpy.init(args=args)
    node = PS5Teleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
