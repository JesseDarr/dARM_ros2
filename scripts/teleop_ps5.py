#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

class PS5Teleop(Node):
    def __init__(self):
        super().__init__('ps5_teleop')
        self.get_logger().info('PS5 Teleop ready')
        self.joy_sub = self.create_subscription(Joy, '/joy', self.handle_joy_input, 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        self.joint_names = [
            'joint_1_joint',
            'joint_2_joint',
            'joint_3_joint',
            'forearm_joint',
            'differential_joint',
            'gripper_joint'
        ]
        self.targets = [0.0] * len(self.joint_names)

    def handle_joy_input(self, msg: Joy):
        # 1) Dead-man’s switch: L1 (button index 4)
        if not msg.buttons[4]:
            return

        # 2) Incremental position step
        step = 0.1
        self.targets[0] += msg.axes[1] * step
        self.targets[1] += msg.axes[0] * step
        # clamp to your own URDF limits:
        self.targets[0] = max(min(self.targets[0],  3.14), -3.14)
        self.targets[1] = max(min(self.targets[1],  2.0),  -2.0)

        out = Float64MultiArray(data=self.targets)
        self.cmd_pub.publish(out)

def main(args=None):
    rclpy.init(args = args)
    node = PS5Teleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
