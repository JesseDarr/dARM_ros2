#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class PS5Teleop(Node):
    def __init__(self):
        super().__init__('ps5_teleop')
        self.get_logger().info('PS5 Teleop ready')
        self.joy_sub = self.create_subscription(Joy, '/joy', self.handle_joy_input, 10)
        self.traj_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # match exactly your arm_controller joints
        self.joint_names = [
            'joint_1_joint',
            'joint_2_joint',
            'joint_3_joint',
            'forearm_joint',
            'differential_joint',
            'gripper_joint'
        ]

    def handle_joy_input(self, msg: Joy):
        jt = JointTrajectory()
        jt.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        # left stick X/Y → first two joints; rest zero
        x = msg.axes[0] * 1.0
        y = msg.axes[1] * 1.0
        pt.positions = [x, y, 0.0, 0.0, 0.0, 0.0]
        pt.time_from_start.sec = 0

        jt.points = [pt]
        self.traj_pub.publish(jt)

def main(args=None):
    rclpy.init(args = args)
    node = PS5Teleop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
