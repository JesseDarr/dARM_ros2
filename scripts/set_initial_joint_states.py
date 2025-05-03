#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

def main():
    rclpy.init()

    node = Node('darm_initial_state')
    pub = node.create_publisher(JointState, '/joint_states', 10)

    # Create the joint state message
    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = [
        'joint_1_joint',
        'joint_2_joint',
        'joint_3_joint',
        'forearm_joint',
        'differential_joint',
        'gripper_joint',
        'finger_1_joint',
        'finger_2_joint'
    ]
    msg.position = [
        0.0,                    # joint_1_joint
        -113.0 * math.pi / 180, # joint_2_joint
        0.0,                    # joint_3_joint
        -122.9 * math.pi / 180, # forearm_joint
        0.0,                    # differential_joint
        0.0,                    # gripper_joint
        0.0,                    # finger_1_joint
        0.0                     # finger_2_joint
    ]

    # Publish a few times to make sure it's received
    for _ in range(10):
        pub.publish(msg)
        time.sleep(0.1)

    print("✅ Initial joint state published.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
