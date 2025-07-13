# darm_test.launch.py
import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PKG         = 'darm_ros2'
URDF_XACRO  = os.path.join(get_package_share_directory(PKG), 'description', 'darm.urdf.xacro')
RVIZ_CONFIG = os.path.join(get_package_share_directory(PKG), 'rviz', 'darm.rviz')

def generate_launch_description():
    robot_description = xacro.process_file(URDF_XACRO).toxml()

    rsp_node = Node(
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': robot_description}]
    )

    jsp_node = Node(
        package    = 'joint_state_publisher',
        executable = 'joint_state_publisher',
        output     = 'screen'
    )

    rviz_node = Node(
        package    = 'rviz2',
        executable = 'rviz2',
        arguments  = ['-d', RVIZ_CONFIG],
        output     = 'screen',
        emulate_tty = True
    )

    return LaunchDescription([rsp_node, jsp_node, rviz_node])