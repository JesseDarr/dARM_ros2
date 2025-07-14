# darm_test.launch.py
import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Convience variables
    pkg_name   = 'darm_ros2'
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'description', 'darm.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # Robot State Publisher
    robot_state_publisher = Node(
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': robot_desc}]
    )

    # Ros2_Control - Controllers
    controller_names = ['joint_state_broadcaster', 'arm_controller', ]
    spawner_nodes    = [
        Node(
            package    = 'controller_manager',
            executable = 'spawner',
            arguments  = [name],
            output     = 'screen'
        ) for name in controller_names
    ]

    # Joy Stick
    joy = Node(
        package    = 'joy',
        executable = 'joy_node',
        name       = 'joy_node',
        output     = 'screen',
        parameters = [{
            'device': '/dev/input/event0',
            'deadzone': 0.05,
            'autorepeat_rate': 30.0
        }]
    )

    # Teleop
    teleop = Node(
        package     = 'darm_ros2',
        executable  = 'teleop_ps5.py',
        name        = 'teleop_ps5',
        output      = 'screen',
        emulate_tty = True,
        arguments   = [
            '--ros-args',                
            '--log-level', 'teleop_ps5:=debug'  
        ],
    )

    # Run the nodes
    return LaunchDescription([
        robot_state_publisher, 
        joy,
        teleop,
        spawner_nodes
    ])