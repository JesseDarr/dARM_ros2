import os
import xacro
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'darm_ros2'
    file_subpath = 'description/darm.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure robot state publisher node
    node_robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen',
        parameters = [{'robot_description': robot_description_raw}]
    )

    # Configure initial state node
    node_darm_initial_state = Node(
        package = 'darm_ros2',
        executable = 'set_initial_joint_states.py',
        output = 'screen'
    )

    # Run the node
    return LaunchDescription([
        node_robot_state_publisher,
        node_darm_initial_state
    ])