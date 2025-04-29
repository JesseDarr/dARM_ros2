import os
import xacro
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'darm'
    file_subpath = 'urdf/darm.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}]
    )

    # Configure initial state node
    node_darm_initial_state = Node(
        package='darm',
        executable='set_initial_joint_states.py',
        output='screen'
    )

    # Configure gazebo
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(get_package_share_directory(pkg_name), 'meshes')
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description', '-entity', 'darm'],
                    output='screen')

    # Run the nodes
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        node_darm_initial_state,
        spawn_entity
    ])