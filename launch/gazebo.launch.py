import os
import xacro
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Setup convience variables
    pkg_name   = 'darm_ros2'
    sim_name   = 'ros_gz_sim'    
    sim_path   = os.path.join(get_package_share_directory(sim_name), 'launch', 'gz_sim.launch.py')
    world_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'basic.sdf')
    model_path = os.path.join(get_package_share_directory(pkg_name), 'meshes')
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'description/darm.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # Configure robot state publisher node
    node_robot_state_publisher = Node(
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description':  robot_desc, 'use_sim_time': True}]
    )

    # Configure initial state node
    node_darm_initial_state = Node(
        package    = pkg_name,
        executable = 'set_initial_joint_states.py',
        output     = 'screen'
    )

    # Configure gazebo
    set_gazebo_model_path = SetEnvironmentVariable(
        name  = 'GAZEBO_MODEL_PATH',
        value = model_path
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_path),
        launch_arguments = {'gz_args': f'-r -v4 {world_path}', 'on_exit_shutdown': 'true'}.items()
    )

    spawn_entity = Node(
        package    = 'ros_gz_sim',
        executable = 'create',
        arguments  = ['-world', 'basic',
                      '-topic', 'robot_description',
                      '-name',  pkg_name],
        output     = 'screen'
    )

    # Run the nodes
    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        node_robot_state_publisher,
        node_darm_initial_state,
        spawn_entity
    ])
