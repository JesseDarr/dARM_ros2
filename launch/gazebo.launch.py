import os
import xacro
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Setup convience variables
    pkg_name   = 'darm_ros2'
    sim_name   = 'ros_gz_sim'    
    sim_path   = os.path.join(get_package_share_directory(sim_name), 'launch', 'gz_sim.launch.py')
    ctrl_mngr  = os.path.join(get_package_share_directory(pkg_name), 'config', 'controllers.yaml')
    world_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'basic.sdf')
    model_path = os.path.join(get_package_share_directory(pkg_name), 'meshes')
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'description/darm.urdf.xacro')
    gz_plugins = os.path.join(get_package_share_directory('gz_ros2_control'), 'gz_hardware_plugins.xml')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # Robot State Publisher
    robot_state_publisher = Node(
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description':  robot_desc, 'publish_robot_description': True, 'use_sim_time': True}]
    )

    # Initial State Node - 1 shot
    set_initial_state = Node(
        package    = pkg_name,
        executable = 'set_initial_joint_states.py',
        output     = 'screen'
    )

    # Gazebo model path
    set_gazebo_model_path = SetEnvironmentVariable(
        name  = 'GAZEBO_MODEL_PATH',
        value = model_path
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_path),
        launch_arguments = {'gz_args': f'-r -v4 {world_path}', 'on_exit_shutdown': 'true'}.items()
    )

    # Spawn Entity
    spawn_entity = Node(
        package    = 'ros_gz_sim',
        executable = 'create',
        arguments  = ['-world', 'basic',
                      '-topic', 'robot_description',
                      '-name',  pkg_name],
        output     = 'screen'
    )

    # Gazebo to ROS2 Bridge
    bridge = Node(
        package    = 'ros_gz_bridge',
        executable = 'parameter_bridge',
        output     = 'screen',
        arguments  = [
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/basic/model/darm_ros2/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/robot_description@std_msgs/msg/String[gz.msgs.StringMsg',
        ],
        remappings = [('/world/basic/model/darm_ros2/joint_state', '/joint_states'),]
    )

    # ROS2_Control - Manager
    controller_manager = Node(
        package    = 'controller_manager',
        executable = 'ros2_control_node',
        parameters = [
            { 'use_sim_time': True },
            { 'resource_manager.plugin_description': gz_plugins },
            ctrl_mngr,
        ],
        output     = 'screen'
    )

    # Ros2_Control - Other Nodes
    controller_names = ['joint_state_broadcaster', 'arm_controller', 'finger_controller',]
    spawner_nodes = [
        Node(
            package    = 'controller_manager',
            executable = 'spawner',
            arguments  = [name],
            output     = 'screen'
        ) for name in controller_names
    ]

    # Run the nodes
    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        robot_state_publisher,
        set_initial_state,
        bridge,
        spawn_entity,

        # ROS 2 Control
        controller_manager,
        *spawner_nodes
    ]
)
