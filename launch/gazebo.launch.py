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
    ctrl_mngr  = os.path.join(get_package_share_directory(pkg_name), 'config', 'controllers.yaml')
    world_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'basic.sdf')
    model_path = os.path.join(get_package_share_directory(pkg_name), 'meshes')
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'description/darm.urdf.xacro')
    gz_plugins = os.path.join(get_package_share_directory('gz_ros2_control'), 'gz_hardware_plugins.xml')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # Configure robot state publisher node
    robot_state_publisher = Node(
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description':  robot_desc, 'use_sim_time': True}]
    )

    # Configure initial state node
    set_initial_state = Node(
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

    bridge = Node(
        package    = 'ros_gz_bridge',
        executable = 'parameter_bridge',
        output     = 'screen',
        arguments  = [
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/basic/model/darm_ros2/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings = [
            ('/world/basic/model/darm_ros2/joint_state', '/joint_states'),
        ]
    )

    # ——————————————————————————————————————————————————————
    # ROS 2 Control manager + controllers
    # ——————————————————————————————————————————————————————

    # 1) controller_manager node
    controller_manager = Node(
        package    = 'controller_manager',
        executable = 'ros2_control_node',
        parameters = [
            { 'robot_description': robot_desc, 'use_sim_time': True },
            { 'resource_manager.plugin_description': gz_plugins },
            ctrl_mngr,
        ],
        output     = 'screen'
    )

    # 2) spawn the joint_state_broadcaster
    spawn_jsb = Node(
        package    = 'controller_manager',
        executable = 'spawner',
        arguments  = ['joint_state_broadcaster'],
        output     = 'screen'
    )

    # 3) spawn your arm trajectory controller
    spawn_arm = Node(
        package    = 'controller_manager',
        executable = 'spawner',
        arguments  = ['arm_controller'],
        output     = 'screen'
    )

    # 4) spawn your finger group position controller
    spawn_finger = Node(
        package    = 'controller_manager',
        executable = 'spawner',
        arguments  = ['finger_controller'],
        output     = 'screen'
    )

    # Run the nodes
    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        robot_state_publisher,
        set_initial_state,
        spawn_entity,
        bridge,

        # ROS 2 Control
        controller_manager,
        spawn_jsb,
        spawn_arm,
        spawn_finger
    ]
)
