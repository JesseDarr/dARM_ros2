import os
import xacro
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable,IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Convience variables
    pkg_name   = 'darm_ros2'
    sim_name   = 'ros_gz_sim'
    sim_path   = os.path.join(get_package_share_directory(sim_name), 'launch', 'gz_sim.launch.py')
    rviz_path  = os.path.join(get_package_share_directory(pkg_name), 'rviz', 'darm.rviz')
    world_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'basic.sdf')
    model_path = os.path.join(get_package_share_directory(pkg_name), 'meshes')
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'description', 'darm.urdf.xacro')    
    robot_desc = xacro.process_file(xacro_file, mappings={'sim': 'true'}).toxml()

    # Robot State Publisher
    robot_state_publisher = Node(
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description':  robot_desc}, {'publish_robot_description': True}, {'use_sim_time': True}]
    )

    # Gazebo model path
    set_gazebo_model_path = SetEnvironmentVariable(
        name  = 'GAZEBO_MODEL_PATH',
        value = model_path
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_path),
        launch_arguments = { 'gz_args': f'-r -v4 {world_path}', 'on_exit_shutdown': 'true' }.items()
    )

    # Spawn Entity
    spawn_entity = Node(
        package    = 'ros_gz_sim',
        executable = 'create',
        arguments  = ['-world', 'basic', '-topic', 'robot_description', '-name', pkg_name],
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

    # RVIZ2
    rviz = Node(
        package    = 'rviz2',
        executable = 'rviz2',
        arguments  = ['-d', rviz_path],
        output     = 'screen',
        emulate_tty = True
    )

    # Run the nodes
    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        robot_state_publisher,
        bridge,
        spawn_entity,
        rviz,

        # Joystick + Teleop
        joy,
        teleop,

        # ROS 2 Control
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[*spawner_nodes]
            )
        ),
    ])