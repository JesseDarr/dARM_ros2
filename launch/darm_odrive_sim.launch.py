from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

PKG = FindPackageShare("darm_ros2")

def generate_launch_description():
    urdf_cmd = Command([
        "xacro ",
        PathJoinSubstitution([PKG, "description", "darm.urdf.xacro"]),
        " sim:=odrive"
    ])
    robot_description = ParameterValue(urdf_cmd, value_type=str)
    controllers_yaml  = PathJoinSubstitution(
        [PKG, "config", "ros2_control_controllers.yaml"])

    rsp = Node(                             # ←— 1. robot-state-publisher
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description":           robot_description,
            "publish_robot_description":   True          # ←— must be True
        }]
    )

    ros2_control = Node(                   # ←— 2. controller manager
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controllers_yaml],
        output="screen"
    )

    mock = ExecuteProcess(                 # ←— 3. mock ODrive CAN node
        cmd=["ros2", "run", "darm_ros2", "mock_odrive.py"],
        output="screen")

    spawners = [
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"], output="screen"),
        Node(package="controller_manager", executable="spawner", arguments=["arm_controller"],          output="screen"),
    ]

    return LaunchDescription([
        rsp,
        ros2_control,
        mock,
        *spawners
    ])
