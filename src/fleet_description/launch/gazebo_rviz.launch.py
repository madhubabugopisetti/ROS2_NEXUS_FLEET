from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_path = get_package_share_directory("fleet_description")

    world_file = os.path.join(pkg_path, "worlds", "world.sdf")
    xacro_file = os.path.join(pkg_path, "urdf", "world.xacro")

    # Start Gazebo Harmonic
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen"
    )

    # Robot description
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str
    )

    # Robot State Publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}]
    )

    # Spawn robot into Gazebo (CORRECT WAY for Jazzy + Harmonic)
    spawn_robot = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-name", "nexus_with_arm",
                "-topic", "robot_description"
            ],
            output="screen"
        )]
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn_robot,
    ])
