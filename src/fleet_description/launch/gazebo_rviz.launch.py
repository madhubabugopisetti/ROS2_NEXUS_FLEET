
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_path = get_package_share_directory("fleet_description")
    world_file = os.path.join(pkg_path, "worlds", "world.sdf")

    # Gazebo Harmonic
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen"
    )

    return LaunchDescription([
        gazebo
    ])
