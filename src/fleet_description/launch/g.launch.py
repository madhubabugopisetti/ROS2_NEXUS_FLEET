from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_path = get_package_share_directory("fleet_description")
    world_file = os.path.join(pkg_path, "worlds", "world.sdf")
    xacro_file = os.path.join(pkg_path, "urdf", "nexus_fleet.xacro")
    rviz_file = os.path.join(pkg_path, "rviz", "fleet.rviz")

    # Gazebo Harmonic
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen"
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": Command(["xacro ", xacro_file])
        }],
        output="screen"
    )

    # Spawn robot from /robot_description
    spawn_robot = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-name", "four_wheel_diff_robot",
                "-topic", "robot_description"
            ],
            output="screen"
        )]
    )

    # --- bridge ---
    bridge = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/arm_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/base_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        output="screen"
    )

    load_jsb = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"],
        output="screen"
    )

    load_arm = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "arm_controller"],
        output="screen"
    )

    delayed_load = TimerAction(
        period=12.0,
        actions=[load_jsb, load_arm]
    )

    # --- RViz ---
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file],
        parameters=[{"use_sim_time": True}],
        output="screen"
    )
    return LaunchDescription([
        gazebo,
        bridge,
        robot_state_publisher,
        spawn_robot,
        delayed_load,
        rviz
    ])
