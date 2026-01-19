from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,

            # Frames (MATCHES YOUR XACRO)
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',

            # Topics (MATCHES YOUR BRIDGE)
            'scan_topic': '/scan',

            # SLAM behavior
            'mode': 'mapping',
            'resolution': 0.05,
            'max_laser_range': 10.0,

            # Performance
            'transform_publish_period': 0.05,
            'map_update_interval': 2.0,

            # Scan matching
            'use_scan_matching': True,
            'use_scan_barycenter': True,

            # Loop closure
            'do_loop_closing': True,
            'loop_search_maximum_distance': 3.0,
        }]
    )

    return LaunchDescription([
        slam_toolbox
    ])
