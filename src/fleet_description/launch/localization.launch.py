from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    pkg = get_package_share_directory('fleet_description')

    map_file = PathJoinSubstitution([pkg, 'maps', 'my_map.yaml'])
    amcl_params = PathJoinSubstitution([pkg, 'config', 'amcl.yaml'])

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_file
        }]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[amcl_params]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    return LaunchDescription([
        map_server,
        amcl,
        lifecycle_manager
    ])
