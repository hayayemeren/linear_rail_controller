import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('linear_rail_controller'),
        'config',
        'demo_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='linear_rail_controller',
            executable='demo',
            name='standalone_rail_demo',
            parameters=[config_file],
            output='screen'
        )
    ])
