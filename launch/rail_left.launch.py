import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Find the parameter file path
    config_file = os.path.join(
        get_package_share_directory('linear_rail_controller'),
        'config',
        'rail_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='linear_rail_controller',       
            executable='absolute_homer_left',       
            name='absolute_homer_left',             
            output='screen',
            parameters=[config_file]  # Loads the YAML parameters into the node
        )
    ])
