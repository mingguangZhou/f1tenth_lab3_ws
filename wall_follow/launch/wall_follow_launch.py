import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_share_dir = get_package_share_directory('wall_follow')
    config_file = os.path.join(package_share_dir, 'config', 'pid_params.yaml')

    return LaunchDescription([
        Node(
            package='wall_follow',
            executable='wall_follow_node',
            name='wall_follow_node',
            output='screen',
            parameters=[config_file]  # Use the constructed absolute path
        )
    ])