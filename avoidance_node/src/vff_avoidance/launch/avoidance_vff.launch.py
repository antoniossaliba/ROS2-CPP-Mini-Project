from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('vff_avoidance')
    config_file = os.path.join(pkg_path, 'config', 'AvoidanceNodeConfig.yaml')

    return LaunchDescription([
        Node(
            package='vff_avoidance',
            executable='avoidance_vff_node',
            name='avoidance_node',
            output='screen',
            parameters=[config_file]
        )
    ])
