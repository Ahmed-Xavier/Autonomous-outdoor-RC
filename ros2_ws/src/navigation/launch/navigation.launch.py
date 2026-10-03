from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('navigation'), 'config',
        'waypoint_manager.yaml')
    return LaunchDescription([
        Node(package='navigation', executable='waypoint_manager',
             name='waypoint_manager', parameters=[config]),
    ])
