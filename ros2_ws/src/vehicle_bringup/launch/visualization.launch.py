from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
import os
import yaml


def _launch_setup(context, *args, **kwargs):
    config_path = os.path.join(
        get_package_share_directory('vehicle_bringup'), 'config', 'bringup.yaml')
    with open(config_path, 'r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file) or {}
    port = config.get('bringup', {}).get('ros__parameters', {}).get(
        'foxglove_port', 8765)
    return [Node(package='foxglove_bridge', executable='foxglove_bridge',
                 name='foxglove_bridge', parameters=[{'port': port}], output='screen')]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_launch_setup)])
