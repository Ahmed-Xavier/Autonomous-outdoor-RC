from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('vehicle_control')
    controller_config = os.path.join(share, 'config', 'vehicle_controller.yaml')
    state_config = os.path.join(share, 'config', 'vehicle_state.yaml')
    return LaunchDescription([
        Node(package='vehicle_control', executable='vehicle_controller',
             name='vehicle_controller', parameters=[controller_config]),
        Node(package='vehicle_control', executable='vehicle_state',
             name='vehicle_state', parameters=[state_config]),
    ])
