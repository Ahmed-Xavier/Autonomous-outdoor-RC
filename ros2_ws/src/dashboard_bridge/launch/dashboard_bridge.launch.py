from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config = os.path.join(
        get_package_share_directory('dashboard_bridge'), 'config', 'dashboard_bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='dashboard_bridge', executable='track_progress',
             name='track_progress', parameters=[config, {'use_sim_time': use_sim_time}]),
        Node(package='dashboard_bridge', executable='fake_vehicle_status',
             name='fake_vehicle_status', parameters=[config, {'use_sim_time': use_sim_time}]),
    ])
