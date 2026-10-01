from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def _include(package, launch_file, arguments=None):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path), launch_arguments=(arguments or {}).items())


def generate_launch_description():
    return LaunchDescription([
        _include('vehicle_description', 'description.launch.py'),
        _include('vehicle_drivers', 'microros_agent.launch.py'),
        _include('localization', 'localization.launch.py', {'use_gps': 'false'}),
        _include('vehicle_control', 'vehicle_control.launch.py'),
        _include('vehicle_bringup', 'visualization.launch.py'),
    ])
