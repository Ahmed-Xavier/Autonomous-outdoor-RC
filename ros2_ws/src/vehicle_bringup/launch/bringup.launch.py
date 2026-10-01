from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
import yaml


def _include(package, launch_file, arguments=None):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path), launch_arguments=(arguments or {}).items())


def _launch_setup(context, *args, **kwargs):
    config_path = os.path.join(
        get_package_share_directory('vehicle_bringup'), 'config', 'bringup.yaml')
    with open(config_path, 'r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file) or {}
    yaml_debug = bool(config.get('bringup', {}).get('ros__parameters', {}).get('debug', False))
    override = LaunchConfiguration('debug').perform(context).strip().lower()
    if override and override not in ('true', 'false'):
        raise ValueError("'debug' must be 'true', 'false', or empty")
    debug = yaml_debug if not override else override == 'true'

    actions = [
        _include('vehicle_description', 'description.launch.py'),
        _include('vehicle_drivers', 'gps.launch.py'),
        _include('vehicle_drivers', 'microros_agent.launch.py'),
        _include('localization', 'localization.launch.py'),
        _include('vehicle_control', 'vehicle_control.launch.py'),
        _include('navigation', 'navigation.launch.py'),
    ]
    if debug:
        actions.append(_include('vehicle_bringup', 'visualization.launch.py'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='',
                              description='Override bringup.yaml debug setting'),
        OpaqueFunction(function=_launch_setup),
    ])
