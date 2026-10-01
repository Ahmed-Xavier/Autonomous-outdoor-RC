from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('vehicle_drivers'), 'config', 'gps.yaml')
    return LaunchDescription([
        Node(package='nmea_navsat_driver', executable='nmea_serial_driver',
             name='nmea_serial_driver', parameters=[config]),
    ])
