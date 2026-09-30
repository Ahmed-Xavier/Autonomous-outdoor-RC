from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('localization'), 'config', 'ekf.yaml')
    return LaunchDescription([
        Node(package='robot_localization', executable='ekf_node', name='ekf_odom',
             parameters=[cfg], remappings=[('odometry/filtered', 'odometry/local')]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_map',
             parameters=[cfg], remappings=[('odometry/filtered', 'odometry/global')]),
        Node(package='robot_localization', executable='navsat_transform_node',
             name='navsat_transform', parameters=[cfg],
             remappings=[('imu', '/esp32/imu'), ('gps/fix', '/fix'),
                         ('odometry/filtered', 'odometry/global'),
                         ('odometry/gps', '/odometry/gps')]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0','0','0','0','0','0','base_link','imu_link']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0','0','0','0','0','0','base_link','gps']),
    ])