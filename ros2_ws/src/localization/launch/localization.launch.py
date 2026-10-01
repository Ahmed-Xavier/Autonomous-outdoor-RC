from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('localization')
    local_cfg = os.path.join(share, 'config', 'ekf_local.yaml')
    global_cfg = os.path.join(share, 'config', 'ekf_global.yaml')
    navsat_cfg = os.path.join(share, 'config', 'navsat.yaml')
    use_gps = LaunchConfiguration('use_gps')

    return LaunchDescription([
        DeclareLaunchArgument('use_gps', default_value='true'),

        Node(
            package='localization',
            executable='imu_stamper.py',
            name='imu_stamper',
            prefix='python3',
            output='screen',
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_odom',
            parameters=[local_cfg],
            remappings=[('odometry/filtered', 'odometry/local')],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_map',
            parameters=[global_cfg],
            remappings=[('odometry/filtered', 'odometry/global')],
            condition=IfCondition(use_gps),
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            parameters=[navsat_cfg],
            condition=IfCondition(use_gps),
            remappings=[
                ('imu', '/imu/data'),
                ('gps/fix', '/fix'),
                ('odometry/filtered', 'odometry/global'),
                ('odometry/gps', '/odometry/gps'),
            ],
        ),
    ])