from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    transport = LaunchConfiguration('transport')
    port = LaunchConfiguration('port')
    return LaunchDescription([
        DeclareLaunchArgument('transport', default_value='udp4'),
        DeclareLaunchArgument('port', default_value='8888'),
        Node(package='micro_ros_agent', executable='micro_ros_agent',
             arguments=[transport, '--port', port], output='screen'),
    ])
