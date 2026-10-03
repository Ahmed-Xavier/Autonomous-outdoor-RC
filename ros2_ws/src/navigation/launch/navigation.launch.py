from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def _launch_setup(context, *args, **kwargs):
    share = get_package_share_directory('navigation')
    manager_config = os.path.join(share, 'config', 'waypoint_manager.yaml')
    gate_config = os.path.join(share, 'config', 'cmd_vel_gate.yaml')
    planner_config = os.path.join(share, 'config', 'local_planner.yaml')
    use_gate = LaunchConfiguration('use_lidar_gate').perform(context).lower() == 'true'
    use_planner = LaunchConfiguration('use_local_planner').perform(context).lower() == 'true'
    if use_planner and not use_gate:
        raise ValueError('use_local_planner requires use_lidar_gate:=true')

    manager = Node(package='navigation', executable='waypoint_manager',
                   name='waypoint_manager', parameters=[manager_config])
    if use_gate:
        manager.remappings = [('/cmd_vel', '/cmd_vel_nav')]
    actions = [manager]
    if use_planner:
        actions.append(Node(package='navigation', executable='local_planner',
                            name='local_planner', parameters=[planner_config]))
    if use_gate:
        gate_params = [gate_config]
        if use_planner:
            gate_params.append({'input_topic': '/cmd_vel_plan'})
        actions.append(Node(package='navigation', executable='cmd_vel_gate',
                            name='cmd_vel_gate', parameters=gate_params))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_lidar_gate', default_value='true',
                              description='Gate all motion through the /scan safety node'),
        DeclareLaunchArgument('use_local_planner', default_value='false',
                              description='Insert the local obstacle planner before the gate'),
        OpaqueFunction(function=_launch_setup),
    ])
