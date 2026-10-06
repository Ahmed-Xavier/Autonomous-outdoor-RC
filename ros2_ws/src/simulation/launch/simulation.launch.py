import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory("simulation")
    desc_share = get_package_share_directory("vehicle_description")
    world = os.path.join(sim_share, "worlds", "empty.sdf")

    # lets Gazebo resolve package://vehicle_description/meshes/...
    resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.dirname(desc_share))

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, "launch", "description.launch.py")),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"),
                         "launch", "gz_sim.launch.py")),
        # -s = server only (no GUI), -r = run immediately
        launch_arguments={"gz_args": f"-r -s {world}"}.items(),
    )

    spawn = TimerAction(period=3.0, actions=[
        Node(package="ros_gz_sim", executable="create",
             arguments=["-topic", "robot_description", "-name", "enimia",
                        "-world", "empty", "-z", "0.1"],
             output="screen")])

    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/world/empty/model/enimia/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            "--ros-args", "-r",
            "/world/empty/model/enimia/joint_state:=/joint_states",
        ],
        output="screen")

    foxglove = Node(package="foxglove_bridge", executable="foxglove_bridge",
                    parameters=[{"port": 8765, "use_sim_time": True}],
                    output="screen")

    return LaunchDescription([resource_path, description, gazebo, spawn, bridge, foxglove])