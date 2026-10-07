import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory("simulation")
    desc_share = get_package_share_directory("vehicle_description")

    # Silesia Ring world
    world = os.path.join(sim_share, "worlds", "silesia_ring.sdf")

    # Gazebo resource paths:
    # - vehicle_description for package://vehicle_description/...
    # - worlds/models for model://silesia_ring
    resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.pathsep.join([
            os.path.dirname(desc_share),
            os.path.join(sim_share, "worlds", "models"),
        ])
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, "launch", "description.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Gazebo with GUI
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        # -r = run immediately
        # No -s, so the Gazebo GUI is launched.
        launch_arguments={"gz_args": f"-r {world}"}.items(),
    )

    spawn = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-topic", "robot_description",
                    "-name", "enimia",
                    "-world", "silesia_ring",
                    "-z", "0.1",
                ],
                output="screen",
            )
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/world/silesia_ring/model/enimia/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",

            # Ground-truth odometry and TF
            "/enimia/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/enimia/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",

            # Drive and steering commands, ROS -> Gazebo
            "/enimia/rear_wheel_cmd@std_msgs/msg/Float64]gz.msgs.Double",
            "/enimia/steer_left_cmd@std_msgs/msg/Float64]gz.msgs.Double",
            "/enimia/steer_right_cmd@std_msgs/msg/Float64]gz.msgs.Double",

            "--ros-args",
            "-r", "/world/silesia_ring/model/enimia/joint_state:=/joint_states",
            "-r", "/enimia/odom:=/odom",
            "-r", "/enimia/tf:=/tf",
        ],
        output="screen",
    )

    # Foxglove bridge
    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        parameters=[
            {
                "port": 8765,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    # World visualization markers
    world_markers = Node(
        package="simulation",
        executable="world_markers.py",
        parameters=[
            {
                "frame_id": "map",
            }
        ],
        output="screen",
    )

    # Real vehicle controller:
    # /cmd_vel -> /esp32/motor_cmd + /esp32/servo_cmd
    vehicle_controller = Node(
        package="vehicle_control",
        executable="vehicle_controller",
        name="vehicle_controller",
        parameters=[
            os.path.join(
                get_package_share_directory("vehicle_control"),
                "config",
                "vehicle_controller.yaml",
            )
        ],
        output="screen",
    )

    # Simulated ESP32:
    # /esp32/* -> Gazebo joints
    esp32_sim_bridge = Node(
        package="simulation",
        executable="esp32_sim_bridge.py",
        output="screen",
    )

    return LaunchDescription([
        resource_path,
        description,
        gazebo,
        spawn,
        bridge,
        foxglove,
        world_markers,
        vehicle_controller,
        esp32_sim_bridge,
    ])