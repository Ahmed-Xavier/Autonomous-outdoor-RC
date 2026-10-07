import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory("simulation")
    desc_share = get_package_share_directory("vehicle_description")
    nav_share = get_package_share_directory("navigation")
    loc_share = get_package_share_directory("localization")
    ctrl_share = get_package_share_directory("vehicle_control")

    # Silesia Ring world
    world = os.path.join(sim_share, "worlds", "silesia_ring.sdf")

    # sim_vehicle.yaml overrides the RC-car footprint/wheelbase with ENIMIA dims
    sim_vehicle_cfg = os.path.join(sim_share, "config", "sim_vehicle.yaml")

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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        # -s = server only, -r = run immediately, --headless-rendering = EGL
        # context for gpu_lidar without a display (required in WSL headless).
        launch_arguments={"gz_args": f"-r -s --headless-rendering {world}"}.items(),
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

    # ------------------------------------------------------------------ #
    # ros_gz_bridge: one parameter_bridge instance for all topics.
    #
    # Notation:
    #   topic@ROS_type[gz_type   →  Gazebo → ROS  (sensor data)
    #   topic@ROS_type]gz_type   →  ROS → Gazebo  (commands)
    #
    # Sensor remaps (gz topic → ROS topic the stack expects):
    #   /lidar_link/scan       → /scan
    #   /imu_link/imu          → /imu/data
    #   /gps_link/navsat       → /fix
    #
    # Ground-truth odometry remaps:
    #   /enimia/odom           → /odom   (used by Foxglove and as EKF input)
    #   /enimia/tf             → /tf
    #
    # /odom is also relayed to /wheel/odom by the relay node below so the
    # EKF local filter gets a velocity source without needing the encoder.
    # ------------------------------------------------------------------ #
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # Core / clock
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # Joint states (for URDF animation in Foxglove)
            "/world/silesia_ring/model/enimia/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            # Ground-truth odometry and TF (map → base_footprint) from Gazebo
            "/enimia/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/enimia/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            # Drive and steering commands, ROS → Gazebo
            "/enimia/rear_wheel_cmd@std_msgs/msg/Float64]gz.msgs.Double",
            "/enimia/steer_left_cmd@std_msgs/msg/Float64]gz.msgs.Double",
            "/enimia/steer_right_cmd@std_msgs/msg/Float64]gz.msgs.Double",
            # ── Sensors (Gazebo → ROS) ──────────────────────────────────
            # LiDAR: safety_stop, cmd_vel_gate, local_planner all read /scan
            "/lidar_link/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            # IMU: ekf_local, ekf_global and navsat_transform read /imu/data
            "/imu_link/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            # GPS: navsat_transform reads /fix
            "/gps_link/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
            # ── Remaps ─────────────────────────────────────────────────
            "--ros-args",
            "-r", "/world/silesia_ring/model/enimia/joint_state:=/joint_states",
            "-r", "/enimia/odom:=/odom",
            "-r", "/enimia/tf:=/tf",
            "-r", "/lidar_link/scan:=/scan",
            "-r", "/imu_link/imu:=/imu/data",
            "-r", "/gps_link/navsat:=/fix",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # Relay /odom → /wheel/odom so the EKF local filter gets a velocity source.
    # The EKF only uses vx from odom0 (see ekf_local.yaml), so ground-truth
    # speed is good enough — the filter will still drift-correct with the IMU.
    relay_wheel_odom = Node(
        package="topic_tools",
        executable="relay",
        arguments=["/odom", "/wheel/odom"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        parameters=[{"port": 8765, "use_sim_time": True}],
        output="screen",
    )

    world_markers = Node(
        package="simulation",
        executable="world_markers.py",
        parameters=[{"frame_id": "map", "use_sim_time": True}],
        output="screen",
    )

    # ── vehicle_controller: /cmd_vel → /esp32/motor_cmd + /esp32/servo_cmd ── #
    vehicle_controller = Node(
        package="vehicle_control",
        executable="vehicle_controller",
        name="vehicle_controller",
        parameters=[
            os.path.join(ctrl_share, "config", "vehicle_controller.yaml"),
            {"use_sim_time": True},
        ],
        output="screen",
    )

    # ── Simulated ESP32: /esp32/* → Gazebo joints ──────────────────────── #
    esp32_sim_bridge = Node(
        package="simulation",
        executable="esp32_sim_bridge.py",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # ── Localization: EKF local + global + navsat_transform ────────────── #
    # use_sim_time is forwarded so nodes subscribe to /clock.
    # imu_stamper will start but sit idle (no /esp32/imu in sim) — harmless.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(loc_share, "launch", "localization.launch.py")
        ),
        launch_arguments={
            "use_gps": "true",
            "use_sim_time": "true",
        }.items(),
    )

    # ── Navigation: waypoint_manager → local_planner → cmd_vel_gate → safety_stop ── #
    # sim_vehicle.yaml is appended after each node's own config to override
    # the RC-car footprint/wheelbase with ENIMIA dimensions.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "use_lidar_gate": "true",
            "use_local_planner": "true",
            "use_sim_time": "true",
            # Pass the sim override config path so navigation.launch.py can
            # append it. navigation.launch.py doesn't support this yet — the
            # override is applied by launching nodes with extra parameters
            # directly below instead.
        }.items(),
    )

    # ── Navigation nodes with ENIMIA overrides (replaces included launch) ─ #
    # navigation.launch.py doesn't accept an extra config path, so we launch
    # the four navigation nodes directly here with sim_vehicle.yaml appended.
    nav_cfg = os.path.join(nav_share, "config")

    waypoint_manager = Node(
        package="navigation",
        executable="waypoint_manager",
        name="waypoint_manager",
        parameters=[
            os.path.join(nav_cfg, "waypoint_manager.yaml"),
            {"use_sim_time": True},
        ],
        remappings=[("/cmd_vel", "/cmd_vel_nav")],
        output="screen",
    )

    local_planner = Node(
        package="navigation",
        executable="local_planner",
        name="local_planner",
        parameters=[
            os.path.join(nav_cfg, "local_planner.yaml"),
            sim_vehicle_cfg,                # overrides wheelbase + footprint
            {"use_sim_time": True},
        ],
        output="screen",
    )

    cmd_vel_gate = Node(
        package="navigation",
        executable="cmd_vel_gate",
        name="cmd_vel_gate",
        parameters=[
            os.path.join(nav_cfg, "cmd_vel_gate.yaml"),
            sim_vehicle_cfg,                # overrides front_x / rear_x / half_width
            {"input_topic": "/cmd_vel_plan", "use_sim_time": True},
        ],
        output="screen",
    )

    safety_stop = Node(
        package="navigation",
        executable="safety_stop",
        name="safety_stop",
        parameters=[
            os.path.join(nav_cfg, "safety_stop.yaml"),
            sim_vehicle_cfg,                # overrides front_offset_m + distances
            {"input_cmd_topic": "/cmd_vel_gated", "use_sim_time": True},
        ],
        output="screen",
    )

    return LaunchDescription([
        resource_path,
        description,
        gazebo,
        spawn,
        bridge,
        relay_wheel_odom,
        foxglove,
        world_markers,
        vehicle_controller,
        esp32_sim_bridge,
        localization,
        waypoint_manager,
        local_planner,
        cmd_vel_gate,
        safety_stop,
    ])
