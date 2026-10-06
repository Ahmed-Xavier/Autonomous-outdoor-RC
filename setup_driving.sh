#!/usr/bin/env bash
# Adds keyboard / cmd_vel driving to the ENIMIA Gazebo simulation.
# Run from the repo root:  bash setup_driving.sh
set -euo pipefail
SIM=ros2_ws/src/simulation
URDF=ros2_ws/src/vehicle_description/urdf/enimia.urdf
[ -d "$SIM" ] && [ -f "$URDF" ] || { echo "Run this from ~/Autonomous-outdoor-RC"; exit 1; }

# 1) cmd_vel -> Gazebo joint commands ---------------------------------------
cat > "$SIM/scripts/cmd_vel_to_gz.py" <<'EOF'
#!/usr/bin/env python3
"""Convert /cmd_vel (Twist) into Gazebo joint commands for the ENIMIA tricycle.

Front: two steered wheels (Ackermann). Rear: one driven wheel on the centre line.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64

WHEELBASE = 1.704        # m, rear wheel centre to front axle
TRACK = 0.567            # m, distance between the front wheels
REAR_RADIUS = 0.18       # m
MAX_STEER = 0.20         # rad, centre steering angle (inner wheel stays < 0.21)
TIMEOUT = 0.5            # s, stop if no command arrives


class CmdVelToGz(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_gz')
        self.rear = self.create_publisher(Float64, '/enimia/rear_wheel_cmd', 10)
        self.left = self.create_publisher(Float64, '/enimia/steer_left_cmd', 10)
        self.right = self.create_publisher(Float64, '/enimia/steer_right_cmd', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_timer(0.05, self.on_timer)
        self.v = 0.0
        self.delta = 0.0
        self.last = self.get_clock().now()

    def on_cmd(self, msg):
        self.v = msg.linear.x
        w = msg.angular.z
        if abs(self.v) > 1e-3:
            delta = math.atan(w * WHEELBASE / self.v)
        else:
            delta = 0.0
        self.delta = max(-MAX_STEER, min(MAX_STEER, delta))
        self.last = self.get_clock().now()

    def on_timer(self):
        age = (self.get_clock().now() - self.last).nanoseconds / 1e9
        v = self.v if age < TIMEOUT else 0.0
        t = math.tan(self.delta)
        left = math.atan2(WHEELBASE * t, WHEELBASE - TRACK / 2.0 * t)
        right = math.atan2(WHEELBASE * t, WHEELBASE + TRACK / 2.0 * t)
        self.rear.publish(Float64(data=v / REAR_RADIUS))
        self.left.publish(Float64(data=left))
        self.right.publish(Float64(data=right))


def main():
    rclpy.init()
    rclpy.spin(CmdVelToGz())


if __name__ == '__main__':
    main()
EOF
chmod +x "$SIM/scripts/cmd_vel_to_gz.py"

# 2) launch file ------------------------------------------------------------
cat > "$SIM/launch/simulation.launch.py" <<'EOF'
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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        # -s = server only, -r = run immediately
        launch_arguments={"gz_args": f"-r -s {world}"}.items(),
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
            # ground-truth odometry and TF (map -> base_footprint) from Gazebo
            "/enimia/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/enimia/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            # drive and steering commands, ROS -> Gazebo
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

    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        parameters=[{"port": 8765, "use_sim_time": True}],
        output="screen",
    )

    world_markers = Node(
        package="simulation",
        executable="world_markers.py",
        parameters=[{"frame_id": "map"}],
        output="screen",
    )

    cmd_vel_to_gz = Node(
        package="simulation",
        executable="cmd_vel_to_gz.py",
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
        cmd_vel_to_gz,
    ])
EOF

# 3) install the new script ---------------------------------------------------
if ! grep -q cmd_vel_to_gz "$SIM/CMakeLists.txt"; then
  sed -i 's#scripts/world_markers.py#scripts/world_markers.py scripts/cmd_vel_to_gz.py#' "$SIM/CMakeLists.txt"
fi
grep -n "PROGRAMS" "$SIM/CMakeLists.txt"

# 4) Gazebo plugins in the URDF ---------------------------------------------
if grep -q "cmd_vel drive plugins" "$URDF"; then
  echo "URDF plugins already present, skipping"
else
python3 - "$URDF" <<'PY'
import sys
p = sys.argv[1]
s = open(p, encoding='utf-8').read()
block = '''
  <!-- cmd_vel drive plugins: rear wheel velocity, front steering, ground-truth odometry -->
  <gazebo>
    <plugin filename="gz-sim-joint-controller-system"
            name="gz::sim::systems::JointController">
      <joint_name>rear_wheel_joint</joint_name>
      <topic>/enimia/rear_wheel_cmd</topic>
    </plugin>
    <plugin filename="gz-sim-joint-position-controller-system"
            name="gz::sim::systems::JointPositionController">
      <joint_name>front_left_steer_joint</joint_name>
      <topic>/enimia/steer_left_cmd</topic>
      <p_gain>10</p_gain>
      <i_gain>0</i_gain>
      <d_gain>1</d_gain>
      <cmd_max>20</cmd_max>
      <cmd_min>-20</cmd_min>
    </plugin>
    <plugin filename="gz-sim-joint-position-controller-system"
            name="gz::sim::systems::JointPositionController">
      <joint_name>front_right_steer_joint</joint_name>
      <topic>/enimia/steer_right_cmd</topic>
      <p_gain>10</p_gain>
      <i_gain>0</i_gain>
      <d_gain>1</d_gain>
      <cmd_max>20</cmd_max>
      <cmd_min>-20</cmd_min>
    </plugin>
    <plugin filename="gz-sim-odometry-publisher-system"
            name="gz::sim::systems::OdometryPublisher">
      <odom_frame>map</odom_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
      <odom_topic>/enimia/odom</odom_topic>
      <tf_topic>/enimia/tf</tf_topic>
      <odom_publish_frequency>30</odom_publish_frequency>
      <dimensions>3</dimensions>
    </plugin>
  </gazebo>

'''
i = s.rindex('</robot>')
open(p, 'w', encoding='utf-8').write(s[:i] + block + s[i:])
print('URDF updated')
PY
fi

python3 -m py_compile "$SIM/scripts/cmd_vel_to_gz.py" "$SIM/launch/simulation.launch.py" && echo syntax_ok
python3 -c "import xml.dom.minidom,sys; xml.dom.minidom.parse('$URDF'); print('urdf_xml_ok')"
echo
echo "Now run:"
echo "  cd ros2_ws && colcon build --packages-select simulation vehicle_description && source install/setup.bash"
echo "  ros2 launch simulation simulation.launch.py"
