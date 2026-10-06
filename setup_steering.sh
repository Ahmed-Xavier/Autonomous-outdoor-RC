#!/usr/bin/env bash
# Links the real vehicle_controller (/cmd_vel -> /esp32/motor_cmd + /esp32/servo_cmd)
# to the Gazebo ENIMIA model. Replaces setup_driving.sh; safe to run after it.
# Run from the repo root:  bash setup_steering.sh
set -euo pipefail
SIM=ros2_ws/src/simulation
URDF=ros2_ws/src/vehicle_description/urdf/enimia.urdf
[ -d "$SIM" ] && [ -f "$URDF" ] || { echo "Run this from ~/Autonomous-outdoor-RC"; exit 1; }

# 1) /esp32/* commands -> Gazebo joint commands ----------------------------------
rm -f "$SIM/scripts/cmd_vel_to_gz.py"
cat > "$SIM/scripts/esp32_sim_bridge.py" <<'EOF'
#!/usr/bin/env python3
"""Stand-in for the ESP32 in simulation.

Reads the same topics the firmware reads (/esp32/motor_cmd, /esp32/servo_cmd,
both std_msgs/Int16, published by vehicle_controller) and drives the Gazebo
joints: rear wheel speed and the two front steering joints (Ackermann).

The servo -> steering angle formula is the one in vehicle_state_node.py.
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int16

# Simulated car geometry (from enimia.urdf)
WHEELBASE = 1.704   # m, rear wheel centre to front axle
TRACK = 0.567       # m, distance between the front wheels
REAR_RADIUS = 0.18  # m
MAX_STEER = 0.20    # rad at the centre; the URDF steer joints stop at +-0.21


class Esp32SimBridge(Node):
    def __init__(self):
        super().__init__('esp32_sim_bridge')
        # Same names and defaults as config/vehicle_state.yaml
        self.declare_parameter('servo_center', 48.0)
        self.declare_parameter('steer_rad_per_servo_deg', math.radians(1.0))
        self.declare_parameter('steer_sign', -1.0)
        # Simulation-only assumptions: calibrate when you know the real car
        self.declare_parameter('motor_max_pwm', 255.0)
        self.declare_parameter('speed_at_full_pwm', 2.0)   # m/s at PWM 255
        self.declare_parameter('command_timeout', 0.5)

        self.rear = self.create_publisher(Float64, '/enimia/rear_wheel_cmd', 10)
        self.left = self.create_publisher(Float64, '/enimia/steer_left_cmd', 10)
        self.right = self.create_publisher(Float64, '/enimia/steer_right_cmd', 10)
        self.create_subscription(Int16, '/esp32/motor_cmd', self.on_motor, 10)
        self.create_subscription(Int16, '/esp32/servo_cmd', self.on_servo, 10)
        self.create_timer(0.05, self.on_timer)

        self.pwm = 0
        self.servo = self.p('servo_center')
        self.last = self.get_clock().now()

    def p(self, name):
        return self.get_parameter(name).value

    def on_motor(self, msg):
        self.pwm = msg.data
        self.last = self.get_clock().now()

    def on_servo(self, msg):
        self.servo = float(msg.data)

    def on_timer(self):
        age = (self.get_clock().now() - self.last).nanoseconds / 1e9
        pwm = self.pwm if age < self.p('command_timeout') else 0
        v = pwm / self.p('motor_max_pwm') * self.p('speed_at_full_pwm')

        # servo above centre = left = positive angle (as in vehicle_state_node)
        delta = (self.p('steer_sign')
                 * (self.p('servo_center') - self.servo)
                 * self.p('steer_rad_per_servo_deg'))
        delta = max(-MAX_STEER, min(MAX_STEER, delta))

        t = math.tan(delta)
        left = math.atan2(WHEELBASE * t, WHEELBASE - TRACK / 2.0 * t)
        right = math.atan2(WHEELBASE * t, WHEELBASE + TRACK / 2.0 * t)
        self.rear.publish(Float64(data=v / REAR_RADIUS))
        self.left.publish(Float64(data=left))
        self.right.publish(Float64(data=right))


def main():
    rclpy.init()
    rclpy.spin(Esp32SimBridge())


if __name__ == '__main__':
    main()
EOF
chmod +x "$SIM/scripts/esp32_sim_bridge.py"

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

    # The real controller: /cmd_vel -> /esp32/motor_cmd + /esp32/servo_cmd.
    # vehicle_state is not started: it needs /esp32/encoder, and Gazebo
    # publishes /joint_states and /odom itself.
    vehicle_controller = Node(
        package="vehicle_control",
        executable="vehicle_controller",
        name="vehicle_controller",
        parameters=[os.path.join(
            get_package_share_directory("vehicle_control"),
            "config", "vehicle_controller.yaml")],
        output="screen",
    )

    # Simulated ESP32: /esp32/* -> Gazebo joints
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
EOF

# 3) install the scripts, declare dependencies -------------------------------
python3 - "$SIM/CMakeLists.txt" "$SIM/package.xml" <<'PY'
import re, sys
cm, px = sys.argv[1], sys.argv[2]
s = open(cm, encoding='utf-8').read()
s = re.sub(r'^install\(PROGRAMS[^\n]*\)\n\n?', '', s, flags=re.M)
line = ('install(PROGRAMS scripts/world_markers.py scripts/esp32_sim_bridge.py '
        'DESTINATION lib/${PROJECT_NAME})\n\n')
s = s.replace('ament_package()', line + 'ament_package()')
open(cm, 'w', encoding='utf-8').write(s)

p = open(px, encoding='utf-8').read()
for dep in ('vehicle_control', 'vehicle_description', 'std_msgs'):
    tag = '<exec_depend>%s</exec_depend>' % dep
    if tag not in p:
        p = p.replace('  <export>', '  ' + tag + '\n  <export>')
open(px, 'w', encoding='utf-8').write(p)
PY
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

python3 -m py_compile "$SIM/scripts/esp32_sim_bridge.py" "$SIM/launch/simulation.launch.py" && echo syntax_ok
python3 -c "import xml.dom.minidom,sys; xml.dom.minidom.parse('$URDF'); print('urdf_xml_ok')"
echo
echo "Now run:"
echo "  cd ros2_ws && colcon build --packages-select simulation vehicle_description vehicle_control && source install/setup.bash"
echo "  ros2 launch simulation simulation.launch.py"
