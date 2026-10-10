import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class FakeVehicleStatus(Node):
    def __init__(self):
        super().__init__('fake_vehicle_status')
        self.declare_parameter('status_odometry_topic', '/odom')
        self.declare_parameter('battery_initial_percent', 100.0)
        self.declare_parameter('battery_drop_percent_per_meter', 0.02)
        self.declare_parameter('ambient_temperature_c', 25.0)
        self.declare_parameter('temperature_rise_c_per_mps', 2.0)
        self.declare_parameter('publish_rate_hz', 2.0)
        self.battery = float(self.get_parameter('battery_initial_percent').value)
        self.battery_drop = float(self.get_parameter('battery_drop_percent_per_meter').value)
        self.ambient_temperature = float(self.get_parameter('ambient_temperature_c').value)
        self.temperature_rise = float(self.get_parameter('temperature_rise_c_per_mps').value)
        self.speed_mps = 0.0
        self.last_position = None
        self.battery_publisher = self.create_publisher(Float32, '/vehicle/battery_percent', 10)
        self.temperature_publisher = self.create_publisher(Float32, '/vehicle/temperature_c', 10)
        topic = self.get_parameter('status_odometry_topic').value
        self.subscription = self.create_subscription(Odometry, topic, self.on_odometry, 10)
        rate = max(0.1, float(self.get_parameter('publish_rate_hz').value))
        self.timer = self.create_timer(1.0 / rate, self.publish_status)

    def on_odometry(self, msg):
        position = msg.pose.pose.position
        current = (float(position.x), float(position.y))
        if self.last_position is not None:
            self.battery = max(0.0, self.battery - self.battery_drop *
                               math.dist(self.last_position, current))
        self.last_position = current
        self.speed_mps = float(msg.twist.twist.linear.x)

    def publish_status(self):
        battery = Float32()
        battery.data = self.battery
        temperature = Float32()
        temperature.data = self.ambient_temperature + self.temperature_rise * abs(self.speed_mps)
        self.battery_publisher.publish(battery)
        self.temperature_publisher.publish(temperature)


def main(args=None):
    rclpy.init(args=args)
    node = FakeVehicleStatus()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
