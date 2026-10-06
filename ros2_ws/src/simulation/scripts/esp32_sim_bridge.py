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
