#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16


class VehicleController(Node):
    """
    Converts ROS 2 /cmd_vel commands into the low-level commands
    expected by the ESP32 micro-ROS firmware.

    Inputs:
        /cmd_vel              geometry_msgs/Twist

    Outputs:
        /esp32/motor_cmd      std_msgs/Int16
        /esp32/servo_cmd      std_msgs/Int16
    """

    def __init__(self):
        super().__init__('vehicle_controller')

        self.declare_parameter('motor_min_pwm', 100)
        self.declare_parameter('motor_max_pwm', 255)
        self.declare_parameter('servo_min', 28)
        self.declare_parameter('servo_center', 48)
        self.declare_parameter('servo_max', 68)
        self.declare_parameter('command_timeout', 0.4)
        self.declare_parameter('linear_deadband', 0.001)
        self.declare_parameter('linear_full_scale', 1.0)
        self.declare_parameter('angular_limit', 1.0)
        self.declare_parameter('control_period', 0.1)

        self.motor_pub = self.create_publisher(
            Int16,
            '/esp32/motor_cmd',
            10
        )

        self.servo_pub = self.create_publisher(
            Int16,
            '/esp32/servo_cmd',
            10
        )

        self.last_cmd_time = self.get_clock().now()

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publish commands periodically so the ESP32 failsafe
        # does not stop the motor during normal teleoperation.
        self.control_timer = self.create_timer(
            self.p('control_period'),
            self.control_loop
        )

        self.last_motor_cmd = 0
        self.last_servo_cmd = self.p('servo_center')

        self.get_logger().info(
            'Vehicle controller started: /cmd_vel -> ESP32'
        )

    def p(self, name):
        return self.get_parameter(name).value

    def cmd_vel_callback(self, msg: Twist):
        """
        Convert Twist into motor and steering commands.
        """

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # ---------------------------------------------------------
        # MOTOR
        # ---------------------------------------------------------
        #
        # linear_x is normalized (a fraction of full PWM), not m/s.
        #
        # Anything non-zero must be at least +/-100 because the
        # physical motor needs that PWM to start moving.
        #

        if abs(linear_x) < self.p('linear_deadband'):
            motor_cmd = 0
        else:
            motor_cmd = int(
                max(
                    self.p('motor_min_pwm'),
                    min(
                        self.p('motor_max_pwm'),
                        abs(linear_x) / self.p('linear_full_scale') * self.p('motor_max_pwm')
                    )
                )
            )

            if linear_x < 0:
                motor_cmd = -motor_cmd

        # ---------------------------------------------------------
        # STEERING
        # ---------------------------------------------------------
        #
        # angular_z is normalized to [-angular_limit, +angular_limit].
        # The mapping is servo_center + angular_z *
        # (servo_center - servo_min), clamped to [servo_min, servo_max].
        # Positive angular_z (left) gives a servo value above center on this car.
        #

        angular_limit = self.p('angular_limit')
        angular_z = max(-angular_limit, min(angular_limit, angular_z))

        servo_cmd = int(
            self.p('servo_center') + angular_z * (
                self.p('servo_center') - self.p('servo_min')
            )
        )

        servo_cmd = max(
            self.p('servo_min'),
            min(self.p('servo_max'), servo_cmd)
        )

        self.last_motor_cmd = motor_cmd
        self.last_servo_cmd = servo_cmd
        self.last_cmd_time = self.get_clock().now()

    def control_loop(self):
        """
        Periodically publish the latest commands.

        If /cmd_vel has stopped arriving, stop the motor and
        return steering to center.
        """

        now = self.get_clock().now()

        elapsed = (
            now - self.last_cmd_time
        ).nanoseconds / 1e9

        if elapsed > self.p('command_timeout'):
            motor_cmd = 0
            servo_cmd = self.p('servo_center')
        else:
            motor_cmd = self.last_motor_cmd
            servo_cmd = self.last_servo_cmd

        motor_msg = Int16()
        motor_msg.data = motor_cmd

        servo_msg = Int16()
        servo_msg.data = servo_cmd

        self.motor_pub.publish(motor_msg)
        self.servo_pub.publish(servo_msg)


def main(args=None):
    rclpy.init(args=args)

    node = VehicleController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Explicit stop command before shutting down.
        stop_motor = Int16()
        stop_motor.data = 0
        node.motor_pub.publish(stop_motor)

        center_servo = Int16()
        center_servo.data = node.p('servo_center')
        node.servo_pub.publish(center_servo)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
