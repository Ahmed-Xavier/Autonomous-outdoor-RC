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

    MOTOR_MIN_PWM = 100
    MOTOR_MAX_PWM = 255

    SERVO_MIN = 30
    SERVO_CENTER = 50
    SERVO_MAX = 70

    # Safety timeout. The ESP32 itself also has a 500 ms failsafe.
    COMMAND_TIMEOUT = 0.4

    def __init__(self):
        super().__init__('vehicle_controller')

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
            0.1,
            self.control_loop
        )

        self.last_motor_cmd = 0
        self.last_servo_cmd = self.SERVO_CENTER

        self.get_logger().info(
            'Vehicle controller started: /cmd_vel -> ESP32'
        )

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
        # Normalize linear velocity from [-1, 1] to [-255, 255].
        #
        # Anything non-zero must be at least +/-100 because the
        # physical motor needs that PWM to start moving.
        #

        if abs(linear_x) < 0.001:
            motor_cmd = 0
        else:
            motor_cmd = int(
                max(
                    self.MOTOR_MIN_PWM,
                    min(
                        self.MOTOR_MAX_PWM,
                        abs(linear_x) * self.MOTOR_MAX_PWM
                    )
                )
            )

            if linear_x < 0:
                motor_cmd = -motor_cmd

        # ---------------------------------------------------------
        # STEERING
        # ---------------------------------------------------------
        #
        # angular.z:
        #
        #   -1.0 -> 70? / 30?
        #    0.0 -> 50
        #   +1.0 -> opposite side
        #
        # ROS convention: positive angular.z = left / CCW.
        #
        # We map positive angular.z to servo values below center.
        # If your physical steering direction is reversed, swap
        # SERVO_MIN and SERVO_MAX in this mapping.
        #

        angular_z = max(-1.0, min(1.0, angular_z))

        servo_cmd = int(
            self.SERVO_CENTER - angular_z * (
                self.SERVO_CENTER - self.SERVO_MIN
            )
        )

        servo_cmd = max(
            self.SERVO_MIN,
            min(self.SERVO_MAX, servo_cmd)
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

        if elapsed > self.COMMAND_TIMEOUT:
            motor_cmd = 0
            servo_cmd = self.SERVO_CENTER
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
        center_servo.data = node.SERVO_CENTER
        node.servo_pub.publish(center_servo)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()