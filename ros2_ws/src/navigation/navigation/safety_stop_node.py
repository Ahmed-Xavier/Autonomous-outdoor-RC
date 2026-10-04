#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class SafetyStop(Node):
    def __init__(self):
        super().__init__('safety_stop')
        for name, param_type in (
                ('scan_topic', Parameter.Type.STRING),
                ('input_cmd_topic', Parameter.Type.STRING),
                ('output_cmd_topic', Parameter.Type.STRING),
                ('obstacle_stop_topic', Parameter.Type.STRING),
                ('half_angle_deg', Parameter.Type.DOUBLE),
                ('stop_distance_m', Parameter.Type.DOUBLE),
                ('clear_distance_m', Parameter.Type.DOUBLE),
                ('front_offset_m', Parameter.Type.DOUBLE),
                ('timeout_s', Parameter.Type.DOUBLE),
                ('loop_period_s', Parameter.Type.DOUBLE),
                ('ignore_sectors', Parameter.Type.DOUBLE_ARRAY)):
            self.declare_parameter(name, param_type)
        self.p = lambda name: self.get_parameter(name).value

        sectors = self.p('ignore_sectors')
        if len(sectors) % 2:
            raise ValueError('ignore_sectors must contain start/end angle pairs in degrees')
        if self.p('clear_distance_m') <= self.p('stop_distance_m'):
            raise ValueError('clear_distance_m must be greater than stop_distance_m')

        self.scan = None
        self.scan_time = None
        self.command = Twist()
        self.obstacle_stop = True
        self.have_status = False

        self.command_pub = self.create_publisher(Twist, self.p('output_cmd_topic'), 10)
        self.stop_pub = self.create_publisher(Bool, self.p('obstacle_stop_topic'), 10)
        self.create_subscription(LaserScan, self.p('scan_topic'), self.on_scan, 10)
        self.create_subscription(Twist, self.p('input_cmd_topic'), self.on_command, 10)
        self.create_timer(self.p('loop_period_s'), self.tick)

    def on_scan(self, msg):
        self.scan = msg
        self.scan_time = self.get_clock().now()

    def on_command(self, msg):
        self.command = msg
        self.publish_command(self.obstacle_stop)

    def is_ignored(self, angle_deg):
        angle_deg = (angle_deg + 180.0) % 360.0 - 180.0
        sectors = self.p('ignore_sectors')
        for index in range(0, len(sectors), 2):
            start = (sectors[index] + 180.0) % 360.0 - 180.0
            end = (sectors[index + 1] + 180.0) % 360.0 - 180.0
            if (start <= end and start <= angle_deg <= end) or (
                    start > end and (angle_deg >= start or angle_deg <= end)):
                return True
        return False

    def nearest_in_cone(self):
        if self.scan is None:
            return None
        nearest = None
        angle = self.scan.angle_min
        half_angle = math.radians(self.p('half_angle_deg'))
        for distance in self.scan.ranges:
            if (math.isfinite(distance) and
                    self.scan.range_min <= distance <= self.scan.range_max and
                    abs(angle) <= half_angle and
                    not self.is_ignored(math.degrees(angle))):
                if nearest is None or distance < nearest:
                    nearest = distance
            angle += self.scan.angle_increment
        return nearest

    def set_stop(self, stopped, reason):
        if not self.have_status or stopped != self.obstacle_stop:
            state = 'STOP' if stopped else 'CLEAR'
            self.get_logger().info(f'obstacle_stop changed to {state}: {reason}')
        self.obstacle_stop = stopped
        self.have_status = True

    def publish_command(self, stopped):
        command = Twist()
        command.linear.x = 0.0 if stopped else self.command.linear.x
        command.angular.z = self.command.angular.z
        command.linear.y = self.command.linear.y
        command.linear.z = self.command.linear.z
        command.angular.x = self.command.angular.x
        command.angular.y = self.command.angular.y
        self.command_pub.publish(command)

    def tick(self):
        now = self.get_clock().now()
        if (self.scan is None or self.scan_time is None or
                (now - self.scan_time).nanoseconds / 1e9 > self.p('timeout_s')):
            self.set_stop(True, 'scan timeout')
        else:
            nearest = self.nearest_in_cone()
            stop_range = self.p('front_offset_m') + self.p('stop_distance_m')
            clear_range = self.p('front_offset_m') + self.p('clear_distance_m')
            if not self.obstacle_stop and nearest is not None and nearest <= stop_range:
                self.set_stop(True, f'obstacle at {nearest:.2f} m from lidar')
            elif self.obstacle_stop and (nearest is None or nearest >= clear_range):
                self.set_stop(False, 'clear distance reached')

        status = Bool()
        status.data = self.obstacle_stop
        self.stop_pub.publish(status)

        if self.obstacle_stop:
            self.publish_command(True)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
