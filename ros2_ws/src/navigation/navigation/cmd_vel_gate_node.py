#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener


def transform_xy(x, y, transform):
    q = transform.transform.rotation
    t = transform.transform.translation
    # Rotate around Z (the mounted scanner has no roll or pitch).
    yaw = math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                     1.0 - 2.0 * (q.y*q.y + q.z*q.z))
    return (t.x + math.cos(yaw)*x - math.sin(yaw)*y,
            t.y + math.sin(yaw)*x + math.cos(yaw)*y)


class CmdVelGate(Node):
    def __init__(self):
        super().__init__('cmd_vel_gate')
        for name, default in (
            ('stop_distance_m', 0.35), ('min_points', 2), ('timeout_s', 0.5),
            ('front_x', 0.2425), ('rear_x', -0.0575), ('half_width', 0.04),
            ('input_topic', '/cmd_vel_nav'), ('output_topic', '/cmd_vel_gated'),
            ('scan_topic', '/scan')):
            self.declare_parameter(name, default)
        self.p = lambda name: self.get_parameter(name).value
        self.command = None
        self.command_time = None
        self.scan = None
        self.scan_time = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Twist, self.p('output_topic'), 10)
        self.status_pub = self.create_publisher(String, '/safety/gate_status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/safety/gate_markers', 10)
        self.create_subscription(Twist, self.p('input_topic'), self.on_command, 10)
        self.create_subscription(LaserScan, self.p('scan_topic'), self.on_scan, 10)
        self.create_timer(0.05, self.tick)

    def on_command(self, msg):
        self.command = msg
        self.command_time = self.get_clock().now()

    def on_scan(self, msg):
        self.scan = msg
        self.scan_time = self.get_clock().now()

    def publish_markers(self, blocked):
        arr = MarkerArray()
        delete = Marker()
        delete.header.frame_id = 'base_link'
        delete.pose.orientation.w = 1.0
        delete.action = Marker.DELETEALL
        arr.markers.append(delete)
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'stop_zone'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.025
        marker.color.r = 1.0 if blocked else 0.0
        marker.color.g = 0.0 if blocked else 1.0
        marker.color.a = 1.0
        x0 = self.p('front_x')
        x1 = x0 + self.p('stop_distance_m')
        w = self.p('half_width')
        marker.points = [Point(x=x0, y=-w), Point(x=x1, y=-w),
                         Point(x=x1, y=w), Point(x=x0, y=w), Point(x=x0, y=-w)]
        arr.markers.append(marker)
        self.marker_pub.publish(arr)

    def tick(self):
        now = self.get_clock().now()
        cmd = Twist()
        status = 'PASS'
        blocked = False
        if self.command is None or (now-self.command_time).nanoseconds/1e9 > self.p('timeout_s'):
            status = 'INPUT_STALE'
        elif self.scan is None or (now-self.scan_time).nanoseconds/1e9 > self.p('timeout_s'):
            status = 'SCAN_STALE'
        else:
            try:
                tr = self.tf_buffer.lookup_transform(
                    'base_link', self.scan.header.frame_id, self.scan.header.stamp)
            except Exception:
                status = 'TF_MISSING'
            else:
                if self.command.linear.x > 0.0:
                    found = 0
                    angle = self.scan.angle_min
                    for r in self.scan.ranges:
                        if math.isfinite(r) and self.scan.range_min <= r <= self.scan.range_max:
                            x, y = transform_xy(r*math.cos(angle), r*math.sin(angle), tr)
                            if self.p('rear_x') <= x <= self.p('front_x') and abs(y) <= self.p('half_width'):
                                angle += self.scan.angle_increment
                                continue
                            if (self.p('front_x') <= x <= self.p('front_x')+self.p('stop_distance_m')
                                    and abs(y) <= self.p('half_width')):
                                found += 1
                        angle += self.scan.angle_increment
                    if found >= self.p('min_points'):
                        status = 'BLOCKED'
                        blocked = True
                    else:
                        cmd = self.command
                else:
                    cmd = self.command
        if status != 'PASS' and self.command is not None:
            # Stop propulsion while retaining the requested steering through
            # the downstream safety_stop command filter.
            cmd.angular.z = self.command.angular.z
        self.pub.publish(cmd)
        s = String()
        s.data = status
        self.status_pub.publish(s)
        self.publish_markers(blocked)

    def destroy_node(self):
        if hasattr(self, 'pub'):
            self.pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelGate()
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
