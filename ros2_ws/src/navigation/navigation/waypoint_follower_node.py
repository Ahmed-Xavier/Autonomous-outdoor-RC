#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

R = 6371000.0

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('goto_waypoint')
        self.declare_parameter('lat', 0.0)
        self.declare_parameter('lon', 0.0)
        self.declare_parameter('tol', 3.0)        # m
        self.declare_parameter('speed', 0.3)      # cmd_vel linear.x
        self.declare_parameter('k_steer', 1.0)    # gain on heading error
        self.declare_parameter('max_steer', 0.5)  # clamp on angular.z
        self.declare_parameter('steer_sign', 1.0) # flip if car turns away
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.declare_parameter('gps_timeout', 3.0)
        self.declare_parameter('loop_period', 0.1)
        self.p = lambda n: self.get_parameter(n).value
        self.fix = None
        self.yaw = None
        self.last_fix_t = None
        self.create_subscription(NavSatFix, '/fix', self.on_fix, 10)
        self.create_subscription(Odometry, '/odometry/global', self.on_odom, 10)
        # Goal from Foxglove (Publish panel, sensor_msgs/NavSatFix)
        self.create_subscription(NavSatFix, '/goal_fix', self.on_goal, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(self.p('loop_period'), self.loop)
        self.get_logger().info('Waiting for /fix and /odometry/global...')

    def on_fix(self, m):
        if m.status.status >= 0:
            self.fix = (m.latitude, m.longitude)
            self.last_fix_t = self.get_clock().now()

    def on_odom(self, m):
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def on_goal(self, m):
        self.set_parameters([
            Parameter('lat', Parameter.Type.DOUBLE, m.latitude),
            Parameter('lon', Parameter.Type.DOUBLE, m.longitude),
        ])
        self.get_logger().info(f'New goal: {m.latitude:.6f}, {m.longitude:.6f}')

    def stop(self):
        self.pub.publish(Twist())

    def loop(self):
        if self.fix is None or self.yaw is None:
            return
        if self.p('lat') == 0.0 and self.p('lon') == 0.0:
            return  # no goal set yet, do not drive toward (0, 0)
        age = (self.get_clock().now() - self.last_fix_t).nanoseconds / 1e9
        if age > self.p('gps_timeout'):
            self.get_logger().warn('GPS fix is old, stopping')
            self.stop(); return
        lat, lon = self.fix
        glat, glon = self.p('lat'), self.p('lon')
        dN = math.radians(glat - lat) * R
        dE = math.radians(glon - lon) * R * math.cos(math.radians(lat))
        dist = math.hypot(dE, dN)
        if dist < self.p('tol'):
            self.get_logger().info(f'Arrived (dist {dist:.1f} m)')
            self.stop()
            rclpy.shutdown(); return
        target = math.atan2(dN, dE)  # 0 = east, counter-clockwise
        yaw = self.yaw + math.radians(self.p('yaw_offset_deg'))
        err = wrap(target - yaw)
        steer = self.p('steer_sign') * self.p('k_steer') * err
        ms = self.p('max_steer')
        steer = max(-ms, min(ms, steer))
        t = Twist()
        t.linear.x = self.p('speed')
        t.angular.z = steer
        self.pub.publish(t)
        self.get_logger().info(
            f'dist {dist:.1f} m  err {math.degrees(err):.0f} deg  steer {steer:.2f}',
            throttle_duration_sec=1.0)

def main():
    rclpy.init()
    n = WaypointFollower()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            n.stop()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()