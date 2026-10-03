#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from .cmd_vel_gate_node import transform_xy


class LocalPlanner(Node):
    def __init__(self):
        super().__init__('local_planner')
        defaults = {
            'horizon_m': 1.0, 'num_arcs': 11, 'safety_margin_m': 0.05,
            'w_smooth': 0.2, 'left_turn_sign': 1.0, 'wheelbase': 0.185,
            'nominal_topic': '/cmd_vel_nav', 'guidance_topic': '/waypoints/guidance',
            'scan_topic': '/scan', 'output_topic': '/cmd_vel_plan',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = lambda name: self.get_parameter(name).value
        self.nominal = None
        self.nominal_time = None
        self.guidance = None
        self.guidance_time = None
        self.scan = None
        self.scan_time = None
        self.previous_steer = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Twist, self.p('output_topic'), 10)
        self.status_pub = self.create_publisher(String, '/planner/status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/planner/markers', 10)
        self.create_subscription(Twist, self.p('nominal_topic'), self.on_nominal, 10)
        self.create_subscription(Float32MultiArray, self.p('guidance_topic'), self.on_guidance, 10)
        self.create_subscription(LaserScan, self.p('scan_topic'), self.on_scan, 10)
        self.create_timer(0.05, self.tick)

    def on_nominal(self, msg):
        self.nominal = msg
        self.nominal_time = self.get_clock().now()

    def on_guidance(self, msg):
        self.guidance = msg.data
        self.guidance_time = self.get_clock().now()

    def on_scan(self, msg):
        self.scan = msg
        self.scan_time = self.get_clock().now()

    def publish_markers(self, arc_points, blocked, chosen):
        array = MarkerArray()
        delete = Marker()
        delete.header.frame_id = 'base_link'
        delete.pose.orientation.w = 1.0
        delete.action = Marker.DELETEALL
        array.markers.append(delete)
        for i, points in enumerate(arc_points):
            m = Marker()
            m.header.frame_id = 'base_link'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'candidate_arcs'
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = 0.025 if i != chosen else 0.06
            m.color.a = 1.0
            if i == chosen:
                m.color.r, m.color.g, m.color.b = 1.0, 0.85, 0.0
            elif blocked[i]:
                m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0
            else:
                m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0
            m.points = [Point(x=x, y=y, z=0.0) for x, y in points]
            array.markers.append(m)
        self.marker_pub.publish(array)

    def publish_result(self, status, command=None, arcs=None, blocked=None, chosen=-1):
        text = String()
        text.data = status
        self.status_pub.publish(text)
        self.pub.publish(command if command is not None else Twist())
        if arcs is not None:
            self.publish_markers(arcs, blocked, chosen)
        else:
            self.publish_markers([], [], -1)

    def tick(self):
        if (self.guidance is None or len(self.guidance) < 5 or
                (self.get_clock().now()-self.guidance_time).nanoseconds/1e9 > 0.5):
            self.publish_result('NO_GUIDANCE')
            return
        distance, err, _index, _total, state = self.guidance[:5]
        if (state != 1.0 or self.nominal is None or self.nominal.linear.x <= 0.0 or
                (self.get_clock().now()-self.nominal_time).nanoseconds/1e9 > 0.5):
            self.publish_result('NO_GUIDANCE')
            return
        if self.scan is None or (self.get_clock().now()-self.scan_time).nanoseconds/1e9 > 0.5:
            self.publish_result('SCAN_STALE')
            return
        try:
            tr = self.tf_buffer.lookup_transform(
                'base_link', self.scan.header.frame_id, self.scan.header.stamp)
        except Exception:
            self.publish_result('SCAN_STALE')
            return

        ranges = []
        angle = self.scan.angle_min
        for r in self.scan.ranges:
            if math.isfinite(r) and self.scan.range_min <= r <= self.scan.range_max:
                x, y = transform_xy(r*math.cos(angle), r*math.sin(angle), tr)
                # URDF-derived base_link footprint: rear=-0.0575, front=0.2425,
                # half-width=0.04 m (chassis width is explicitly assumed).
                if not (-0.0575 <= x <= 0.2425 and abs(y) <= 0.04):
                    ranges.append((x, y))
            angle += self.scan.angle_increment

        horizon = self.p('horizon_m')
        goal_distance = min(max(0.0, distance), horizon)
        goal = (goal_distance*math.cos(err), goal_distance*math.sin(err))
        n = max(1, int(self.p('num_arcs')))
        values = [-0.9 + 1.8*i/(n-1) if n > 1 else 0.0 for i in range(n)]
        arc_points, arc_blocked, candidates = [], [], []
        margin = self.p('safety_margin_m')
        for i, steer in enumerate(values):
            curvature = math.tan(self.p('left_turn_sign') * steer * math.radians(20.0)) / self.p('wheelbase')
            pts = []
            collision = False
            steps = max(10, int(horizon/0.025))
            for step in range(steps+1):
                s = horizon*step/steps
                if abs(curvature) < 1e-9:
                    x, y, yaw = s, 0.0, 0.0
                else:
                    x = math.sin(curvature*s)/curvature
                    y = (1.0-math.cos(curvature*s))/curvature
                    yaw = curvature*s
                pts.append((x, y))
                c, sn = math.cos(yaw), math.sin(yaw)
                # Sweep the rectangular vehicle footprint at each sampled pose.
                for ox, oy in ranges:
                    dx, dy = ox-x, oy-y
                    local_x = c*dx + sn*dy
                    local_y = -sn*dx + c*dy
                    if (-0.0575-margin <= local_x <= 0.2425+margin and
                            abs(local_y) <= 0.04+margin):
                        collision = True
                        break
                if collision:
                    break
            arc_points.append(pts)
            arc_blocked.append(collision)
            if not collision:
                end_x, end_y = pts[-1]
                cost = math.hypot(end_x-goal[0], end_y-goal[1]) + self.p('w_smooth')*abs(steer-self.previous_steer)
                candidates.append((cost, i, steer))

        if not candidates:
            self.publish_result('BLOCKED', arcs=arc_points, blocked=arc_blocked)
            return
        _, idx, steer = min(candidates)
        self.previous_steer = steer
        cmd = Twist()
        cmd.linear.x = self.nominal.linear.x
        cmd.angular.z = steer
        self.publish_result('OK', cmd, arc_points, arc_blocked, idx)


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
