#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_localization.srv import FromLL, ToLL
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from .cmd_vel_gate_node import transform_xy

def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')
        defaults = {
            'tol': 3.0, 'speed': 0.0, 'steer_sign': 1.0, 'k_steer': 1.0,
            'max_steer': 0.5, 'yaw_offset_deg': 0.0, 'gps_timeout': 2.0,
            'loop_period': 0.1, 'from_ll_service': '/fromLL',
            'to_ll_service': '/navsat_transform/toLL',
            'markers_topic': '/waypoints/markers',
            'waypoint_marker_scale': 0.35, 'target_marker_scale': 0.55,
            'text_height': 0.5, 'line_width': 0.08,
            'guidance_topic': '/waypoints/guidance',
            # Foxglove "Publish pose" button topic (3D panel → toolbar → arrow icon).
            # Set to '' to disable.
            'goal_pose_topic': '/goal_pose',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = lambda name: self.get_parameter(name).value

        self.state = 'COLLECTING'
        self.waypoints = []  # Each entry has authoritative lat/lon and optional map cache.
        self.index = 0
        self.fix = None
        self.last_fix_time = None
        self.yaw = None
        self.map_position = None
        self.pending_to_ll = False
        self.pending_start = False
        self.to_ll_generation = 0
        self.pending_from_ll_index = None
        self.from_ll_generation = 0
        self.last_from_ll_attempt = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        marker_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, self.p('markers_topic'), marker_qos)
        self.guidance_pub = self.create_publisher(
            Float32MultiArray, self.p('guidance_topic'), 10)
        self.from_ll_client = self.create_client(FromLL, self.p('from_ll_service'))
        self.to_ll_client = self.create_client(ToLL, self.p('to_ll_service'))

        self.create_subscription(NavSatFix, '/fix', self.on_fix, 10)
        self.create_subscription(Odometry, '/odometry/global', self.on_odom, 10)
        # /clicked_point  — Foxglove 3D panel → "Publish clicked location" (crosshair icon)
        self.create_subscription(PointStamped, '/clicked_point', self.on_click, 10)
        # /goal_pose      — Foxglove 3D panel → "Publish pose" (arrow icon); same logic
        goal_topic = self.p('goal_pose_topic')
        if goal_topic:
            self.create_subscription(PoseStamped, goal_topic, self.on_goal_pose, 10)
        self.create_service(Trigger, '/waypoints/add_here', self.add_here)
        self.create_service(Trigger, '/waypoints/start', self.start)
        self.create_service(Trigger, '/waypoints/clear', self.clear)
        self.create_service(Trigger, '/waypoints/undo', self.undo)
        self.create_timer(self.p('loop_period'), self.loop)
        self.stop()
        self.publish_markers()
        self.get_logger().info('Waypoint manager ready in COLLECTING state')

    def on_fix(self, msg):
        if msg.status.status >= 0:
            self.fix = (msg.latitude, msg.longitude)
            self.last_fix_time = self.get_clock().now()

    def on_odom(self, msg):
        position = msg.pose.pose.position
        self.map_position = (position.x, position.y)
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z))

    def stop(self):
        self.cmd_pub.publish(Twist())

    def append_waypoint(self, lat, lon, map_xy=None):
        entry = {'lat': lat, 'lon': lon, 'map': map_xy}
        self.waypoints.append(entry)
        self.get_logger().info(
            f'Added waypoint {len(self.waypoints)}: {lat:.8f}, {lon:.8f}')
        self.publish_markers()

    def add_here(self, _request, response):
        if self.state != 'COLLECTING':
            response.success = False
            response.message = f'Cannot add a waypoint while {self.state}; clear first'
            return response
        if self.pending_start:
            response.success = False
            response.message = 'Cannot add a waypoint while route coordinates are refreshing'
            return response
        if self.fix is None or self.last_fix_time is None:
            response.success = False
            response.message = 'No /fix received yet'
            return response
        age = (self.get_clock().now() - self.last_fix_time).nanoseconds / 1e9
        if age > self.p('gps_timeout'):
            response.success = False
            response.message = f'/fix is stale ({age:.1f} s old)'
            return response
        lat, lon = self.fix
        self.append_waypoint(lat, lon)
        response.success = True
        response.message = f'Added waypoint {len(self.waypoints)}: {lat:.8f}, {lon:.8f}'
        return response

    def start(self, _request, response):
        if self.state != 'COLLECTING':
            response.success = False
            response.message = f'Cannot start while {self.state}; clear first'
            return response
        if self.pending_start:
            response.success = False
            response.message = 'Route map coordinates are already being refreshed'
            return response
        if not self.waypoints:
            response.success = False
            response.message = 'Add at least one waypoint before starting'
            return response
        # Latitude/longitude remain authoritative. Refresh all cached map
        # coordinates because the localization datum may have changed.
        self.pending_start = True
        self.from_ll_generation += 1
        self.pending_from_ll_index = None
        self.last_from_ll_attempt.clear()
        for waypoint in self.waypoints:
            waypoint['map'] = None
        self.publish_markers()
        self.request_from_ll()
        response.success = True
        response.message = (
            f'Refreshing map coordinates for {len(self.waypoints)} waypoint(s); '
            'route will start when ready')
        return response

    def begin_running_if_ready(self):
        if (self.pending_start and self.waypoints and
                all(waypoint['map'] is not None for waypoint in self.waypoints)):
            self.pending_start = False
            self.index = 0
            self.state = 'RUNNING'
            self.get_logger().info(
                f'Started route with {len(self.waypoints)} waypoint(s)')
            self.publish_markers()

    def clear(self, _request, response):
        self.stop()
        self.waypoints.clear()
        self.last_from_ll_attempt.clear()
        self.pending_from_ll_index = None
        self.from_ll_generation += 1
        self.pending_start = False
        self.pending_to_ll = False
        self.to_ll_generation += 1
        self.state = 'COLLECTING'
        self.index = 0
        self.publish_markers()
        response.success = True
        response.message = 'Cleared waypoints; state is COLLECTING'
        return response

    def undo(self, _request, response):
        if self.pending_start:
            response.success = False
            response.message = 'Cannot undo while route map coordinates are refreshing'
            return response
        if self.state != 'COLLECTING':
            response.success = False
            response.message = f'Cannot undo while {self.state}; clear first'
            return response
        if not self.waypoints:
            response.success = False
            response.message = 'No waypoint to undo'
            return response
        removed = self.waypoints.pop()
        response.success = True
        response.message = (
            f'Removed waypoint {len(self.waypoints) + 1}: '
            f"{removed['lat']:.8f}, {removed['lon']:.8f}")
        self.publish_markers()
        return response

    def on_click(self, msg):
        if self.pending_start:
            self.get_logger().warning(
                'Ignoring map click while route map coordinates are refreshing',
                throttle_duration_sec=5.0)
            return
        if self.state != 'COLLECTING':
            self.get_logger().warning(
                f'Ignoring map click while {self.state}', throttle_duration_sec=5.0)
            return

        x = msg.point.x
        y = msg.point.y
        frame_id = msg.header.frame_id or 'map'

        if frame_id != 'map':
            try:
                tr = self.tf_buffer.lookup_transform(
                    'map', frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2))
                x, y = transform_xy(x, y, tr)
            except Exception as exc:
                self.get_logger().warning(
                    f"Could not transform clicked point from frame '{frame_id}' to 'map': {exc}",
                    throttle_duration_sec=5.0)
                return

        if self.pending_to_ll:
            self.get_logger().warning('toLL request already pending; dropping map click',
                                       throttle_duration_sec=5.0)
            return
        if not self.to_ll_client.service_is_ready():
            self.get_logger().warning(
                f"Service {self.p('to_ll_service')} is not ready; ignoring map click",
                throttle_duration_sec=5.0)
            return

        request = ToLL.Request()
        request.map_point.x = x
        request.map_point.y = y
        request.map_point.z = 0.0
        self.pending_to_ll = True
        generation = self.to_ll_generation
        future = self.to_ll_client.call_async(request)
        future.add_done_callback(
            lambda result, px=x, py=y, g=generation:
                self.on_to_ll_response(result, px, py, g))

    def on_to_ll_response(self, future, x, y, generation):
        if generation != self.to_ll_generation:
            return
        self.pending_to_ll = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f'toLL request failed: {exc}')
            return
        if self.state != 'COLLECTING':
            return
        ll = response.ll_point
        self.append_waypoint(ll.latitude, ll.longitude, (x, y))

    def on_goal_pose(self, msg):
        """Handle a PoseStamped from Foxglove's 'Publish pose' arrow.

        Constructs a PointStamped and delegates to on_click (which will
        automatically transform from whatever frame the pose arrived in to map).
        """
        point = PointStamped()
        point.header = msg.header
        point.point = msg.pose.position
        self.on_click(point)

    def request_from_ll(self):
        if self.pending_from_ll_index is not None or not self.waypoints:
            return
        now = self.get_clock().now()
        candidate = None
        for index, waypoint in enumerate(self.waypoints):
            if waypoint['map'] is not None:
                continue
            last = self.last_from_ll_attempt.get(index)
            if last is None or (now - last).nanoseconds / 1e9 >= 1.0:
                candidate = index
                break
        if candidate is None:
            return
        if not self.from_ll_client.service_is_ready():
            self.get_logger().warning(
                f"Service {self.p('from_ll_service')} is not ready; will retry",
                throttle_duration_sec=5.0)
            return
        index = candidate
        generation = self.from_ll_generation
        waypoint = self.waypoints[index]
        request = FromLL.Request()
        request.ll_point.latitude = waypoint['lat']
        request.ll_point.longitude = waypoint['lon']
        request.ll_point.altitude = 0.0
        self.pending_from_ll_index = index
        self.last_from_ll_attempt[index] = now
        future = self.from_ll_client.call_async(request)
        future.add_done_callback(
            lambda result, i=index, g=generation: self.on_from_ll_response(result, i, g))

    def on_from_ll_response(self, future, index, generation):
        if generation != self.from_ll_generation:
            return
        if self.pending_from_ll_index == index:
            self.pending_from_ll_index = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f'fromLL request for waypoint {index + 1} failed: {exc}')
            return
        if index >= len(self.waypoints):
            return
        point = response.map_point
        self.waypoints[index]['map'] = (point.x, point.y)
        self.publish_markers()
        self.begin_running_if_ready()

    def loop(self):
        if self.state == 'COLLECTING':
            self.stop()
            self.request_from_ll()
            self.begin_running_if_ready()
            return
        if self.state != 'RUNNING':
            self.stop()
            return
        self.request_from_ll()
        if (self.fix is None or self.yaw is None or self.map_position is None or
                self.last_fix_time is None):
            self.stop()
            self.get_logger().warning('GPS fix or global odometry missing; stopping',
                                      throttle_duration_sec=5.0)
            return
        age = (self.get_clock().now() - self.last_fix_time).nanoseconds / 1e9
        if age > self.p('gps_timeout'):
            self.stop()
            self.get_logger().warning('/fix is stale; stopping', throttle_duration_sec=5.0)
            return
        waypoint = self.waypoints[self.index]
        if waypoint['map'] is None:
            self.stop()
            self.get_logger().warning(
                'Target map coordinate missing; stopping', throttle_duration_sec=5.0)
            return
        x, y = self.map_position
        target_x, target_y = waypoint['map']
        d_east = target_x - x
        d_north = target_y - y
        distance = math.hypot(d_east, d_north)
        if distance <= self.p('tol'):
            self.stop()
            self.index += 1
            if self.index >= len(self.waypoints):
                self.state = 'DONE'
                self.get_logger().info('Route complete; state is DONE')
            else:
                self.get_logger().info(f'Advancing to waypoint {self.index + 1}')
            self.publish_markers()
            return
        target = math.atan2(d_north, d_east)
        yaw = self.yaw + math.radians(self.p('yaw_offset_deg'))
        error = wrap(target - yaw)
        # State codes: 0=COLLECTING, 1=RUNNING, 2=DONE.
        guidance = Float32MultiArray()
        guidance.data = [float(distance), float(error), float(self.index),
                         float(len(self.waypoints)), 1.0]
        self.guidance_pub.publish(guidance)
        steer = self.p('steer_sign') * self.p('k_steer') * error
        limit = self.p('max_steer')
        steer = max(-limit, min(limit, steer))
        command = Twist()
        command.linear.x = self.p('speed')
        command.angular.z = steer
        self.cmd_pub.publish(command)

    def publish_markers(self):
        array = MarkerArray()
        delete = Marker()
        delete.header.frame_id = 'map'
        delete.header.stamp = self.get_clock().now().to_msg()
        delete.action = Marker.DELETEALL
        delete.pose.orientation.w = 1.0
        array.markers.append(delete)

        converted = [(i, w['map']) for i, w in enumerate(self.waypoints)
                     if w['map'] is not None]
        if converted:
            line = self.new_marker(Marker.LINE_STRIP, 1)
            line.scale.x = self.p('line_width')
            line.color.r, line.color.g, line.color.b, line.color.a = 0.2, 0.6, 1.0, 1.0
            line.points = [self.marker_point(x, y, 0.0) for _, (x, y) in converted]
            array.markers.append(line)
            for index, (x, y) in converted:
                is_target = self.state == 'RUNNING' and index == self.index
                sphere = self.new_marker(Marker.SPHERE, 1000 + index)
                sphere.pose.position = self.marker_point(x, y, 0.0)
                size = self.p('target_marker_scale') if is_target else self.p('waypoint_marker_scale')
                sphere.scale.x = sphere.scale.y = sphere.scale.z = size
                if is_target:
                    sphere.color.r, sphere.color.g, sphere.color.b = 1.0, 0.85, 0.0
                else:
                    sphere.color.r, sphere.color.g, sphere.color.b = 0.1, 0.7, 1.0
                sphere.color.a = 1.0
                array.markers.append(sphere)
                label = self.new_marker(Marker.TEXT_VIEW_FACING, 2000 + index)
                label.pose.position = self.marker_point(x, y, self.p('text_height'))
                label.scale.z = self.p('text_height')
                label.color.r = label.color.g = label.color.b = label.color.a = 1.0
                label.text = str(index + 1)
                array.markers.append(label)
        self.marker_pub.publish(array)

    def new_marker(self, marker_type, marker_id):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'waypoints'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    @staticmethod
    def marker_point(x, y, z):
        point = Point()
        point.x, point.y, point.z = x, y, z
        return point


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
