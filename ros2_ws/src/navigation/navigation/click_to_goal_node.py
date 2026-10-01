#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from robot_localization.srv import ToLL
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener


class ClickToGoal(Node):
    def __init__(self):
        super().__init__('click_to_goal')

        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('goal_topic', '/goal_fix')
        self.declare_parameter('to_ll_service', '/toLL')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_pub = self.create_publisher(
            NavSatFix, self.get_parameter('goal_topic').value, 10)
        self.to_ll_client = self.create_client(
            ToLL, self.get_parameter('to_ll_service').value)
        self.click_sub = self.create_subscription(
            PointStamped,
            self.get_parameter('clicked_point_topic').value,
            self.on_click,
            10,
        )

    def on_click(self, msg):
        point_map = msg
        if msg.header.frame_id != 'map':
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', msg.header.frame_id, Time())
                point_map = do_transform_point(msg, transform)
            except TransformException as exc:
                self.get_logger().warning(
                    f'Could not transform clicked point from '
                    f"'{msg.header.frame_id}' to 'map': {exc}; click ignored")
                return

        if not self.to_ll_client.service_is_ready():
            self.get_logger().warning('no GPS datum yet, click ignored')
            return

        request = ToLL.Request()
        request.map_point.x = point_map.point.x
        request.map_point.y = point_map.point.y
        request.map_point.z = point_map.point.z

        future = self.to_ll_client.call_async(request)
        future.add_done_callback(
            lambda result: self.on_to_ll_response(
                result, point_map.point.x, point_map.point.y))

    def on_to_ll_response(self, future, x, y):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f'toLL request failed: {exc}')
            return

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps_link'
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = response.ll_point.latitude
        fix.longitude = response.ll_point.longitude
        fix.altitude = 0.0
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.goal_pub.publish(fix)
        self.get_logger().info(
            f'Clicked map point x={x:.3f}, y={y:.3f} -> '
            f'goal lat={fix.latitude:.8f}, lon={fix.longitude:.8f}')


def main(args=None):
    rclpy.init(args=args)
    node = ClickToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
