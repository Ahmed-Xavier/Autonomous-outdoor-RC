import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry

BIG = 1e6


class OdometryNode(Node):
    """Turns encoder tick count into wheel speed (m/s) on /wheel/odom."""

    def __init__(self):
        super().__init__('odometry_node')
        self.declare_parameter('ticks_per_meter', 0.0)
        self.declare_parameter('encoder_topic', '/esp32/encoder')
        self.declare_parameter('speed_variance', 0.05)

        self.last_ticks = None
        self.last_time = None

        self.pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        topic = self.get_parameter('encoder_topic').value
        self.create_subscription(Int32, topic, self.on_encoder, 10)

    def on_encoder(self, msg):
        tpm = self.get_parameter('ticks_per_meter').value
        if tpm <= 0.0:
            self.get_logger().error(
                'ticks_per_meter is 0.0 - set it in config.yaml',
                throttle_duration_sec=5.0)
            return

        now = self.get_clock().now()
        if self.last_ticks is None:
            self.last_ticks, self.last_time = msg.data, now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt < 1e-4:
            return
        dticks = msg.data - self.last_ticks
        self.last_ticks, self.last_time = msg.data, now

        speed = dticks / dt / tpm

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = speed

        pose_cov = [0.0] * 36
        twist_cov = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):
            pose_cov[i] = BIG
            twist_cov[i] = BIG
        twist_cov[0] = self.get_parameter('speed_variance').value
        odom.pose.covariance = pose_cov
        odom.twist.covariance = twist_cov

        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
