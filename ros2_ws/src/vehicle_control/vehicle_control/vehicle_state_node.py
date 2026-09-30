#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16, Int32
from tf2_ros import TransformBroadcaster


class VehicleState(Node):
    """
    /esp32/servo_cmd + /esp32/encoder  ->  /joint_states + TF odom->base_footprint
    Dead reckoning with a bicycle model (no IMU yet).
    """

    def __init__(self):
        super().__init__('vehicle_state')

        # Geometry (same numbers as the URDF)
        self.declare_parameter('wheel_radius', 0.025)
        self.declare_parameter('wheel_base', 0.185)
        self.declare_parameter('track_width', 0.11)

        # Calibration: PLACEHOLDERS, measure on the real car
        self.declare_parameter('ticks_per_meter', 1000.0)
        self.declare_parameter('encoder_sign', 1.0)
        self.declare_parameter('servo_center', 50.0)
        self.declare_parameter('steer_rad_per_servo_deg', math.radians(1.0))
        self.declare_parameter('steer_sign', -1.0)

        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.delta = 0.0          # steering angle at the centre (rad), + = left
        self.last_ticks = None
        self.spin = {'rl': 0.0, 'rr': 0.0, 'fl': 0.0, 'fr': 0.0}

        self.tf_broadcaster = TransformBroadcaster(self)
        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.create_subscription(
            Int32, '/esp32/encoder', self.on_encoder, qos_profile_sensor_data)
        self.create_subscription(
            Int16, '/esp32/servo_cmd', self.on_servo, qos_profile_sensor_data)

        self.create_timer(1.0 / 30.0, self.publish_state)
        self.get_logger().info('vehicle_state started (placeholder calibration)')

    def p(self, name):
        return self.get_parameter(name).value

    def on_servo(self, msg):
        # servo below centre = left = positive steering angle
        self.delta = (self.p('steer_sign')
                      * (self.p('servo_center') - msg.data)
                      * self.p('steer_rad_per_servo_deg'))

    def on_encoder(self, msg):
        if self.last_ticks is None:
            self.last_ticks = msg.data
            return
        d = (self.p('encoder_sign') * (msg.data - self.last_ticks)
             / self.p('ticks_per_meter'))
        self.last_ticks = msg.data
        self.integrate(d)

    def integrate(self, d):
        L = self.p('wheel_base')
        tw = self.p('track_width')
        r = self.p('wheel_radius')
        t = math.tan(self.delta)

        dyaw = d * t / L
        self.x += d * math.cos(self.yaw + dyaw / 2.0)
        self.y += d * math.sin(self.yaw + dyaw / 2.0)
        self.yaw += dyaw

        # Each wheel travels a different distance in a turn.
        # The turn centre is at (0, R) in base_link, R = L / tan(delta).
        wheels = {'rl': (0.0, tw / 2), 'rr': (0.0, -tw / 2),
                  'fl': (L, tw / 2), 'fr': (L, -tw / 2)}
        for k, (px, py) in wheels.items():
            if abs(t) < 1e-6:
                dist = d
            else:
                R = L / t
                dist = d * math.hypot(px, py - R) / abs(R)
            self.spin[k] = (self.spin[k] + dist / r) % (2.0 * math.pi)

    def publish_state(self):
        now = self.get_clock().now().to_msg()
        L = self.p('wheel_base')
        tw = self.p('track_width')
        t = math.tan(self.delta)

        # Ackermann: the inner wheel turns more than the outer one
        delta_left = math.atan2(L * t, L - (tw / 2) * t)
        delta_right = math.atan2(L * t, L + (tw / 2) * t)

        js = JointState()
        js.header.stamp = now
        js.name = [
            'rear_left_wheel_joint', 'rear_right_wheel_joint',
            'front_left_steer_joint', 'front_right_steer_joint',
            'front_left_wheel_joint', 'front_right_wheel_joint',
        ]
        js.position = [
            self.spin['rl'], self.spin['rr'],
            delta_left, delta_right,
            self.spin['fl'], self.spin['fr'],
        ]
        self.js_pub.publish(js)

        if self.p('publish_tf'):
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.p('odom_frame')
            tf.child_frame_id = self.p('base_frame')
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = math.sin(self.yaw / 2.0)
            tf.transform.rotation.w = math.cos(self.yaw / 2.0)
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
