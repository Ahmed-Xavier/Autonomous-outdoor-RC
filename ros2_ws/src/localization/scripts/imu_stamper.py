#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuStamper(Node):
    def __init__(self):
        super().__init__('imu_stamper')
        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        self.create_subscription(Imu, '/esp32/imu', self.cb, 10)

    def cb(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ImuStamper()
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