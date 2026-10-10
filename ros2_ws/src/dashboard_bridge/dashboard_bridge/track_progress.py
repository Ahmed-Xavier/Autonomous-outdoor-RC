import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class TrackProgress(Node):
    def __init__(self):
        super().__init__('track_progress')
        self.declare_parameter('track_path', 'track.json')
        self.declare_parameter('pixel_center_x', 300.0)
        self.declare_parameter('pixel_center_y', 300.0)
        self.declare_parameter('meters_per_pixel', 1.4321)
        self.declare_parameter('track_search_window', 20)
        self.declare_parameter('odometry_topic', '/odometry/global')

        track_path = self.get_parameter('track_path').value
        if not os.path.isabs(track_path):
            track_path = os.path.join(
                get_package_share_directory('dashboard_bridge'), 'config', track_path)
        with open(track_path, encoding='utf-8') as track_file:
            raw_points = json.load(track_file)['points']

        cx = float(self.get_parameter('pixel_center_x').value)
        cy = float(self.get_parameter('pixel_center_y').value)
        scale = float(self.get_parameter('meters_per_pixel').value)
        self.points = [((float(px) - cx) * scale, (cy - float(py)) * scale)
                       for px, py in raw_points]
        if len(self.points) < 2:
            raise ValueError('track.json must contain at least two points')
        self.segment_lengths = [
            math.dist(self.points[i], self.points[(i + 1) % len(self.points)])
            for i in range(len(self.points))]
        self.total_length = sum(self.segment_lengths)
        self.cumulative = [0.0]
        for length in self.segment_lengths:
            self.cumulative.append(self.cumulative[-1] + length)
        self.search_window = max(1, int(self.get_parameter('track_search_window').value))
        self.last_index = None
        self.publisher = self.create_publisher(Float32, '/track/progress', 10)
        topic = self.get_parameter('odometry_topic').value
        self.subscription = self.create_subscription(Odometry, topic, self.on_odometry, 10)

    def on_odometry(self, msg):
        position = msg.pose.pose.position
        x, y = float(position.x), float(position.y)
        count = len(self.points)
        if self.last_index is None:
            candidates = range(count)
        else:
            candidates = {(self.last_index + offset) % count
                          for offset in range(-self.search_window, self.search_window + 1)}

        best_distance_sq = float('inf')
        best_index = 0
        best_fraction = 0.0
        for index in candidates:
            ax, ay = self.points[index]
            bx, by = self.points[(index + 1) % count]
            dx, dy = bx - ax, by - ay
            length_sq = dx * dx + dy * dy
            fraction = 0.0 if length_sq == 0 else max(
                0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length_sq))
            near_x, near_y = ax + fraction * dx, ay + fraction * dy
            distance_sq = (x - near_x) ** 2 + (y - near_y) ** 2
            if distance_sq < best_distance_sq:
                best_distance_sq = distance_sq
                best_index = index
                best_fraction = fraction

        self.last_index = best_index
        along_track = self.cumulative[best_index] + best_fraction * self.segment_lengths[best_index]
        message = Float32()
        message.data = float(along_track / self.total_length) if self.total_length else 0.0
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = TrackProgress()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
