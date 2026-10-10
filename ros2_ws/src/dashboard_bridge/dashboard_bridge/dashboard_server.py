import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from dashboard_bridge.render import fit_track, render


class DashboardServer(Node):
    def __init__(self):
        super().__init__('dashboard_server')
        for parameter in ('track_path', 'port', 'rotate', 'speed_max',
                          'stale_timeout_s', 'track_smoothing_sigma',
                          'supersample_factor', 'map_box'):
            self.declare_parameter(parameter)

        track_path = self.get_parameter('track_path').value
        if not os.path.isabs(track_path):
            track_path = os.path.join(
                get_package_share_directory('dashboard_bridge'), 'config', track_path)
        with open(track_path, encoding='utf-8') as track_file:
            raw_points = json.load(track_file)['points']

        map_box = self.get_parameter('map_box').value
        self.map_box = [int(coordinate) for coordinate in map_box]
        smoothing = float(self.get_parameter('track_smoothing_sigma').value)
        self.track = fit_track(raw_points, self.map_box, smoothing)
        self.port = int(self.get_parameter('port').value)
        self.rotate = int(self.get_parameter('rotate').value)
        self.speed_max = float(self.get_parameter('speed_max').value)
        self.stale_timeout_s = float(self.get_parameter('stale_timeout_s').value)
        self.supersample_factor = int(self.get_parameter('supersample_factor').value)

        self._telemetry_lock = threading.Lock()
        self._telemetry = {
            'speed': {'value': None, 'arrival': None},
            'battery': {'value': None, 'arrival': None},
            'temperature': {'value': None, 'arrival': None},
            'progress': {'value': None, 'arrival': None},
        }
        self._frame_lock = threading.Lock()
        self._frame = 0

        self.create_subscription(Odometry, '/odom', self._on_odometry, 10)
        self.create_subscription(Float32, '/track/progress',
                                 lambda msg: self._record('progress', msg.data), 10)
        self.create_subscription(Float32, '/vehicle/battery_percent',
                                 lambda msg: self._record('battery', msg.data), 10)
        self.create_subscription(Float32, '/vehicle/temperature_c',
                                 lambda msg: self._record('temperature', msg.data), 10)

        handler = self._make_handler()
        self.http_server = ThreadingHTTPServer(('0.0.0.0', self.port), handler)
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever, name='dashboard-http', daemon=True)
        self.http_thread.start()
        print(f'Dashboard on http://0.0.0.0:{self.port}/dash.png', flush=True)

    def _record(self, name, value):
        with self._telemetry_lock:
            self._telemetry[name]['value'] = float(value)
            self._telemetry[name]['arrival'] = time.monotonic()

    def _on_odometry(self, message):
        self._record('speed', message.twist.twist.linear.x * 3.6)

    def _snapshot(self):
        now = time.monotonic()
        with self._telemetry_lock:
            snapshot = {}
            for name, item in self._telemetry.items():
                fresh = (item['arrival'] is not None and
                         now - item['arrival'] <= self.stale_timeout_s)
                snapshot[name] = item['value'] if fresh else None
        return snapshot

    def _render_image(self):
        with self._frame_lock:
            self._frame += 1
            frame = self._frame
        values = self._snapshot()
        return render(
            track=self.track,
            speed=values['speed'],
            battery=values['battery'],
            temp=values['temperature'],
            progress=values['progress'],
            frame=frame,
            clock_text=time.strftime('%H:%M:%S'),
            rotate=self.rotate,
            speed_max=self.speed_max,
            map_box=self.map_box,
            supersample_factor=self.supersample_factor,
        )

    def _make_handler(self):
        dashboard = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                path = urlsplit(self.path).path
                if path.endswith('/favicon.ico'):
                    self.send_error(404)
                    return
                if path not in ('/', '/dash.png'):
                    self.send_error(404)
                    return
                image = dashboard._render_image()
                self.send_response(200)
                self.send_header('Content-Type', 'image/png')
                self.send_header('Content-Length', str(len(image)))
                self.end_headers()
                self.wfile.write(image)

            def log_message(self, *args):
                pass

        return Handler

    def destroy_node(self):
        self.http_server.shutdown()
        self.http_server.server_close()
        self.http_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
