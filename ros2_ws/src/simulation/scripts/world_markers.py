#!/usr/bin/env python3
"""Publish the Silesia Ring world as RViz/Foxglove markers."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

PKG = 'package://simulation/'
MESH_DIR = PKG + 'worlds/models/silesia_ring/meshes/'
GROUND = MESH_DIR + 'silesia_ring.glb'
BOUNDS = MESH_DIR + 'track_boundaries.obj'


class WorldMarkers(Node):
    def __init__(self):
        super().__init__('world_markers')
        self.declare_parameter('frame_id', 'map')
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(MarkerArray, '/world_markers', qos)
        self.timer = self.create_timer(1.0, self.publish)
        self.publish()

    def _mesh(self, mid, name, uri, rgba=None):
        m = Marker()
        m.header.frame_id = self.get_parameter('frame_id').value
        m.ns, m.id = name, mid
        m.type, m.action = Marker.MESH_RESOURCE, Marker.ADD
        m.mesh_resource = uri
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        if rgba is None:
            m.mesh_use_embedded_materials = True
            m.color.a = 1.0
        else:
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
        return m

    def publish(self):
        arr = MarkerArray()

        # Ground: Foxglove already rotates it, so no extra rotation.
        arr.markers.append(self._mesh(0, 'ground', GROUND))

        self.pub.publish(arr)


def main():
    rclpy.init()
    rclpy.spin(WorldMarkers())


if __name__ == '__main__':
    main()
