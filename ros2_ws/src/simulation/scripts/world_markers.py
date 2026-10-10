#!/usr/bin/env python3
"""Publish the selected simulation environment as Foxglove MarkerArray geometry."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

GROUND = 'package://simulation/worlds/models/silesia_ring/meshes/silesia_ring.glb'

class WorldMarkers(Node):
    def __init__(self):
        super().__init__('world_markers')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('world', 'silesia_ring')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(MarkerArray, '/world_markers', qos)
        self.timer = self.create_timer(1.0, self.publish)
        self.publish()

    def marker(self, mid, ns, kind, xyz, scale, rgba, yaw=0.0):
        m=Marker(); m.header.frame_id=self.get_parameter('frame_id').value
        m.ns, m.id, m.type, m.action=ns, mid, kind, Marker.ADD
        m.pose.position.x,m.pose.position.y,m.pose.position.z=xyz
        m.pose.orientation.z,m.pose.orientation.w=math.sin(yaw/2),math.cos(yaw/2)
        m.scale.x,m.scale.y,m.scale.z=scale
        m.color.r,m.color.g,m.color.b,m.color.a=rgba
        return m

    def publish(self):
        arr=MarkerArray()
        if self.get_parameter('world').value == 'silesia_ring':
            m=self.marker(0,'silesia_ring_ground',Marker.MESH_RESOURCE,(0,0,0),(1,1,1),(1,1,1,1))
            m.mesh_resource=GROUND; m.mesh_use_embedded_materials=True; arr.markers.append(m)
        else:
            cube=Marker.CUBE; asphalt=(.16,.17,.18,1); white=(.95,.95,.95,1)
            arr.markers.append(self.marker(0,'challenge_ground',cube,(0,0,-.12),(100,70,.2),(.58,.60,.61,1)))
            for i,(x,y,sx,sy) in enumerate([(0,-15,60,6),(0,15,60,6),(-30,0,6,30),(30,0,6,30)],1):
                arr.markers.append(self.marker(i,'road',cube,(x,y,0),(sx,sy,.04),asphalt))
            for i,(x,y,sx,sy,yaw) in enumerate([(0,-18,60,.12,0),(0,-12,60,.12,0),(0,18,60,.12,0),(0,12,60,.12,0),(-33,0,30,.12,math.pi/2),(33,0,30,.12,math.pi/2)],5):
                arr.markers.append(self.marker(i,'edge',cube,(x,y,.025),(sx,sy,.02),white,yaw))
            arr.markers.append(self.marker(11,'start_finish',cube,(-10,-15,.035),(.15,6,.02),(1,.82,.02,1)))
            arr.markers.append(self.marker(12,'stop_line',cube,(20,-15,.035),(.15,6,.02),(1,.32,.02,1)))
            for i,(x,y) in enumerate([(8,-13),(14,-17),(0,12.7)],13):
                arr.markers.append(self.marker(i,'obstacles',Marker.CYLINDER,(x,y,.3),(.36,.36,.6),(.95,.36,.02,1)))
            arr.markers.append(self.marker(16,'stop_sign',cube,(22,-11.3,1.3),(.12,.85,.85),(.8,.02,.02,1)))
        self.pub.publish(arr)

def main():
    rclpy.init(); node=WorldMarkers()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
