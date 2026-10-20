#!/usr/bin/env python3
"""Static TF + RViz markers for the NRDS warehouse AMR fleet (AMR_1 ... AMR_N).

Publishes, from the roster the launch file writes (robot id -> spawn pose):
  map -> AMR_i/odom                        (spawn pose; DiffDrive odometry starts at 0 there)
  AMR_i/base_link -> AMR_i/base_link/lidar_front_right, lidar_rear_left
                                           (scan frame ids after ros_gz_bridge maps '::' to '/')
a MarkerArray on /amr_markers with a box + AMR_i label per robot, locked to AMR_i/base_link,
and the roofless warehouse mesh on /warehouse_mesh (latched) so RViz shows the building.
"""
import math
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

# Lidar mounts from models/amr_01/model.sdf: (x, y, z, yaw)
LIDARS = {
    'lidar_front_right': (0.565, -0.355, 0.1225, -0.7854),
    'lidar_rear_left': (-0.565, 0.355, 0.1225, 2.3562),
}


def transform(parent, child, x, y, z, yaw, stamp):
    t = TransformStamped()
    t.header.frame_id, t.header.stamp, t.child_frame_id = parent, stamp, child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = float(x), float(y), float(z)
    t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return t


class StaticFrames(Node):
    def __init__(self, spawn_file):
        super().__init__('nrds_static_frames')
        with open(spawn_file) as f:
            self.robots = yaml.safe_load(f)['robots']
        stamp = self.get_clock().now().to_msg()
        tfs = []
        for name, p in self.robots.items():
            tfs.append(transform('map', f'{name}/odom', p['x'], p['y'], p.get('z', 0), p.get('yaw', 0), stamp))
            for lidar, (x, y, z, yaw) in LIDARS.items():
                tfs.append(transform(f'{name}/base_link', f'{name}/base_link/{lidar}', x, y, z, yaw, stamp))
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster.sendTransform(tfs)
        self.marker_pub = self.create_publisher(MarkerArray, 'amr_markers', 1)
        self.create_timer(1.0, self.publish_markers)
        self.mesh_pub = self.create_publisher(
            Marker, 'warehouse_mesh', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.mesh_pub.publish(self.warehouse_mesh())
        self.get_logger().info(f'Published {len(tfs)} static transforms for {len(self.robots)} robots')

    def warehouse_mesh(self):
        mesh = Marker()
        mesh.header.frame_id = 'map'
        mesh.ns, mesh.id, mesh.type = 'warehouse', 0, Marker.MESH_RESOURCE
        mesh.mesh_resource = 'file://' + get_package_share_directory('nrds_warehouse_sim') + \
            '/models/nrds_warehouse/meshes/nrds_warehouse_no_roof.glb'
        mesh.mesh_use_embedded_materials = True
        # glTF is Y-up: roll +90 deg, same as the Gazebo visual pose
        mesh.pose.orientation.x = mesh.pose.orientation.w = math.sqrt(0.5)
        mesh.scale.x = mesh.scale.y = mesh.scale.z = 1.0
        return mesh

    def publish_markers(self):
        markers = MarkerArray()
        for i, name in enumerate(self.robots):
            body = Marker()
            body.header.frame_id = f'{name}/base_link'
            body.ns, body.id, body.type, body.frame_locked = 'amr_body', i, Marker.CUBE, True
            body.pose.position.z = 0.2
            body.pose.orientation.w = 1.0
            body.scale.x, body.scale.y, body.scale.z = 1.2, 0.78, 0.4
            body.color.r, body.color.g, body.color.b, body.color.a = 0.35, 0.5, 0.65, 1.0
            label = Marker()
            label.header.frame_id = f'{name}/base_link'
            label.ns, label.id, label.type, label.frame_locked = 'amr_label', i, Marker.TEXT_VIEW_FACING, True
            label.text = name
            label.pose.position.z = 1.0
            label.pose.orientation.w = 1.0
            label.scale.z = 0.6
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            markers.markers += [body, label]
        self.marker_pub.publish(markers)


def main():
    rclpy.init(args=sys.argv)
    node = StaticFrames(rclpy.utilities.remove_ros_args(sys.argv)[1])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
