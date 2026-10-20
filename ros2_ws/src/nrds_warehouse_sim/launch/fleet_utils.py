"""Fleet helpers for the NRDS warehouse launch files (no ROS imports, unit-testable).

* :func:`roster` reads ``config/fleet.yaml`` (+ charger poses from ``config/locations.yaml``)
  and returns ``[(robot_id, x, y, yaw, spawn_station)]`` for ``AMR_1 ... AMR_N``.
* :func:`robot_sdf` renders one AMR-01 from ``models/amr_01/model.sdf`` with a unique
  model name, namespaced odometry/TF frames, a sensor profile, and a chassis contact sensor.
* :func:`bridge_config` writes the ros_gz_bridge YAML for the whole fleet.
"""

from __future__ import annotations

import os
import re

import yaml

LIDARS = {
    "lidar_front_right": (0.565, -0.355, 0.1225, -0.7854),
    "lidar_rear_left": (-0.565, 0.355, 0.1225, 2.3562),
}


def roster(pkg: str, count: int | None = None) -> list[tuple[str, float, float, float, str]]:
    with open(os.path.join(pkg, "config", "fleet.yaml")) as f:
        fleet = yaml.safe_load(f)
    with open(os.path.join(pkg, "config", "locations.yaml")) as f:
        loc = yaml.safe_load(f)
    poses = {f"charger_{k}": (v["x"], v["y"], v.get("yaw", 0.0)) for k, v in (loc.get("chargers") or {}).items()}
    poses.update({k: (v["x"], v["y"], v.get("yaw", 0.0)) for k, v in (fleet.get("parking") or {}).items()})

    def num(rid: str) -> int:
        return int(rid.split("_")[1])

    out = []
    for rid in sorted(fleet["robots"], key=num):
        st = fleet["robots"][rid]["spawn"]
        x, y, yaw = poses[st]
        out.append((rid, float(x), float(y), float(yaw), st))
    return out[: count] if count else out


def robot_sdf(pkg: str, rid: str, profile: str = "fleet", lidar_hz: float = 10.0, beams: int = 271) -> str:
    """AMR-01 SDF for one robot.

    ``light``: IMU + contact; lidar scans are emulated on the CPU (20-robot default on a 4 GB GPU)
    ``fleet``: 2 GPU lidars at 10 Hz / 271 beams + IMU + contact
    ``full`` : every sensor of the model, as authored
    """
    with open(os.path.join(pkg, "models", "amr_01", "model.sdf")) as f:
        sdf = f.read()
    sdf = re.sub(r"<!--.*?-->", "", sdf, flags=re.S)
    sdf = sdf.replace('<model name="amr_01">', f'<model name="{rid}">')
    if profile == "light":
        # physics + IMU + contact only; scans come from nrda_fleet_ros/lidar_emulator (CPU ray casting)
        sdf = re.sub(r'\s*<sensor name="(front_depth|upward_camera|lidar_front_right|lidar_rear_left)" type="[a-z_]+">.*?</sensor>',
                     "", sdf, flags=re.S)
    # lidars render only visibility flag 2: the lidar proxy of the warehouse, robots, chargers, obstacles
    sdf = sdf.replace("<lidar>\n", "<lidar>\n          <visibility_mask>2</visibility_mask>\n")
    if profile == "fleet":
        sdf = re.sub(r'\s*<sensor name="front_depth" type="rgbd_camera">.*?</sensor>', "", sdf, flags=re.S)
        sdf = re.sub(r'\s*<sensor name="upward_camera" type="camera">.*?</sensor>', "", sdf, flags=re.S)
        sdf = sdf.replace("<update_rate>15</update_rate>", f"<update_rate>{lidar_hz:g}</update_rate>")
        sdf = sdf.replace("<samples>541</samples>", f"<samples>{int(beams)}</samples>")
    contact = (
        '\n      <sensor name="chassis_contact" type="contact">\n'
        '        <update_rate>50</update_rate>\n'
        '        <always_on>true</always_on>\n'
        '        <contact><collision>chassis</collision></contact>\n'
        '      </sensor>')
    sdf = sdf.replace("    </link>\n    <link name=\"left_wheel\">", contact + "\n    </link>\n    <link name=\"left_wheel\">", 1)
    sdf = sdf.replace(
        "<odom_publish_frequency>50</odom_publish_frequency>",
        f"<odom_publish_frequency>50</odom_publish_frequency>\n      <frame_id>{rid}/odom</frame_id>\n"
        f"      <child_frame_id>{rid}/base_link</child_frame_id>")
    # true world pose with the model name in the header (dynamic_pose/info carries no frame names through
    # ros_gz_bridge): lidar emulator + independent safety monitor
    sdf = sdf.replace(
        '<plugin filename="gz-sim-joint-state-publisher-system"',
        '<plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">\n'
        '      <publish_link_pose>false</publish_link_pose>\n      <publish_visual_pose>false</publish_visual_pose>\n'
        '      <publish_collision_pose>false</publish_collision_pose>\n      <publish_sensor_pose>false</publish_sensor_pose>\n'
        '      <publish_nested_model_pose>false</publish_nested_model_pose>\n      <publish_model_pose>true</publish_model_pose>\n'
        '      <use_pose_vector_msg>true</use_pose_vector_msg>\n      <static_publisher>false</static_publisher>\n'
        '      <update_frequency>20</update_frequency>\n    </plugin>\n    '
        '<plugin filename="gz-sim-joint-state-publisher-system"', 1)
    return sdf


def bridge_config(robots: list[str], profile: str = "fleet", world: str = "nrds_warehouse") -> list[dict]:
    cfg = [dict(ros_topic_name="/clock", gz_topic_name="/clock", ros_type_name="rosgraph_msgs/msg/Clock",
                gz_type_name="gz.msgs.Clock", direction="GZ_TO_ROS")]
    for rid in robots:
        base = f"/world/{world}/model/{rid}/link/base_link/sensor"
        cfg += [
            # true world pose (PosePublisher, child_frame_id = robot id): lidar emulator + safety monitor only
            dict(ros_topic_name="/nrda/ground_truth", gz_topic_name=f"/model/{rid}/pose",
                 ros_type_name="tf2_msgs/msg/TFMessage", gz_type_name="gz.msgs.Pose_V", direction="GZ_TO_ROS"),
            dict(ros_topic_name=f"/{rid}/cmd_vel", gz_topic_name=f"/model/{rid}/cmd_vel",
                 ros_type_name="geometry_msgs/msg/Twist", gz_type_name="gz.msgs.Twist", direction="ROS_TO_GZ"),
            dict(ros_topic_name=f"/{rid}/odom", gz_topic_name=f"/model/{rid}/odometry",
                 ros_type_name="nav_msgs/msg/Odometry", gz_type_name="gz.msgs.Odometry", direction="GZ_TO_ROS"),
            dict(ros_topic_name="/tf", gz_topic_name=f"/model/{rid}/tf",
                 ros_type_name="tf2_msgs/msg/TFMessage", gz_type_name="gz.msgs.Pose_V", direction="GZ_TO_ROS"),
            dict(ros_topic_name=f"/{rid}/contacts", gz_topic_name=f"{base}/chassis_contact/contact",
                 ros_type_name="ros_gz_interfaces/msg/Contacts", gz_type_name="gz.msgs.Contacts",
                 direction="GZ_TO_ROS"),
            dict(ros_topic_name=f"/{rid}/imu", gz_topic_name=f"{base}/imu/imu",
                 ros_type_name="sensor_msgs/msg/Imu", gz_type_name="gz.msgs.IMU", direction="GZ_TO_ROS"),
        ]
        if profile != "light":
            cfg += [
                dict(ros_topic_name=f"/{rid}/scan_front", gz_topic_name=f"{base}/lidar_front_right/scan",
                     ros_type_name="sensor_msgs/msg/LaserScan", gz_type_name="gz.msgs.LaserScan",
                     direction="GZ_TO_ROS"),
                dict(ros_topic_name=f"/{rid}/scan_rear", gz_topic_name=f"{base}/lidar_rear_left/scan",
                     ros_type_name="sensor_msgs/msg/LaserScan", gz_type_name="gz.msgs.LaserScan",
                     direction="GZ_TO_ROS"),
            ]
        if profile == "full":
            cam = f"{base}/front_depth"
            cfg += [
                dict(ros_topic_name=f"/{rid}/front_camera/image", gz_topic_name=f"{cam}/image",
                     ros_type_name="sensor_msgs/msg/Image", gz_type_name="gz.msgs.Image", direction="GZ_TO_ROS"),
                dict(ros_topic_name=f"/{rid}/front_camera/depth", gz_topic_name=f"{cam}/depth_image",
                     ros_type_name="sensor_msgs/msg/Image", gz_type_name="gz.msgs.Image", direction="GZ_TO_ROS"),
            ]
    return cfg


def floor_boxes(pkg: str) -> dict:
    path = os.path.join(pkg, "config", "floor_boxes.yaml")
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        return (yaml.safe_load(f) or {}).get("boxes") or {}


def box_sdf(name: str, length: float, width: float, height: float) -> str:
    """A static cardboard box with packing tape: a test obstacle the robots do not have in their map."""
    return f"""<?xml version='1.0'?><sdf version='1.9'><model name='{name}'><static>true</static><link name='box'>
<collision name='c'><pose>0 0 {height / 2} 0 0 0</pose><geometry><box><size>{length} {width} {height}</size></box></geometry></collision>
<visual name='v'><pose>0 0 {height / 2} 0 0 0</pose><geometry><box><size>{length} {width} {height}</size></box></geometry>
<material><ambient>0.72 0.52 0.30 1</ambient><diffuse>0.72 0.52 0.30 1</diffuse></material></visual>
<visual name='tape'><pose>0 0 {height + 0.002} 0 0 0</pose><geometry><box><size>{length} 0.08 0.004</size></box></geometry>
<material><ambient>0.9 0.85 0.6 1</ambient><diffuse>0.9 0.85 0.6 1</diffuse></material></visual>
</link></model></sdf>"""
