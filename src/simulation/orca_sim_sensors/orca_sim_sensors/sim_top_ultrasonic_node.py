#!/usr/bin/env python3
"""Synthetic top ultrasonic (ice ceiling range) for the Polaris simulation.

A perfect sensor: one ray from the sensor mount along body +z (straight up in base_link),
cast against the ice layer and the water surface. Publishes what the vehicle's
ultrasonic_driver publishes in the 'top' namespace:

  /top/ultrasonic/distance   std_msgs/Float32   range in metres

The obstacles come from the world file Gazebo loads, so they always match the physics:
every <box> collision of the model named ``ice_model_name`` (axis-aligned, as in
ice.world), plus the water surface plane z = ``water_surface_z``, which the sensor sees
from below through the hole. With no ice model in the world (sand.world), only the
surface. No hit within ``max_range_m`` (2.5 m): 0.0, as the real sensor reports out of range.

Mount position (base_link, FLU), derived from the vehicle's parameters:
  Keller pressure sensor  0.005607 -0.000299 0.117520   (keller_26x_pkg launch, static TF)
  tower top               Keller + 0.136 m in z           (ice_touch_detection pressure_sensor_offset_m)
  ultrasonic              tower top - 0.148 m in z        (tower_height_m)
                          and 0.490 m in front of it in x (tower_horizontal_offset_m)
so ``distance`` is 0.148 m when the tower top touches a level ice ceiling.

Ground truth: Gazebo /odom, base_link pose in the world frame (ENU, water surface z = 0).
"""

import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32

from orca_sim_sensors.common import quat_to_matrix

_KELLER_XYZ = (0.005607, -0.000299, 0.117520)
_PRESSURE_BELOW_TOWER_TOP_M = 0.136
_TOWER_ABOVE_SENSOR_M = 0.148
_TOWER_BEHIND_SENSOR_M = 0.490
DEFAULT_SENSOR_XYZ = [
    _KELLER_XYZ[0] + _TOWER_BEHIND_SENSOR_M,
    _KELLER_XYZ[1],
    _KELLER_XYZ[2] + _PRESSURE_BELOW_TOWER_TOP_M - _TOWER_ABOVE_SENSOR_M,
]


def _pose_xyz(elem):
    """Translation of an SDF <pose> child; rejects rotated boxes (ray-cast assumes AABBs)."""
    pose = elem.find('pose') if elem is not None else None
    if pose is None or not pose.text:
        return np.zeros(3)
    v = [float(x) for x in pose.text.split()]
    if any(abs(a) > 1e-9 for a in v[3:6]):
        raise ValueError(f'rotated pose not supported: {pose.text.strip()}')
    return np.array(v[:3])


def load_ice_boxes(world_file, model_name):
    """(min_corner, max_corner) world-frame AABBs of the model's box collisions."""
    root = ET.parse(world_file).getroot()
    boxes = []
    for model in root.iter('model'):
        if model.get('name') != model_name:
            continue
        model_xyz = _pose_xyz(model)
        for link in model.findall('link'):
            link_xyz = model_xyz + _pose_xyz(link)
            for col in link.findall('collision'):
                size = col.find('geometry/box/size')
                if size is None:
                    continue
                centre = link_xyz + _pose_xyz(col)
                half = np.array([float(x) for x in size.text.split()]) / 2.0
                boxes.append((centre - half, centre + half))
    return boxes


def ray_box(origin, direction, lo, hi):
    """Distance along the ray to the box (slab test), 0 if inside, None if missed."""
    t_near, t_far = -math.inf, math.inf
    for i in range(3):
        if abs(direction[i]) < 1e-12:
            if origin[i] < lo[i] or origin[i] > hi[i]:
                return None
            continue
        t1 = (lo[i] - origin[i]) / direction[i]
        t2 = (hi[i] - origin[i]) / direction[i]
        t_near = max(t_near, min(t1, t2))
        t_far = min(t_far, max(t1, t2))
    if t_near > t_far or t_far < 0.0:
        return None
    return max(t_near, 0.0)


class SimTopUltrasonicNode(Node):
    def __init__(self):
        super().__init__('sim_top_ultrasonic_node')
        default_world = os.path.join(
            get_package_share_directory('orca_description'), 'worlds', 'ice.world')
        self.declare_parameter('world_file', default_world)
        self.declare_parameter('ice_model_name', 'ice_layer')
        self.declare_parameter('sensor_xyz', DEFAULT_SENSOR_XYZ)
        self.declare_parameter('water_surface_z', 0.0)
        self.declare_parameter('max_range_m', 2.5)
        self.declare_parameter('out_of_range_value', 0.0)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('topic', '/top/ultrasonic/distance')

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self._sensor = np.array([float(v) for v in p('sensor_xyz')])
        self._surface_z = float(p('water_surface_z'))
        self._max_range = float(p('max_range_m'))
        self._out_of_range = float(p('out_of_range_value'))
        self._boxes = load_ice_boxes(p('world_file'), p('ice_model_name'))

        self._pub = self.create_publisher(Float32, p('topic'), 10)
        self._odom = None
        self.create_subscription(Odometry, '/odom', self._on_odom, qos_profile_sensor_data)
        self.create_timer(1.0 / float(p('rate_hz')), self._tick)
        self.get_logger().info(
            f"Top ultrasonic at base_link {np.round(self._sensor, 4).tolist()}, "
            f"{len(self._boxes)} ice box(es) from '{p('ice_model_name')}' in "
            f"{p('world_file')}, max range {self._max_range:.1f} m")

    def _on_odom(self, msg):
        self._odom = msg

    def range_from_pose(self, position, rotation):
        """Distance from the sensor to the first obstacle along body +z, or None."""
        origin = position + rotation @ self._sensor
        direction = rotation[:, 2]
        hits = [d for d in (ray_box(origin, direction, lo, hi) for lo, hi in self._boxes)
                if d is not None]
        if origin[2] < self._surface_z and direction[2] > 1e-9:
            hits.append((self._surface_z - origin[2]) / direction[2])
        if not hits or min(hits) > self._max_range:
            return None
        return min(hits)

    def _tick(self):
        if self._odom is None:
            return
        pose = self._odom.pose.pose
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        distance = self.range_from_pose(position, quat_to_matrix(pose.orientation))
        if distance is None:
            distance = self._out_of_range
        self._pub.publish(Float32(data=float(distance)))


def main():
    rclpy.init()
    node = SimTopUltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
