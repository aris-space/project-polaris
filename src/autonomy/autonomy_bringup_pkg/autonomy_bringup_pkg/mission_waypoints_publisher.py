#!/usr/bin/env python3
"""Publish a WGS84 mission CSV's waypoints as GeoJSON for Foxglove visualization."""

import csv
import json
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from ament_index_python.packages import get_package_share_directory
from foxglove_msgs.msg import GeoJSON


def default_mission_csv_path() -> str:
    share = get_package_share_directory('autonomy_bringup_pkg')
    return f'{share}/missions/goldbach_straightline_wgs84_mission.csv'


class MissionWaypointsPublisher(Node):
    """Publishes mission CSV waypoints as a GeoJSON FeatureCollection for Foxglove.

    Publishes once at startup and once on every mission_csv parameter change.
    Uses Transient Local QoS so late subscribers (Foxglove reconnect) get the
    current mission on connection without needing a periodic republish.
    """

    def __init__(self):
        super().__init__("mission_waypoints_publisher")

        self.declare_parameter("mission_csv", default_mission_csv_path())

        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(
            GeoJSON, "/mission_waypoints", latched_qos
        )

        self._dirty = False
        self.add_on_set_parameters_callback(self._validate_params)
        self.create_timer(0.1, self._check_and_publish)

        self._publish_mission()
        self.get_logger().info(
            "MissionWaypointsPublisher ready (topic=/mission_waypoints)"
        )

    def _load_waypoints(self, csv_path: str):
        points = []
        with open(csv_path, newline='') as f:
            reader = csv.DictReader(f)
            if reader.fieldnames is None or 'lat' not in reader.fieldnames or 'lon' not in reader.fieldnames:
                raise ValueError("CSV missing required 'lat'/'lon' columns")
            idx = 0
            for row in reader:
                if row is None:
                    continue
                lat_s = (row.get('lat') or '').strip()
                lon_s = (row.get('lon') or '').strip()
                if not lat_s or not lon_s:
                    continue
                try:
                    lat = float(lat_s)
                    lon = float(lon_s)
                except ValueError:
                    continue
                points.append((idx, lat, lon))
                idx += 1
        return points

    def _to_geojson(self, points):
        features = [
            {
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": [lon, lat]},
                "properties": {"id": pid},
            }
            for pid, lat, lon in points
        ]
        return json.dumps({"type": "FeatureCollection", "features": features})

    def _publish_mission(self):
        csv_path = str(self.get_parameter("mission_csv").value)
        if not csv_path or not os.path.isfile(csv_path):
            self.get_logger().error(f"Mission CSV not found: {csv_path}")
            return
        try:
            points = self._load_waypoints(csv_path)
        except (OSError, ValueError) as e:
            self.get_logger().error(f"Failed to read mission CSV {csv_path}: {e}")
            return
        if not points:
            self.get_logger().error(f"No valid waypoints in {csv_path}")
            return
        msg = GeoJSON()
        msg.geojson = self._to_geojson(points)
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published mission: {len(points)} waypoints from {csv_path}"
        )

    def _validate_params(self, params):
        for p in params:
            if p.name == "mission_csv" and not isinstance(p.value, str):
                return SetParametersResult(
                    successful=False, reason="mission_csv must be a string"
                )
        if any(p.name == "mission_csv" for p in params):
            self._dirty = True
        return SetParametersResult(successful=True)

    def _check_and_publish(self):
        if self._dirty:
            self._dirty = False
            self._publish_mission()


def main(args=None):
    rclpy.init(args=args)
    node = MissionWaypointsPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
