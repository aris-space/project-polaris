import json
import math

import numpy as np
import pyproj
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from foxglove_msgs.msg import GeoJSON


GRID_PARAMS = {
    "entry_lat",
    "entry_lon",
    "heading_deg",
    "drive_out_m",
    "rotation_deg",
    "grid_size_m",
    "spacing_m",
}


class GridPublisher(Node):
    """Publishes a square survey grid as a GeoJSON FeatureCollection for Foxglove.

    Publishes once at startup and once on every grid-parameter change.
    Uses Transient Local QoS so late subscribers (Foxglove reconnect) get the
    current grid on connection without needing a periodic republish.
    """

    def __init__(self):
        super().__init__("grid_publisher")

        self.declare_parameter("entry_lat", 45.97073358962203)
        self.declare_parameter("entry_lon", 7.7136995051485115)
        self.declare_parameter("heading_deg", 0.0)
        self.declare_parameter("drive_out_m", 50.0)
        self.declare_parameter("rotation_deg", 0.0)
        self.declare_parameter("grid_size_m", 100.0)
        self.declare_parameter("spacing_m", 10.0)

        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(
            GeoJSON, "/measurement_grid", latched_qos
        )

        self._dirty = False
        self.add_on_set_parameters_callback(self._validate_params)
        self.create_timer(0.1, self._check_and_publish)

        self._publish_grid()
        self.get_logger().info("GridPublisher ready (topic=/measurement_grid)")

    def _compute_grid(self):
        """Return grid points and marker dict with entry/anchor positions."""
        entry_lat = float(self.get_parameter("entry_lat").value)
        entry_lon = float(self.get_parameter("entry_lon").value)
        heading_deg = float(self.get_parameter("heading_deg").value)
        drive_out = float(self.get_parameter("drive_out_m").value)
        rotation = float(self.get_parameter("rotation_deg").value)
        grid_size = float(self.get_parameter("grid_size_m").value)
        spacing = float(self.get_parameter("spacing_m").value)

        utm_zone = int((entry_lon + 180) / 6) + 1
        proj = pyproj.Proj(proj="utm", zone=utm_zone, ellps="WGS84")

        h_drive = math.radians(heading_deg)
        drive = np.array([math.sin(h_drive), math.cos(h_drive)])

        entry_utm = np.array(proj(entry_lon, entry_lat))
        anchor_utm = entry_utm + drive_out * drive

        h_grid = math.radians(heading_deg + rotation)
        forward = np.array([math.sin(h_grid), math.cos(h_grid)])
        right = np.array([math.cos(h_grid), -math.sin(h_grid)])

        lower_left = anchor_utm - (grid_size / 2.0) * right

        steps = np.arange(0, grid_size + spacing / 2, spacing)
        points = []
        i = 0
        for dx in steps:
            for dy in steps:
                pt = lower_left + dx * right + dy * forward
                lon, lat = proj(pt[0], pt[1], inverse=True)
                points.append((i, float(lat), float(lon)))
                i += 1

        anchor_lon, anchor_lat = proj(anchor_utm[0], anchor_utm[1], inverse=True)
        markers = {
            "entry": (entry_lat, entry_lon),
            "anchor": (float(anchor_lat), float(anchor_lon)),
        }
        return points, markers

    def _to_geojson(self, points, markers):
        features = [
            {
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": [lon, lat]},
                "properties": {"id": pid},
            }
            for pid, lat, lon in points
        ]
        for label, (lat, lon) in markers.items():
            features.append({
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": [lon, lat]},
                "properties": {"id": label},
            })
        return json.dumps({"type": "FeatureCollection", "features": features})

    def _publish_grid(self):
        points, markers = self._compute_grid()
        msg = GeoJSON()
        msg.geojson = self._to_geojson(points, markers)
        self.publisher.publish(msg)
        self.get_logger().info(f"Published grid: {len(points)} points")

    def _validate_params(self, params):
        for p in params:
            if p.name in GRID_PARAMS and not isinstance(p.value, (int, float)):
                return SetParametersResult(
                    successful=False, reason=f"{p.name} must be numeric"
                )
        if any(p.name in GRID_PARAMS for p in params):
            self._dirty = True
        return SetParametersResult(successful=True)

    def _check_and_publish(self):
        if self._dirty:
            self._dirty = False
            self._publish_grid()


def main(args=None):
    rclpy.init(args=args)
    node = GridPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
