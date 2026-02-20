from .brping import Ping1D
from rclpy.node import Node
import rclpy
import csv
import json
import os
from datetime import datetime
from std_msgs.msg import String
from config_pkg.constants import Logs, Comms


class Ice_Measurement(Node):
    """
    Node that uses the PingSonar measurements and writes the intensity and timestamps to a csv
    """

    def __init__(self):
        super().__init__("ice_measurement_publisher")

        self.ping = Ping1D()  # initializes object
        self.ping.connect_udp(Comms.JETSON_IP_ADDRESS, Comms.PING_SONAR_PORT)  # specifies relevant port

        self.initialization = self.ping.initialize()

        if not self.initialization:
            self.get_logger().error("Failed to initialize measurement device")
            return
        else:
            self.get_logger().info("Measurement device initialized")

        # relevant parameters to configure
        self.scan_start = 0
        self.scan_length = 1000
        self.number_bins = 200
        self.speed_of_sound = 1500000
        self.ping_interval = 0.05

        self.bin_time = (2 * self.scan_length) / (
            self.number_bins * self.speed_of_sound
        )

        # set functions based on the config factors
        if not self.ping.set_range(self.scan_start, self.scan_length, verify=True):
            self.get_logger().error("Failed to set range")
        else:
            self.get_logger().info("Range set")
         # scan_start, scan_length in mm
        if not self.ping.set_oss_profile_configuration(self.number_bins, 0, 0, verify=True):
            self.get_logger().error("Failed to set profile configuration")
        else:
            self.get_logger().info("Profile configuration set")
            
    
        self.recording = False
        self.mode_sub = self.create_subscription(
            String, "/ping_sonar/mode", self.mode_callback, 10
        )

        # CSV setup
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = os.path.expanduser(Logs.LOG_DIR)
        os.makedirs(log_dir, exist_ok=True)
        self.csv_file = open(
            os.path.join(log_dir, f"ice_data_{ts}.csv"), "w", newline=""
        )
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp", "ping_number", "bin_index", "intensity"])

        self.first_timestamp = None
        self.timer = self.create_timer(self.ping_interval, self.logging_cb)

        self.distance_publisher = self.create_publisher(String, "/ping_sonar/distance", 10)
        self.profile_publisher = self.create_publisher(String, "/ping_sonar/profile", 10)

    def mode_callback(self, msg):
        if msg.data == "start":
            self.recording = True
            self.get_logger().info("Recording started")
        elif msg.data == "stop":
            self.recording = False
            self.get_logger().info("Recording stopped")

    def logging_cb(self):
        if not self.recording:
            return
        
        profile = self.ping.get_profile()
        distance = self.ping.get_distance_simple()

        if profile is None:
            return
        if distance is None:
            return

        msg_distance = String()
        msg_distance.data = json.dumps({
            "distance": distance["distance"],
            "confidence": distance["confidence"],
        })

        msg_profile = String()
        msg_profile.data = json.dumps({
            "scan_start": profile["scan_start"],
            "scan_length": profile["scan_length"],
            "ping_number": profile["ping_number"],
            "profile_data": list(profile["profile_data"]),
        })

        self.distance_publisher.publish(msg_distance)
        self.profile_publisher.publish(msg_profile)

        if self.first_timestamp is None:
            self.first_timestamp = (
                self.get_clock().now().nanoseconds / 1e9
            )

        ping_num = profile["ping_number"]
        for i, intensity in enumerate(profile["profile_data"]):
            timestamp = (
                self.first_timestamp
                + (
                    ping_num * self.ping_interval
                )  # we adapt the csv timestamps based on the index of the bin and the index of the ping
                + (i * self.bin_time)
            )
            self.csv_writer.writerow([f"{timestamp:.6f}", ping_num, i, intensity])

        self.csv_file.flush()
        self.get_logger().info(f"Ping {ping_num}")


def main(args=None):
    rclpy.init(args=args)
    node = Ice_Measurement()
    rclpy.spin(node)
    node.csv_file.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
