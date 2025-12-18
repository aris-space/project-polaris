from ping_sonar.brping import Ping1D
from rclpy.node import Node
from std_msgs.msg import String


class Ice_Measurement(Node):
    """
    Node that uses the PingSonar measurements and writes the important values to a csv
    """

    def __init__(self):
        super().__init__("ice_measurement_publisher")

        self.ping = Ping1D()
        self.ping.connect_serial("/dev/ttyUSB0", 115200)

        self.initialization = self.ping.initialize()

        if self.initialization:
            self.get_logger().info("Measurement device initalized")

        self.measurement_mode = self.create_subscription(
            String, "measurement/mode", self.measurement_mode_cb, 10
        )


# def measurement_mode_cb (self, msg):
