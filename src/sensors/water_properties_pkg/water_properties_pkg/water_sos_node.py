import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


def _speed_of_sound_fresh_water(temp_c: float) -> float:
    """Marczak (1997) formula for speed of sound in pure water, valid 0-95 °C at 1 atm."""
    T = temp_c
    return (
        1.402385e3
        + 5.038813 * T
        - 5.799136e-2 * T**2
        + 3.287156e-4 * T**3
        - 1.398845e-6 * T**4
        + 2.787860e-9 * T**5
    )


class WaterSosNode(Node):
    """Compute and publish speed of sound in fresh water on demand.

    Subscribes to the Keller water temperature topic. A ``std_srvs/Trigger`` service call
    computes the Marczak (1997) speed-of-sound estimate from the latest temperature sample
    and publishes it to a latched (transient-local) topic.

    Nothing is published until the service is explicitly called, so spurious values while
    the vehicle is in air are avoided. The latched QoS ensures late subscribers (DVL/SBL
    config GUIs) still receive the last computed value.
    """

    def __init__(self) -> None:
        super().__init__("water_sos_node")

        self.declare_parameter(
            "temperature_topic", "/sensors/keller26x/water_temperature_degc"
        )
        self.declare_parameter("sos_topic", "/sensors/water/speed_of_sound_m_s")

        temp_topic = str(self.get_parameter("temperature_topic").value)
        sos_topic = str(self.get_parameter("sos_topic").value)

        self._latest_temp_c: float | None = None

        self._temp_sub = self.create_subscription(
            Temperature, temp_topic, self._temp_callback, 10
        )

        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._sos_pub = self.create_publisher(Float64, sos_topic, latched_qos)

        self._service = self.create_service(
            Trigger, "compute_water_sos", self._compute_sos_callback
        )

        self.get_logger().info(
            f"Water SoS node ready. Temperature from {temp_topic}. "
            f"Call ~/compute_water_sos to compute and publish to {sos_topic}."
        )

    def _temp_callback(self, msg: Temperature) -> None:
        self._latest_temp_c = float(msg.temperature)

    def _compute_sos_callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self._latest_temp_c is None:
            response.success = False
            response.message = (
                "No temperature sample received yet — is the Keller node running?"
            )
            self.get_logger().warning(response.message)
            return response

        sos = _speed_of_sound_fresh_water(self._latest_temp_c)
        out = Float64()
        out.data = sos
        self._sos_pub.publish(out)

        response.success = True
        response.message = (
            f"Speed of sound: {sos:.2f} m/s at {self._latest_temp_c:.2f} °C "
            f"(Marczak 1997, fresh water)"
        )
        self.get_logger().info(f"SoS published: {response.message}")
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaterSosNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
