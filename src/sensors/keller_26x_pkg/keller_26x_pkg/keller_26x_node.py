import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float64

from config_pkg.constants import Ports

from keller_protocol import keller_protocol as kp

# use pip install keller-protocol
# https://github.com/KELLERAGfuerDruckmesstechnik/keller_protocol_python


class Keller26xNode(Node):

    def __init__(self):
        super().__init__("keller_26x_pressure")

        self.declare_parameter(
            "surface_pressure_topic", "/sensors/pressure/p_surface_pa"
        )
        self.declare_parameter(
            "gauge_pressure_topic", "/sensors/keller26x/gauge_pressure"
        )
        self.declare_parameter("abs_pressure_topic", "/sensors/keller26x/abs_pressure")
        self.declare_parameter(
            "water_temperature_topic", "/sensors/keller26x/water_temperature_degc"
        )
        self.declare_parameter("default_atmospheric_pressure_pa", 101325.0)
        self.declare_parameter("pressure_frame_id", "keller_pressure_link")
        self.declare_parameter("publish_frequency_hz", 30.0)
        self.declare_parameter("dev_port", Ports.KELLER_SENSOR)

        self.dev_port = str(self.get_parameter("dev_port").value)
        self.publish_frequency_hz = float(
            self.get_parameter("publish_frequency_hz").value
        )

        self.address = 1
        self.bus = None
        self._connect_bus(self.dev_port)
        self.p1_Pa = 0.0
        self.p1_gauge_pa = 0.0
        self.p1_gauge_raw_pa = 0.0
        self.water_temperature_c = 0.0
        self.serial_number = None
        self._surface_pressure_pa = float(
            self.get_parameter("default_atmospheric_pressure_pa").value
        )
        self._surface_pressure_source = "default"
        self._gauge_offset_pa = 0.0
        self._latest_gauge_raw_pa = None
        self._pending_gauge_offset_capture = False
        self.f73_channels = {
            "CH0": 0,
            "P1": 1,
            "P2": 2,
            "T": 3,
            "TOB1": 4,
            "TOB2": 5,
            "ConTc": 10,
            "ConRaw": 11,
        }

        surface_pressure_topic = str(self.get_parameter("surface_pressure_topic").value)
        gauge_pressure_topic = str(self.get_parameter("gauge_pressure_topic").value)
        abs_pressure_topic = str(self.get_parameter("abs_pressure_topic").value)
        water_temperature_topic = str(
            self.get_parameter("water_temperature_topic").value
        )
        self.pressure_frame_id = str(self.get_parameter("pressure_frame_id").value)

        self.gauge_pub = self.create_publisher(
            FluidPressure,
            gauge_pressure_topic,
            10,
        )
        self.abs_pub = self.create_publisher(
            FluidPressure,
            abs_pressure_topic,
            10,
        )
        self.water_temperature_pub = self.create_publisher(
            Temperature,
            water_temperature_topic,
            10,
        )
        self.surface_pressure_sub = self.create_subscription(
            Float64,
            surface_pressure_topic,
            self.surface_pressure_callback,
            10,
        )

        timer_period = 1.0 / self.publish_frequency_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.water_temperature_timer = self.create_timer(
            1.0 / 5.0,
            self.water_temperature_timer_callback,
        )
        self.add_on_set_parameters_callback(self.on_parameter_change)

        self.get_logger().info(
            f"Started Keller26x pressure node. Gauge topic: {gauge_pressure_topic}. "
            f"Absolute topic: {abs_pressure_topic}. Using default atmospheric pressure "
            f"Water temperature topic: {water_temperature_topic} (5.0 Hz). "
            f"{self._surface_pressure_pa:.2f} Pa until override on {surface_pressure_topic}. "
            f"Message frame_id={self.pressure_frame_id}. Publish frequency: {self.publish_frequency_hz} Hz. "
            f"Device port: {self.dev_port}."
        )

    def _connect_bus(self, port: str) -> None:
        if self.bus is not None:
            # Gracefully release the previous serial handle when switching ports.
            close_fn = getattr(self.bus, "close", None)
            if callable(close_fn):
                close_fn()

        self.bus = kp.KellerProtocol(
            port=port,
            baud_rate=9600,
            timeout=0.3,
            echo=False,
        )
        self.init_f48()

    def _update_timer_frequency(self, frequency_hz: float) -> None:
        timer_period = 1.0 / frequency_hz
        self.timer.cancel()
        self.destroy_timer(self.timer)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def on_parameter_change(self, params):
        result = SetParametersResult(successful=True)

        for param in params:
            if param.name == "publish_frequency_hz":
                try:
                    new_frequency = float(param.value)
                except (TypeError, ValueError):
                    result.successful = False
                    result.reason = "publish_frequency_hz must be a number"
                    return result

                if new_frequency <= 0.0:
                    result.successful = False
                    result.reason = "publish_frequency_hz must be > 0"
                    return result

                self.publish_frequency_hz = new_frequency
                self._update_timer_frequency(self.publish_frequency_hz)
                self.get_logger().info(
                    f"Updated publish frequency to {self.publish_frequency_hz:.2f} Hz"
                )

            elif param.name == "dev_port":
                new_port = str(param.value).strip()
                if not new_port:
                    result.successful = False
                    result.reason = "dev_port must be a non-empty string"
                    return result

                try:
                    self._connect_bus(new_port)
                except Exception as exc:
                    result.successful = False
                    result.reason = f"Failed to connect dev_port '{new_port}': {exc}"
                    return result

                self.dev_port = new_port
                self.get_logger().info(f"Updated Keller sensor port to {self.dev_port}")

        return result

    def init_f48(self):
        """
        To be able to communicate with the transmitter you will have to use F48 first to initialize.
        """
        self.bus.f48(self.address)

    def measure_p1(self) -> float:
        """Get pressure P1

        :return: pressure
        """
        pressure = self.bus.f73(self.address, self.f73_channels["P1"])
        return pressure

    def measure_water_temperature(self) -> float:
        """Get water temperature from channel T in degC."""
        return self.bus.f73(self.address, self.f73_channels["T"])

    """
    calibration callback flow (surface_pressure_callback):
    - on startup, use default surface pressure and add the measured gauge raw pressure to that to get absolute pressure. Log that we are using the default surface pressure.
    - when a surface pressure message is received (via surface_pressure_sub), we update the surface pressure and capture the gauge offset.
    - that gauge offset is then applied to all subsequent gauge pressure measurements and absolute pressure is recalculated accordingly.
    """

    def surface_pressure_callback(self, msg: Float64) -> None:
        new_surface_pressure_pa = float(msg.data)
        if abs(new_surface_pressure_pa - self._surface_pressure_pa) > 1e-6:
            self._surface_pressure_pa = new_surface_pressure_pa
            previous_source = self._surface_pressure_source
            self._surface_pressure_source = "topic"

            if self._latest_gauge_raw_pa is None:
                self._pending_gauge_offset_capture = True
                self.get_logger().warning(
                    "Surface calibration updated but no gauge sample yet. "
                    "Gauge offset will be captured on next pressure sample."
                )
            else:
                self._gauge_offset_pa = self._latest_gauge_raw_pa
                self._pending_gauge_offset_capture = False

            if previous_source == "default":
                self.get_logger().info(
                    f"Surface pressure override received: {self._surface_pressure_pa:.2f} Pa "
                    f"(replacing default literature value), gauge_offset={self._gauge_offset_pa:.2f} Pa."
                )
            else:
                self.get_logger().info(
                    f"Surface pressure calibration updated to {self._surface_pressure_pa:.2f} Pa, "
                    f"gauge_offset={self._gauge_offset_pa:.2f} Pa."
                )

    def timer_callback(self):
        p1_bar = self.measure_p1()
        self.p1_gauge_raw_pa = p1_bar * 100000.0
        self._latest_gauge_raw_pa = self.p1_gauge_raw_pa

        if self._pending_gauge_offset_capture:
            self._gauge_offset_pa = self.p1_gauge_raw_pa
            self._pending_gauge_offset_capture = False

        self.p1_gauge_pa = self.p1_gauge_raw_pa - self._gauge_offset_pa

        gauge_msg = FluidPressure()
        gauge_msg.header.stamp = self.get_clock().now().to_msg()
        gauge_msg.header.frame_id = self.pressure_frame_id
        gauge_msg.fluid_pressure = self.p1_gauge_pa
        self.gauge_pub.publish(gauge_msg)

        self.p1_Pa = self.p1_gauge_pa + self._surface_pressure_pa
        abs_msg = FluidPressure()
        abs_msg.header.stamp = self.get_clock().now().to_msg()
        abs_msg.header.frame_id = self.pressure_frame_id
        abs_msg.fluid_pressure = self.p1_Pa
        self.abs_pub.publish(abs_msg)

        self.get_logger().info(
            f"keller_gauge_pressure={self.p1_gauge_pa:.2f} Pa, "
            f"keller_gauge_raw_pressure={self.p1_gauge_raw_pa:.2f} Pa, "
            f"gauge_offset={self._gauge_offset_pa:.2f} Pa, "
            f"keller_abs_pressure={self.p1_Pa:.2f} Pa, "
            f"atm_source={self._surface_pressure_source}",
            throttle_duration_sec=5.0,
        )

    def water_temperature_timer_callback(self) -> None:
        try:
            reading = self.measure_water_temperature()
        except Exception as exc:
            self.get_logger().warning(f"Failed to read water temperature: {exc}")
            return

        if math.isnan(reading):
            self.get_logger().warning("Water temperature reading is NaN — skipping publish")
            return

        self.water_temperature_c = reading

        temp_msg = Temperature()
        temp_msg.header.stamp = self.get_clock().now().to_msg()
        temp_msg.header.frame_id = self.pressure_frame_id
        temp_msg.temperature = self.water_temperature_c
        temp_msg.variance = 0.0
        self.water_temperature_pub.publish(temp_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Keller26xNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
