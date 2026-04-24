import rclpy
from rclpy.node import Node
from dalybms import DalyBMS
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue, DiagnosticArray
from rcl_interfaces.msg import SetParametersResult


DEFAULT_THRESHOLDS = {
    "battery_warn_current": -20.0,
    "battery_error_current": -30.0,
    "battery_warn_voltage": 14.0,
    "battery_error_voltage": 13.0,
    "battery_warn_soc": 40.0,
    "battery_error_soc": 20.0,
}


class BMSNode(Node):
    def __init__(self):
        super().__init__("bms_node")

        for name, default in DEFAULT_THRESHOLDS.items():
            self.declare_parameter(name, default)
            setattr(self, name, float(self.get_parameter(name).value))

        self.add_on_set_parameters_callback(self._on_param_change)

        self.bms = DalyBMS()
        self.bms.connect("/dev/uart_port_4")
        self.get_logger().info("BMS node connected")

        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "diagnostics", 10
        )
        self.timer = self.create_timer(1.0, self.timer_callback)

    def _on_param_change(self, params):
        for p in params:
            if p.name in DEFAULT_THRESHOLDS:
                if not isinstance(p.value, (int, float)):
                    return SetParametersResult(
                        successful=False, reason=f"{p.name} must be numeric"
                    )
                setattr(self, p.name, float(p.value))
        return SetParametersResult(successful=True)

    def timer_callback(self):

        try:
            soc = self.bms.get_soc() or {}
        except Exception as e:
            self.get_logger().error(f"BMS read failed: {e}")
            return

        voltage = float(soc.get("total_voltage", 0.0))
        current = float(soc.get("current", 0.0))
        soc_pct = float(soc.get("soc_percent", 0.0))

        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()

        # Battery current
        status_current = DiagnosticStatus()
        status_current.name = "Battery: Current"
        status_current.level = DiagnosticStatus.OK
        status_current.message = f"{current:.2f}A"
        status_current.values = [KeyValue(key="current_A", value=f"{current:.2f}")]
        diag_msg.status.append(status_current)

        if current < self.battery_error_current:
            status_current.level = DiagnosticStatus.ERROR
            status_current.message = (
                f"Current draw is critically high (current: {current:.2f}A)"
            )
        elif current < self.battery_warn_current:
            status_current.level = DiagnosticStatus.WARN
            status_current.message = f"Current draw is high (current: {current:.2f}A)"
        else:
            status_current.level = DiagnosticStatus.OK
            status_current.message = f"{current:.2f}A"

        # Battery voltage
        status_voltage = DiagnosticStatus()
        status_voltage.name = "Battery: Voltage"
        status_voltage.level = DiagnosticStatus.OK
        status_voltage.message = f"{voltage:.2f}V"
        status_voltage.values = [KeyValue(key="voltage", value=f"{voltage:.2f}V")]

        if voltage < self.battery_error_voltage:
            status_voltage.level = DiagnosticStatus.ERROR
            status_voltage.message = (
                f"Voltage is critically low (voltage: {voltage:.2f}V)"
            )
        elif voltage < self.battery_warn_voltage:
            status_voltage.level = DiagnosticStatus.WARN
            status_voltage.message = (
                f"Voltage is close to minimum (voltage: {voltage:.2f}V)"
            )
        else:
            status_voltage.level = DiagnosticStatus.OK
            status_voltage.message = f"{voltage:.2f}V"

        diag_msg.status.append(status_voltage)

        status_pct = DiagnosticStatus()
        status_pct.name = "Battery: Percentage"
        status_pct.level = DiagnosticStatus.OK
        status_pct.message = f"{soc_pct:.2f}%"
        status_pct.values = [KeyValue(key="soc_percent", value=f"{soc_pct:.2f}%")]

        if soc_pct < self.battery_error_soc:
            status_pct.level = DiagnosticStatus.ERROR
            status_pct.message = (
                f"State of Charge is critically low (soc_pct: {soc_pct:.2f}%)"
            )
        elif soc_pct < self.battery_warn_soc:
            status_pct.level = DiagnosticStatus.WARN
            status_pct.message = f"State of Charge is low (soc_pct: {soc_pct:.2f}%)"
        else:
            status_pct.level = DiagnosticStatus.OK
            status_pct.message = f"{soc_pct:.2f}%"

        diag_msg.status.append(status_pct)

        self.diagnostic_publisher.publish(diag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BMSNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
