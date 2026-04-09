"""
Step inputs mode node for PID tuning.

This node is active when current_mode == 'step_inputs_mode'. It selects one of
three submodes using ROS parameters:
    - /tuning/active/attitude  (bool)
    - /tuning/active/depthhold (bool)
    - /tuning/active/poshold   (bool)

Submode behavior:
  - Default: all /tuning/active/* false → vehicle stays in MANUAL (mode 19).
  - attitude: sends Pixhawk mode STABILIZATION and publishes attitude step
              targets in degrees on /pixhawk/attitude_step_cmd.
  - depthhold: sends Pixhawk mode ALT_HOLD and publishes a local NED position
               target on /pixhawk/position_step_cmd (typically z-only step).
  - poshold: sends Pixhawk mode POSHOLD and publishes local NED position step
             targets on /pixhawk/position_step_cmd.
  - When all /tuning/active/* are false again → switch back to MANUAL (mode 19).

Step target parameters:
    - tuning/target/attitude_roll_deg, tuning/target/attitude_pitch_deg, tuning/target/attitude_yaw_deg
    - tuning/target/depthhold_z_m (NED z target in meters)
    - tuning/target/poshold_x_m, tuning/target/poshold_y_m, tuning/target/poshold_z_m (NED frame)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray


class StepInputsModeNode(Node):
    def __init__(self):
        super().__init__("step_inputs_mode")

        self.current_mode = ""
        self._last_active_submode = None
        self._last_published_pixhawk_mode = None

        # Submode selectors (default all false → MANUAL / mode 19, not STABILIZE / mode 0).
        self.declare_parameter("/tuning/active/attitude", False)
        self.declare_parameter("/tuning/active/depthhold", False)
        self.declare_parameter("/tuning/active/poshold", False)

        # Step target parameters.
        self.declare_parameter("tuning/target/attitude_roll_deg", 0.0)
        self.declare_parameter("tuning/target/attitude_pitch_deg", 0.0)
        self.declare_parameter("tuning/target/attitude_yaw_deg", 0.0)

        self.declare_parameter("tuning/target/depthhold_z_m", -1.0)

        self.declare_parameter("tuning/target/poshold_x_m", 0.0)
        self.declare_parameter("tuning/target/poshold_y_m", 0.0)
        self.declare_parameter("tuning/target/poshold_z_m", -1.0)

        self.mode_subscription = self.create_subscription(
            String,
            "/mode_control/current_mode",
            self.mode_callback,
            10,
        )

        self.pixhawk_mode_publisher = self.create_publisher(
            String,
            "/pixhawk/mode_cmd",
            10,
        )

        self.attitude_step_publisher = self.create_publisher(
            Float32MultiArray,
            "/pixhawk/attitude_step_cmd",
            10,
        )

        self.position_step_publisher = self.create_publisher(
            Float32MultiArray,
            "/pixhawk/position_step_cmd",
            10,
        )

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("StepInputsModeNode: Node has been initialized")

    def mode_callback(self, msg):
        self.current_mode = msg.data

    def timer_callback(self):
        if self.current_mode != "step_inputs_mode":
            return

        submode = self._get_active_submode()
        if submode is None:
            self._publish_mode_cmd("MANUAL")
            return

        if submode == "attitude":
            self._publish_mode_cmd("STABILIZATION")
            self._publish_attitude_step()
        elif submode == "depthhold":
            self._publish_mode_cmd("ALT_HOLD")
            self._publish_depthhold_step()
        elif submode == "poshold":
            self._publish_mode_cmd("POSHOLD")
            self._publish_position_step()

    def _get_active_submode(self):
        attitude = (
            self.get_parameter("/tuning/active/attitude")
            .get_parameter_value()
            .bool_value
        )
        depthhold = (
            self.get_parameter("/tuning/active/depthhold")
            .get_parameter_value()
            .bool_value
        )
        poshold = (
            self.get_parameter("/tuning/active/poshold")
            .get_parameter_value()
            .bool_value
        )

        active = []
        if attitude:
            active.append("attitude")
        if depthhold:
            active.append("depthhold")
        if poshold:
            active.append("poshold")

        if len(active) == 1:
            return active[0]

        if len(active) > 1:
            self.get_logger().warn(
                "Multiple PID tuning submodes are true. Using priority: attitude > depthhold > poshold"
            )
            if attitude:
                return "attitude"
            if depthhold:
                return "depthhold"
            return "poshold"

        return None

    def _publish_mode_cmd(self, mode_name):
        """Publish /pixhawk/mode_cmd when the requested mode changes (incl. MANUAL / mode 19)."""
        if self._last_published_pixhawk_mode == mode_name:
            return
        msg = String()
        msg.data = mode_name
        self.pixhawk_mode_publisher.publish(msg)
        self._last_published_pixhawk_mode = mode_name
        if mode_name == "MANUAL":
            self._last_active_submode = None
        else:
            self._last_active_submode = self._mode_to_submode(mode_name)

    def _publish_attitude_step(self):
        msg = Float32MultiArray()
        roll = (
            self.get_parameter("tuning/target/attitude_roll_deg")
            .get_parameter_value()
            .double_value
        )
        pitch = (
            self.get_parameter("tuning/target/attitude_pitch_deg")
            .get_parameter_value()
            .double_value
        )
        yaw = (
            self.get_parameter("tuning/target/attitude_yaw_deg")
            .get_parameter_value()
            .double_value
        )
        msg.data = [float(roll), float(pitch), float(yaw)]
        self.attitude_step_publisher.publish(msg)

    def _publish_depthhold_step(self):
        msg = Float32MultiArray()
        z = (
            self.get_parameter("tuning/target/depthhold_z_m")
            .get_parameter_value()
            .double_value
        )
        msg.data = [0.0, 0.0, float(z)]
        self.position_step_publisher.publish(msg)

    def _publish_position_step(self):
        msg = Float32MultiArray()
        x = (
            self.get_parameter("tuning/target/poshold_x_m")
            .get_parameter_value()
            .double_value
        )
        y = (
            self.get_parameter("tuning/target/poshold_y_m")
            .get_parameter_value()
            .double_value
        )
        z = (
            self.get_parameter("tuning/target/poshold_z_m")
            .get_parameter_value()
            .double_value
        )
        msg.data = [float(x), float(y), float(z)]
        self.position_step_publisher.publish(msg)

    @staticmethod
    def _mode_to_submode(mode_name):
        if mode_name == "STABILIZATION":
            return "attitude"
        if mode_name == "ALT_HOLD":
            return "depthhold"
        if mode_name == "POSHOLD":
            return "poshold"
        return ""

    @staticmethod
    def _submode_to_mode(submode):
        if submode == "attitude":
            return "STABILIZATION"
        if submode == "depthhold":
            return "ALT_HOLD"
        if submode == "poshold":
            return "POSHOLD"
        return ""


def main(args=None):
    rclpy.init(args=args)
    node = StepInputsModeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
