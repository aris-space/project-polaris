import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn


class MavlinkBridgeReceiver(Node):
    """
    Node that is supposed to translate ROS2 messages that it receives to Mavlink for the pixhawk
    """

    def __init__(self):
        # "mavlink_bridge" is the name of the node
        super().__init__("mavlink_bridge_receiver")

        # configures serial port the pixhawk is connected to and the baud rate
        self.port = mavutil.mavlink_connection("/dev/ttyTHS1", baud=57600)

        # Wait for a heartbeat so we know the target system IDs. Code can get stuck here meaning we didn't receive any heartbeat
        self.port.wait_heartbeat()
        self.get_logger().info(
            f"Heartbeat received from system {self.port.target_system}"
        )

        # Subscribe to RC override messages from ROS2 topic "pixhawk/rc_override" and then calls the rc_override_cb (translator) function when a message arrives. Accepts only RCIn messages
        self.rc_override_subscriber = self.create_subscription(
            OverrideRCIn,
            "pixhawk/rc_override",
            self.rc_override_cb,
            10,  # overrideRCIn is a 8 integer array, so the function currently only accepts that input type
        )

        self.manual_control_subscriber = self.create_subscription(
            String, "pixhawk/manual_control", self.manual_control_cb, 10 #TODO: Change the topic and message type to what gleb defined!
        )

        # subscribe to the pixhawk/mode_cmd topic and calls mode_selection_cb
        self.mode_selection_subscriber = self.create_subscription(
            String, "pixhawk/mode_cmd", self.mode_selection_cb, 10
        )

        self.get_logger().info("MavlinkBridgeReceiver: Node has been initialized")

    def rc_override_cb(self, msg):
        """
        Called automatically when a message arrives on "pixhawk/rc_override" topic.
        Converts the ROS2 OverrideRCIn message to MAVLink RC_OVERRIDE and sends it to Pixhawk.
        """
        self.get_logger().info(f"Received ROS2 RC override: {msg.channels}")

        channels = msg.channels

        # Send MAVLink RC_CHANNELS_OVERRIDE message
        # Arguments: target_system, target_component and the different RCOverride values in channels
        self.port.mav.rc_channels_override_send(
            self.port.target_system,  # Target system ID
            self.port.target_component,  # Target component ID
            channels[0],
            channels[1],
            channels[2],
            channels[3],
            channels[4],
            channels[5],
            channels[6],
            channels[7],
        )

    def manual_control_cb(self, msg):
        """
        Called when a message arrives in the pixhawk/manual_control topic. The message should contain the surge, sway, heave, roll, pitch and yaw values for the manual control command.
        """
        self.send_6dof_command(msg.data) #TODO: Change this to the correct message type and extract the control input values from the message

        #self.send_4dof_command(msg.data) 
        

    def send_4dof_command(self, control_input):
        """
        Input values: -1000 to 1000 (except heave, see below)
        """
        surge, sway, heave, yaw = control_input
        self.port.mav.manual_control_send(
            self.port.target_system,
            int(surge),  # x: Forward/Back
            int(sway),  # y: Left/Right
            int(heave),  # z: Up/Down (range 0-1000, 500 is neutral)
            int(yaw),  # r: Yaw
            0,  # buttons bitmask
        )

    def send_6dof_command(self, control_input):
        """
        Note: Extension fields (s, t) are usually enabled in
        newer MAVLink 2.0 implementations. This has to be tested!
        Input values: -1000 to 1000 (except heave, see below)
        """
        surge, sway, heave, yaw, roll, pitch = control_input
        self.port.mav.manual_control_send(
            self.port.target_system,
            int(surge),  # x
            int(sway),  # y
            int(heave),  # z (0-1000)
            int(yaw),  # r
            0,  # buttons
            int(roll),  # s (Extension 1)
            int(pitch),  # t (Extension 2)
        )

    def mode_selection_cb(self, msg):
        """
        Called when a message arrives in the pixhawk/mode_cmd topic. if the message is mapped to "ALT_HOLD" then the sub will perform that function"
        """
        self.get_logger().info(f"Received ROS2 RC Mode message: {msg.data}")
        if msg.data == "ALT_HOLD":
            # Set mode to ALT_HOLD (Depth Hold for ArduSub)
            # id mappings:{'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3, 'GUIDED': 4, 'CIRCLE': 7, 'SURFACE': 9, 'POSHOLD': 16, 'MANUAL': 19}
            # Base mode 209 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
            mode_id = 2
            self.port.mav.set_mode_send(
                self.port.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
            self.get_logger().info("Sent ALT_HOLD mode command")

        elif msg.data == "MANUAL":
            mode_id = 19
            self.port.mav.set_mode_send(
                self.port.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
            self.get_logger().info("Sent MANUAL mode command")


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridgeReceiver()
    rclpy.spin(node)  # Keeps the node running and processing callbacks
    rclpy.shutdown()
