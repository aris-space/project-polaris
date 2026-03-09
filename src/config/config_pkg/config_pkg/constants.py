"""
Change ENVIRONMENT to the appropriate value based on the testing environment:
- "POOL" for pool testing
- "OPEN_WATER" for open water testing
- "ICE_LAKE" for ice lake testing
"""

ENVIRONMENT = "POOL"  # Options: "POOL", "OPEN_WATER", "ICE_LAKE"
CONTROLLER_LAYOUT = "FOXGLOVE"  # Options: "DESKTOP", "JETSON", "FOXGLOVE"


"""
Access the constants in this file using: from config_pkg.constants import SubConfig, JoyPS4, JoyControlMapping, Comms.
Example usage:
from config_pkg.constants import JoyControlMapping
# Accessing a specific constant
emergency_stop_button_idx = JoyControlMapping.EMERGENCY_STOP_BUTTON_IDX
"""


class SubConfig:
    MAX_DEPTH = 30.0  # meters
    LEAK_THRESHOLD = 500  # Analog value


class Ports:
    FRONT_ULTRASONIC_PORT = "/dev/uart_port_1"
    PING_SONAR_PORT = "/dev/uart_port_2"
    TOP_ULTRASONIC_PORT = "/dev/uart_port_3"
    ARDUINO_PORT = "/dev/arduino_nano"
    USB_CAM_FRONT_PORT = "/dev/cam_front"
    USB_CAM_TUBE_PORT = "/dev/cam_tube"


class Logs:
    LOG_DIR = "~/polaris_logs"  # Directory to save logs
    ROSBAG_DIR = "/ros2_ws/recordings/"  # Directory to save rosbag recordings


class JoyPS4:
    if CONTROLLER_LAYOUT == "DESKTOP":
        # General Button Mapping (0-indexed)
        X = 0
        CIRCLE = 1
        TRIANGLE = 2
        SQUARE = 3
        L1 = 4
        R1 = 5
        SHARE_BUTTON = 8
        OPTIONS_BUTTON = 9
        PS_BUTTON = 10
        L3_BUTTON = 11  # Left stick click
        R3_BUTTON = 12  # Right stick click

        # General Axis Mapping (0-indexed)
        LEFT_STICK_X_AXIS = 1  # Up = 1.0, Down = -1.0
        LEFT_STICK_Y_AXIS = 0  # Left = 1.0, Right = -1.0
        RIGHT_STICK_X_AXIS = 4  # Up = 1.0, Down = -1.0
        L2_TRIGGER_AXIS = 2  # Fully out = -1.0, Fully in = 1.0
        R2_TRIGGER_AXIS = 5  # Fully out = -1.0, Fully in = 1.0
        RIGHT_STICK_Y_AXIS = 3  # Left = 1.0, Right = -1.0
        DPAD_HORIZONTAL_AXIS = 6  # Left = 1.0, Right = -1.0
        DPAD_VERTICAL_AXIS = 7  # Up = 1.0, Down = -1.0

    elif CONTROLLER_LAYOUT == "JETSON":
        # General Button Mapping (0-indexed)
        X = 0
        CIRCLE = 1
        SQUARE = 2
        TRIANGLE = 3
        SHARE_BUTTON = 4
        PS_BUTTON = 5
        OPTIONS_BUTTON = 6
        L3_BUTTON = 7  # Left stick click
        R3_BUTTON = 8  # Right stick click
        L1 = 9
        R1 = 10
        DPAD_UP = 11
        DPAD_DOWN = 12
        DPAD_LEFT = 13
        DPAD_RIGHT = 14
        TOUCHPAD_BUTTON = 15  # Touchpad Button

        # General Axis Mapping (0-indexed)
        LEFT_STICK_Y_AXIS = 0  # left = 1.0, right = -1.0
        LEFT_STICK_X_AXIS = 1  # down = -1.0, up = 1.0
        RIGHT_STICK_Y_AXIS = 2  # Left = 1.0, Right = -1.0
        RIGHT_STICK_X_AXIS = 3  # down = -1.0, up = 1.0
        L2_TRIGGER_AXIS = 4  # Fully out = -1.0, Fully in = 1.0
        R2_TRIGGER_AXIS = 5  # Fully out = -1.0, Fully in = 1.0

    elif CONTROLLER_LAYOUT == "FOXGLOVE":
        # General Button Mapping (0-indexed)
        X = 0
        CIRCLE = 1
        SQUARE = 2
        TRIANGLE = 3
        L1 = 4
        R1 = 5
        L2 = 6
        R2 = 7
        SHARE_BUTTON = 8
        OPTIONS_BUTTON = 9
        L3 = 10
        R3 = 11
        DPAD_UP = 12
        DPAD_DOWN = 13
        DPAD_LEFT = 14
        DPAD_RIGHT = 15
        PS_BUTTON = 16
        TOUCHPAD_BUTTON = 17  # Touchpad Button

        # General Axis Mapping (0-indexed)
        LEFT_STICK_Y_AXIS = 0  # left = 1.0, right = -1.0
        LEFT_STICK_X_AXIS = 1  # down = -1.0, up = 1.0
        L2_TRIGGER_AXIS = 4  # Fully out = -1.0, Fully in = 1.0
        RIGHT_STICK_Y_AXIS = 2  # Left = 1.0, Right = -1.0
        RIGHT_STICK_X_AXIS = 3  # down = -1.0, up = 1.0
        R2_TRIGGER_AXIS = 5  # Fully out = -1.0, Fully in = 1.0


class JoyControlMapping:

    # Control Specific Button Mapping
    SETTING_SAFETY_BUTTON_IDX = JoyPS4.SQUARE  # Square Button
    MODE_SAFETY_BUTTON_IDX = JoyPS4.X  # X Button
    SETTING_STABILIZATION_BUTTON_IDX = JoyPS4.SHARE_BUTTON  # Share Button
    EMERGENCY_STOP_BUTTON_IDX_LEFT = JoyPS4.L3
    EMERGENCY_STOP_BUTTON_IDX_RIGHT = JoyPS4.R3  # R3 Button
    ROLL_RATE_NEGATIVE_AXIS_IDX = JoyPS4.L1  # L1 Button
    ROLL_RATE_POSITIVE_AXIS_IDX = JoyPS4.R1  # R1 Button

    if CONTROLLER_LAYOUT == "DESKTOP":
        # DESKTOP Mode Axis Mapping (Requires Safety Button Pressed)
        MODE_MANUAL_AXES_IDX = JoyPS4.DPAD_VERTICAL_AXIS
        MODE_ALT_HOLD_AXES_IDX = JoyPS4.DPAD_HORIZONTAL_AXIS
        MODE_SPARE_1_DPAD_AXES_IDX = JoyPS4.DPAD_HORIZONTAL_AXIS
        MODE_SPARE_2_DPAD_AXES_IDX = JoyPS4.DPAD_VERTICAL_AXIS
        SETTING_ARM_DISARM_AXIS_IDX = JoyPS4.DPAD_VERTICAL_AXIS
        SETTING_STABILIZATION_AXIS_IDX = JoyPS4.DPAD_HORIZONTAL_AXIS

    else:
        # JETSON & FOXGLOVE Mode Button Mapping (Requires Safety Button Pressed)
        MODE_MANUAL_BUTTON_IDX = JoyPS4.DPAD_LEFT  # D-pad Left
        MODE_ALT_HOLD_BUTTON_IDX = JoyPS4.DPAD_UP  # D-pad Up
        MODE_SPARE_1_DPAD_BUTTON_IDX = JoyPS4.DPAD_RIGHT  # D-pad Right
        MODE_SPARE_2_DPAD_BUTTON_IDX = JoyPS4.DPAD_DOWN  # D-pad Down
        SETTING_ARM_BUTTON_IDX = JoyPS4.DPAD_UP  # D-pad Up
        SETTING_DISARM_BUTTON_IDX = JoyPS4.DPAD_DOWN  # D-pad Down
        SETTING_STABILIZATION_BUTTON_IDX = JoyPS4.DPAD_LEFT  # D-pad Left

    LINEAR_SPEED_X_AXIS_IDX = JoyPS4.LEFT_STICK_X_AXIS  # Left Stick X-Axis
    LINEAR_SPEED_Y_AXIS_IDX = JoyPS4.LEFT_STICK_Y_AXIS  # Left Stick Y-Axis
    LINEAR_SPEED_Z_FORWARD_AXIS_IDX = JoyPS4.L2_TRIGGER_AXIS  # L2 Trigger Axis
    LINEAR_SPEED_Z_BACKWARD_AXIS_IDX = JoyPS4.R2_TRIGGER_AXIS  # R2 Trigger Axis
    PITCH_RATE_AXIS_IDX = JoyPS4.RIGHT_STICK_X_AXIS  # Right Stick X-Axis
    YAW_RATE_AXIS_IDX = JoyPS4.RIGHT_STICK_Y_AXIS  # Right Stick Y-Axis


class Comms:
    IP_ADDRESS = "XXX.XXX.X.XX"  # Tethered IP
    SERIAL_PORT1 = "/dev/ttyTHS1"
    SUB_QOS_DEPTH = 10
    SERIAL1_BAUD_RATE = 57600
