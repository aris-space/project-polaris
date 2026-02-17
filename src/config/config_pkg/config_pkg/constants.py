"""
Change ENVIRONMENT to the appropriate value based on the testing environment:
- "POOL" for pool testing
- "OPEN_WATER" for open water testing
- "ICE_LAKE" for ice lake testing
"""

ENVIRONMENT = "POOL"  # Options: "POOL", "OPEN_WATER", "ICE_LAKE"
CONTROLLER_LAYOUT = "JETSON"  # Options: "DESKTOP", "JETSON"


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


if CONTROLLER_LAYOUT == "DESKTOP":

    class JoyPS4:
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
        L2_TRIGGER_AXIS = 2  # Fully out = 1.0, Fully in = -1.0
        R2_TRIGGER_AXIS = 5  # Fully out = 1.0, Fully in = -1.0
        RIGHT_STICK_Y_AXIS = 3  # Left = 1.0, Right = -1.0
        DPAD_HORIZONTAL_AXIS = 6  # Left = 1.0, Right = -1.0
        DPAD_VERTICAL_AXIS = 7  # Up = 1.0, Down = -1.0

elif CONTROLLER_LAYOUT == "JETSON":

    class JoyPS4:
        # General Button Mapping (0-indexed)
        X = 0
        CIRCLE = 1
        TRIANGLE = 3
        SQUARE = 2
        L1 = 4
        R1 = 5
        SHARE_BUTTON = 8
        OPTIONS_BUTTON = 9
        PS_BUTTON = 10
        L3_BUTTON = 11  # Left stick click
        R3_BUTTON = 12  # Right stick click
        DPAD_UP = 13
        DPAD_DOWN = 14
        DPAD_LEFT = 15
        DPAD_RIGHT = 16


        # General Axis Mapping (0-indexed)
        LEFT_STICK_X_AXIS = 0  # Left = -1.0, Right = 1.0
        LEFT_STICK_Y_AXIS = 1  # Up = -1.0, Down = 1.0
        RIGHT_STICK_X_AXIS = 2  # Left = -1.0, Right = 1.0
        RIGHT_STICK_Y_AXIS = 3  # Up = -1.0, Down = 1.0
        L2_TRIGGER_AXIS = 4  # Fully out = -1.0, Fully in = 1.0
        R2_TRIGGER_AXIS = 5  # Fully out = -1.0, Fully in = 1.0


class JoyControlMapping:

    # Control Specific Button Mapping
    SETTING_SAFETY_BUTTON_IDX = JoyPS4.CIRCLE  # Circle Button
    MODE_SAFETY_BUTTON_IDX = JoyPS4.TRIANGLE  # Square Button
    SETTING_ARM_BUTTON_IDX = JoyPS4.OPTIONS_BUTTON  # Options Button
    SETTING_STABILIZATION_BUTTON_IDX = JoyPS4.SHARE_BUTTON  # R1 Button
    EMERGENCY_STOP_BUTTON_IDX = JoyPS4.PS_BUTTON  # Triangle Button
    ROLL_RATE_NEGATIVE_AXIS_IDX = JoyPS4.L1  # L1 Button
    ROLL_RATE_POSITIVE_AXIS_IDX = JoyPS4.R1  # R1 Button

    if CONTROLLER_LAYOUT == "JETSON":
        MODE_DPAD_UP = JoyPS4.DPAD_UP  # D-pad Up
        MODE_DPAD_DOWN = JoyPS4.DPAD_DOWN  # D-pad Down
        MODE_DPAD_LEFT = JoyPS4.DPAD_LEFT  # D-pad Left
        MODE_DPAD_RIGHT = JoyPS4.DPAD_RIGHT  # D-pad Right
    else: 
        # Control Specific Axis Mapping
        MODE_DPAD_HORIZONTAL_AXIS_IDX = (
            JoyPS4.DPAD_HORIZONTAL_AXIS
        )  # Left = -1.0 = TBD, Right = 1.0 = TBD
        MODE_DPAD_UP_AXIS_IDX = (
            JoyPS4.DPAD_VERTICAL_AXIS
        )  # Up = 1.0 = MANUAL, Down = -1.0 = DEPTH HOLD
    LINEAR_SPEED_X_AXIS_IDX = JoyPS4.LEFT_STICK_X_AXIS  # Left Stick X-Axis
    LINEAR_SPEED_Y_AXIS_IDX = JoyPS4.LEFT_STICK_Y_AXIS  # Left Stick Y-Axis
    LINEAR_SPEED_Z_FORWARD_AXIS_IDX = JoyPS4.L2_TRIGGER_AXIS  # L2 Trigger Axis
    LINEAR_SPEED_Z_BACKWARD_AXIS_IDX = JoyPS4.R2_TRIGGER_AXIS  # R2 Trigger Axis
    PITCH_RATE_AXIS_IDX = JoyPS4.RIGHT_STICK_X_AXIS  # Right Stick X-Axis
    YAW_RATE_AXIS_IDX = JoyPS4.RIGHT_STICK_Y_AXIS  # Right Stick Y-Axis


class Comms:
    IP_ADDRESS = "XXX.XXX.X.XX"  # Tethered IP
