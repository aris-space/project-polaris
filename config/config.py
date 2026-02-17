"""
Configuration Class for global settings.
USEAGE:
from config.config import Config
pwm_max = Config.get_pwm_max()
"""


class Config:
    pwm = {"max": 1900, "min": 1100, "init": 1500}

    joystick = {
        "x_button": 0,
        "o_button": 1,
        "triangle_button": 2,
        "square_button": 3,
        "left_stick_horizontal_axis": 0,
        "left_stick_vertical_axis": 1,
        "right_stick_horizontal_axis": 2,
        "right_stick_vertical_axis": 3,
        "l2_axis": 4,
        "r2_axis": 5,
        "l1_button": 4,
        "r1_button": 5,

    }

    def __init__(self):
        pass

    @classmethod
    def get_pwm_max(self):
        return self.pwm["max"]

    @classmethod
    def get_pwm_min(self):
        return self.pwm["min"]

    @classmethod
    def get_pwm_init(self):
        return self.pwm["init"]

    @classmethod
    def get_joy_r2_axis(self):
        return self.joystick["r2_axis"]

    @classmethod
    def get_joy_l2_axis(self):
        return self.joystick["l2_axis"]

    @classmethod
    def get_joy_x_button(self):
        return self.joystick["x_button"]

    @classmethod
    def get_joy_o_button(self):
        return self.joystick["o_button"]

    @classmethod
    def get_joy_left_stick_vertical_axis(self):
        return self.joystick["left_stick_vertical_axis"]

    @classmethod
    def get_joy_left_stick_horizontal_axis(self):
        return self.joystick["left_stick_horizontal_axis"]

    @classmethod
    def get_joy_right_stick_vertical_axis(self):
        return self.joystick["right_stick_vertical_axis"]

    @classmethod
    def get_joy_right_stick_horizontal_axis(self):
        return self.joystick["right_stick_horizontal_axis"]

    @classmethod
    def get_joy_triangle_button(self):
        return self.joystick["triangle_button"]

    @classmethod
    def get_joy_l1_button(self):
        return self.joystick["l1_button"]

    @classmethod
    def get_joy_r1_button(self):
        return self.joystick["r1_button"]


if __name__ == "__main__":
    cfg = Config()
    print(cfg.get_pwm_max())
