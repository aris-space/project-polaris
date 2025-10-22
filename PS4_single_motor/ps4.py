import rclpy
from rclpy.node import Node


def mapper(input_value):
    """
    Maps the input_value from the range [1, -1] to the range [0, 100].

    Parameters:
    input_value (float): A float value in the range [1, -1].

    Returns:
    int: An integer value in the range [0, 100].
    """
    
    if input_value > 1 or input_value < -1:
        raise ValueError("Input value must be in the range [1, -1]")
    
    mapped_value = int(((-input_value + 1) / 2) * 100)
    return mapped_value

