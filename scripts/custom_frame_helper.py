import numpy as np

"""
plug in the positions of hte center of mass and the various thrusters in [x,y,z]. The x, y, z are according to the coordinate system in onshape. the roll axis is the negative y axis, the pitch axis is the positive x axis, and the yaw axis is the positive z axis.
"""
com_position = np.array([-0.0153, 4.03e-4, -0.0305])
thruster_1_pos = np.array(
    [-0.9153, 0, -5.12e-4]
)  # thruster in negative y direction (forward thruster)
thruster_2_pos = np.array(
    [-0.6323, 0.0249, 0]
)  # back thruster in x direction (lateral thruster)
thruster_3_pos = np.array(
    [0.7173, 0.0249, 0]
)  # front thruster in x direction (lateral thruster)
thruster_4_pos = np.array(
    [-0.5693, -0.12835, 0.009]
)  # right thruster in z direction (throttle thruster)
thruster_5_pos = np.array(
    [-0.5693, 0.12835, 0.009]
)  # left thruster in z direction (throttle thruster)
thruster_6_pos = np.array([0.6173, 0, -0.0249])  #

thruster_positions = np.array(
    [
        thruster_1_pos,
        thruster_2_pos,
        thruster_3_pos,
        thruster_4_pos,
        thruster_5_pos,
        thruster_6_pos,
    ]
)


thruster_forces = np.array(
    [[1, 0, 0], [0, 1, 0], [0, 1, 0], [0, 0, 1], [0, 0, 1], [0, 0, 1]]
)


def attitude_calculator(com_position, thruster_positions, thruster_forces):
    """
    Calculates the roll, pitch, and yaw contributions of each thrusters and then also subsequently normalizes according to the maximum value of each contribution.
    """

    attitude = np.zeros_like(thruster_positions)

    for i in range(6):
        attitude[i] = np.cross(thruster_positions[i] - com_position, thruster_forces[i])

    for i in range(3):

        max_contribution = np.max(np.abs(attitude[:, i]))

        if max_contribution != 0:
            attitude[:, i] = attitude[:, i] / max_contribution

    # extract the torques in the x, y, z onshape coordinate system
    torque_x = attitude[:, 0]
    torque_y = attitude[:, 1]
    torque_z = attitude[:, 2]

    # Map them to ArduSub's expected [Roll, Pitch, Yaw] order
    roll_col = torque_x
    pitch_col = -torque_y
    yaw_col = -torque_z

    # Reassemble the attitude matrix with the new column order
    attitude = np.column_stack((roll_col, pitch_col, yaw_col))

    return attitude


print(attitude_calculator(com_position, thruster_positions, thruster_forces))
