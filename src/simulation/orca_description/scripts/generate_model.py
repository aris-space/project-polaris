#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2022 Clyde McQueen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Generate the model.sdf file by substituting strings of the form "@foo" with calculated values

The SDF file uses the ArduPilotPlugin COMMAND control method; this sends commands to a specified
ign-transport topic rather than directly controlling a joint.

We use the COMMAND method to send commands to the Gazebo Sim ThrusterPlugin. The ThrusterPlugin
supports 2 control methods:
      control thrust via /cmd_thrust
      control angular velocity via /cmd_vel

The ThrusterPlugin uses the Fossen equation to relate thrust to angular velocity, and will apply
thrust force to the joint and spin the propeller. Propellers have bounding boxes and inertia, so
spinning the propeller does affect the simulation.
"""

import math
import re
import sys

# SDF 1.9 supports degrees="true"; provide some nice vars for earlier versions
d180 = math.pi
d90 = d180 / 2
d45 = d90 / 2
d135 = d90 + d45

# --- Rigid body (CAD / spreadsheet), body frame at base_link ---
mass = 32.412
ixx = 0.195
ixy = 0.041
ixz = 0.071
iyy = 11.51 # should be 9.216 but multiplied again by 1.5
iyz = 0.0
izz = 11.475 # should be 9.18 but multiplied again by 1.5

# Center of mass / buoyancy reference. Polaris CAD: model frame ≡ CoG (mass_z = 0).
mass_z = 0.0
volume_z = 0.06

# Water density for HydrodynamicsPlugin, ThrusterPlugin, and displaced-mass comment
fluid_density = 997.0

# Visual mesh extent (display only)
visual_x = 0.457
visual_y = 0.338
visual_z = 0.25

# --- Buoyancy: small box (stable with gz buoyancy + heightmap); slightly positively buoyant ---
buoyancy_adjustment = 0.05
displaced_mass = mass + buoyancy_adjustment
collision_x = 0.457
collision_y = 0.338
collision_z = displaced_mass / (collision_x * collision_y * fluid_density)

# --- Hydrodynamic *drag*: slender hull cylinder (areas only; not the buoyancy collision shape) ---
hull_length = 1.78
hull_radius = 0.16
cd_axial = 0.85
cd_cross = 0.68
_area_axial = math.pi * hull_radius**2
_area_cross = 2.0 * hull_radius * hull_length

# Added mass (xDotU = X_u_dot, ...): gz applies F ≈ -Ma * Δv/Δt — keep well below dry mass on
# translational diagonals to avoid DART startup spikes. Full vehicle sheet had Y_v_dot ~ 34 kg;
# we use modest fractions here; raise slowly if the sim stays stable.
_added_mass_surge = 0.67
_added_mass_sway_heave = 0.15 * mass  # ~3.24 kg
_added_mass_roll = 0.08
_added_mass_pitch_yaw = 1.5
xDotU = _added_mass_surge
yDotV = _added_mass_sway_heave
zDotW = _added_mass_sway_heave
kDotP = _added_mass_roll
mDotQ = _added_mass_pitch_yaw
nDotR = _added_mass_pitch_yaw

# Quadratic damping: slender cylinder, F = 0.5 * rho * Cd * A * |v| * v (plugin uses coeffs
# on u|u|, etc.; signs negative for dissipative force, matching gz Orca4 convention).
xUabsU = -0.5 * fluid_density * cd_axial * _area_axial
yVabsV = -0.5 * fluid_density * cd_cross * _area_cross
zWabsW = yVabsV
# Roll: scale from lateral drag × (diameter / length); pitch/yaw moment from cross-flow on body
kPabsP = yVabsV * (2.0 * hull_radius / hull_length)
mQabsQ = -0.25 * fluid_density * cd_cross * hull_radius * hull_length**2
nRabsR = mQabsQ

# Linear damping: not on the datasheet, but required for low-speed decay (quadratic → 0 as v → 0,
# so without this the vehicle coasts indefinitely after disarm). Tied to the quadratic terms via a
# crossover velocity: linear damping dominates below v_cross, quadratic above. Tune v_cross_* if
# settling looks too sluggish (raise) or too damped (lower).
v_cross_lin = 0.5  # m/s
v_cross_ang = 1.0  # rad/s
xU = xUabsU * v_cross_lin
yV = yVabsV * v_cross_lin
zW = zWabsW * v_cross_lin
kP = kPabsP * v_cross_ang
mQ = mQabsQ * v_cross_ang
nR = nRabsR * v_cross_ang

# Thruster placement (Polaris CAD, body frame ≡ CoG, Gazebo FLU: x-fwd, y-port, z-up).
# Each MOT_n: position (xyz) and orientation (rpy) chosen so that with joint axis
# <0 0 -1> and positive thrust_coefficient, a positive PWM produces force in the
# direction listed below. Mixer matrix in fork's AP_Motors6DOF.cpp:181-186 is
# self-consistent with these positions and force directions (verified via cross-product
# attitude calculator).

# MOT_1 (PWM 1) — surge, force +X (fwd)
t1_x, t1_y, t1_z = -0.864,  0.000,  0.000
t1_R, t1_P, t1_Y =  0.0,   -d90,    0.0

# MOT_2 (PWM 2) — rear lateral, force +Y (port)
t2_x, t2_y, t2_z = -0.537,  0.000,  0.000
t2_R, t2_P, t2_Y =  d90,    0.0,    0.0

# MOT_3 (PWM 3) — left vertical, force +Z (up)
t3_x, t3_y, t3_z = -0.493,  0.148,  0.035
t3_R, t3_P, t3_Y =  d180,   0.0,    0.0

# MOT_4 (PWM 4) — right vertical, force +Z (up)
t4_x, t4_y, t4_z = -0.493, -0.148,  0.035
t4_R, t4_P, t4_Y =  d180,   0.0,    0.0

# MOT_5 (PWM 5) — front vertical, force +Z (up)
t5_x, t5_y, t5_z =  0.600,  0.000,  0.000
t5_R, t5_P, t5_Y =  d180,   0.0,    0.0

# MOT_6 (PWM 6) — front lateral, force +Y (port)
t6_x, t6_y, t6_z =  0.695,  0.000,  0.000
t6_R, t6_P, t6_Y =  d90,    0.0,    0.0

# Propeller link parameters
propeller_size = "0.1 0.02 0.01"
propeller_mass = 0.002
propeller_ixx = 0.001
propeller_iyy = 0.001
propeller_izz = 0.001

# ThrusterPlugin parameters
propeller_diameter = 0.1
thrust_coefficient = 0.02

# Max thrust force, N
# Both forward and reverse thrust must be the same
max_thrust = 50

# ArduPilotPlugin control parameters
servo_min = 1100
servo_max = 1900
control_offset = -0.5

# From the command line
use_angvel_cmd = False

# Set by update_globals()
cw_control_multiplier = 0   # Thrusters 3, 4 and 6
ccw_control_multiplier = 0  # Thrusters 1, 2 and 5
thruster1_topic = "/model/orca4/joint/thruster1_joint/cmd_"
thruster2_topic = "/model/orca4/joint/thruster2_joint/cmd_"
thruster3_topic = "/model/orca4/joint/thruster3_joint/cmd_"
thruster4_topic = "/model/orca4/joint/thruster4_joint/cmd_"
thruster5_topic = "/model/orca4/joint/thruster5_joint/cmd_"
thruster6_topic = "/model/orca4/joint/thruster6_joint/cmd_"


# Fossen equation, see "Guidance and Control of Ocean Vehicles" p. 246
def thrust_to_ang_vel(thrust):
    assert thrust >= 0
    assert thrust_coefficient >= 0
    return math.sqrt(thrust / (fluid_density * thrust_coefficient * pow(propeller_diameter, 4)))


def update_globals():
    global cw_control_multiplier
    global ccw_control_multiplier
    global thruster1_topic
    global thruster2_topic
    global thruster3_topic
    global thruster4_topic
    global thruster5_topic
    global thruster6_topic

    if use_angvel_cmd:
        print("control method: angular velocity")
        thruster1_topic += "vel"
        thruster2_topic += "vel"
        thruster3_topic += "vel"
        thruster4_topic += "vel"
        thruster5_topic += "vel"
        thruster6_topic += "vel"

        # Angular velocity range in rad/s
        # Thrust ~ sqrt(angular velocity), so the curves are quite different
        # Reverse the angular velocity for thrusters 3, 4 and 6
        cw_control_multiplier = -thrust_to_ang_vel(max_thrust) * 2
        ccw_control_multiplier = thrust_to_ang_vel(max_thrust) * 2
    else:
        print("control method: thrust force")
        thruster1_topic += "thrust"
        thruster2_topic += "thrust"
        thruster3_topic += "thrust"
        thruster4_topic += "thrust"
        thruster5_topic += "thrust"
        thruster6_topic += "thrust"

        # Force range [-50, 50] in N
        cw_control_multiplier = max_thrust * 2
        ccw_control_multiplier = max_thrust * 2


def generate_model(input_path, output_path):
    s = open(input_path, "r").read()
    pattern = re.compile(r"@(\w+)")
    # globals()['foo'] will return the value of foo
    # TODO(clyde) trim floats
    s = re.sub(pattern, lambda m: str(globals()[m.group(1)]), s)
    open(output_path, "w").write(s)


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage:")
        print("generate_model.py infile outfile 0|1")
        print("0: control thrust force")
        print("1: control angular velocity")
        exit(-100)

    use_angvel_cmd = bool(int(sys.argv[3]))

    update_globals()

    generate_model(sys.argv[1], sys.argv[2])
