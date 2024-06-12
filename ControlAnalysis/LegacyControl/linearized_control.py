import numpy as np
import control
import matplotlib.pyplot as plt

# Constants
g = 3.71  # m/s^2

# Parameters
mass = 60.0  # kg
arm_length = 2.0  # m
torque_thrust_ratio = 0.1  # m^-1

# Estimated weight
estimated_weight = mass * g

# Thrust range per motor
hover_thrust_per_motor = estimated_weight / 6
thrust_min = hover_thrust_per_motor * 0.25
thrust_max = hover_thrust_per_motor * 1.5

Jxx = 5.0  # kg*m^2
Jyy = 5.0  # kg*m^2
Jzz = 8.0  # kg*m^2

# Transfer functions for the plant
phi_tf = control.TransferFunction([arm_length], [Jxx, 0, 0])
theta_tf = control.TransferFunction([arm_length], [Jyy, 0, 0])
psi_tf = control.TransferFunction([arm_length], [Jzz, 0, 0])
Z_tf = control.TransferFunction([1], [mass, 0])

# Function to plot root locus
def plot_root_locus(tf, title):
    plt.figure()
    control.root_locus(tf)
    plt.title(title)
    plt.xlabel('Real Axis')
    plt.ylabel('Imaginary Axis')
    plt.grid(True)
    plt.show()

# Plot root locus for each transfer function
plot_root_locus(phi_tf, 'Root Locus of Phi')
plot_root_locus(theta_tf, 'Root Locus of Theta')
plot_root_locus(psi_tf, 'Root Locus of Psi')
plot_root_locus(Z_tf, 'Root Locus of Z')

# Based on the root locus, choose desired pole locations
desired_poles_phi = [-0.5, -0.5]
desired_poles_theta = [-0.5, -0.5]
desired_poles_psi = [-0.5, -0.5]
desired_poles_z = [-0.5, -0.5]

# Function to tune PID controller based on desired poles
def tune_pid_with_desired_poles(tf, desired_poles):
    # Calculate PID parameters using desired poles
    kp, ki, kd = control.acker(tf.den[0][0], tf.num[0][0], desired_poles)
    pid_tf = control.TransferFunction([kd, kp, ki], [1, 0])
    return pid_tf

# Tune PID controllers using desired poles
pid_phi = tune_pid_with_desired_poles(phi_tf, desired_poles_phi)
pid_theta = tune_pid_with_desired_poles(theta_tf, desired_poles_theta)
pid_psi = tune_pid_with_desired_poles(psi_tf, desired_poles_psi)
pid_z = tune_pid_with_desired_poles(Z_tf, desired_poles_z)

# Plot step response for closed-loop transfer functions with tuned parameters
def plot_step_response(tf, pid_tf, title):
    closed_loop_tf = control.feedback(pid_tf * tf)
    t, y = control.step_response(closed_loop_tf)
    plt.figure()
    plt.plot(t, y)
    plt.title(title)
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude')
    plt.grid(True)
    plt.show()

plot_step_response(phi_tf, pid_phi, 'Step Response - Closed Loop Phi (with tuned parameters)')
plot_step_response(theta_tf, pid_theta, 'Step Response - Closed Loop Theta (with tuned parameters)')
plot_step_response(psi_tf, pid_psi, 'Step Response - Closed Loop Psi (with tuned parameters)')
plot_step_response(Z_tf, pid_z, 'Step Response - Closed Loop Altitude (Z) (with tuned parameters)')
