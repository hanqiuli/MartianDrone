import numpy as np
from scipy.optimize import linprog

# Constants
g = 9.81  # m/s^2

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

# Function to calculate max moments based on thrust configuration
def calculate_max_moments(thrust_min, thrust_max, hover_thrust_per_motor, arm_length, torque_thrust_ratio, estimated_weight):
    def optimize_moment(c):
        # Objective: maximize or minimize c^T * T
        # Constraints: A_eq * T = b_eq and bounds on T
        A_eq = np.ones((1, 6))
        b_eq = np.array([estimated_weight])
        bounds = [(thrust_min, thrust_max) for _ in range(6)]

        res_max = linprog(-c, A_eq=A_eq, b_eq=b_eq, bounds=bounds)
        res_min = linprog(c, A_eq=A_eq, b_eq=b_eq, bounds=bounds)
        max_moment = -res_max.fun if res_max.success else None
        min_moment = res_min.fun if res_min.success else None

        return max(abs(max_moment), abs(min_moment))

    # Define coefficients for roll (Mx), pitch (My), and yaw (Mz)
    c_roll = np.array([arm_length/2, -arm_length, -arm_length/2, arm_length/2, arm_length, arm_length/2])
    c_pitch = np.array([-arm_length*np.sqrt(3)/2, 0, arm_length*np.sqrt(3)/2, arm_length*np.sqrt(3)/2, 0, -arm_length*np.sqrt(3)/2])
    c_yaw = np.array([-torque_thrust_ratio, torque_thrust_ratio, -torque_thrust_ratio, torque_thrust_ratio, -torque_thrust_ratio, torque_thrust_ratio])

    max_moment_x = optimize_moment(c_roll)
    max_moment_y = optimize_moment(c_pitch)
    max_moment_z = optimize_moment(c_yaw)

    return max_moment_x, max_moment_y, max_moment_z

max_moment_x, max_moment_y, max_moment_z = calculate_max_moments(thrust_min, thrust_max, hover_thrust_per_motor, arm_length, torque_thrust_ratio, estimated_weight)

print("Max Moment X:", max_moment_x, "N*m")
print("Max Moment Y:", max_moment_y, "N*m")
print("Max Moment Z:", max_moment_z, "N*m")

# Implement the previous linearized control code with these calculated moments
import control
import matplotlib.pyplot as plt

# Transfer functions for the plant
phi_tf = control.TransferFunction([arm_length], [5, 0, 0])
theta_tf = control.TransferFunction([arm_length], [5, 0, 0])
psi_tf = control.TransferFunction([arm_length], [8, 0, 0])
Z_tf = control.TransferFunction([1], [mass, 0])

# Function to tune PID parameters with actuator saturation
def tune_pid_with_saturation(tf, kp, ki, kd, max_output):
    # PID controller transfer function
    pid_tf = control.TransferFunction([kd, kp, ki], [1, 0])
    # Open-loop transfer function
    open_loop_tf = pid_tf * tf
    # Closed-loop transfer function
    closed_loop_tf = control.feedback(open_loop_tf)
    
    # Apply saturation
    t, y = control.step_response(closed_loop_tf)
    y = np.clip(y, -max_output, max_output)
    
    return closed_loop_tf, t, y

# Set initial PID parameters for each axis
attitude_pids = [10, 3, 10]
kp_phi, ki_phi, kd_phi = attitude_pids
kp_theta, ki_theta, kd_theta = attitude_pids
kp_psi, ki_psi, kd_psi = attitude_pids
kp_z, ki_z, kd_z = 10, 5, 20

# Tune PID controllers with saturation
closed_loop_phi, t_phi, y_phi = tune_pid_with_saturation(phi_tf, kp_phi, ki_phi, kd_phi, max_moment_x)
closed_loop_theta, t_theta, y_theta = tune_pid_with_saturation(theta_tf, kp_theta, ki_theta, kd_theta, max_moment_y)
closed_loop_psi, t_psi, y_psi = tune_pid_with_saturation(psi_tf, kp_psi, ki_psi, kd_psi, max_moment_z)
closed_loop_z, t_z, y_z = tune_pid_with_saturation(Z_tf, kp_z, ki_z, kd_z, hover_thrust_per_motor)

# Plot step response for closed-loop transfer functions with saturation
plt.figure(figsize=(15, 10))
plt.subplot(2, 2, 1)
plt.plot(t_phi, y_phi)
plt.title('Step Response - Closed Loop Phi (with saturation)')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')

plt.subplot(2, 2, 2)
plt.plot(t_theta, y_theta)
plt.title('Step Response - Closed Loop Theta (with saturation)')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')

plt.subplot(2, 2, 3)
plt.plot(t_psi, y_psi)
plt.title('Step Response - Closed Loop Psi (with saturation)')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')

plt.subplot(2, 2, 4)
plt.plot(t_z, y_z)
plt.title('Step Response - Closed Loop Altitude (Z) (with saturation)')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')

plt.tight_layout()
plt.show()

# Gain and Phase Margin
gm_phi, pm_phi, wg_phi, wp_phi = control.margin(closed_loop_phi)
gm_theta, pm_theta, wg_theta, wp_theta = control.margin(closed_loop_theta)
gm_psi, pm_psi, wg_psi, wp_psi = control.margin(closed_loop_psi)
gm_z, pm_z, wg_z, wp_z = control.margin(closed_loop_z)

print("Gain Margin and Phase Margin for Phi: GM =", gm_phi, "PM =", pm_phi)
print("Gain Margin and Phase Margin for Theta: GM =", gm_theta, "PM =", pm_theta)
print("Gain Margin and Phase Margin for Psi: GM =", gm_psi, "PM =", pm_psi)
print("Gain Margin and Phase Margin for Z: GM =", gm_z, "PM =", pm_z)

# Overshoot and Settling Time
info_phi = control.step_info(closed_loop_phi)
info_theta = control.step_info(closed_loop_theta)
info_psi = control.step_info(closed_loop_psi)
info_z = control.step_info(closed_loop_z)

print("Overshoot and Settling Time for Phi:", info_phi['Overshoot'], info_phi['SettlingTime'])
print("Overshoot and Settling Time for Theta:", info_theta['Overshoot'], info_theta['SettlingTime'])
print("Overshoot and Settling Time for Psi:", info_psi['Overshoot'], info_psi['SettlingTime'])
print("Overshoot and Settling Time for Z:", info_z['Overshoot'], info_z['SettlingTime'])

