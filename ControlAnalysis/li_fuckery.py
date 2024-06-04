import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Hexacopter parameters
m = 0.468  # mass (kg)
g = 9.81   # gravity (m/s^2)
Ixx = 4.856e-3  # Inertia around x-axis
Iyy = 4.856e-3  # Inertia around y-axis
Izz = 8.801e-3  # Inertia around z-axis
k = 2.980e-6    # lift constant
b = 1.140e-7    # drag constant
l = 0.225       # distance from center to rotor

# Quaternion functions
def quaternion_to_rotation_matrix(q):
    q0, q1, q2, q3 = q
    return np.array([
        [q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2 - q0*q3), 2*(q0*q2 + q1*q3)],
        [2*(q1*q2 + q0*q3), q0**2 - q1**2 + q2**2 - q3**2, 2*(q2*q3 - q0*q1)],
        [2*(q1*q3 - q0*q2), 2*(q0*q1 + q2*q3), q0**2 - q1**2 - q2**2 + q3**2]
    ])

def quaternion_derivative(q, angular_velocity):
    q0, q1, q2, q3 = q
    p, q, r = angular_velocity
    q_dot = 0.5 * np.array([
        -q1*p - q2*q - q3*r,
        q0*p - q3*q + q2*r,
        q3*p + q0*q - q1*r,
        -q2*p + q1*q + q0*r
    ])
    return q_dot

def quaternion_error(q_desired, q_actual):
    q_conj = np.array([q_actual[0], -q_actual[1], -q_actual[2], -q_actual[3]])
    q_error = quaternion_multiply(q_desired, q_conj)
    return q_error

def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

# Translational dynamics
def translational_dynamics(thrust, q, velocity):
    R = quaternion_to_rotation_matrix(q)
    gravity = np.array([0, 0, -m * g])
    thrust_force = np.array([0, 0, thrust])
    acceleration = (R @ thrust_force + gravity) / m
    return acceleration

# Rotational dynamics
def rotational_dynamics(tau, angular_velocity):
    p, q, r = angular_velocity
    I = np.diag([Ixx, Iyy, Izz])
    I_inv = np.linalg.inv(I)
    gyro_effects = np.cross(angular_velocity, I @ angular_velocity)
    angular_acceleration = I_inv @ (tau - gyro_effects)
    return angular_acceleration

# PD controller
def pd_controller(q_error, angular_velocity_error, Kp, Kd):
    tau = Kp * q_error[1:4] + Kd * angular_velocity_error
    return tau

# System dynamics
def system_dynamics(t, state, q_desired, angular_velocity_desired, Kp, Kd):
    position = state[0:3]
    velocity = state[3:6]
    q = state[6:10]
    angular_velocity = state[10:13]

    q_error = quaternion_error(q_desired, q)
    angular_velocity_error = angular_velocity_desired - angular_velocity

    thrust = m * g  # For simplicity, assume hover thrust
    tau = pd_controller(q_error, angular_velocity_error, Kp, Kd)

    acceleration = translational_dynamics(thrust, q, velocity)
    angular_acceleration = rotational_dynamics(tau, angular_velocity)
    q_dot = quaternion_derivative(q, angular_velocity)

    return np.hstack((velocity, acceleration, q_dot, angular_acceleration))

# Initial conditions
initial_position = np.array([0, 0, 0])
initial_velocity = np.array([0, 0, 0])
initial_q = np.array([1, 0, 0, 0])  # No initial rotation
initial_angular_velocity = np.array([0, 0, 0])
initial_state = np.hstack((initial_position, initial_velocity, initial_q, initial_angular_velocity))

# Desired state
q_desired = np.array([0.92387953, 0, 0.38268343, 0])  # Desired 45-degree rotation around z-axis
angular_velocity_desired = np.array([0, 0, 0])

# PD gains
Kp = np.array([1, 1, 1])
Kd = np.array([0.1, 0.1, 0.1])

# Time span for the simulation
t_span = (0, 10)  # 10 seconds
