import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint

sys.path.append('.')
from legacy.DesignTools.environment_properties import ENV as ENVdict


class PIDController:
    def __init__(self, Kp, Ki, Kd, set_point, clip_range):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.set_point = set_point
        self.integral = 0
        self.previous_error = 0
        self.type=PID_type
        self.clip_range = clip_range

    def update(self, measurement, dt):
        error = self.set_point - measurement
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        return np.clip(self.Kp * error + self.Ki * self.integral + self.Kd * derivative, self.clip_range[0], self.clip_range[1])


class HexacopterModel:
    """
        Hexacopter dynamics model
    """

    def __init__(self, mass, moment_inertia, moment_inertia_prop, *, torque_thrust_ratio, omega_thrust_ratio, ENV=None, arm_length=2):
        """
        Class initializer

        Positional arguments:
            mass                [kg]            float       mass of the drone in kg
            moment_inertia      [kg m^2]        array-like  lists of moment of inertia of the drone in, [J_x, J_y, J_z]
            moment_inertia_prop [kg m^2]        float       moment of inertia of the propellers + motors

        Keyword arguments:
            ENV                 [N/A]           dict        dictionary containing the environmental properties
            arm_length          [m]             float       length of the arm of the drone
            torque_thrust_ratio [m^-1]          float       ratio of reactive torque to thrust of propellers
            omega_thrust_ratio  [rad (Ns)^-1]   float       ratio of angular velocity to thrust of propellers
            arm_length          [m]             float       length of the arm of the drone
        """

        self.mass = mass
        self.moment_inertia = moment_inertia
        self.environment = ENV if ENV else ENVdict
        self.moment_inertia_prop = moment_inertia_prop
        self.arm_length = arm_length
        self.torque_thrust_ratio = torque_thrust_ratio
        self.omega_thrust_ratio = omega_thrust_ratio

        self.transformation_matrix = np.array([
            [-1, -1, -1, -1, -1, -1],
            [0, -arm_length * np.sqrt(3) / 2, -arm_length * np.sqrt(3) / 2, 0, arm_length * np.sqrt(3) / 2,
             arm_length * np.sqrt(3) / 2],
            [arm_length, 0.5 * arm_length, -0.5 * arm_length, -arm_length, -0.5 * arm_length, 0.5 * arm_length],
            [-torque_thrust_ratio, torque_thrust_ratio, -torque_thrust_ratio, torque_thrust_ratio,
             -torque_thrust_ratio, torque_thrust_ratio]
        ])

    # Hexacopter dynamics
    def hexacopter_dynamics(self, state, t, u1, u2, u3, u4):
        x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r = state

        J = self.moment_inertia
        J_mp = self.moment_inertia_prop
        m = self.mass
        K_MT = self.torque_thrust_ratio
        K_omegaT = self.omega_thrust_ratio


        # Translational dynamics
        x_ddot = (u1/m) * (np.cos(phi) * np.sin(theta) * np.cos(psi) + np.sin(phi) * np.sin(psi))
        y_ddot = (u1/m) * (np.cos(phi) * np.sin(theta) * np.sin(psi) - np.sin(phi) * np.cos(psi))
        z_ddot = (u1/m) * np.cos(phi) * np.cos(theta) - g
        
        # Rotational dynamics
        phi_dot = p + q * np.sin(phi) * np.tan(theta) + r * np.cos(phi) * np.tan(theta)
        theta_dot = q * np.cos(phi) - r * np.sin(phi)
        psi_dot = q * np.sin(phi) / np.cos(theta) + r * np.cos(phi) / np.cos(theta)
        
        p_dot = (J[1,1] - J[2,2]) * q * r / J[0,0] + u2 / J[0,0] #TODO: Check whether we should include propeller inertia effects
        q_dot = (J[2,2] - J[0,0]) * p * r / J[1,1] + u3 / J[1,1]
        r_dot = (J[0,0] - J[1,1]) * p * q / J[2,2] + u4 / J[2,2]
        
        return [x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot, x_ddot, y_ddot, z_ddot, p_dot, q_dot, r_dot]

# Initialize PID controllers for altitude and attitude
pid_altitude = PIDController(Kp_alt, Ki_alt, Kd_alt, desired_state[2])
pid_roll = PIDController(Kp_att, Ki_att, Kd_att, desired_state[3])
pid_pitch = PIDController(Kp_att, Ki_att, Kd_att, desired_state[4])
pid_yaw = PIDController(Kp_att, Ki_att, Kd_att, desired_state[5])

# Simulation loop
dt = t[1] - t[0]
states = [initial_state]
for i in range(1, len(t)):
    current_state = states[-1]
    z = current_state[2]
    phi = current_state[3]
    theta = current_state[4]
    psi = current_state[5]
    
    pid_altitude.set_point = desired_z[i]  # Update the desired altitude at each time step
    
    u1 = m * (g + pid_altitude.update(z, dt))
    u2 = pid_roll.update(phi, dt)
    u3 = pid_pitch.update(theta, dt)
    u4 = pid_yaw.update(psi, dt)
    
    new_state = odeint(hexacopter_dynamics, current_state, [0, dt], args=(u1, u2, u3, u4))[-1]
    states.append(new_state)

# Convert states to numpy array for easier plotting
states = np.array(states)

# Plotting
plt.figure()
plt.subplot(3, 1, 1)
plt.plot(t, states[:, 2], label='z')
plt.plot(t, desired_z, 'r--', label='desired z')
plt.xlabel('Time [s]')
plt.ylabel('Altitude [m]')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(t, states[:, 3], label='phi')
plt.plot(t, np.zeros_like(t), 'r--', label='desired phi')
plt.xlabel('Time [s]')
plt.ylabel('Roll [rad]')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(t, states[:, 4], label='theta')
plt.plot(t, np.zeros_like(t), 'r--', label='desired theta')
plt.xlabel('Time [s]')
plt.ylabel('Pitch [rad]')
plt.legend()

plt.tight_layout()
plt.show()

