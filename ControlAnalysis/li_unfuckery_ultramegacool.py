import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint

sys.path.append('.')
from legacy.DesignTools.environment_properties import ENV as ENVdict


class PIDController:
    def __init__(self, Kp, Ki, Kd, wind_up_limit, *, set_point=0, PID_type='altitude'):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.wind_up_limit = wind_up_limit

        self.set_point = set_point

        self.integral = 0
        self.previous_error = 0
        self.type=PID_type

    def update(self, measurement, dt):
        # Error term
        error = self.set_point - measurement

        # Integral term
        self.integral += error * dt
        if self.wind_up_limit is not None:
            self.integral = np.clip(self.integral, -self.wind_up_limit, self.wind_up_limit)

        # Derivative term
        derivative = (error - self.previous_error) / dt
        self.previous_error = error

        # Clipped return
        if self.type == 'altitude':
            return np.clip(self.Kp * error + self.Ki * self.integral + self.Kd * derivative, -0.5*g, 0.5*g) #TODO: proper clipping here
        else:
            return self.Kp * error + self.Ki * self.integral + self.Kd * derivative


class HexacopterModel:
    """
        Hexacopter dynamics model
    """

    def __init__(self, mass, moment_inertia, moment_inertia_prop, pid_params, *, torque_thrust_ratio, omega_thrust_ratio, ENV=None, arm_length=2):
        """
        Class initializer

        Positional arguments:
            mass                [kg]            float       mass of the drone in kg
            moment_inertia      [kg m^2]        array-like  lists of moment of inertia of the drone in, [J_x, J_y, J_z]
            moment_inertia_prop [kg m^2]        float       moment of inertia of the propellers + motors
            pid_params          [N/A]           list        List of pid parameters, sorted per pid: [gain_list, ]

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

        self.setup_pids(pid_params)

        self.transformation_matrix = np.array([
            [-1, -1, -1, -1, -1, -1],
            [0, -arm_length * np.sqrt(3) / 2, -arm_length * np.sqrt(3) / 2, 0, arm_length * np.sqrt(3) / 2,
             arm_length * np.sqrt(3) / 2],
            [arm_length, 0.5 * arm_length, -0.5 * arm_length, -arm_length, -0.5 * arm_length, 0.5 * arm_length],
            [-torque_thrust_ratio, torque_thrust_ratio, -torque_thrust_ratio, torque_thrust_ratio,
             -torque_thrust_ratio, torque_thrust_ratio]
        ])

    # Hexacopter dynamics
    def hexacopter_dynamics(self, state, t, inputs):
        x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r = state

        u1, u2, u3, u4, *unassigned_inputs = inputs

        if len(unassigned_inputs) > 0:
            print(f'Warning: there are {len(unassigned_inputs)} too many control inputs specified in the dynamics system call')

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

    def setup_pids(self, pid_params):
        # Initialize PID controllers for altitude and attitude
        self.pid_list_control = [
            PIDController(*pid_params.pop(0), PID_type = 'altitude'),
            PIDController(*pid_params.pop(0), PID_type = 'attitude'),   # roll
            PIDController(*pid_params.pop(0), PID_type = 'attitude'),   # pitch
            PIDController(*pid_params.pop(0), PID_type = 'attitude')    # yaw
            ]

        # x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r
        self.state_to_pid_control_map = [None, None, 0, 1, 2, 3, None, None, None, None, None, None]

        self.pid_control_to_state_map = [2, 3, 4, 5]
        
        self.pid_control_to_input_map = [0, 1, 2, 3]

    def get_control(self, state, set_point, dt):
        inputs = [0]*len(self.pid_control_to_input_map)

        for i, pid in enumerate(self.pid_list_control):
            state_id = self.pid_control_to_state_map[i]
            pid.set_point = set_point[state_id]

            input_id = self.pid_control_to_input_map[i]
            inputs[input_id] = pid.update(state[state_id], dt)
        
        return inputs
    
    def clip_inputs(self, inputs, thrust_range=[-0.5, 0.5], moment_range=[-1, 1]):
        # Clip max thrust
        inputs[0] = np.clip(inputs[0], thrust_range[0], thrust_range[1])
        # Clip maximum moments
        inputs[2]
        
    def simulate

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

