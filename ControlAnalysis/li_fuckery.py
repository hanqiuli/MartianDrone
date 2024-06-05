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
            return np.clip(self.Kp * error + self.Ki * self.integral + self.Kd * derivative, -0.25, 0.25) #TODO: proper clipping here
        elif self.type == 'attitude':
            return np.clip(self.Kp * error + self.Ki * self.integral + self.Kd * derivative, -0.25, 0.25) #TODO: proper clipping here
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
        
        mass_skew_factor = 0  # the fraction that the base thrust is off of the hover thrust
        self.estimated_mass = mass*(1+mass_skew_factor)
        self.estimated_moment_inertia = np.max(moment_inertia*(1+mass_skew_factor))

        self.setup_pids(pid_params)

        self.transformation_matrix = np.array([
            [-1, -1, -1, -1, -1, -1],
            [0, -arm_length * np.sqrt(3) / 2, -arm_length * np.sqrt(3) / 2, 0, arm_length * np.sqrt(3) / 2,
             arm_length * np.sqrt(3) / 2],
            [arm_length, 0.5 * arm_length, -0.5 * arm_length, -arm_length, -0.5 * arm_length, 0.5 * arm_length],
            [-torque_thrust_ratio, torque_thrust_ratio, -torque_thrust_ratio, torque_thrust_ratio,
             -torque_thrust_ratio, torque_thrust_ratio]
        ])

        self.input_type_list = ['altitude', 'attitude', 'attitude', 'attitude']  

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
        g = self.environment['g']


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
        self.input_to_pid_control_map = [0, 1, 2, 3]

    def clip_inputs(self, inputs, thrust_range=[-0.5, 0.5], moment_range=[-1, 1]):
        #TODO have function to map these to thrusters and clip these while retaining relative moments
        for i, input_type in enumerate(self.input_type_list):
            if 'thruster' == input_type:
                inputs[i] = np.clip(inputs[i], thrust_range[0], thrust_range[1])
            elif 'altitude' == input_type:
                inputs[i] = np.clip(inputs[i], thrust_range[0], thrust_range[1])
            elif 'attitude' == input_type:
                inputs[i] = np.clip(inputs[i], moment_range[0], moment_range[1])
            else:
                raise ValueError('Undefined input type in input type list')

        return inputs

    def mod_inputs(self, inputs):
        # M = pid * J * g / L
        # T = (1+pid)*mass estimated*g

        for i, input_type in enumerate(self.input_type_list):
            if 'thruster' == input_type:
                pass
            elif 'altitude' == input_type:
                inputs[i] = (inputs[i] + 1) * self.estimated_mass * self.environment['g']
            elif 'attitude' == input_type:
                inputs[i] *= self.estimated_moment_inertia * self.environment['g'] / self.arm_length
            else:
                raise ValueError('Undefined input type in input type list')
        
        return inputs
            

    def get_control(self, state, set_point, dt, *, thrust_range=[-0.5, 0.5], moment_range=[-1, 1]):
        inputs = [0]*len(self.pid_control_to_input_map)

        for i, pid in enumerate(self.pid_list_control):
            state_id = self.pid_control_to_state_map[i]
            pid.set_point = set_point[state_id]

            input_id = self.pid_control_to_input_map[i]
            inputs[input_id] = pid.update(state[state_id], dt)

        inputs = self.clip_inputs(inputs, thrust_range=thrust_range, moment_range=moment_range)
        inputs = self.mod_inputs(inputs)
        
        return inputs
    
    def simulate(self, times, initial_state, setpoints):

        """
        Simulate the hexacopter dynamics
        times:  array-like - time points for the simulation
        initial_state: array-like - initial state of the hexacopter [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]
        setpoints: array-like - setpoints for the simulation [z, phi, theta, psi], this should be the same length as max_time/time_step
        """
        desired_x, desired_y, desired_z, desired_phi, desired_theta, desired_psi, desired_xdot, desired_ydot, desired_zdot, desired_p, desired_q, desired_r = setpoints
        time_step = times[1] - times[0]
        states = [initial_state]
        for i in range(1, len(times)):
            current_state = states[-1]
            inputs = self.get_control(current_state, [desired_x[i], desired_y[i], desired_z[i], desired_phi[i], desired_theta[i], desired_psi[i], desired_xdot[i], desired_ydot[i], desired_zdot[i], desired_p[i], desired_q[i], desired_r[i]], time_step)
            inputs = tuple([inputs],)
            new_state = odeint(self.hexacopter_dynamics, current_state, [0, time_step], args=inputs)[-1]
            states.append(new_state)
        
        return np.array(states), times
    
    @staticmethod
    def plot_figures(states, times, setpoints):

        """
            Plot the simulation results
        """

        desired_x, desired_y, desired_z, desired_phi, desired_theta, desired_psi, desired_xdot, desired_ydot, desired_zdot, desired_p, desired_q, desired_r = setpoints

        plt.figure()
        plt.subplot(4, 1, 1)
        plt.plot(times, states[:, 0], label='x')
        plt.plot(times, desired_x, 'r--', label='desired x')
        plt.plot(times, states[:, 1], label='y')
        plt.plot(times, desired_y, 'g--', label='desired y')
        plt.xlabel('Time [s]')
        plt.ylabel('Position [m]')
        plt.legend()

        plt.subplot(4, 1, 2)
        plt.plot(times, states[:, 2], label='z')
        plt.plot(times, desired_z, 'r--', label='desired z')
        plt.xlabel('Time [s]')
        plt.ylabel('Altitude [m]')
        plt.ylim([0, 100])
        plt.legend()

        plt.subplot(4, 1, 3)
        plt.plot(times, states[:, 3], label='phi')
        plt.plot(times, desired_phi, 'r--', label='desired phi')
        plt.xlabel('Time [s]')
        plt.ylabel('Roll [rad]')
        plt.legend()

        plt.subplot(4, 1, 4)
        plt.plot(times, states[:, 4], label='theta')
        plt.plot(times, desired_theta, 'r--', label='desired theta')
        plt.xlabel('Time [s]')
        plt.ylabel('Pitch [rad]')
        plt.legend()

        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    
    mass = 60.0
    moment_inertia = np.diag([5, 5, 5])
    moment_inertia_prop = 0.01
    pid_params = [[12, 0.5, 12, 3], [5, 0.2, 3, 10], [5, 0.2, 3, 10], [5, 0.2, 3, 10]]
    torque_thrust_ratio = 0.1
    omega_thrust_ratio = 0.1
    ENV = ENVdict

    hexacopter = HexacopterModel(mass, moment_inertia, moment_inertia_prop, pid_params, torque_thrust_ratio=torque_thrust_ratio, omega_thrust_ratio=omega_thrust_ratio, ENV=ENV)

    t = np.linspace(0, 50, 10000)
    initial_state = np.zeros(12)
    # initial altitude is nonzero
    initial_state[2] = 50 

    # desired_z = np.zeros_like(t)
    # desired_z[t <= 5] = np.linspace(0, 20, len(t[t <= 5]))
    # desired_z[(t > 5) & (t <= 10)] = 20
    # desired_z[(t > 10) & (t <= 20)] = 30
    # desired_z[(t > 20) & (t <= 30)] = 10
    # desired_z[(t > 30) & (t <= 32)] = 20
    # desired_z[t > 32] = 20

    desired_x = np.zeros_like(t)
    desired_y = np.zeros_like(t)
    desired_z = np.ones_like(t) * 50 # should hover
    desired_phi = np.sin(t/2)/10
    desired_theta = np.zeros_like(t)
    desired_psi = np.zeros_like(t)
    desired_xdot = np.zeros_like(t)
    desired_ydot = np.zeros_like(t)
    desired_zdot = np.zeros_like(t)
    desired_p = np.zeros_like(t)
    desired_q = np.zeros_like(t)
    desired_r = np.zeros_like(t)
    
    desired_states = [desired_x, desired_y, desired_z, desired_phi, desired_theta, desired_psi, desired_xdot, desired_ydot, desired_zdot, desired_p, desired_q, desired_r]

    # desired_states = [
    #     np.zeros_like(t),
    #     np.zeros_like(t),
    #     # z, phi, theta, psi
    #     desired_z,
    #     np.zeros_like(t),
    #     np.zeros_like(t),
    #     np.zeros_like(t),
    #     np.zeros_like(t),
    #     np.zeros_like(t),
    #     np.zeros_like(t),
    #     np.zeros_like(t),
    #     np.zeros_like(t),
    #     np.zeros_like(t)
    # ]

    states, times = hexacopter.simulate(t, initial_state, desired_states)
    hexacopter.plot_figures(states, times, desired_states)


    
    