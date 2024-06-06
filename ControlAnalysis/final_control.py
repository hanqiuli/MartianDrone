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

        # # Clipped return
        # if self.type == 'altitude':
        #     return np.clip(self.Kp * error + self.Ki * self.integral + self.Kd * derivative, -0.25, 0.25) #TODO: proper clipping here
        # elif self.type == 'attitude':
        #     return np.clip(self.Kp * error + self.Ki * self.integral + self.Kd * derivative, -0.25, 0.25) #TODO: proper clipping here
        # else:
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative


class MotorResponse:
    def __init__(self, time_constant):
        """
        Initialize the FirstOrderLag class.

        :param time_constant: The time constant of the first-order lag.
        :param dt: The timestep for the simulation.
        """
        self.time_constant = time_constant
        self.output = 0.0  # Initial output value

    def get_actual_torque(self, input_torque, dt):
        """
        Update the output based on the input signal.

        :param input_torque: The current input signal value.
        :return: The updated output signal value.
        """
        self.output += (dt / self.time_constant) * (input_torque - self.output)
        return self.output

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
        self.setup_motor_responses(0.15)

        self.thrust_map = np.array([
            [1, 1, 1, 1, 1, 1],
            [-arm_length/2, -arm_length, -arm_length/2, arm_length/2, arm_length, arm_length/2],
            [-arm_length*np.sqrt(3)/2, 0, arm_length*np.sqrt(3)/2, arm_length*np.sqrt(3)/2, 0, -arm_length*np.sqrt(3)/2],
            [-torque_thrust_ratio, torque_thrust_ratio, -torque_thrust_ratio, torque_thrust_ratio,
             -torque_thrust_ratio, torque_thrust_ratio]
        ])

        self.inverted_thrust_map = np.linalg.pinv(self.thrust_map)

        self.input_type_list = ['altitude', 'attitude', 'attitude', 'attitude']  

    # Hexacopter dynamics
    def hexacopter_dynamics(self, state, t, inputs):
        x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r = state

        u1, u2, u3, u4, *unassigned_inputs = inputs

        if len(unassigned_inputs) > 0:
            print(f'Warning: there are {len(unassigned_inputs)} too many control inputs specified in the dynamics system call')

        J = self.moment_inertia
        m = self.mass
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

    def setup_motor_responses(self, time_constant):
        self.motor_response_list = [
            MotorResponse(time_constant),
            MotorResponse(time_constant),
            MotorResponse(time_constant),
            MotorResponse(time_constant),
            MotorResponse(time_constant),
            MotorResponse(time_constant)
        ]
        
    def clip_thrusters(self, thruster_inputs, thrust_range):
        dimensionalized_thrust_range = [thrust_limit*self.estimated_mass*self.environment['g']/6 for thrust_limit in thrust_range]
        return np.clip(thruster_inputs, *dimensionalized_thrust_range)

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

    def get_control(self, state, set_point, dt, *, thrust_range=[0, 1.5]):
        inputs = [0]*len(self.pid_control_to_input_map)

        for i, pid in enumerate(self.pid_list_control):
            state_id = self.pid_control_to_state_map[i]
            pid.set_point = set_point[state_id]

            input_id = self.pid_control_to_input_map[i]
            inputs[input_id] = pid.update(state[state_id], dt)
        
        inputs = self.mod_inputs(inputs) 
        thruster_inputs = self.inverted_thrust_map @ np.array(inputs)
        thruster_inputs = self.clip_thrusters(thruster_inputs, thrust_range)
        for i, motor_response in enumerate(self.motor_response_list):
            thruster_inputs[i] = motor_response.get_actual_torque(thruster_inputs[i], dt)
        inputs = self.thrust_map @ thruster_inputs

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
        thruster_values = []
        for i in range(1, len(times)):
            current_state = states[-1]
            inputs = self.get_control(current_state, [desired_x[i], desired_y[i], desired_z[i], desired_phi[i], desired_theta[i], desired_psi[i], desired_xdot[i], desired_ydot[i], desired_zdot[i], desired_p[i], desired_q[i], desired_r[i]], time_step)
            inputs = tuple([inputs],)
            new_state = odeint(self.hexacopter_dynamics, current_state, [0, time_step], args=inputs)[-1]
            states.append(new_state)
            thruster_values.append(self.inverted_thrust_map @ np.array(inputs[0]))
        
        return np.array(states), times, thruster_values
    
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
    moment_inertia = np.diag([5, 5, 8])
    moment_inertia_prop = 0.01
    pid_params = [[1000, 0.5, 17, 3], [5, 0.2, 3, 10], [5, 0.2, 3, 10], [5, 0.2, 3, 10]]
    torque_thrust_ratio = 0.1
    omega_thrust_ratio = 0.1
    ENV = ENVdict

    hexacopter = HexacopterModel(mass, moment_inertia, moment_inertia_prop, pid_params, torque_thrust_ratio=torque_thrust_ratio, omega_thrust_ratio=omega_thrust_ratio, ENV=ENV)

    t = np.linspace(0, 50, 10000)
    initial_state = np.zeros(12)
    # initial altitude is nonzero
    initial_state[2] = 0

    

    desired_x = np.zeros_like(t)
    desired_y = np.zeros_like(t)
    # desired_z = np.ones_like(t) * 50 # should hover

    desired_z = np.zeros_like(t)
    desired_z[t <= 5] = np.linspace(0, 20, len(t[t <= 5]))
    desired_z[(t > 5) & (t <= 10)] = 20
    desired_z[(t > 10) & (t <= 20)] = 30
    desired_z[(t > 20) & (t <= 30)] = 10
    desired_z[(t > 30) & (t <= 32)] = 20
    desired_z[t > 32] = 20

    desired_phi = np.sin(t/2)/10
    desired_phi = np.zeros_like(t)
    # desired_theta = np.ones_like(t) * 0.05
    desired_theta = np.zeros_like(t)
    # desired_theta = np.sin(t/2 + np.pi/2)/10
    desired_psi = np.zeros_like(t)
    desired_xdot = np.zeros_like(t)
    desired_ydot = np.zeros_like(t)
    desired_zdot = np.zeros_like(t)
    desired_p = np.zeros_like(t)
    desired_q = np.zeros_like(t)
    desired_r = np.zeros_like(t)
    
    desired_states = [desired_x, desired_y, desired_z, desired_phi, desired_theta, desired_psi, desired_xdot, desired_ydot, desired_zdot, desired_p, desired_q, desired_r]


    states, times, thruster_values = hexacopter.simulate(t, initial_state, desired_states)
    hexacopter.plot_figures(states, times, desired_states)
    # plt.plot(times[:-1], thruster_values)
    # plt.legend(['1', '2', '3', '4', '5', '6'])
    # plt.show()


    
    