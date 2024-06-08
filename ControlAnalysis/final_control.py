import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint      # type: ignore
from scipy.optimize import lsq_linear   # type: ignore

sys.path.append('.')
from legacy.DesignTools.environment_properties import ENV as ENVdict


class ThrusterMapping:
    def __init__(self, A, weights, x_min, x_max):
        """
        Initialize the solver with the control effectiveness matrix, weight matrix, and constraints.

        :param A: Control effectiveness matrix (m x n)
        :param weights: Weight matrix (n x n) or vector of weights (n)
        :param x_min: Lower bounds for the thruster outputs (n-dimensional vector)
        :param x_max: Upper bounds for the thruster outputs (n-dimensional vector)
        """
        self.A = A
        self.W = np.diag(weights) if len(weights.shape) == 1 else weights
        self.x_min = x_min
        self.x_max = x_max

    def optimal(self, b):
        """
        Solve the constrained weighted least squares problem.

        :param b: Desired total thrust and moments (m-dimensional vector)
        :return: Optimal thruster outputs (n-dimensional vector)
        """
        # Weighted A and b
        WA = self.W @ self.A
        Wb = self.W @ b

        # Solve the constrained least squares problem
        res = lsq_linear(WA, Wb, bounds=(self.x_min, self.x_max))

        if res.success:
            return res.x
        raise ValueError("Constrained least squares solution could not be found.")


class PIDController:
    def __init__(self, Kp, Ki, Kd, wind_up_limit, *, set_point=0, clip_limit=np.Infinity, PID_type='altitude'):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.wind_up_limit = wind_up_limit
        self.clip_limit = clip_limit

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
        #     return np.clip(self.Kp * error + self.Ki * self.integral + self.Kd * derivative, -0.5, 0.5) #TODO: proper clipping here
        # elif self.type == 'attitude':
        #     return np.clip(self.Kp * error + self.Ki * self.integral + self.Kd * derivative, -0.5, 0.5) #TODO: proper clipping here
        # else:
        return np.clip(self.Kp * error + self.Ki * self.integral + self.Kd * derivative, -self.clip_limit, self.clip_limit)


class MotorResponse:
    def __init__(self, time_constant):
        """
        Initialize the FirstOrderLag class.

        :param time_constant: The time constant of the first-order lag.
        :param dt: The timestep for the simulation.
        """
        self.time_constant = time_constant
        self.output = 35.0  # Initial output value

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

    def __init__(self, mass, moment_inertia, moment_inertia_prop, pid_params, *, \
            propellor_thrust_coefficient, propellor_power_coefficient, propellor_radius, \
            thrust_to_weight_range=None, ENV=None, arm_length=2, motor_lag_term_time_constant=0.1):
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
        self.arm_length = arm_length
        self.environment = ENV if ENV else ENVdict

        self.moment_inertia_prop = moment_inertia_prop
        self.propellor_radius = propellor_radius
        self.propellor_thrust_coefficient = propellor_thrust_coefficient
        self.propellor_power_coefficient = propellor_power_coefficient

        # Calculate constants for the hexacopter
        self.b = self.propellor_radius ** 4 * self.propellor_thrust_coefficient * self.environment['rho'] * np.pi
        self.d = self.propellor_radius ** 5 * self.propellor_power_coefficient * self.environment['rho'] * np.pi
        self.torque_thrust_ratio = self.b / self.d
        
        mass_skew_factor = 0.0  # the fraction that the base thrust is off of the hover thrust
        self.estimated_mass = mass*(1+mass_skew_factor)
        self.estimated_weight = self.estimated_mass*self.environment['g']
        inertia_skew_factor = np.random.uniform(-0.02, 0.02)
        self.estimated_moment_inertia = np.max(moment_inertia*(1+inertia_skew_factor))

        self.setup_pids(pid_params)
        self.setup_motor_responses(motor_lag_term_time_constant)

        L = arm_length                      # alias for arm length to make matrix more readable
        L_2 = arm_length/2                  # alias for arm length over 2 to make matrix more readable
        Lsqrt3_2 = arm_length*np.sqrt(3)/2  # alias for arm length times sqrt(3) over 2 to make matrix more readable
        tt_ratio = self.torque_thrust_ratio # alias for torque to thrust ratio to make matrix more readable
        self.thrust_map = np.array([
            [ 1,         1,          1,         1,           1,          1          ],
            [-L_2 ,     -L,         -L_2 ,      L_2 ,        L,          L_2        ],
            [-Lsqrt3_2,  0,          Lsqrt3_2,  Lsqrt3_2,    0,         -Lsqrt3_2   ],
            [-tt_ratio,  tt_ratio,  -tt_ratio,  tt_ratio,   -tt_ratio,   tt_ratio   ]
        ])

        if thrust_to_weight_range is None:
            raise ValueError('Thrust to weight range not specified, set as a list of two items [min, max]')
        self.thrust_to_weight_range = np.array(thrust_to_weight_range)
        self.thruster_range = self.thrust_to_weight_range*self.estimated_weight/6

        control_axis_weights = [
            np.array([4, 10, 10, 3]),   # Nominal weights
            np.array([8, 10, 10, 5]),   # Altitude recovery weights
            np.array([1, 10, 10, 2])    # Attitude recovery weights
        ]

        self.inverted_thrust_map = [
            ThrusterMapping(self.thrust_map, control_axis_weights[0], *self.thruster_range),
            ThrusterMapping(self.thrust_map, control_axis_weights[1], *self.thruster_range),
            ThrusterMapping(self.thrust_map, control_axis_weights[2], *self.thruster_range)
        ]

        self.input_type_list = ['altitude', 'attitude', 'attitude', 'attitude']

        self.crashed = False

    # Hexacopter dynamics
    def hexacopter_dynamics(self, state, _, thruster_inputs):

        """
        Hexacopter dynamics model

        Positional arguments:
        state       [m, m, m, rad, rad, rad, m/s, m/s, m/s, rad/s, rad/s, rad/s]  array-like  state of the hexacopter [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]
        inputs      [T]          array-like  control inputs [t1, t2, t3, t4, t5, t6]
        """

        x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r = state

        inputs = self.thrust_map @ thruster_inputs
        u1, u2, u3, u4, *unassigned_inputs = inputs
        omega_rotors = np.sqrt(thruster_inputs / self.b)

        

        if len(unassigned_inputs) > 0:
            print(f'Warning: there are {len(unassigned_inputs)} \
            too many control inputs specified in the dynamics system call')

        J = self.moment_inertia
        Jr = self.moment_inertia_prop
        m = self.mass
        g = self.environment['g']
        omega_r = -omega_rotors[0] + omega_rotors[1] - omega_rotors[2] + omega_rotors[3] - omega_rotors[4] + omega_rotors[5]


        # Translational dynamics
        x_ddot = (u1/m) * (np.cos(phi) * np.sin(theta) * np.cos(psi) + np.sin(phi) * np.sin(psi))
        y_ddot = (u1/m) * (np.cos(phi) * np.sin(theta) * np.sin(psi) - np.sin(phi) * np.cos(psi))
        z_ddot = (u1/m) * np.cos(phi) * np.cos(theta) - g

        # Rotational dynamics
        phi_dot = p + q * np.sin(phi) * np.tan(theta) + r * np.cos(phi) * np.tan(theta)
        theta_dot = q * np.cos(phi) - r * np.sin(phi)
        psi_dot = q * np.sin(phi) / np.cos(theta) + r * np.cos(phi) / np.cos(theta)

        #TODO: Check whether we should include propeller inertia effects
        p_dot = (J[1,1] - J[2,2]) * q * r / J[0,0] + u2 / J[0,0] - Jr * omega_r * q / J[0, 0]
        q_dot = (J[2,2] - J[0,0]) * p * r / J[1,1] + u3 / J[1,1] + Jr * omega_r * p / J[1, 1]
        r_dot = (J[0,0] - J[1,1]) * p * q / J[2,2] + u4 / J[2,2]

        return [x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot, x_ddot, y_ddot, z_ddot, p_dot, q_dot, r_dot]

    def setup_pids(self, pid_params):

        """
        Initialize the PID controllers for the hexacopter

        Positional arguments:
        pid_params      [N/A]           array/LIL        List of pid parameters, sorted per pid: [gain_list, gain_list, ...]
        """

        # Initialize PID controllers for altitude and attitude
        self.pid_list_control = []

        for i, param in enumerate(pid_params):
            common_params = param[0:4]
            if len(param) > 4:
                pid_controller = PIDController(*common_params, PID_type='altitude' if i == 0 else 'attitude', clip_limit=param[4])
            else:
                pid_controller = PIDController(*common_params, PID_type='altitude' if i == 0 else 'attitude')
            self.pid_list_control.append(pid_controller)

        # x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r
        self.state_to_pid_control_map = [None, None, 0, 1, 2, 3, None, None, None, None, None, None]
        self.pid_control_to_state_map = [2, 3, 4, 5]
        
        self.pid_control_to_input_map = [0, 1, 2, 3]
        self.input_to_pid_control_map = [0, 1, 2, 3]

    def setup_motor_responses(self, time_constant):

        """
        Initialize the motor response lags for the hexacopter, at a given time constant
        """

        self.motor_response_list = [
            MotorResponse(time_constant),
            MotorResponse(time_constant),
            MotorResponse(time_constant),
            MotorResponse(time_constant),
            MotorResponse(time_constant),
            MotorResponse(time_constant)
        ]
    
    def clip_inputs(self, inputs):
        # relative to hover
        thrust = np.clip(inputs[0], *(self.thrust_to_weight_range-1))

        moment_max = 4 * (np.max(np.abs(self.thrust_to_weight_range-1)))*self.arm_length
        moment = np.clip(inputs[1:3], -moment_max, moment_max)

        torque_max = 6 * self.torque_thrust_ratio*np.mean(np.abs(self.thrust_to_weight_range-1))
        torque = np.clip(inputs[3], -torque_max, torque_max)

        return [thrust, *moment, torque]

    def clip_thrusters(self, thruster_inputs):
        """
        Clip the thruster inputs to the thrust range. Note that the thrust range is in absolute terms, not in T/W

        Positional arguments:
        thruster_inputs     [-]           array-like  thruster inputs
        """
        return np.clip(thruster_inputs, *self.thruster_range)

    def mod_inputs(self, inputs):

        for i, input_type in enumerate(self.input_type_list):
            # M = pid * mass estimated * g * L
            # T = (1+pid)*mass estimated*g
            if 'altitude' == input_type:
                inputs[i] = (inputs[i] + 1) * self.estimated_weight
            elif 'attitude' == input_type:
                inputs[i] *= self.estimated_mass * self.environment['g']/self.arm_length
            else:
                raise ValueError('Undefined input type in input type list')

        return inputs

    def get_flight_mode(self, state, set_point):

        x, y, z, phi, theta, psi, xdot, ydot, zdot, p, q, r = state
        target_x, target_y, target_z, target_phi, target_theta, target_psi, \
            target_xdot, target_ydot, target_zdot, target_p, target_q, target_r = set_point

        max_allowed_angle = 45 * np.pi/180
        # Attitude recovery
        if abs(phi) > max_allowed_angle or abs(theta) > max_allowed_angle:
            return ['attitude', 2]  # axis weight set 2
        
        # Altitude recovery
        if z < target_z - 20:
            return ['altitude', 1]
        
        # Close to target
        delta_x = target_x-x
        delta_y = target_y-y
        velocity_magnitude = np.linalg.norm([xdot, ydot])
        landing_radius = 2
        close_radius = 30
        proximity = np.linalg.norm([delta_x, delta_y])
            # Landing
        if (proximity < landing_radius) and (velocity_magnitude < 1):
            return ['land', 1]
            # Stopping
        if (proximity < close_radius):
            return ['close', 0]
        
        # Unexpectedly low/climbing
        if z < 20:
            return ['climb', 1]
            # Lower than expected

        # Nominal flight
        return ['nominal', 0]

    def get_control(self, state, set_point, dt, weight_set):

        """
        Get the control inputs for the hexacopter, applying the proper modifications

        Positional arguments:

        state       [m, m, m, rad, rad, rad, m/s, m/s, m/s, rad/s, rad/s, rad/s]  array-like  state of the hexacopter [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]
        set_point   [m, m, m, rad, rad, rad, m/s, m/s, m/s, rad/s, rad/s, rad/s]  array-like  setpoints for the hexacopter [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]
        dt          [s]            float       time step
        thrust_range [-]           array-like  thrust range for the thrusters (with respect to T/W)
        """

        inputs = np.zeros_like(self.pid_control_to_input_map)

        for i, pid in enumerate(self.pid_list_control):
            state_id = self.pid_control_to_state_map[i]
            pid.set_point = set_point[state_id]

            input_id = self.pid_control_to_input_map[i]
            inputs[input_id] = pid.update(state[state_id], dt)


        inputs = self.clip_inputs(inputs)
        inputs = self.mod_inputs(inputs)
        thruster_inputs = self.inverted_thrust_map[weight_set].optimal(np.array(inputs))
        thruster_inputs = self.clip_thrusters(thruster_inputs)


        for i, motor_response in enumerate(self.motor_response_list):
            thruster_inputs[i] = motor_response.get_actual_torque(thruster_inputs[i], dt)

        return thruster_inputs

    def get_flight(self, state, set_point, dt):
        flight_mode, weight_set = self.get_flight_mode(state, set_point)

        thruster_inputs = self.get_control(state, set_point, dt, weight_set)

        return thruster_inputs, flight_mode

    def simulate(self, times, initial_state, setpoints):

        """
        Simulate the hexacopter dynamics
        times:  array-like - time points for the simulation
        initial_state: array-like - initial state of the hexacopter
        setpoints: array-like - setpoints for the simulation, same length as max_time/time_step
        """

        self.crashed = False
        time_step = times[1] - times[0]
        states = [initial_state]
        thruster_values = []
        flight_mode_list = []

        for i in range(1, len(times)):
            current_state = states[-1]

            # get inputs from flight plan and axis controller
            thruster_inputs, flight_mode = self.get_flight(current_state, \
                [setpoint[i] for setpoint in setpoints] ,time_step)
            
            # simulate to next timestep
            inputs = tuple([thruster_inputs],)
            new_state = odeint(self.hexacopter_dynamics, current_state, [0, time_step], args=inputs)[-1]

            # save values for analysis
            states.append(new_state)
            thruster_values.append(thruster_inputs)
            flight_mode_list.append(flight_mode)

            # Check if the drone has crashed
            if new_state[2] < 0:
                self.crashed = True
                break
           
        times = times[:len(states)]
        
        return np.array(states), times, thruster_values, flight_mode_list
    
    @staticmethod
    def plot_figures(states, times, setpoints, thruster_values, flight_mode_list):

        """
            Plot the simulation results
        """

        desired_x, desired_y, desired_z, desired_phi, desired_theta, desired_psi, \
        desired_xdot, desired_ydot, desired_zdot, desired_p, desired_q, desired_r = setpoints
        # clipping the setpoints to the length of the simulation
        desired_x = desired_x[:len(times)]
        desired_y = desired_y[:len(times)]
        desired_z = desired_z[:len(times)]
        desired_phi = desired_phi[:len(times)]
        desired_theta = desired_theta[:len(times)]
        desired_psi = desired_psi[:len(times)]
        desired_xdot = desired_xdot[:len(times)]
        desired_ydot = desired_ydot[:len(times)]
        desired_zdot = desired_zdot[:len(times)]
        desired_p = desired_p[:len(times)]
        desired_q = desired_q[:len(times)]
        desired_r = desired_r[:len(times)]

        # Position plot
        plt.figure()
        plt.subplot(2, 1, 1)
        plt.plot(times, states[:, 0], label='x')
        plt.plot(times, desired_x, 'r--', label='desired x')
        plt.plot(times, states[:, 1], label='y')
        plt.plot(times, desired_y, 'g--', label='desired y')
        plt.xlabel('Time [s]')
        plt.ylabel('Position [m]')
        plt.ylim([-100, 100])
        plt.legend()

        plt.subplot(2, 1, 2)
        plt.plot(times, states[:, 2], label='z')
        plt.plot(times, desired_z, 'r--', label='desired z')
        plt.xlabel('Time [s]')
        plt.ylabel('Altitude [m]')
        plt.ylim([-20, 100])
        plt.legend()

        plt.tight_layout()
        plt.savefig('ControlAnalysis/figures/position.png', dpi=300)

        # Attitude plot
        plt.figure()
        plt.subplot(3, 1, 1)
        plt.plot(times, states[:, 3], label='phi')
        plt.plot(times, desired_phi, 'r--', label='desired phi')
        plt.xlabel('Time [s]')
        plt.ylabel('Roll [rad]')
        plt.ylim([-3, 3])
        plt.legend()

        plt.subplot(3, 1, 2)
        plt.plot(times, states[:, 4], label='theta')
        plt.plot(times, desired_theta, 'r--', label='desired theta')
        plt.xlabel('Time [s]')
        plt.ylabel('Pitch [rad]')
        plt.ylim([-3, 3])
        plt.legend()

        plt.subplot(3, 1, 3)
        plt.plot(times, states[:, 5], label='psi')
        plt.plot(times, desired_psi, 'r--', label='desired psi')
        plt.xlabel('Time [s]')
        plt.ylabel('Yaw [rad]')
        plt.ylim([-3, 3])
        plt.legend()

        plt.tight_layout()
        plt.savefig('ControlAnalysis/figures/angles.png', dpi=300)

        # Velocity plot
        plt.figure()
        plt.subplot(2, 1, 1)
        plt.plot(times, states[:, 6], label='x_dot')
        plt.plot(times, desired_xdot, 'r--', label='desired x_dot')
        plt.plot(times, states[:, 7], label='y_dot')
        plt.plot(times, desired_ydot, 'g--', label='desired y_dot')
        plt.xlabel('Time [s]')
        plt.ylabel('Horizontal velocity [m/s]')
        plt.ylim([-10, 10])
        plt.legend()

        plt.subplot(2, 1, 2)
        plt.plot(times, states[:, 8], label='z_dot')
        plt.plot(times, desired_zdot, 'r--', label='desired z_dot')
        plt.xlabel('Time [s]')
        plt.ylabel('Altitude velocity [m/s]')
        plt.ylim([-10, 10])

        plt.tight_layout()
        plt.savefig('ControlAnalysis/figures/velocity.png', dpi=300)

        # Attitude rate plot
        plt.figure()
        plt.subplot(3, 1, 1)
        plt.plot(times, states[:, 9], label='p')
        plt.plot(times, desired_p, 'r--', label='desired p')
        plt.xlabel('Time [s]')
        plt.ylabel('Roll rate [rad/s]')
        plt.ylim([-3, 3])
        plt.legend()

        plt.subplot(3, 1, 2)
        plt.plot(times, states[:, 10], label='q')
        plt.plot(times, desired_q, 'r--', label='desired q')
        plt.xlabel('Time [s]')
        plt.ylabel('Pitch rate [rad/s]')
        plt.ylim([-3, 3])
        plt.legend()

        plt.subplot(3, 1, 3)
        plt.plot(times, states[:, 11], label='r')
        plt.plot(times, desired_r, 'r--', label='desired r')
        plt.xlabel('Time [s]')
        plt.ylabel('Yaw rate [rad/s]')
        plt.ylim([-3, 3])
        plt.legend()


        plt.tight_layout()
        plt.savefig('ControlAnalysis/figures/angular_velocity.png', dpi=300)

        # Thruster plot
        plt.figure(figsize=(20, 4.8))
        plt.plot(times[:-1], thruster_values)
        plt.legend(['1', '2', '3', '4', '5', '6'])
        plt.xlabel('Time [s]')
        plt.ylabel('Thruster values [N]')
        plt.tight_layout()
        plt.savefig('ControlAnalysis/figures/thrusters.png', dpi=1200)

        # Flight mode plot
        flight_modes = sorted(list(set(flight_mode_list)))
        mode_to_index = {status: index for index, status in enumerate(flight_modes)}
        flight_mode_list_indexed = [mode_to_index[mode] for mode in flight_mode_list]

        mode_matrix = np.zeros((len(flight_modes), len(flight_mode_list)), dtype=int)
        mode_matrix[flight_mode_list_indexed, np.arange(len(flight_mode_list))] = 1

        plt.figure(figsize=(20, 4.8))
        for row_index, mode in enumerate(flight_modes):
            plt.plot(times[:-1], mode_matrix[row_index], linestyle='--', label=mode)
            plt.fill_between(times[:-1], mode_matrix[row_index], where=mode_matrix[row_index] == 1, 
                    interpolate=True, alpha=0.3, hatch='//')
        plt.legend()
        plt.xlabel('Time [s]')
        plt.ylabel('Active [True/False]')
        plt.tight_layout()
        plt.savefig('ControlAnalysis/figures/flight_mode.png', dpi=1200)


        plt.close('all')


if __name__ == "__main__":

    mass = 60.0
    moment_inertia = np.diag([5, 5, 8])
    moment_inertia_prop = 0.01
    pid_params = [[20, 2, 20, 3, 10], [6, 0.3, 4, 10], [6, 0.3, 4, 10], [8, 0.4, 16, 10]]
    torque_thrust_ratio = 0.1
    omega_thrust_ratio = 0.1
    ENV = ENVdict

    thrust_to_weight_range = [0.6, 1.3]

    hexacopter = HexacopterModel(mass, moment_inertia, moment_inertia_prop, pid_params, \
        propellor_power_coefficient=0.5, propellor_radius=1.3, propellor_thrust_coefficient=1.2, thrust_to_weight_range=thrust_to_weight_range, ENV=ENV)

    delta_t = 0.02
    t = np.arange(0, 350, delta_t)

    initial_state = np.zeros(12)
    initial_state[2] = 5

    desired_x = np.ones_like(t)*150
    desired_y = np.ones_like(t)*100

    desired_z = np.zeros_like(t)
    desired_z[t <= 5] = np.linspace(0, 20, len(t[t <= 5]))
    desired_z[(t > 5) & (t <= 10)] = 20
    desired_z[(t > 10) & (t <= 30)] = 30
    desired_z[(t > 30) & (t <= 42)] = 20
    desired_z[(t > 42) & (t <= 60)] = 60
    desired_z[(t > 60) & (t <= 250)] = np.sin(t[(t > 60) & (t <= 250)]*np.pi/10)*5 + 50
    desired_z[(t > 250)] = 80

    desired_phi = np.zeros_like(t)
    desired_phi[(t > 100)] = np.sin(t[t > 100]*np.pi/11)*7*np.pi/180

    desired_theta = np.zeros_like(t)
    desired_theta[(t > 150)] = np.sin(t[t > 150]*np.pi/15+2)*9*np.pi/180

    desired_psi = np.zeros_like(t)
    desired_psi[(t > 180) & (t <= 220)] = np.sin(t[(t > 180) & (t <= 220)]*np.pi/12+2)*9*np.pi/180
    desired_psi[(t > 220)] = np.linspace(0, 2*np.pi, len(t[t > 220]))

    desired_xdot = np.zeros_like(t)
    desired_ydot = np.zeros_like(t)
    desired_zdot = np.zeros_like(t)
    desired_p = np.zeros_like(t)
    desired_q = np.zeros_like(t)
    desired_r = np.zeros_like(t)

    desired_states = [desired_x, desired_y, desired_z, desired_phi, desired_theta, desired_psi, \
         desired_xdot, desired_ydot, desired_zdot, desired_p, desired_q, desired_r]


    states, times, thruster_values, flight_mode_list = hexacopter.simulate(t, initial_state, desired_states)
    hexacopter.plot_figures(states, times, desired_states, thruster_values, flight_mode_list)
