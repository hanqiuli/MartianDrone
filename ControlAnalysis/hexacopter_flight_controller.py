import sys
from turtle import setposition

import numpy as np
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
        res = lsq_linear(WA, Wb, bounds=(self.x_min, self.x_max), method='bvls')

        if res.success:
            return res.x
        print(res)
        raise ValueError("Constrained least squares solution could not be found.")
    

class PIDController:
    def __init__(self, Kp, Ki, Kd, wind_up_limit, clip_limit=np.Infinity, *, set_point=0, PID_type='altitude'):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.wind_up_limit = wind_up_limit
        self.clip_limit = clip_limit

        self.set_point = set_point

        self.integral = 0.0
        self.previous_error = 0.0
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

        return np.clip(self.Kp * error \
                       + self.Ki * self.integral \
                       + self.Kd * derivative, \
                        -self.clip_limit, self.clip_limit)


class FlightController:
    def __init__(self, estimated_mass, pid_params, *, arm_length=2, ENV=None, thrust_to_weight_range=None, \
                propellor_thrust_coefficient=None, propellor_power_coefficient=None, propellor_radius=None):
        '''
        
        pid_params          [N/A]           list        List of pid parameters, sorted per pid: [gain_list, ]
        '''
        if propellor_thrust_coefficient is None:
            raise ValueError('Propellor thrust coefficient not specified, set as a float')
        if propellor_power_coefficient is None:
            raise ValueError('Propellor power coefficient not specified, set as a float')
        if propellor_radius is None:
            raise ValueError('Propellor radius not specified, set as a float')
        if thrust_to_weight_range is None:
            raise ValueError('Thrust to weight range not specified, set as a list of two items [min, max]')
        
        self.environment = ENV if ENV else ENVdict

        self.setup_pids(pid_params)
        self.last_control_loop_index = -1
        
        self.mass = estimated_mass
        self.weight = self.mass*self.environment['g']
        self.arm_length = arm_length

        ## Calculate constants for the hexacopter torque
        self.b = propellor_radius ** 4 * propellor_thrust_coefficient * self.environment['rho'] * np.pi
        self.d = propellor_radius ** 5 * propellor_power_coefficient * self.environment['rho'] * np.pi
        self.torque_thrust_ratio = self.b / self.d

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

        
        self.thrust_to_weight_range = np.array(thrust_to_weight_range)
        self.thruster_range = self.thrust_to_weight_range*self.weight/6

        control_axis_weights = [
            np.array([10, 5, 5, 5]),   # Nominal weights
            np.array([13, 5, 5, 5]),   # Altitude recovery weights
            np.array([2, 8, 8, 8])    # Attitude recovery weights
        ]

        self.inverted_thrust_map = [
            ThrusterMapping(self.thrust_map, control_axis_weights[0], *self.thruster_range),
            ThrusterMapping(self.thrust_map, control_axis_weights[1], *self.thruster_range),
            ThrusterMapping(self.thrust_map, control_axis_weights[2], *self.thruster_range)
        ]

        self.input_type_list = ['altitude', 'attitude', 'attitude', 'attitude']

    def setup_pids(self, pid_params):

        """
        Initialize the PID controllers for the hexacopter

        Positional arguments:
        pid_params      [N/A]           array/LIL        List of pid parameters, sorted per pid: [gain_list, gain_list, ...]
        """

         # Initialize PID controllers for altitude and attitude
        self.pid_list = [
            PIDController(*pid_params[0], PID_type = 'attitude'),   # roll
            PIDController(*pid_params[1], PID_type = 'attitude'),   # pitch
            PIDController(*pid_params[2], PID_type = 'attitude'),   # yaw
            PIDController(*pid_params[3], PID_type = 'position'),   # body x
            PIDController(*pid_params[4], PID_type = 'position'),   # body y
            PIDController(*pid_params[5], PID_type = 'position'),   # z
            PIDController(*pid_params[6], PID_type = 'velocity'),   # Body velocity x
            PIDController(*pid_params[7], PID_type = 'velocity'),   # Body velocity y
            PIDController(*pid_params[8], PID_type = 'velocity'),   # Velocity z
            ]

        # x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r
        self.state_to_pid_map = [3, 4, 5, 0, 1, 2, 6, 7, 8, None, None, None]
        self.pid_to_state_map = [3, 4, 5, 0, 1, 2, 6, 7, 8]

        # Control loops:
        # 0: Direct angle and height control
        # 1: Velocity and height control
        # 2: Position and height control
        # 3: Position and vertical velocity control
        # Structured as:
        # [thrust loop, moment_roll loop, moment_pitch loop, moment_yaw loop]
        control_loop_0 = [
            [5],
            [0],
            [1],
            [2]
        ]
        control_loop_1 = [
            [5],
            [7, 0],
            [6, 1],
            [2]
        ]
        control_loop_2 = [
            [5],
            [4, 7, 0],
            [3, 6, 1],
            [2]
        ]
        control_loop_3 = [
            [8],
            [4, 7, 0],
            [3, 6, 1],
            [2]
        ]
        self.control_loop = [control_loop_0, control_loop_1, control_loop_2, control_loop_3]
    
    def transform_state_rotation(self, state, psi):
        '''Transforms the state from the intertial frame to the body frame'''

        # 2D rotation matrix using phi as angle
        rotation_matrix = np.array([[np.cos(psi), np.sin(psi)], 
                                   [-np.sin(psi),  np.cos(psi)]])
        
        # modify x and y
        if np.any(np.isnan(state[0:2])):
            pass
        else:
            state[0:2] = np.dot(rotation_matrix, state[0:2])

        # modify x_dot and y_dot
        if np.any(np.isnan(state[6:8])):
            pass
        else:
            state[6:8]  = np.dot(rotation_matrix, state[6:8])

        return state

    def clip_inputs(self, inputs):
        '''Clips the non-dimensional thrust differential and non-dimensional moments'''
        thrust = np.clip(inputs[0], *(self.thrust_to_weight_range-1))

        moment_max = (np.max(np.abs(self.thrust_to_weight_range-1)))*self.arm_length / 4
        moment = np.clip(inputs[1:3], -moment_max, moment_max)

        torque_max = 6 * np.mean(np.abs(self.thrust_to_weight_range-1)) / self.torque_thrust_ratio
        torque = np.clip(inputs[3], -torque_max, torque_max)

        return [thrust, *moment, torque]
    
    def scale_inputs(self, inputs):
        '''Scales the inputs to absolute values and adds level hover thrust'''
        for i, input_type in enumerate(self.input_type_list):
            # M = pid * mass estimated * g * L
            # T = (1+pid)*mass estimated*g
            if 'altitude' == input_type:
                inputs[i] = (inputs[i] + 1) * self.weight
            elif 'attitude' == input_type:
                inputs[i] *= self.weight * self.arm_length
            else:
                raise ValueError('Undefined input type in input type list')
            
        return inputs
    
    def get_control(self, state, set_point, weight_set, control_loop_index, dt):
        """
        flight_mode, weight_set = self.get_flight_mode(state, target)
        control_loop, set_point  = self.get_flight_configuration(flight_mode, state, target)

        thruster_inputs  = self.get_control(state, set_point, dt, weight_set)
        """
        inputs = np.zeros(4)

        transformed_state = self.transform_state_rotation(np.array(state, dtype=float), psi=state[5])
        transformed_setpoint = self.transform_state_rotation(np.array(set_point, dtype=float), psi=state[5])

        # Reset PIDs on switch to control loop
        if control_loop_index != self.last_control_loop_index:

            for i, pid in enumerate(self.pid_list):
                if not any(i in loop for loop in self.control_loop[self.last_control_loop_index]):
                    pid.integral = 0.0
                    pid.previous_error = 0.0 #TODO: Check for shock on control loop switching
            
            self.last_control_loop_index  = control_loop_index
        # Get target input by going through every control loop
        for i, loop in enumerate(self.control_loop[control_loop_index]):
            next_target = transformed_setpoint[self.pid_to_state_map[loop[0]]]

            # Update PIDs and get axis inputs, store intermediate values as setpoints
            for pid_index in loop:
                # Keep track of intermediate PID targets
                state_index = self.pid_to_state_map[pid_index]
                if np.isnan(transformed_setpoint[state_index]):
                    set_point[state_index] = next_target
                
                # Get PID and update to next target
                pid = self.pid_list[pid_index]
                
                pid.set_point = next_target
                next_target = pid.update(transformed_state[self.pid_to_state_map[pid_index]], dt)

            inputs[i] = next_target

        # Convert axis inputs to thruster inputs
        inputs = self.clip_inputs(inputs)
        inputs = self.scale_inputs(inputs)
        thruster_inputs = self.inverted_thrust_map[weight_set].optimal(np.array(inputs))
        

        
        return thruster_inputs, set_point, inputs

        

    


