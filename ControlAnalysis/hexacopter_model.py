import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.append('.')
from legacy.DesignTools.environment_properties import ENV as ENVdict
from motor_response import MotorResponse


class HexacopterModel:
    """Hexacopter dynamics model"""
    def __init__(self, mass, moment_inertia, moment_inertia_prop, *, \
            propellor_thrust_coefficient, propellor_power_coefficient, propellor_radius, \
            thrust_to_weight_range=None, ENV=None, arm_length=2, motor_natural_frequency=20, motor_damping=1):
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

        self.environment = ENV if ENV else ENVdict

        self.setup_motor_responses(motor_natural_frequency, motor_damping)

        self.mass = mass
        self.moment_inertia = moment_inertia
        self.arm_length = arm_length

        self.moment_inertia_prop = moment_inertia_prop
        self.propellor_radius = propellor_radius
        self.propellor_thrust_coefficient = propellor_thrust_coefficient
        self.propellor_power_coefficient = propellor_power_coefficient

        # Calculate constants for the hexacopter torque
        self.b = propellor_radius ** 4 * propellor_thrust_coefficient * self.environment['rho'] * np.pi
        self.d = propellor_radius ** 5 * propellor_power_coefficient * self.environment['rho'] * np.pi
        self.torque_thrust_ratio = self.b / self.d
        
        if thrust_to_weight_range is None:
            raise ValueError('Thrust to weight range not specified, set as a list of two items [min, max]')
        self.thrust_to_weight_range = np.array(thrust_to_weight_range)
        self.thruster_range = self.thrust_to_weight_range * self.mass*self.environment['g'] / 6

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

        self.crashed = False

    # Hexacopter dynamics
    def hexacopter_dynamics(self, state, omega_rotors, inputs):

        """
        Hexacopter dynamics model

        Positional arguments:
        state       [m, m, m, rad, rad, rad, m/s, m/s, m/s, rad/s, rad/s, rad/s]  array-like  state of the hexacopter [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]
        inputs      [T]          array-like  control inputs [t1, t2, t3, t4, t5, t6]
        """

        x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r = state

        u1, u2, u3, u4, *unassigned_inputs = inputs
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

    # Thruster dynamics
    def setup_motor_responses(self, *args):
        """
        Initialize the motor response lags for the hexacopter
        """

        self.motor_response_list = [
            MotorResponse(*args),
            MotorResponse(*args),
            MotorResponse(*args),
            MotorResponse(*args),
            MotorResponse(*args),
            MotorResponse(*args)
        ]

    def clip_thrusters(self, thruster_inputs):
        """
        Clip the thruster inputs to the thrust range. Note that the thrust range is in absolute terms, not in T/W

        Positional arguments:
        thruster_inputs     [-]           array-like  thruster inputs
        """
        return np.clip(thruster_inputs, *self.thruster_range)

    # Full simulation
    def simulate_response(self, _, state, thruster_inputs, dt):
        '''Full model simulation involving both the kinematics as well as the thruster response and limits'''
        # thruster response
        thruster_inputs = self.clip_thrusters(thruster_inputs)
        for i, motor_response in enumerate(self.motor_response_list):
            thruster_inputs[i] = motor_response.get_actual_torque(thruster_inputs[i], dt)
        thruster_inputs = self.clip_thrusters(thruster_inputs)

        # input mapping
        inputs = self.thrust_map @ thruster_inputs
        omega_rotors = np.sqrt(thruster_inputs / self.b)

        # kinematic model
        return self.hexacopter_dynamics(state, omega_rotors, inputs)

