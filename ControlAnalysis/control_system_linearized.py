import sys

import control as ct
import matplotlib.pyplot as plt
import numpy as np

sys.path.append('.')
from legacy.DesignTools.environment_properties import ENV as ENVdict

class DroneSystem:
    """
    This class defines the nonlinear dynamics of a drone system, taken from the sketchy paper.
    """

    def __init__(self, mass, moment_inertia, moment_inertia_prop, torque_thrust_ratio, omega_thrust_ratio, *, ENV=ENVdict, arm_length=2):
        """
        Class initializer

        Positional arguments:
            mass                [kg]            float       mass of the drone in kg
            moment_inertia      [kg m^2]        array-like  lists of moment of inertia of the drone in, [J_x, J_y, J_z]
            moment_inertia_prop [kg m^2]        float       moment of inertia of the propellers + motors
            torque_thrust_ratio [m^-1]          float       ratio of reactive torque to thrust of propellers
            omega_thrust_ratio  [rad (Ns)^-1]   float       ratio of angular velocity to thrust of propellers

        Keyword arguments:
            ENV                 [N/A]           dict        dictionary containing the environmental properties
            arm_length          [m]             float       length of the arm of the drone
        """

        self.mass = mass
        self.moment_inertia = moment_inertia
        self.moment_inertia_prop = moment_inertia_prop
        self.torque_thrust_ratio = torque_thrust_ratio
        self.omega_thrust_ratio = omega_thrust_ratio
        self.environment = ENV
        self.arm_length = arm_length

    def create_state_space(self):
        """
        This function creates the state space representation of the drone system.

        Returns:
            sys         [StateSpace]    state space representation of the drone system
        """ 

        Jx, Jy, Jz = self.moment_inertia
        k_MT = self.torque_thrust_ratio
        k_omega = self.omega_thrust_ratio
        L = self.arm_length

        # Define the state space matrices
        A = np.zeros((12, 12))
        A[9:12, 0:3] = np.eye(3)
        A[6:9, 3:6] = np.array([[0, 1, 0], [1, 0, 0], [0, 0, 1]])
        
        # This B is using inputs: T, Mx, My, Mz
        B_orig = np.zeros((12, 4))
        B_orig[0, 3] = -1/self.mass
        B_orig[3:6, 1:4] = np.array([[1/Jx, 0, 0],[0, 1/Jy, 0],[0, 0, 1/Jz]])

        # We need to convert the inputs to the propeller speeds
        # This is the conversion matrix

        transformation_matrix = np.array([
            [1,     1,                   1,                 1,      1,              1],
            [0,     -L*np.sqrt(3)/2,    -L*np.sqrt(3)/2,    0,      L*np.sqrt(3)/2, L*np.sqrt(3)/2],
            [L,     0.5*L,              -0.5*L,             -L,     -0.5*L,         0.5*L],
            [-k_MT, k_MT,               -k_MT,              k_MT,   -k_MT,          k_MT]
        ])

        B = np.dot(B_orig, transformation_matrix)

        C = np.eye(12)

        D = np.zeros((12, 6))

        # Create the state space representation
        sys = ct.StateSpace(A, B, C, D)

        self.linear_system = sys

        return sys
    

if __name__ == "__main__":
    # Define the drone system parameters
    mass = 1.5
    moment_inertia = [0.1, 0.1, 0.2]
    moment_inertia_prop = 0.05
    torque_thrust_ratio = 0.1
    omega_thrust_ratio = 0.1

    # Create an instance of the DroneSystem class
    drone = DroneSystem(mass, moment_inertia, moment_inertia_prop, torque_thrust_ratio, omega_thrust_ratio)

    # Create the state space representation of the drone system
    sys = drone.create_state_space()

    # Print the state space representation
    print(sys)

    # Simulate the system

    # Define the time vector
    t = np.linspace(0, 10, 1000)

    # Define the input vector
    u = np.ones((6, t.size)) * 10

    # Define the initial state vector
    x0 = np.zeros(12)

    # Simulate the system
    t, y = ct.forced_response(sys, t, u, x0)

    # Plot the results
    plt.plot(t, y[9], label='x')
    plt.plot(t, y[10], label='y')
    plt.plot(t, y[11], label='z')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.legend()
    plt.grid()
    plt.show()