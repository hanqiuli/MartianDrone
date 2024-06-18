import numpy as np
import control as ct
import matplotlib.pyplot as plt

class DroneSystem:
    """
    This class defines the nonlinear dynamics of a drone system, taken from the sketchy paper.
    """

    def __init__(self, mass, moment_inertia, moment_inertia_prop, torque_thrust_ratio, omega_thrust_ratio, *, ENV=None, arm_length=2):
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
        A[0:3, 6:9] = np.eye(3)  # position to velocity
        A[3:6, 9:12] = np.eye(3)  # orientation to angular velocity
        
        B_orig = np.zeros((12, 4))
        B_orig[6:9, 0] = np.array([0, 0, 1/self.mass])  # force input to linear acceleration
        B_orig[9:12, 1:4] = np.diag([1/Jx, 1/Jy, 1/Jz])  # torques to angular acceleration

        # This is the transformation matrix for a hexa-copter
        # transformation_matrix = np.array([
        #     [1, 1, 1, 1, 1, 1],
        #     [0, -L*np.sqrt(3)/2, -L*np.sqrt(3)/2, 0, L*np.sqrt(3)/2, L*np.sqrt(3)/2],
        #     [-L, -L/2, L/2, L, L/2, -L/2],
        #     [-k_MT, k_MT, -k_MT, k_MT, -k_MT, k_MT]
        # ])

        B = B_orig

        C = np.eye(12)

        D = np.zeros((12, 4))

        # Create the state space representation
        sys = ct.StateSpace(A, B, C, D)

        self.linear_system = sys

        return sys

    def design_pd_controller(self, omega_n=1, zeta=0.5):
        """
        This function designs a PD controller for the hexa-copter system.

        Positional arguments:
            omega_n     [rad/s]     float       natural frequency of the closed-loop system
            zeta        [-]         float       damping ratio of the closed-loop system

        Returns:
            Kp, Kd      [N/A]       float       proportional and derivative gains of the PD controller
        """
        Kp = omega_n**2
        Kd = 2*zeta*omega_n
        return Kp, Kd

    def simulate(self, t, r):
        """
        This function simulates the response of the hexa-copter system to a given reference input.

        Positional arguments:
            t       [s]     array-like       time vector
            r       [m]     array-like       reference input

        Returns:
            y       [m]     array-like       system response
        """
        # Implement the control law
        Kp, Kd = self.design_pd_controller()

        # Creating separate PD controllers for each axis
        pd_controllers = []
        for i in range(6):
            A_controller = np.array([[0, 1], [0, 0]])
            B_controller = np.array([[0], [1]])
            C_controller = np.array([[Kp, Kd]])
            D_controller = np.array([[0]])
            pd_controllers.append(ct.StateSpace(A_controller, B_controller, C_controller, D_controller))

        # Connecting PD controllers in feedback with the system
        sys_closed_loop = self.linear_system
        for pd_controller in pd_controllers:
            sys_closed_loop = ct.feedback(sys_closed_loop, pd_controller, sign=-1)

        # Simulate the system response
        t_out, y_out, x_out = ct.forced_response(sys_closed_loop, T=t, U=r)

        return t_out, y_out

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

    # Define the simulation parameters
    t = np.linspace(0, 10, 1000)
    r = np.ones((6, len(t)))  # step input for 6 inputs

    # Simulate the system response
    t_out, y_out = drone.simulate(t, r)

    # Plot the system response
    plt.figure()
    for i in range(y_out.shape[0]):
        plt.plot(t_out, y_out[i, :], label=f'State {i+1}')
    plt.xlabel('Time [s]')
    plt.ylabel('Response')
    plt.title('Hexa-copter System Response')
    plt.legend()
    plt.grid(True)
    plt.show()
