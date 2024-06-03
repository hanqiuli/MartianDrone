import sys

import control as ctrl
import matplotlib.pyplot as plt
import numpy as np

sys.path.append('.')
from legacy.DesignTools.environment_properties import ENV as ENVdict


class PIDController(ctrl.NonlinearIOSystem):
    """
    This class defines a PID controller system using the control library.
    """
    def __init__(self, Kp, Ki, Kd, thruster_weights):
        super().__init__(updfcn=self.update, outfcn=self.output, inputs=2, outputs=6, states=3, name='pid')
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.thruster_weights = thruster_weights

    def update(self, t, x, u, params):
        e = u[0] - u[1]  # Error signal
        dxdt = [e, x[0] + e, e - x[1]]  # Integrate error and differentiate error
        return dxdt

    def output(self, t, x, u, params):
        e = u[0] - u[1]  # Error signal
        u_pid = self.Kp * e + self.Ki * x[0] + self.Kd * (e - x[1])
        return u_pid * self.thruster_weights
    

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
        self.torque_thrust_ratio = k_MT = torque_thrust_ratio
        self.omega_thrust_ratio = omega_thrust_ratio
        self.environment = ENV
        self.arm_length = L = arm_length

        self.transformation_matrix = np.array([
            [1,     1,                   1,                 1,      1,              1],
            [0,     -L*np.sqrt(3)/2,    -L*np.sqrt(3)/2,    0,      L*np.sqrt(3)/2, L*np.sqrt(3)/2],
            [L,     0.5*L,              -0.5*L,             -L,     -0.5*L,         0.5*L],
            [-k_MT, k_MT,               -k_MT,              k_MT,   -k_MT,          k_MT]
        ])
        
    def nonlinear_system_update(self, t, x, u, params=None):
        '''Drone dynamic system update function'''
        # Extract state variables
        v_xr, v_yr, v_zr = x[0], x[1], x[2]
        omega_xr, omega_yr, omega_zr = x[3], x[4], x[5]
        Theta, Phi, Psi = x[6], x[7], x[8]

        # Calculate relevant parameters from the input signal
        # Note, the input signal is a 6x1 vector [T1, T2, T3, T4, T5, T6]

        # Constants (These need to be defined based on your system)
        g = self.environment['g']
        J_x, J_y, J_z = self.moment_inertia
        k_omegaT = self.omega_thrust_ratio
        k_MT = self.torque_thrust_ratio
        J_mp = self.moment_inertia_prop
        m = self.mass
        L = self.arm_length

        T, M_x, M_y, M_z = np.dot(self.transformation_matrix, u)

        # Trigonometric functions for readability
        s_Theta, c_Theta = np.sin(Theta), np.cos(Theta)
        s_Phi, c_Phi = np.sin(Phi), np.cos(Phi)
        s_Psi, c_Psi = np.sin(Psi), np.cos(Psi)
        t_Theta = np.tan(Theta)

        # State equations
        dv_xr = -v_xr * omega_yr + v_yr * omega_zr - g * s_Theta
        dv_yr = -v_xr * omega_zr + v_zr * omega_xr + g * c_Theta * s_Phi
        dv_zr = -v_yr * omega_xr + v_xr * omega_yr + g * c_Theta * c_Phi - T / m
        domega_xr = (1 / J_x) * (-omega_yr * omega_zr * (J_z - J_y) + M_x + (k_omegaT / k_MT) * J_mp * M_z * omega_yr)
        domega_yr = (1 / J_y) * (-omega_xr * omega_zr * (J_x - J_z) + M_y + (k_omegaT / k_MT) * J_mp * M_z * omega_xr)
        domega_zr = M_z / J_z
        dTheta = omega_yr * c_Phi - omega_zr * s_Phi
        dPhi = omega_xr + omega_yr * s_Phi * t_Theta + omega_zr * c_Phi * t_Theta
        dPsi = omega_yr * s_Phi / c_Theta + omega_zr * c_Phi / c_Theta

        # Position equations
        dX = c_Psi * c_Theta * v_xr + (-s_Psi * c_Phi + c_Psi * s_Theta * s_Phi) * v_yr + (s_Psi * s_Phi + c_Psi * s_Theta * c_Phi) * v_zr
        dY = s_Psi * c_Theta * v_xr + (c_Psi * c_Phi + s_Psi * s_Theta * s_Phi) * v_yr + (-c_Psi * s_Phi + s_Psi * s_Theta * c_Phi) * v_zr
        dZ = -s_Theta * v_xr + c_Theta * s_Phi * v_yr + c_Theta * c_Phi * v_zr

        return [dv_xr, dv_yr, dv_zr, domega_xr, domega_yr, domega_zr, dTheta, dPhi, dPsi, dX, dY, dZ]

    # Define the nonlinear system output function
    def nonlinear_system_output(self, t, x, u, params):
        # Outputs is the full state and input
        y = np.append(x, u)
        return y

    def create_nonlinear_system(self):
        # Define the nonlinear system
        nonlinear_system = ctrl.NonlinearIOSystem(
            updfcn=self.nonlinear_system_update,  # Update function
            outfcn=self.nonlinear_system_output,  # Output function
            inputs=6, outputs=18, states=12, name='nonlinear_system'
        )
        return nonlinear_system
    
    def simulate_nonlinear_system(self, time, input_signal, initial_state):
        # Create the nonlinear system
        nonlinear_system = self.create_nonlinear_system()
        # Simulate the response of the nonlinear system
        time, response = ctrl.input_output_response(nonlinear_system, time, input_signal, X0=initial_state)
        return time, response
    
    def add_pid_controller(self, Kp, Ki, Kd, setpoint_index=2):
        """
        Add a PID controller to regulate the setpoint for a specific output variable.

        Parameters:
            Kp, Ki, Kd : PID controller gains
            setpoint_index : The index of the output variable to regulate
        """
        # Create PID controller for each individual euler angle
        pid_controller_theta = PIDController(Kp, Ki, Kd, thruster_weights=self.transformation_matrix[1], name=)
        pid_controller_phi = PIDController(Kp, Ki, Kd, thruster_weights=self.transformation_matrix[2])
        pid_controller_psi = PIDController(Kp, Ki, Kd, thruster_weights=self.transformation_matrix[3])

        sum_block = ctrl.summing_junction(inputs=['sysB.y[0]', 'sysC.y[0]'], outputs=['sum0'], name='sum')
        
        # Create the closed-loop system by connecting the PID controller to the nonlinear drone system
        closed_loop = ctrl.interconnect(
            [self.create_nonlinear_system(), pid_controller],
            connections=[
                ['nonlinear_system.u[0]', 'pid.y[0]'],  # Control input to the drone system
                [f'pid.u[1]', f'nonlinear_system.y[{setpoint_index}]']  # Feedback from the system (measured variable)
            ],
            inplist=['pid.u[0]'],  # Ensure setpoint input is listed
            outlist=[f'nonlinear_system.u[0]']  # System output to monitor
        )
        return closed_loop
    
    def plot_all_responses(self, time, response):
        # plot the response of the system
        fig, ax = plt.subplots(3, 4, figsize=(20, 10))
        ax[0, 0].plot(time, response[0])
        ax[0, 0].set_title('v_xr')
        ax[0, 1].plot(time, response[1])
        ax[0, 1].set_title('v_yr')
        ax[0, 2].plot(time, response[2])
        ax[0, 2].set_title('v_zr')
        ax[0, 3].plot(time, response[3])
        ax[0, 3].set_title('omega_xr')
        ax[1, 0].plot(time, response[4])
        ax[1, 0].set_title('omega_yr')
        ax[1, 1].plot(time, response[5])
        ax[1, 1].set_title('omega_zr')
        ax[1, 2].plot(time, response[6])
        ax[1, 2].set_title('Theta')
        ax[1, 3].plot(time, response[7])
        ax[1, 3].set_title('Phi')
        ax[2, 0].plot(time, response[8])
        ax[2, 0].set_title('Psi')
        ax[2, 1].plot(time, response[9])
        ax[2, 1].set_title('X')
        ax[2, 2].plot(time, response[10])
        ax[2, 2].set_title('Y')
        ax[2, 3].plot(time, response[11])
        ax[2, 3].set_title('Z')
        plt.show()

    

if __name__ == "__main__":
    # Define the drone parameters
    mass = 20
    moment_inertia = [5, 5, 10]
    moment_inertia_prop = 0.01
    torque_thrust_ratio = 0.1
    omega_thrust_ratio = 0.1

    # Create the drone system
    drone_system = DroneSystem(mass, moment_inertia, moment_inertia_prop, torque_thrust_ratio, omega_thrust_ratio)
    
    # Define the PID controller parameters
    Kp = 0.1
    Ki = 0.01
    Kd = 0.001

    # Control in theta
    setpoint_index = 6  

    # Define the time vector for the simulation
    time = np.linspace(0, 5, 1000)

    # Define the setpoint signal (desired altitude)
    setpoint = np.ones_like(time) * 5 * (np.pi / 180)  # Step input for the desired altitude

    # Create the closed-loop system
    closed_loop_system = drone_system.add_pid_controller(Kp, Ki, Kd, setpoint_index)

    # Initial state of the system
    initial_state = np.zeros(12)

    # Create the input signal with the correct shape
    input_signals = np.ones((1, len(time)))
    input_signals[0, :] = setpoint  # Setpoint signal

    # Simulate the response of the closed-loop system
    rtol = 1e-6
    atol = 1e-8
    time, response = ctrl.input_output_response(closed_loop_system, time, input_signals, X0=initial_state, rtol=rtol, atol=atol)
    
    # Plot the response (specifically for the altitude)
    plt.plot(time, response[0], label='Altitude (Z)')
    plt.xlabel('Time (s)')
    plt.ylabel('Altitude (m)')
    plt.title('Altitude Response with PID Control')
    plt.grid(True)
    plt.legend()
    plt.show()
    
    # # Define the time vector for the simulation
    # time = np.linspace(0, 1, 1000)
    # # Define the input signal

    # input_signal = np.zeros((6, len(time)))
    # input_signal[0, :] = 50
    # input_signal[1, :] = 50
    # input_signal[2, :] = 50
    # input_signal[3, :] = 50
    # input_signal[4, :] = 50
    # input_signal[5, :] = 50
    
    # # Initial state of the system
    # initial_state = np.zeros(12)

    # # Simulate the response of the nonlinear system
    # time, response = drone_system.simulate_nonlinear_system(time, input_signal, initial_state)
    
    # # Plot the response
    # drone_system.plot_all_responses(time, response)



