'''Nonlinear variant of the control system'''
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
    def __init__(self, Kp, Ki, Kd, Kii, max_output, min_output=0, name_input='pid', number_of_inputs=2, number_of_outputs=4):
        super().__init__(updfcn=self.update, outfcn=self.output, inputs=number_of_inputs, outputs=number_of_outputs, states=3, name=name_input)
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Kii = Kii
        self.min_output = min_output
        self.max_output = max_output

    def update(self, t, x, u, params):
        e = u[0] - u[1]  # Error signal
        dxdt = [e, x[0] + e, x[0]]  # Integrate error and differentiate error
        # Integral term, Position plus error, error minus derivative
        return dxdt

    def output(self, t, x, u, params):
        e = u[0] - u[1]  # Error signal
        u_pid = self.Kp * e + self.Ki * x[0] + self.Kd * (e - x[1]) + self.Kii * x[2]
        return np.clip(u_pid, self.min_output, self.max_output)

class DroneSystem:
    """
    This class defines the nonlinear dynamics of a drone system, taken from the sketchy paper.
    """

    def __init__(self, mass, moment_inertia, moment_inertia_prop, torque_thrust_ratio, omega_thrust_ratio, *, ENV=ENVdict, arm_length=2, trolling=True):
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
        self.trolling = trolling
        
    def nonlinear_system_update(self, t, x, u, params=None):
        '''Drone dynamic system update function'''
        # Extract state variables
        v_xr, v_yr, v_zr = x[0], x[1], x[2]
        omega_xr, omega_yr, omega_zr = x[3], x[4], x[5]
        Theta, Phi, Psi = x[6], x[7], x[8]

        # Calculate relevant parameters from the input signal
        # Note, the input signal is a 4x1 vector [T, M_x, M_y, M_z]

        # Constants (These need to be defined based on your system)
        g = self.environment['g']
        J_x, J_y, J_z = self.moment_inertia
        k_omegaT = self.omega_thrust_ratio
        k_MT = self.torque_thrust_ratio
        J_mp = self.moment_inertia_prop
        m = self.mass

        T, M_x, M_y, M_z = u

        # T += self.mass*g

        # Trigonometric functions for readability
        if not self.trolling:
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


        else:
            s_Theta, c_Theta = np.sin(Theta), np.cos(Theta)
            s_Phi, c_Phi = np.sin(Phi), np.cos(Phi)
            s_Psi, c_Psi = np.sin(Psi), np.cos(Psi)
            t_Theta = np.tan(Theta)
            dv_xr = - g * s_Theta
            dv_yr = + g * c_Theta * s_Phi
            dv_zr = + g * c_Theta * c_Phi - T / m
            domega_xr = (1 / J_x) * ( M_x )
            domega_yr = (1 / J_y) * ( M_y )
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
            inputs=4, outputs=16, states=12, name='drone_system'
        )
        self.system = nonlinear_system
    
    def simulate_nonlinear_system(self, time, input_signal, initial_state):
        # Create the nonlinear system
        self.create_nonlinear_system()
        # Simulate the response of the nonlinear system
        time, response = ctrl.input_output_response(self.system, time, input_signal, X0=initial_state)
        return time, response

    def add_feedback(self, gain_list):
        # Gainlist is a list of tuples with (kp, ki, kd)
        # Create the PID controller for height
        pid_controller_height = PIDController(*gain_list[0], name_input='pid_height', max_output=300, min_output=10, number_of_outputs=1) # Only commands thrust
        # Create the PID controller for roll
        pid_controller_roll = PIDController(*gain_list[1], name_input='pid_roll', max_output=4, number_of_outputs=1) # Commands torque around y
        # Create the PID controller for pitch
        pid_controller_pitch = PIDController(*gain_list[2], name_input='pid_pitch',max_output=4, number_of_outputs=1) # Commands torque around x
        # Create the PID controller for yaw
        pid_controller_yaw = PIDController(*gain_list[3], name_input='pid_yaw',max_output=4, number_of_outputs=1) # Commands torque around z

        #Define the system
        self.create_nonlinear_system()
        drone_system = self.system

        # Create the closed-loop system

        connections=[
            # Connect the output of the PID to the input of the system
            ['drone_system.u[0]', 'pid_height.y[0]'],
            ['drone_system.u[2]', 'pid_roll.y[0]'],
            ['drone_system.u[1]', 'pid_pitch.y[0]'],
            ['drone_system.u[3]', 'pid_yaw.y[0]'],
            # Connect the output of the system to the input of the PID
            ['pid_height.u[0]', 'drone_system.y[11]'],
            ['pid_roll.u[0]', 'drone_system.y[7]'],
            ['pid_pitch.u[0]', 'drone_system.y[6]'],
            ['pid_yaw.u[0]', 'drone_system.y[8]'],
        ]

        input_list = ['pid_height.u[0]', 'pid_roll.u[0]', 'pid_pitch.u[0]', 'pid_yaw.u[0]']
        output_list = ['drone_system.y[11]', 'drone_system.y[6]', 'drone_system.y[7]', 'drone_system.y[8]']
        closed_sys = ctrl.interconnect([drone_system, pid_controller_height, pid_controller_roll, pid_controller_pitch, pid_controller_yaw], connections, input_list, output_list)
        
        print(closed_sys)

        return closed_sys
        

    

if __name__ == "__main__":
    # Define the drone parameters
    mass = 60
    moment_inertia = [5, 5, 10]
    moment_inertia_prop = 0.01
    torque_thrust_ratio = 0.1
    omega_thrust_ratio = 0.1

    # Create the drone system
    drone_system = DroneSystem(mass, moment_inertia, moment_inertia_prop, torque_thrust_ratio, omega_thrust_ratio)
    
    # Define the PID controller parameters
    gain_list = [
        [50, 0.5, 0, 1],  # height
        [1, 0.5, 0, 0],  # roll
        [1, 0.5, 0, 0],  # pitch
        [1, 0.5, 0, 0],  # yaw
    ]

    # Define the time vector for the simulation
    dt = 0.01
    time = np.arange(0, 100, dt)

    # Define the setpoint signals for roll, pitch, and yaw
    setpoint_roll = np.zeros_like(time)
    setpoint_pitch = np.zeros_like(time)  # Desired pitch angle in radians
    # setpoint_pitch = np.ones_like(time) * 5 * np.pi / 180  
    setpoint_yaw = np.zeros_like(time)
    setpoint_height = np.ones_like(time) * 50

    # # Stack the setpoint signals
    setpoints = np.vstack((setpoint_height, setpoint_roll, setpoint_pitch, setpoint_yaw))

    # Create the closed-loop system
    closed_loop_system = drone_system.add_feedback(gain_list)

    # Initial state
    X0 = [0]*24
    X0[11] = -50

    # Simulate the response of the closed-loop system
    time, response = ctrl.input_output_response(closed_loop_system, time, setpoints, X0=X0)

    # Plot the response of the closed-loop system
    plt.figure()
    plt.plot(time, response[2], label='Roll angle')
    plt.plot(time, response[1], label='Pitch angle')
    plt.plot(time, response[3], label='Yaw angle')
    plt.plot(time, -response[0], label='Height')
    plt.legend()
    plt.xlabel('Time [s]')
    plt.ylabel('response')
    plt.title('Response of the closed-loop system')
    plt.show()

