import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

class PIDController:
    """
    PID Controller for the drone system
    """
    def __init__(self, Kp, Ki, Kd, u_min=0, u_max=40):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.previous_error = 0
        self.u_min = u_min
        self.u_max = u_max

    def compute_control(self, setpoint, measurement, dt):
        error = setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error

        u_pid = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return np.clip(u_pid, self.u_min, self.u_max)

class DroneSystem:
    def __init__(self, mass, moment_inertia, moment_inertia_prop, torque_thrust_ratio, omega_thrust_ratio, ENV, arm_length=2):
        self.mass = mass
        self.moment_inertia = moment_inertia
        self.moment_inertia_prop = moment_inertia_prop
        self.torque_thrust_ratio = torque_thrust_ratio
        self.omega_thrust_ratio = omega_thrust_ratio
        self.environment = ENV
        self.arm_length = arm_length
        
        self.transformation_matrix = np.array([
            [-1,     -1,                   -1,                 -1,      -1,              -1],
            [0,     -arm_length*np.sqrt(3)/2,    -arm_length*np.sqrt(3)/2,    0,      arm_length*np.sqrt(3)/2, arm_length*np.sqrt(3)/2],
            [arm_length,     0.5*arm_length,              -0.5*arm_length,             -arm_length,     -0.5*arm_length,         0.5*arm_length],
            [-torque_thrust_ratio, torque_thrust_ratio,               -torque_thrust_ratio,              torque_thrust_ratio,   -torque_thrust_ratio,          torque_thrust_ratio]
        ])
        
    def nonlinear_system_dynamics(self, t, state, inputs):
        v_xr, v_yr, v_zr, omega_xr, omega_yr, omega_zr, Theta, Phi, Psi, X, Y, Z = state
        T, M_x, M_y, M_z = np.dot(self.transformation_matrix, inputs)

        g = self.environment['g']
        J_x, J_y, J_z = self.moment_inertia
        k_omegaT = self.omega_thrust_ratio
        k_MT = self.torque_thrust_ratio
        J_mp = self.moment_inertia_prop
        m = self.mass
        L = self.arm_length

        s_Theta, c_Theta = np.sin(Theta), np.cos(Theta)
        s_Phi, c_Phi = np.sin(Phi), np.cos(Phi)
        s_Psi, c_Psi = np.sin(Psi), np.cos(Psi)
        t_Theta = np.tan(Theta)

        dv_xr = -v_xr * omega_yr + v_yr * omega_zr - g * s_Theta
        dv_yr = -v_xr * omega_zr + v_zr * omega_xr + g * c_Theta * s_Phi
        dv_zr = -v_yr * omega_xr + v_xr * omega_yr + g * c_Theta * c_Phi - T / m
        domega_xr = (1 / J_x) * (-omega_yr * omega_zr * (J_z - J_y) + M_x + (k_omegaT / k_MT) * J_mp * M_z * omega_yr)
        domega_yr = (1 / J_y) * (-omega_xr * omega_zr * (J_x - J_z) + M_y + (k_omegaT / k_MT) * J_mp * M_z * omega_xr)
        domega_zr = M_z / J_z
        dTheta = omega_yr * c_Phi - omega_zr * s_Phi
        dPhi = omega_xr + omega_yr * s_Phi * t_Theta + omega_zr * c_Phi * t_Theta
        dPsi = omega_yr * s_Phi / c_Theta + omega_zr * c_Phi / c_Theta

        dX = c_Psi * c_Theta * v_xr + (-s_Psi * c_Phi + c_Psi * s_Theta * s_Phi) * v_yr + (s_Psi * s_Phi + c_Psi * s_Theta * c_Phi) * v_zr
        dY = s_Psi * c_Theta * v_xr + (c_Psi * c_Phi + s_Psi * s_Theta * s_Phi) * v_yr + (-c_Psi * s_Phi + s_Psi * s_Theta * c_Phi) * v_zr
        dZ = -s_Theta * v_xr + c_Theta * s_Phi * v_yr + c_Theta * c_Phi * v_zr

        return np.array([dv_xr, dv_yr, dv_zr, domega_xr, domega_yr, domega_zr, dTheta, dPhi, dPsi, dX, dY, dZ])

    def simulate(self, initial_state, setpoints, time, pid_controllers, hover_thrust):
        
        num_steps = len(time)
        state = initial_state
        states = np.zeros((num_steps, len(state)))
        states[0, :] = state
        thrust_history = np.zeros((num_steps, 6))
        
        for i in range(1, num_steps):
            dt = time[i] - time[i-1]
            current_setpoints = setpoints[:, i]
            controls = np.zeros(6)
            for j, pid in enumerate(pid_controllers):
                control_input = pid.compute_control(current_setpoints[j], state[setpoint_indices[j]], dt)
                controls[j] = control_input

            thrust = np.clip(controls, 0, 40)
            x_dot = self.nonlinear_system_dynamics(time[i], state, thrust)
            state += x_dot * dt

            states[i, :] = state
            thrust_history[i, :] = thrust
        
        return states, thrust_history

    def plot_response(self, time, response, setpoint_indices):
        fig, ax = plt.subplots(3, 4, figsize=(20, 10))
        ax[0, 0].plot(time, response[:, 0])
        ax[0, 0].set_title('v_xr')
        ax[0, 1].plot(time, response[:, 1])
        ax[0, 1].set_title('v_yr')
        ax[0, 2].plot(time, response[:, 2])
        ax[0, 2].set_title('v_zr')
        ax[0, 3].plot(time, response[:, 3])
        ax[0, 3].set_title('omega_xr')
        ax[1, 0].plot(time, response[:, 4])
        ax[1, 0].set_title('omega_yr')
        ax[1, 1].plot(time, response[:, 5])
        ax[1, 1].set_title('omega_zr')
        ax[1, 2].plot(time, response[:, 6])
        ax[1, 2].set_title('Theta')
        ax[1, 3].plot(time, response[:, 7])
        ax[1, 3].set_title('Phi')
        ax[2, 0].plot(time, response[:, 8])
        ax[2, 0].set_title('Psi')
        ax[2, 1].plot(time, response[:, 9])
        ax[2, 1].set_title('X')
        ax[2, 2].plot(time, response[:, 10])
        ax[2, 2].set_title('Y')
        ax[2, 3].plot(time, response[:, 11])
        ax[2, 3].set_title('Z')
        plt.show()

if __name__ == "__main__":
    mass = 60
    moment_inertia = [5, 5, 10]
    moment_inertia_prop = 0.01
    torque_thrust_ratio = 0.1
    omega_thrust_ratio = 0.1
    ENV = {'g': 3.71}
    arm_length = 2

    drone_system = DroneSystem(mass, moment_inertia, moment_inertia_prop, torque_thrust_ratio, omega_thrust_ratio, ENV, arm_length)

    Kp = 1.0
    Ki = 0.1
    Kd = 0.5

    setpoint_indices = [6, 7, 8, 11]
    
    time = np.linspace(0, 500, 10000)

    setpoint_roll = np.zeros_like(time)
    setpoint_pitch = np.zeros_like(time) * 5 * np.pi / 180
    setpoint_yaw = np.zeros_like(time)
    setpoint_height = np.ones_like(time) * -1000

    setpoints = np.vstack((setpoint_roll, setpoint_pitch, setpoint_yaw, setpoint_height))

    pid_roll = PIDController(Kp, Ki, Kd)
    pid_pitch = PIDController(Kp, Ki, Kd)
    pid_yaw = PIDController(Kp, Ki, Kd)
    pid_height = PIDController(Kp, Ki, Kd)

    pid_controllers = [pid_roll, pid_pitch, pid_yaw, pid_height]

    initial_state = np.zeros(12)
    initial_state[11] = -80

    hover_thrust = mass * ENV['g'] / 6  # Dividing total thrust required for hover by the number of rotors

    response, inputs = drone_system.simulate(initial_state, setpoints, time, pid_controllers, hover_thrust)

    plt.plot(time, response[:, setpoint_indices[1]], label='Pitch response')
    plt.plot(time, response[:, setpoint_indices[3]], label='Height response')
    plt.xlabel('Time (s)')
    plt.ylabel('Response')
    plt.title('Pitch and Height Response with PID Control')
    plt.grid(True)
    plt.legend()
    plt.show()
    
    for i in range(0, 6):
        plt.plot(time, inputs[:, i], label=f'Thrust {i}')
    
    plt.xlabel('Time (s)')
    plt.ylabel('Thrust')
    plt.title('Thrust Response with PID Control')
    plt.grid(True)
    plt.legend()
    plt.show()
