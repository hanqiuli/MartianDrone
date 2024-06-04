import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

class PIDController:
    """
    PID Controller for the drone system
    """
    def __init__(self, Kp, Ki, Kd, thruster_weights):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.thruster_weights = thruster_weights
        self.integral = 0
        self.previous_error = 0

    def compute_control(self, setpoint, measurement, dt):
        error = setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error

        u_pid = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return u_pid * self.thruster_weights

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
            [1,     1,                   1,                 1,      1,              1],
            [0,     -arm_length*np.sqrt(3)/2,    -arm_length*np.sqrt(3)/2,    0,      arm_length*np.sqrt(3)/2, arm_length*np.sqrt(3)/2],
            [arm_length,     0.5*arm_length,              -0.5*arm_length,             -arm_length,     -0.5*arm_length,         0.5*arm_length],
            [-torque_thrust_ratio, torque_thrust_ratio,               -torque_thrust_ratio,              torque_thrust_ratio,   -torque_thrust_ratio,          torque_thrust_ratio]
        ])
        
    def nonlinear_system_dynamics(self, t, state, inputs):
        v_xr, v_yr, v_zr, omega_xr, omega_yr, omega_zr, Theta, Phi, Psi, X, Y, Z = state
        u = np.clip(inputs, 10, 40)  # Clip the input signal to be between 10 and 40
        T, M_x, M_y, M_z = np.dot(self.transformation_matrix, u)

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

        return [dv_xr, dv_yr, dv_zr, domega_xr, domega_yr, domega_zr, dTheta, dPhi, dPsi, dX, dY, dZ]

    def simulate(self, initial_state, setpoints, time, pid_controllers, hover_thrust):
        dt = time[1] - time[0]
        num_steps = len(time)
        state = initial_state
        states = np.zeros((num_steps, len(state)))
        states[0, :] = state
        inputs = np.ones(6) * hover_thrust
        
        for i in range(1, num_steps):
            current_setpoints = setpoints[:, i]
            controls = np.zeros(6)
            for j, pid in enumerate(pid_controllers):
                controls += pid.compute_control(current_setpoints[j], state[j + 6], dt)
            inputs = np.clip(controls + hover_thrust, 10, 40)
            state = solve_ivp(lambda t, x: self.nonlinear_system_dynamics(t, x, inputs), [0, dt], state).y[:, -1]
            states[i, :] = state
        
        return states

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
    ENV = {'g': 9.81}
    arm_length = 2

    drone_system = DroneSystem(mass, moment_inertia, moment_inertia_prop, torque_thrust_ratio, omega_thrust_ratio, ENV, arm_length)

    Kp = 0.7
    Ki = 0.1
    Kd = 0

    setpoint_indices = {'roll': 7, 'pitch': 6, 'yaw': 8, 'height': 11}
    
    time = np.linspace(0, 500, 10000)

    setpoint_roll = np.zeros_like(time)
    setpoint_pitch = np.ones_like(time) * 5 * np.pi / 180
    setpoint_yaw = np.zeros_like(time)
    setpoint_height = np.ones_like(time) * -80

    setpoints = np.vstack((setpoint_roll, setpoint_pitch, setpoint_yaw, setpoint_height))

    pid_roll = PIDController(Kp, Ki, Kd, drone_system.transformation_matrix[2])
    pid_pitch = PIDController(Kp, Ki, Kd, drone_system.transformation_matrix[1])
    pid_yaw = PIDController(Kp, Ki, Kd, drone_system.transformation_matrix[3])
    pid_height = PIDController(Kp, Ki, Kd, -drone_system.transformation_matrix[0])

    pid_controllers = [pid_roll, pid_pitch, pid_yaw, pid_height]

    initial_state = np.zeros(12)
    initial_state[11] = -80

    hover_thrust = mass * ENV['g'] / 6  # Dividing total thrust required for hover by the number of rotors

    response = drone_system.simulate(initial_state, setpoints, time, pid_controllers, hover_thrust)

    plt.plot(time, response[:, setpoint_indices['pitch']], label='Pitch response')
    plt.plot(time, response[:, 11], label='Height response')
    plt.xlabel('Time (s)')
    plt.ylabel('Response')
    plt.title('Pitch and Height Response with PID Control')
    plt.grid(True)
    plt.legend()
    plt.show()
    
    for i in range(12, 18):
        plt.plot(time, response[:, i], label=f'Thrust {i-11}')
    
    plt.xlabel('Time (s)')
    plt.ylabel('Thrust')
    plt.title('Thrust Response with PID Control')
    plt.grid(True)
    plt.legend()
    plt.show()
