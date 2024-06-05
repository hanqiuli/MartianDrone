import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# Define hexacopter parameters
m = 100.0  # mass of the hexacopter
g = 3.71  # gravitational acceleration
J = np.diag([5, 5, 5])  # moment of inertia matrix

# Define PID gains
Kp_alt = 12
Ki_alt = 0.5
Kd_alt = 12

Kp_att = 1.46
Ki_att = 2.151
Kd_att = 0.326

# Define initial conditions
initial_state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]

# Define time vector
t = np.linspace(0, 50, 1000)

# Generate the doublet signal for altitude
desired_z = np.zeros_like(t)
desired_z[t <= 5] = np.linspace(0, 20, len(t[t <= 5]))
desired_z[(t > 5) & (t <= 10)] = 20
desired_z[(t > 10) & (t <= 20)] = 30
desired_z[(t > 20) & (t <= 30)] = 10
desired_z[(t > 30) & (t <= 32)] = 20
desired_z[t > 32] = 20

desired_state = [0, 0, 20, 0, 0, 0]  # [x, y, z, phi, theta, psi]

# PID Controller
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
        self.integral = np.clip(self.integral, -self.wind_up_limit, self.wind_up_limit)
        if self.type=='altitude':
            print(self.integral)

        # Derivative term
        derivative = (error - self.previous_error) / dt
        self.previous_error = error

        # Clipped return
        if self.type == 'altitude':
            return np.clip(self.Kp * error + self.Ki * self.integral + self.Kd * derivative, -0.5*m*g, 0.5*g)
        else:
            return self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
# Hexacopter dynamics
def hexacopter_dynamics(state, t, u1, u2, u3, u4):
    x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r = state
    
    # Translational dynamics
    x_ddot = (u1/m) * (np.cos(phi) * np.sin(theta) * np.cos(psi) + np.sin(phi) * np.sin(psi))
    y_ddot = (u1/m) * (np.cos(phi) * np.sin(theta) * np.sin(psi) - np.sin(phi) * np.cos(psi))
    z_ddot = (u1/m) * np.cos(phi) * np.cos(theta) - g
    
    # Rotational dynamics
    phi_dot = p + q * np.sin(phi) * np.tan(theta) + r * np.cos(phi) * np.tan(theta)
    theta_dot = q * np.cos(phi) - r * np.sin(phi)
    psi_dot = q * np.sin(phi) / np.cos(theta) + r * np.cos(phi) / np.cos(theta)
    
    p_dot = (J[1,1] - J[2,2]) * q * r / J[0,0] + u2 / J[0,0]
    q_dot = (J[2,2] - J[0,0]) * p * r / J[1,1] + u3 / J[1,1]
    r_dot = (J[0,0] - J[1,1]) * p * q / J[2,2] + u4 / J[2,2]
    
    return [x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot, x_ddot, y_ddot, z_ddot, p_dot, q_dot, r_dot]

# Initialize PID controllers for altitude and attitude
pid_altitude = PIDController(Kp_alt, Ki_alt, Kd_alt, 3, set_point=desired_state[2], PID_type = 'altitude')
pid_roll = PIDController(Kp_att, Ki_att, Kd_att, 10, set_point=desired_state[3], PID_type = 'attitude')
pid_pitch = PIDController(Kp_att, Ki_att, Kd_att, 10, set_point=desired_state[4], PID_type = 'attitude')
pid_yaw = PIDController(Kp_att, Ki_att, Kd_att, 10, set_point=desired_state[5], PID_type = 'attitude')

# Simulation loop
dt = t[1] - t[0]
states = [initial_state]
for i in range(1, len(t)):
    current_state = states[-1]
    z = current_state[2]
    phi = current_state[3]
    theta = current_state[4]
    psi = current_state[5]
    
    pid_altitude.set_point = desired_z[i]  # Update the desired altitude at each time step
    
    u1 = m * g + pid_altitude.update(z, dt)
    u2 = pid_roll.update(phi, dt)
    u3 = pid_pitch.update(theta, dt)
    u4 = pid_yaw.update(psi, dt)
    
    new_state = odeint(hexacopter_dynamics, current_state, [0, dt], args=(u1, u2, u3, u4))[-1]
    states.append(new_state)

# Convert states to numpy array for easier plotting
states = np.array(states)

# Plotting
plt.figure()
plt.subplot(3, 1, 1)
plt.plot(t, states[:, 2], label='z')
plt.plot(t, desired_z, 'r--', label='desired z')
plt.xlabel('Time [s]')
plt.ylabel('Altitude [m]')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(t, states[:, 8], label='z_dot')
plt.xlabel('Time [s]')
plt.ylabel('Altitude [m]')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(t, states[:, 3], label='phi')
plt.plot(t, np.zeros_like(t), 'r--', label='desired phi')
plt.xlabel('Time [s]')
plt.ylabel('Roll [rad]')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(t, states[:, 4], label='theta')
plt.plot(t, np.zeros_like(t), 'r--', label='desired theta')
plt.xlabel('Time [s]')
plt.ylabel('Pitch [rad]')
plt.legend()

plt.tight_layout()
plt.show()
