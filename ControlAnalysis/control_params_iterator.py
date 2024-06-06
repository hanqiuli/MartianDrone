import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint

sys.path.append('.')
from legacy.DesignTools.environment_properties import ENV as ENVdict
from ControlAnalysis.final_control import HexacopterModel


def RMSdiff(a, b):
    max_length = max(np.size(a), np.size(b))
    return np.sqrt(np.sum((a-b)**2) / max_length)


if __name__ == "__main__":
    
    mass = 60.0
    moment_inertia = np.diag([5, 5, 8])
    moment_inertia_prop = 0.01
    torque_thrust_ratio = 0.1
    omega_thrust_ratio = 0.1
    ENV = ENVdict

    pid_params = [[10, 0.5, 6, 3], [5, 0.2, 3, 10], [5, 0.2, 3, 10], [5, 0.2, 3, 10]]
    hexacopter = HexacopterModel(mass, moment_inertia, moment_inertia_prop, pid_params, torque_thrust_ratio=torque_thrust_ratio, omega_thrust_ratio=omega_thrust_ratio, ENV=ENV)

    delta_t = 0.05
    t = np.arange(0, 350, delta_t)
    initial_state = np.zeros(12)
    # initial altitude is nonzero
    initial_state[2] = 3

    

    desired_x = np.zeros_like(t)
    desired_y = np.zeros_like(t)
    # desired_z = np.ones_like(t) * 50 # should hover

    desired_z = np.zeros_like(t)
    desired_z[t <= 5] = np.linspace(0, 20, len(t[t <= 5]))
    desired_z[(t > 5) & (t <= 10)] = 20
    desired_z[(t > 10) & (t <= 20)] = 30
    desired_z[(t > 20) & (t <= 30)] = 5
    desired_z[(t > 30) & (t <= 32)] = 20
    desired_z[(t > 32) & (t <= 50)] = 60
    desired_z[(t > 50) & (t <= 250)] = np.sin(t[(t > 50) & (t <= 250)]*np.pi/5)*5 + 50
    desired_z[(t > 250)] = 100


    desired_phi = np.zeros_like(t)
    desired_phi[(t > 100)] = np.sin(t[t > 100]*np.pi/5)*7*np.pi/180
    # desired_theta = np.ones_like(t) * 0.05
    desired_theta = np.zeros_like(t)
    desired_theta[(t > 150)] = np.sin(t[t > 150]*np.pi/5+2)*9*np.pi/180
    # desired_theta = np.sin(t/2 + np.pi/2)/10
    desired_psi = np.zeros_like(t)
    desired_psi[(t > 180) & (t <= 220)] = np.sin(t[(t > 180) & (t <= 220)]*np.pi/5+2)*9*np.pi/180
    desired_psi[(t > 220)] = np.linspace(0, 2*np.pi, len(t[t > 220]))
    desired_xdot = np.zeros_like(t)
    desired_ydot = np.zeros_like(t)
    desired_zdot = np.zeros_like(t)
    desired_p = np.zeros_like(t)
    desired_q = np.zeros_like(t)
    desired_r = np.zeros_like(t)
    
    desired_states = [desired_x, desired_y, desired_z, desired_phi, desired_theta, desired_psi, desired_xdot, desired_ydot, desired_zdot, desired_p, desired_q, desired_r]

    bestRMS = np.Infinity
    for i in range(10000):
        pid_params = np.random.uniform(0, 30, [4, 4])
        pid_params[3] = pid_params[2]
        

        # Simulate the response of the closed-loop system
        try:
            states, times, thruster_values = hexacopter.simulate(t, initial_state, desired_states)
        except Exception as e:
            if isinstance(e, (KeyboardInterrupt, SystemExit)):
                print(i)
                raise e
            continue
        
        rms = RMSdiff(states[:, 2], desired_z)
        if rms < bestRMS and not hexacopter.crashed:
            bestRMS = rms
            print(rms)
            print(pid_params)
            print()
            best_gain_list = pid_params


    hexacopter.plot_figures(states, times, desired_states)
    plt.plot(times[:-1], thruster_values)
    plt.legend(['1', '2', '3', '4', '5', '6'])
    plt.show()


    
    