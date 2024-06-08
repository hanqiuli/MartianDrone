import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint
from scipy import optimize

sys.path.append('.')
from legacy.DesignTools.environment_properties import ENV as ENVdict
from ControlAnalysis.final_control import HexacopterModel


def RMSdiff(a, b):
    max_length = max(np.size(a), np.size(b))
    return np.sqrt(np.sum((a-b)**2) / max_length)


def getRMS(pid_params_flattened):
    global desired_states, t, fcount

    fcount += 1
    if fcount % 100 == 0:
        print(fcount)


    mass = 60.0
    moment_inertia = np.diag([5, 5, 8])
    moment_inertia_prop = 0.01
    torque_thrust_ratio = 0.1
    omega_thrust_ratio = 0.1
    ENV = ENVdict
    thrust_to_weight_range = [0.6, 1.2]

    pid_params = np.array([ [11.98595877,  4.93789508, 27.97515155, 16.86759748, np.inf],
                            [ 3.95433974,  7.06347748,  2.30512318, 22.40876810, np.inf],
                            [ 3.95433974,  7.06347748,  2.30512318, 22.40876810, np.inf],
                            [10.25419113,  5.83153413, 11.13413042,  8.72583077, np.inf]])

    

    pid_params = pid_params_flattened.reshape((4, 5))
    hexacopter = HexacopterModel(mass, moment_inertia, moment_inertia_prop, pid_params, \
            torque_thrust_ratio=torque_thrust_ratio, thrust_to_weight_range=thrust_to_weight_range, \
            omega_thrust_ratio=omega_thrust_ratio, ENV=ENV)

    
    initial_state = np.zeros(12)
    # initial altitude is nonzero
    initial_state[2] = 3

    # Simulate the response of the closed-loop system
    states, times, thruster_values = hexacopter.simulate(t, initial_state, desired_states)

    rms = 0
    rms += RMSdiff(states[:, 2], desired_z[:np.size(states[:, 2])])
    rms += RMSdiff(states[:, 3], desired_phi[:np.size(states[:, 3])])
    rms += RMSdiff(states[:, 4], desired_theta[:np.size(states[:, 4])])
    rms += RMSdiff(states[:, 5], desired_psi[:np.size(states[:, 5])])
    if hexacopter.crashed:
        rms *= (100000/(np.size(states[:, 0])*delta_t) + 100)
    
    return rms

if __name__ == "__main__":
    fcount = -1

    delta_t = 0.05
    t = np.arange(0, 350, delta_t)

    desired_x = np.zeros_like(t)
    desired_y = np.zeros_like(t)

    desired_z = np.zeros_like(t)
    desired_z[t <= 5] = np.linspace(0, 20, len(t[t <= 5]))
    desired_z[(t > 5) & (t <= 10)] = 20
    desired_z[(t > 10) & (t <= 20)] = 30
    desired_z[(t > 20) & (t <= 30)] = 15
    desired_z[(t > 30) & (t <= 32)] = 20
    desired_z[(t > 32) & (t <= 50)] = 60
    desired_z[(t > 50) & (t <= 250)] = np.sin(t[(t > 50) & (t <= 250)]*np.pi/10)*5 + 50
    desired_z[(t > 250)] = 80

    desired_phi = np.zeros_like(t)
    desired_phi[(t > 100)] = np.sin(t[t > 100]*np.pi/11)*7*np.pi/180

    desired_theta = np.zeros_like(t)
    desired_theta[(t > 150)] = np.sin(t[t > 150]*np.pi/15+2)*9*np.pi/180

    desired_psi = np.zeros_like(t)
    desired_psi[(t > 180) & (t <= 220)] = np.sin(t[(t > 180) & (t <= 220)]*np.pi/12+2)*9*np.pi/180
    desired_psi[(t > 220)] = np.linspace(0, 2*np.pi, len(t[t > 220]))

    desired_xdot = np.zeros_like(t)
    desired_ydot = np.zeros_like(t)
    desired_zdot = np.zeros_like(t)
    desired_p = np.zeros_like(t)
    desired_q = np.zeros_like(t)
    desired_r = np.zeros_like(t)
    
    desired_states = [desired_x, desired_y, desired_z, desired_phi, desired_theta, desired_psi, desired_xdot, desired_ydot, desired_zdot, desired_p, desired_q, desired_r]



    lower_bounds = np.zeros([4, 5])
    higher_bounds = np.zeros([4, 5])
    higher_bounds[:, 0:4] = 50
    higher_bounds[:, 4] = 500

    bounds = list(zip(lower_bounds.flatten(), higher_bounds.flatten()))
    
    # Perform global optimization using dual annealing
    #print("starting")
    #result = optimize.dual_annealing(getRMS, bounds)
    #print(result)

    # Reshape the result back to the original 4x5 shape
    #optimized_params = result.x.reshape((4, 5))
    #print(optimized_params)

    mass = 60.0
    moment_inertia = np.diag([5, 5, 8])
    moment_inertia_prop = 0.01
    torque_thrust_ratio = 0.1
    omega_thrust_ratio = 0.1
    ENV = ENVdict
    thrust_to_weight_range = [0.6, 1.5]    
    

    #pid_params = optimized_params
    pid_params = np.array([ [11.98595877,  4.93789508, 27.97515155, 16.86759748, np.inf],
                            [ 3.95433974,  7.06347748,  2.30512318, 22.40876810, np.inf],
                            [ 3.95433974,  7.06347748,  2.30512318, 22.40876810, np.inf],
                            [10.25419113,  5.83153413, 11.13413042,  8.72583077, np.inf]])
    hexacopter = HexacopterModel(mass, moment_inertia, moment_inertia_prop, pid_params, \
            torque_thrust_ratio=torque_thrust_ratio, thrust_to_weight_range=thrust_to_weight_range, \
            omega_thrust_ratio=omega_thrust_ratio, ENV=ENV)


    initial_state = np.zeros(12)

    # Simulate the response of the closed-loop system
    states, times, thruster_values = hexacopter.simulate(t, initial_state, desired_states)
    HexacopterModel.plot_figures(states, times, desired_states, thruster_values)
    plt.plot(times[:-1], thruster_values)
    plt.legend(['1', '2', '3', '4', '5', '6'])
    plt.show()
