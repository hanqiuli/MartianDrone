import sys
import os
import random as rand
import re
import math

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import scienceplots

sys.path.append('.')
from ControlAnalysis.hexacopter_model import HexacopterModel
from ControlAnalysis.hexacopter_flight_computer import FlightComputer


def rolling_average(arr, n):
    kernel = np.ones(n) / n
    axis = 0
    rolling_avg = np.apply_along_axis(lambda m: np.convolve(m, kernel, mode='valid'), axis, arr)
    pad_size = arr.shape[axis] - rolling_avg.shape[axis]
    pad_before = pad_size // 2
    pad_after = pad_size - pad_before
    
    pad_width = [(0, 0)] * arr.ndim
    pad_width[axis] = (pad_before, pad_after)
    
    padded_avg = np.pad(rolling_avg, pad_width, mode='edge')
    
    return padded_avg


def solve_ivp(dydx_func, y0, t_eval, args):
    """
    Solve an initial value problem using the forward euler method.
    """
    

    extra = []
    if len(t_eval) < 2:
        t_eval = [0, t_eval[0]]
    y = np.zeros([len(t_eval), len(y0)])
    y[0] = y0

    for i, t in enumerate(t_eval[1:]):
        dt = t_eval[i+1] - t_eval[i]
        dydx, *extra = dydx_func(t, y[i], *args)
        y[i+1] = y[i] + dt*dydx

    return y, *extra


def find_highest_index(directory, pattern_prefix, pattern_suffix):
    # Regular expression pattern to match file_name_{i}_suffix
    pattern = re.compile(rf"{re.escape(pattern_prefix)}(\d+){re.escape(pattern_suffix)}")
    
    # List all files in the directory
    files = os.listdir(directory)
    
    # Extract numbers from files matching the pattern
    indices = []
    for file in files:
        match = pattern.match(file)
        if match:
            indices.append(int(match.group(1)))
    
    # Find the highest number
    if indices:
        return max(indices)
    else:
        return -1


def filepath(filename, path):
   folders = os.path.join(*path)
   os.makedirs(folders, exist_ok=True)

   fullpath = os.path.join(folders, filename)
   return fullpath


def plot_figures(states, times, setpoints, thruster_values, flight_mode_list, input_array, disturbance, time_step=0.01, dpi=150):
    """
        Plot the simulation results
    """
    path = ['ControlAnalysis', 'Figures']
    #plt.style.use('science')


    desired_x, desired_y, desired_z, desired_phi, desired_theta, desired_psi, \
    desired_xdot, desired_ydot, desired_zdot, desired_p, desired_q, desired_r = setpoints.T
    # clipping the setpoints to the length of the simulation
    desired_x = desired_x[:len(times)]
    desired_y = desired_y[:len(times)]
    desired_z = desired_z[:len(times)]
    desired_phi = desired_phi[:len(times)]
    desired_theta = desired_theta[:len(times)]
    desired_psi = desired_psi[:len(times)]
    desired_xdot = desired_xdot[:len(times)]
    desired_ydot = desired_ydot[:len(times)]
    desired_zdot = desired_zdot[:len(times)]
    desired_p = desired_p[:len(times)]
    desired_q = desired_q[:len(times)]
    desired_r = desired_r[:len(times)]

    # Position plot
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(times, states[:, 0], label='x')
    plt.plot(times, desired_x, 'r--', label='desired x')
    plt.plot(times, states[:, 1], label='y')
    plt.plot(times, desired_y, 'g--', label='desired y')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    #plt.ylim([290, 310])
    plt.legend()

    r = np.linalg.norm(states[:, 0:2]-np.array([desired_x, desired_y]).T, axis=1)

    plt.subplot(3, 1, 2)
    plt.plot(times, r, label='r')
    plt.plot(times, np.ones_like(times)*125, 'r--', label='close range')
    plt.plot(times, np.ones_like(times)*5, 'b--', label='land range')
    plt.plot(times, np.ones_like(times)*0.4, 'g--', label='accuracy range')
    plt.text(times[-1], r[-1], f'r = {round(r[-1], 2)}', fontsize=7, ha='right', color='k', alpha=0.7)
    print(f'r = {round(r[-1], 2)}')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.yscale('log')
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(times, states[:, 2], label='z')
    plt.plot(times, desired_z, 'r--', label='desired z')
    plt.xlabel('Time [s]')
    plt.ylabel('Altitude [m]')
    plt.ylim([0, 150])
    plt.legend()

    plt.tight_layout()
    plt.savefig(filepath('position.png', path), dpi=dpi)

    # Attitude plot
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(times, states[:, 3], label='phi')
    plt.plot(times, desired_phi, 'r--', label='desired phi')
    plt.xlabel('Time [s]')
    plt.ylabel('Roll [rad]')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(times, states[:, 4], label='theta')
    plt.plot(times, desired_theta, 'r--', label='desired theta')
    plt.xlabel('Time [s]')
    plt.ylabel('Pitch [rad]')
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(times, states[:, 5], label='psi')
    plt.plot(times, desired_psi, 'r--', label='desired psi')
    plt.xlabel('Time [s]')
    plt.ylabel('Yaw [rad]')
    plt.legend()

    plt.tight_layout()
    plt.savefig(filepath('angles.png', path), dpi=dpi)

    # Velocity plot
    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(times, states[:, 6], label='x_dot')
    plt.plot(times, desired_xdot, 'r--', label='desired x_dot')
    plt.plot(times, states[:, 7], label='y_dot')
    plt.plot(times, desired_ydot, 'g--', label='desired y_dot')
    plt.xlabel('Time [s]')
    plt.ylabel('Horizontal velocity [m/s]')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(times, states[:, 8], label='z_dot')
    plt.plot(times, desired_zdot, 'r--', label='desired z_dot')
    plt.xlabel('Time [s]')
    plt.ylabel('Altitude velocity [m/s]')
    plt.ylim([-20, 20])

    plt.tight_layout()
    plt.savefig(filepath('velocity.png', path), dpi=dpi)

    plt.figure()
    half_time = np.size(times)//2
    plt.plot(times[half_time:], states[half_time:, 6], label='x_dot')
    plt.plot(times[half_time:], desired_xdot[half_time:], 'r--', label='desired x_dot')
    plt.plot(times[half_time:], states[half_time:, 7], label='y_dot')
    plt.plot(times[half_time:], desired_ydot[half_time:], 'g--', label='desired y_dot')
    plt.xlabel('Time [s]')
    plt.ylabel('Horizontal velocity [m/s]')
    plt.legend()

    plt.tight_layout()
    plt.savefig(filepath('velocitylanding.png', path), dpi=dpi)


    # Attitude rate plot
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(times, states[:, 9], label='p')
    plt.plot(times, desired_p, 'r--', label='desired p')
    plt.xlabel('Time [s]')
    plt.ylabel('Roll rate [rad/s]')
    plt.ylim([-3, 3])
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(times, states[:, 10], label='q')
    plt.plot(times, desired_q, 'r--', label='desired q')
    plt.xlabel('Time [s]')
    plt.ylabel('Pitch rate [rad/s]')
    plt.ylim([-3, 3])
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(times, states[:, 11], label='r')
    plt.plot(times, desired_r, 'r--', label='desired r')
    plt.xlabel('Time [s]')
    plt.ylabel('Yaw rate [rad/s]')
    plt.ylim([-3, 3])
    plt.legend()


    plt.tight_layout()
    plt.savefig(filepath('angular_velocity.png', path), dpi=dpi)

    # Thruster plot
    plt.figure(figsize=(30, 4.8*1.5))
    plt.plot(times[:-1], thruster_values)
    plt.legend(['1', '2', '3', '4', '5', '6'])
    plt.xlabel('Time [s]')
    plt.ylabel('Thruster values [N]')
    plt.tight_layout()
    plt.savefig(filepath('thrusters.png', path), dpi=dpi)

    # Thruster plot
    plt.figure(figsize=(30, 4.8*3))
    plt.plot(times[:-1], input_array[:times.shape[0]-1])
    plt.legend(['T', 'Mx', 'My', 'Mz'])
    plt.xlabel('Time [s]')
    plt.ylabel('Thruster values [N]')
    plt.tight_layout()
    plt.savefig(filepath('inputs.png', path), dpi=dpi)

    # Flight mode plot
    flight_modes = sorted(list(set(flight_mode_list)))
    mode_to_index = {status: index for index, status in enumerate(flight_modes)}
    flight_mode_list_indexed = [mode_to_index[mode] for mode in flight_mode_list]

    mode_matrix = np.zeros((len(flight_modes), len(flight_mode_list)), dtype=int)
    mode_matrix[flight_mode_list_indexed, np.arange(len(flight_mode_list))] = 1

    plt.figure(figsize=(20, 4.8))
    for row_index, mode in enumerate(flight_modes):
        plt.plot(times[:-1], mode_matrix[row_index], linestyle='--', label=mode)
        plt.fill_between(times[:-1], mode_matrix[row_index], where=mode_matrix[row_index] == 1, 
                interpolate=True, alpha=0.3, hatch='//')
    plt.legend()
    plt.xlabel('Time [s]')
    plt.ylabel('Active [True/False]')
    plt.tight_layout()
    plt.savefig(filepath('flight_mode.png', path), dpi=dpi)

    mpl.rcParams['agg.path.chunksize'] = 10000
    # Disturbance
    floating_average_disturbance = rolling_average(disturbance, int(3/time_step))
    print(disturbance.shape)
    plt.figure(figsize=(30, 4.8*3))
    plt.plot(times[:-1], floating_average_disturbance[:-1, :3])
    plt.legend(['a_x', 'a_y', 'a_z'])
    plt.xlabel('Time [s]')
    plt.ylabel('Disturbance values [N] or [Nm]')
    plt.tight_layout()
    plt.savefig(filepath('turbulence_axis.png', path), dpi=dpi)

    plt.figure(figsize=(30, 4.8*3))
    plt.plot(times[:-1], floating_average_disturbance[:-1, 3:])
    plt.legend(['Mx', 'My', 'Mz'])
    plt.xlabel('Time [s]')
    plt.ylabel('Disturbance values [N] or [Nm]')
    plt.tight_layout()
    plt.savefig(filepath('turbulence_angle.png', path), dpi=dpi)

    delta_t = times[1:] - times[:-1]
    disturbance_integral = np.cumsum(disturbance, axis=0)[:-1] * delta_t[:, None]

    plt.figure(figsize=(30, 4.8*3))
    plt.plot(times[:-1], disturbance_integral)
    plt.legend(['a_x', 'a_y', 'a_z', 'Mx', 'My', 'Mz'])
    plt.xlabel('Time [s]')
    plt.ylabel('Disturbance values [N] or [Nm]')
    plt.tight_layout()
    plt.savefig(filepath('turbulence_integral.png', path), dpi=dpi)
    

    plt.close('all')


def simulate(times, initial_state, target, *, \
            flight_computer_args, flight_computer_kwargs, \
            model_args, model_kwargs):

    """
    Simulate the hexacopter dynamics and decision making

    Positional arguments:
    times:          array-like      time points for the simulation
    initial_state:  array-like      initial state of the hexacopter
    target:         array-like      x, y, psi: target position and final heading

    Keyword arguments:
    flight_computer_args        positional arguments for the flight computer
    flight_computer_kwargs      keyword arguments for the flight computer
    model_args                  positional arguments for the hexacopter model
    model_kwargs                keyword arguments for the hexacopter model
    """

    # Initialize objects
    flight_computer = FlightComputer(*flight_computer_args, **flight_computer_kwargs)
    model = HexacopterModel(*model_args, **model_kwargs)
    
    # Initialize histories
    states = [initial_state]
    thruster_values = []
    flight_mode_list = []
    set_points_array = np.zeros([len(times), 12])
    input_array = np.zeros([len(times), 4])
    disturbance_array = np.zeros([len(times), 6])

    for i in range(1, len(times)):
        current_state = states[-1]
        time_step = times[i] - times[i-1]

        # get inputs from flight plan and axis controller
        thruster_inputs, flight_mode, set_point, inputs = \
            flight_computer.get_flight(current_state, target, time_step)
        
        # simulate to next timestep
        simulation_args = (thruster_inputs, time_step)
        new_state, disturbance = solve_ivp(model.simulate_response, current_state, [0, time_step], args=simulation_args)
        new_state = new_state[-1]

        # save values for analysis
        states.append(new_state)
        thruster_values.append(thruster_inputs)
        flight_mode_list.append(flight_mode)
        set_points_array[i] = set_point
        input_array[i] = inputs
        disturbance_array[i] = disturbance

        # Check if the drone has crashed
        if new_state[2] < 0:
            model.crashed = True
            break
        
    times = times[:len(states)]
    set_points_array = set_points_array[:len(states)]
    flight_mode_list = flight_mode_list[:len(states)]
    input_array = input_array[:len(states)]
    disturbance_array = disturbance_array[:len(states)]
    
    thruster_values = thruster_values[:len(states)]
    
    return np.array(states), times, np.array(set_points_array), thruster_values, flight_mode_list, input_array, model.crashed, disturbance_array

def plot_figures_new(states, times, setpoints, thruster_values, flight_mode_list, input_array, disturbance, time_step=0.01, dpi=300, legend_fontsize=8):
    """
        Plot the simulation results, final version
    """
    path = ['ControlAnalysis', 'Figures']
    #plt.style.use('science')


    desired_x, desired_y, desired_z, desired_phi, desired_theta, desired_psi, \
    desired_xdot, desired_ydot, desired_zdot, desired_p, desired_q, desired_r = setpoints.T
    # clipping the setpoints to the length of the simulation
    desired_x = desired_x[:len(times)]
    desired_y = desired_y[:len(times)]
    desired_z = desired_z[:len(times)]
    desired_phi = desired_phi[:len(times)]
    desired_theta = desired_theta[:len(times)]
    desired_psi = desired_psi[:len(times)]
    desired_xdot = desired_xdot[:len(times)]
    desired_ydot = desired_ydot[:len(times)]
    desired_zdot = desired_zdot[:len(times)]
    desired_p = desired_p[:len(times)]
    desired_q = desired_q[:len(times)]
    desired_r = desired_r[:len(times)]

    plt.style.use('science')
    # Position plot
    plt.figure(figsize=(6, 4))
    plt.plot(times, states[:, 0], label=r'$x$')
    plt.plot(times, desired_x, 'r--', label=r'$x^*$')
    plt.plot(times, states[:, 1], label=r'$y$')
    plt.plot(times, desired_y, 'g--', label=r'$y^*$')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.legend(fontsize=legend_fontsize)
    plt.savefig(filepath('position_xy.png', path), dpi=dpi)
    

    plt.figure(figsize=(6, 4))
    r = np.linalg.norm(states[:, 0:2]-np.array([desired_x, desired_y]).T, axis=1)
    plt.plot(times, r, label='r')
    plt.plot(times, np.ones_like(times)*125, 'r--', label='Close range')
    plt.plot(times, np.ones_like(times)*5, 'b--', label='Land range')
    plt.plot(times, np.ones_like(times)*0.4, 'g--', label='Accuracy range')
    plt.text(times[-1], r[-1], f'r = {round(r[-1], 2)}', fontsize=7, ha='right', color='k', alpha=0.7)
    print(f'r = {round(r[-1], 2)}')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.yscale('log')
    plt.legend(fontsize=legend_fontsize)
    plt.savefig(filepath('landing_position.png', path), dpi=dpi)

    plt.figure(figsize=(6, 4))
    plt.plot(times, states[:, 2], label=r'$z$')
    plt.plot(times, desired_z, 'r--', label=r'$z^*$')
    plt.xlabel('Time [s]')
    plt.ylabel('Altitude [m]')
    plt.ylim([0, 150])
    plt.legend(fontsize=legend_fontsize)
    plt.savefig(filepath('position_z.png', path), dpi=dpi)

    plt.figure(figsize=(12, 8))
    plt.subplot(2, 1, 1)
    plt.plot(times, states[:, 0], label=r'$x$')
    plt.plot(times, desired_x, 'r--', label=r'$x^*$')
    plt.plot(times, states[:, 1], label=r'$y$')
    plt.plot(times, desired_y, 'g--', label=r'$y^*$')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.legend(fontsize=legend_fontsize)

    plt.subplot(2, 1, 2)
    plt.plot(times, states[:, 2], label=r'$z$')
    plt.plot(times, desired_z, 'r--', label=r'$z^*$')
    plt.xlabel('Time [s]')
    plt.ylabel('Altitude [m]')
    plt.ylim([0, 150])
    plt.legend(fontsize=legend_fontsize)
    plt.savefig(filepath('position_combined_xyz.png', path), dpi=dpi)

    # Attitude plot
    plt.figure(figsize=(12, 10))
    plt.subplot(3, 1, 1)
    plt.plot(times, states[:, 3], label=r'$\phi$')
    plt.plot(times, desired_phi, 'r--', label=r'$\phi ^*$')
    plt.xlabel('Time [s]')
    plt.ylabel('Roll [rad]')
    plt.legend(fontsize=legend_fontsize)

    plt.subplot(3, 1, 2)
    plt.plot(times, states[:, 4], label=r'$\theta$')
    plt.plot(times, desired_theta, 'r--', label=r'$\theta ^*$')
    plt.xlabel('Time [s]')
    plt.ylabel('Pitch [rad]')
    plt.legend(fontsize=legend_fontsize)

    plt.subplot(3, 1, 3)
    plt.plot(times, states[:, 5], label=r'$\psi$')
    plt.plot(times, desired_psi, 'r--', label=r'$\psi ^*$')
    plt.xlabel('Time [s]')
    plt.ylabel('Yaw [rad]')
    plt.legend(fontsize=legend_fontsize)
    plt.savefig(filepath('angles.png', path), dpi=dpi)

    # Velocity plot
    plt.figure(figsize=(12, 8))
    plt.subplot(2, 1, 1)
    plt.plot(times, states[:, 6], label=r'$\dot{x}$')
    plt.plot(times, desired_xdot, 'r--', label=r'$\dot{x}^*$')
    plt.plot(times, states[:, 7], label=r'$\dot{y}$')
    plt.plot(times, desired_ydot, 'g--', label=r'$\dot{y}^*$')
    plt.xlabel('Time [s]')
    plt.ylabel('Horizontal velocity [m/s]')
    plt.legend(fontsize=legend_fontsize)

    plt.subplot(2, 1, 2)
    plt.plot(times, states[:, 8], label=r'$\dot{z}$')
    plt.plot(times, desired_zdot, 'r--', label=r'$\dot{z}^*$')
    plt.xlabel('Time [s]')
    plt.ylabel('Altitude velocity [m/s]')
    plt.ylim([-20, 20])
    plt.legend(fontsize=legend_fontsize)

    plt.savefig(filepath('velocity.png', path), dpi=dpi)

    plt.figure(figsize=(6, 4))
    half_time = np.size(times)//2
    plt.plot(times[half_time:], states[half_time:, 6], label=r'$\dot{x}$')
    plt.plot(times[half_time:], desired_xdot[half_time:], 'r--', label=r'$\dot{x}^*$')
    plt.plot(times[half_time:], states[half_time:, 7], label=r'$\dot{y}$')
    plt.plot(times[half_time:], desired_ydot[half_time:], 'g--', label=r'$\dot{y}^*$')
    plt.xlabel('Time [s]')
    plt.ylabel('Horizontal velocity [m/s]')
    plt.legend(fontsize=legend_fontsize)
    plt.savefig(filepath('velocity_landing.png', path), dpi=dpi)


    # Attitude rate plot
    plt.figure(figsize=(12, 10))
    plt.subplot(3, 1, 1)
    plt.plot(times, states[:, 9], label=r'$p$')
    plt.plot(times, desired_p, 'r--', label=r'$p^*$')
    plt.xlabel('Time [s]')
    plt.ylabel('Roll rate [rad/s]')
    plt.ylim([-3, 3])
    plt.legend(fontsize=legend_fontsize)

    plt.subplot(3, 1, 2)
    plt.plot(times, states[:, 10], label=r'$q$')
    plt.plot(times, desired_q, 'r--', label=r'$q^*$')
    plt.xlabel('Time [s]')
    plt.ylabel('Pitch rate [rad/s]')
    plt.ylim([-3, 3])
    plt.legend(fontsize=legend_fontsize)

    plt.subplot(3, 1, 3)
    plt.plot(times, states[:, 11], label=r'$r$')
    plt.plot(times, desired_r, 'r--', label=r'$r^*$')
    plt.xlabel('Time [s]')
    plt.ylabel('Yaw rate [rad/s]')
    plt.ylim([-3, 3])
    plt.legend(fontsize=legend_fontsize)
    plt.tight_layout()
    plt.savefig(filepath('angular_velocity.png', path), dpi=dpi)

    # Thruster plot
    plt.figure(figsize=(6, 4))
    plt.plot(times[:-1], thruster_values)
    plt.legend(['1', '2', '3', '4', '5', '6'], fontsize=legend_fontsize)
    plt.xlabel('Time [s]')
    plt.ylabel('Thruster values [N]')
    plt.tight_layout()
    plt.savefig(filepath('thrusters.png', path), dpi=dpi)

    # Thruster plot
    plt.figure(figsize=(6, 4))
    plt.plot(times[:-1], input_array[:times.shape[0]-1])
    plt.legend([r'$T$', r'$M_x$', r'$M_y$', r'$M_z$'], fontsize=legend_fontsize)
    plt.xlabel('Time [s]')
    plt.ylabel('Thruster values [N]')
    plt.tight_layout()
    plt.savefig(filepath('inputs.png', path), dpi=dpi)

    # Flight mode plot
    flight_modes = sorted(list(set(flight_mode_list)))
    mode_to_index = {status: index for index, status in enumerate(flight_modes)}
    flight_mode_list_indexed = [mode_to_index[mode] for mode in flight_mode_list]

    mode_matrix = np.zeros((len(flight_modes), len(flight_mode_list)), dtype=int)
    mode_matrix[flight_mode_list_indexed, np.arange(len(flight_mode_list))] = 1

    plt.figure(figsize=(12, 5))
    for row_index, mode in enumerate(flight_modes):
        plt.plot(times[:-1], mode_matrix[row_index], linestyle='--', label=mode)
        plt.fill_between(times[:-1], mode_matrix[row_index], where=mode_matrix[row_index] == 1, 
                interpolate=True, alpha=0.3, hatch='//')
    plt.legend(fontsize=legend_fontsize)
    plt.xlabel('Time [s]')
    plt.ylabel('Active [True/False]')
    plt.tight_layout()
    plt.savefig(filepath('flight_mode.png', path), dpi=dpi)

    mpl.rcParams['agg.path.chunksize'] = 10000
    # Disturbance
    floating_average_disturbance = rolling_average(disturbance, int(3/time_step))
    print(disturbance.shape)

    plt.figure(figsize=(6, 4))
    plt.plot(times[:-1], floating_average_disturbance[:-1, :3])
    plt.legend([r'$a_x$', r'$a_y$', r'$a_z$'], fontsize=legend_fontsize)
    plt.xlabel('Time [s]')
    plt.ylabel('Disturbance values [N] or [Nm]')
    plt.tight_layout()
    plt.savefig(filepath('turbulence_axis.png', path), dpi=dpi)

    plt.figure(figsize=(6, 4))
    plt.plot(times[:-1], floating_average_disturbance[:-1, 3:])
    plt.legend([r'$M_x$', r'$M_y$', r'$M_z$'], fontsize=legend_fontsize)
    plt.xlabel('Time [s]')
    plt.ylabel('Disturbance values [N] or [Nm]')
    plt.tight_layout()
    plt.savefig(filepath('turbulence_angle.png', path), dpi=dpi)

    delta_t = times[1:] - times[:-1]
    disturbance_integral = np.cumsum(disturbance, axis=0)[:-1] * delta_t[:, None]

    plt.figure(figsize=(6, 4))
    plt.plot(times[:-1], disturbance_integral)
    plt.legend([r'$a_x$', r'$a_y$', r'$a_z$', r'$M_x$', r'$M_y$', r'$M_z$'], fontsize=legend_fontsize)
    plt.xlabel('Time [s]')
    plt.ylabel('Disturbance values [N] or [Nm]')
    plt.tight_layout()
    plt.savefig(filepath('turbulence_integral.png', path), dpi=dpi)

    plt.close('all')


