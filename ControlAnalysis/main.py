import sys
import os
import random as rand
import re
import math

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

sys.path.append('.')
from hexacopter_model import HexacopterModel
from hexacopter_flight_computer import FlightComputer


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


def plot_figures(states, times, setpoints, thruster_values, flight_mode_list, input_array, disturbance):

    """
        Plot the simulation results
    """
    path = ['ControlAnalysis', 'Figures']


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
    plt.plot(times, np.ones_like(times)*0.8, 'g--', label='accuracy range')
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
    plt.savefig(filepath('position.png', path), dpi=300)

    # Attitude plot
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(times, states[:, 3], label='phi')
    #plt.plot(times, desired_phi, 'r--', label='desired phi')
    plt.xlabel('Time [s]')
    plt.ylabel('Roll [rad]')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(times, states[:, 4], label='theta')
    #plt.plot(times, desired_theta, 'r--', label='desired theta')
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
    plt.savefig(filepath('angles.png', path), dpi=300)

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
    plt.savefig(filepath('velocity.png', path), dpi=300)

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
    plt.savefig(filepath('angular_velocity.png', path), dpi=300)

    # Thruster plot
    plt.figure(figsize=(30, 4.8*1.5))
    plt.plot(times[:-1], thruster_values)
    plt.legend(['1', '2', '3', '4', '5', '6'])
    plt.xlabel('Time [s]')
    plt.ylabel('Thruster values [N]')
    plt.tight_layout()
    plt.savefig(filepath('thrusters.png', path), dpi=300)

    # Thruster plot
    plt.figure(figsize=(30, 4.8*3))
    plt.plot(times[:-1], input_array[:times.shape[0]-1])
    plt.legend(['T', 'Mx', 'My', 'Mz'])
    plt.xlabel('Time [s]')
    plt.ylabel('Thruster values [N]')
    plt.tight_layout()
    plt.savefig(filepath('inputs.png', path), dpi=300)

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
    plt.savefig(filepath('flight_mode.png', path), dpi=600)

    mpl.rcParams['agg.path.chunksize'] = 10000
    # Disturbance
    plt.figure(figsize=(30, 4.8*3))
    plt.plot(times[:1000], disturbance[:1000, :])
    plt.legend(['a_x', 'a_y', 'a_z', 'Mx', 'My', 'Mz'])
    plt.xlabel('Time [s]')
    plt.ylabel('Disturbance values [N] or [Nm]')
    plt.tight_layout()
    plt.savefig(filepath('turbulence.png', path), dpi=300)

    delta_t = times[1:] - times[:-1]
    disturbance_integral = np.cumsum(disturbance, axis=0)[:-1] * delta_t[:, None]

    plt.figure(figsize=(30, 4.8*3))
    plt.plot(times[:1000], disturbance_integral[:1000, :])
    plt.legend(['a_x', 'a_y', 'a_z', 'Mx', 'My', 'Mz'])
    plt.xlabel('Time [s]')
    plt.ylabel('Disturbance values [N] or [Nm]')
    plt.tight_layout()
    plt.savefig(filepath('turbulence_integral.png', path), dpi=300)
    

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



if __name__ == "__main__":
    time_step = 0.001   # [s]
    time_end = 2     # [s]

    sensitivity_iter_max = 600

    # Target position: x, y, psi(heading)
    target = np.array([200, 300, 1.7])
    print(f"distance = {np.linalg.norm(target[0:2])}")

    # True mass constants
    mass= 60.0    # point to vary around in the sensitivy analysis
    moment_inertia = np.diag([5, 5, 8])
    moment_inertia_prop = 0.096

    thrust_to_weight_range = [0.7, 1.3]
    skew_factor = 1.0
    
    speed_target = 30
    speed_closing = 3
    precision_speed = speed_closing/10

    max_angle = 30 * np.pi/180
    # PID_Params given in the following structure: 
    # [P, I, D, wind_up_limit, clip_limit]
    pid_params = [
        # Angles -> Thruster_inputs
        [0.4, 0.05, 0.8, 3], # Phi
        [0.4, 0.05, 0.8, 3], # Theta
        [0.5, 0.1, 0.3, 3], # Psi
        # Positions -> Angles
        [0.2, 0.001, 1.7, 1, precision_speed],   # X
        [0.2, 0.001, 1.7, 1, precision_speed],   # Y
        [1.7, 0.2, 6, 2, 10],   # exception in Z: Position -> thrust
        # Velocities -> Angles
        [3, 0.008, 5, 10, max_angle],   # Velocity X
        [-3, -0.008, -5, 10, max_angle],   # Velocity Y
        [10, 0.016, 4, 20],   # exception in Z: Velocity -> thrust
        ]

    arm_length = 2
    propellor_thrust_coefficient = 0.02
    propellor_power_coefficient = 0.0032
    propellor_radius = 1.3

    motor_natural_frequency = 15
    motor_damping = 1.05

    # Class parameters
    flight_controller_args = [
        mass*skew_factor,
        pid_params
    ]

    flight_controller_kwargs = {
        'arm_length': arm_length,
        'thrust_to_weight_range': thrust_to_weight_range,
        'propellor_thrust_coefficient': propellor_thrust_coefficient,
        'propellor_power_coefficient': propellor_power_coefficient,
        'propellor_radius': propellor_radius,
    }

    flight_computer_args = [
        speed_target,
        speed_closing
    ]

    flight_computer_kwargs = {
        'FlightController_args': flight_controller_args,
        'FlightController_kwargs': flight_controller_kwargs,
    }

    model_args = [
        mass,
        moment_inertia,
        moment_inertia_prop
    ]

    model_kwargs = {
        'propellor_thrust_coefficient': propellor_thrust_coefficient,
        'propellor_power_coefficient': propellor_power_coefficient,
        'propellor_radius': propellor_radius,
        'thrust_to_weight_range': thrust_to_weight_range,
        'arm_length': arm_length,
        'motor_natural_frequency': motor_natural_frequency,
        'motor_damping': motor_damping
    }

    times = np.arange(0, time_end + time_step, time_step)

    initial_state = np.zeros(12)
    initial_state[2] = 1

    
    
    # Simulate
    print('Simulating...')
    states, times, setpoints, \
    thruster_values, flight_mode_list, \
    input_array, crashed, disturbance = simulate(times, initial_state, target,\
                                                    flight_computer_args=flight_computer_args, \
                                                    flight_computer_kwargs=flight_computer_kwargs,\
                                                    model_args=model_args, model_kwargs=model_kwargs)

    print('Plotting...')
    plot_figures(states, times, setpoints, thruster_values, flight_mode_list, input_array, disturbance)
    
    
    # Mass
    mass_varied = mass
    # Thrust
    thrust_varied = thrust_to_weight_range
    # Mass skew factor
    skew_factor_varied = 1

    # sensitivity analysis
    data_path = ['ControlAnalysis', 'Data']

    i_max = 4*math.floor(sensitivity_iter_max/4)


    try:
        start_i = find_highest_index(os.path.join(*data_path), 'flight_data_', '.npz')+1
    except FileNotFoundError as e:
        print(e)
        start_i = 0
    
    print('Sensitivity analysis...')
    for i in range(start_i, start_i+i_max):
        if i >= 400:
            print('Sensitivity analysis complete...')
            break

        print(f'Iteration {i}: {round((i-start_i) / i_max, 4)}')

        times = np.arange(0, time_end + time_step, time_step)

        # Mass
        if 0 <= i < i_max//4 or i >= i_max//4*3:
            mass_varied = rand.uniform(0.9, 1.1) * mass
        else:
            mass_varied = mass
        # Thrust
        if i_max//4 <= i < 2*i_max//4 or i >= i_max//4*3:
            thrust_varied = np.random.uniform(0.9, 1.1, 2) * thrust_to_weight_range
        else:
            thrust_varied = thrust_to_weight_range
        # Mass skew factor
        if 2*i_max//4 <= i < 3*i_max//4 or i >= i_max//4*3:
            skew_factor_varied = rand.uniform(0.95, 1.05)
        else:
            skew_factor_varied = 1

        

        # Class parameters
        flight_controller_args = [
            mass_varied*skew_factor_varied,
            pid_params
        ]

        flight_controller_kwargs = {
            'arm_length': arm_length,
            'thrust_to_weight_range': thrust_varied,
            'propellor_thrust_coefficient': propellor_thrust_coefficient,
            'propellor_power_coefficient': propellor_power_coefficient,
            'propellor_radius': propellor_radius,
        }

        flight_computer_args = [
            speed_target,
            speed_closing
        ]

        flight_computer_kwargs = {
            'FlightController_args': flight_controller_args,
            'FlightController_kwargs': flight_controller_kwargs,
        }

        model_args = [
            mass_varied,
            moment_inertia,
            moment_inertia_prop
        ]

        model_kwargs = {
            'propellor_thrust_coefficient': propellor_thrust_coefficient,
            'propellor_power_coefficient': propellor_power_coefficient,
            'propellor_radius': propellor_radius,
            'thrust_to_weight_range': thrust_varied,
            'arm_length': arm_length,
            'motor_natural_frequency': motor_natural_frequency,
            'motor_damping': motor_damping
        }
        

        states, times, setpoints, \
        thruster_values, flight_mode_list, \
        input_array, crashed, disturbance = simulate(times, initial_state, target,\
                                                    flight_computer_args=flight_computer_args, \
                                                    flight_computer_kwargs=flight_computer_kwargs,\
                                                    model_args=model_args, model_kwargs=model_kwargs)
        

        flight_modes = ['nominal', 'close', 'land', 'climb', 'altitude', 'attitude']
        mode_to_index = {status: index for index, status in enumerate(flight_modes)}

        flight_mode_list_indexed = [mode_to_index[mode] for mode in flight_mode_list]
        
        np.savez(filepath(f'flight_data_{i}.npz', data_path), 
        states=states, times=times, setpoints=setpoints, thruster_values=thruster_values, 
        flight_mode_list=flight_mode_list_indexed, input_array=input_array, crashed=crashed,
        mass=mass_varied, skew_factor=skew_factor_varied, thrust=thrust_varied)

