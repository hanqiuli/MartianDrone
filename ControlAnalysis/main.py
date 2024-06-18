import sys
import os
import random as rand
import math

import numpy as np

sys.path.append('.')
from ControlAnalysis.helper_funcs import find_highest_index, plot_figures, plot_figures_new, simulate, filepath


if __name__ == "__main__":
    do_data_analysis = False
    skip_found_files = True

    time_step = 0.01  # [s]
    time_end = 180    # [s]

    sensitivity_iter_max = 1000

    # Target position: x, y, psi(heading)
    target = np.array([200, 300, 1.7])
    #target = np.array([0, 0, 0])
    print(f"distance = {np.linalg.norm(target[0:2])}")

    # True mass constants
    mass= 48.67162169
    moment_inertia = np.diag([32.209, 32.209, 63.639])
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
        [1.4, 0.13, 2.2, 0.3], # Phi
        [1.4, 0.13, 2.2, 0.3], # Theta
        [0.5, 0.1, 1.3, 0.6], # Psi

        # Positions -> Angles
        [0.3, 0.0002, 0.35, 5, precision_speed],   # X
        [0.3, 0.0002, 0.35, 5, precision_speed],   # Y
        [0.7, 0.3, 3.0, 2, 10],   # exception in Z: Position -> thrust

        # Velocities -> Angles
        [0.1, 0.009, 0.2, 7, max_angle],   # Velocity X
        [-0.1, -0.009, -0.2, 7, max_angle],   # Velocity Y
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

    print('Plotting final figures...')
    plot_figures_new(states, times, setpoints, thruster_values, flight_mode_list, input_array, disturbance, legend_fontsize=10)

    

    if do_data_analysis:
        pass
    else:
        quit()

    # sensitivity analysis
    data_path = ['ControlAnalysis', 'Data']

    i_max = 5*math.floor(sensitivity_iter_max/5)

    if skip_found_files:
        try:
            start_i = find_highest_index(os.path.join(*data_path), 'flight_data_', '.npz')+1
        except FileNotFoundError as e:
            print(e)
            start_i = 0
    else:
        start_i = 0
    
    print('Accuracy analysis...')
    for i in range(start_i, start_i+i_max):
        if i >= sensitivity_iter_max:
            print('Sensitivity analysis complete...')
            break

        print(f'Iteration {i}: {round((i-start_i) / (i_max-start_i), 4)}')

        times = np.arange(0, time_end + time_step, time_step)        
    
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
        mass=mass, skew_factor=skew_factor, thrust=thrust_to_weight_range)

