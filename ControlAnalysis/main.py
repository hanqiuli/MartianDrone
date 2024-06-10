import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp # type: ignore

sys.path.append('.')
from hexacopter_model import HexacopterModel
from hexacopter_flight_computer import FlightComputer


def plot_figures(states, times, setpoints, thruster_values, flight_mode_list, input_array):

    """
        Plot the simulation results
    """

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
    # text with opacity of 0.7
    plt.text(times[-1], r[-1], f'r = {round(r[-1], 2)}', fontsize=7, ha='right', color='k', alpha=0.7)
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
    plt.savefig('ControlAnalysis/figures/position.png', dpi=300)

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
    plt.savefig('ControlAnalysis/figures/angles.png', dpi=300)

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
    plt.savefig('ControlAnalysis/figures/velocity.png', dpi=300)

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
    plt.savefig('ControlAnalysis/figures/angular_velocity.png', dpi=300)

    # Thruster plot
    plt.figure(figsize=(30, 4.8*1.5))
    plt.plot(times[:-1], thruster_values)
    plt.legend(['1', '2', '3', '4', '5', '6'])
    plt.xlabel('Time [s]')
    plt.ylabel('Thruster values [N]')
    plt.tight_layout()
    plt.savefig('ControlAnalysis/figures/thrusters.png', dpi=300)

    # Thruster plot
    plt.figure(figsize=(30, 4.8*3))
    plt.plot(times[:-1], input_array[:times.shape[0]-1])
    plt.legend(['T', 'Mx', 'My', 'Mz'])
    plt.xlabel('Time [s]')
    plt.ylabel('Thruster values [N]')
    plt.tight_layout()
    plt.savefig('ControlAnalysis/figures/inputs.png', dpi=300)

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
    plt.savefig('ControlAnalysis/figures/flight_mode.png', dpi=600)


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

    for i in range(1, len(times)):
        current_state = states[-1]
        time_step = times[i] - times[i-1]

        # get inputs from flight plan and axis controller
        thruster_inputs, flight_mode, set_point, inputs = \
            flight_computer.get_flight(current_state, target, time_step)
        
        # simulate to next timestep
        simulation_args = (thruster_inputs, time_step)
        new_state = solve_ivp(model.simulate_response,  [0, time_step], current_state, args=simulation_args, method='RK23').y[:, -1]

        # save values for analysis
        states.append(new_state)
        thruster_values.append(thruster_inputs)
        flight_mode_list.append(flight_mode)
        set_points_array[i] = set_point
        input_array[i] = inputs

        # Check if the drone has crashed
        if new_state[2] < 0:
            model.crashed = True
            break
        
    times = times[:len(states)]
    
    return np.array(states), times, np.array(set_points_array), thruster_values, flight_mode_list, input_array



if __name__ == "__main__":
    time_step = 0.01

    # Target position: x, y, psi(heading)
    target = np.array([1237, 900, 1.7])

    # Define constants
    mass = 60.0
    moment_inertia = np.diag([5, 5, 8])
    moment_inertia_prop = 0.01
    
    # PID_Params given in the following structure: 
    # [P, I, D, wind_up_limit, clip_limit]
    
    max_angle = 30 * np.pi/180
    precision_speed = 0.3
    pid_params = [
        # Angles -> Thruster_inputs
        [0.4, 0.05, 0.8, 3], # Phi
        [0.4, 0.05, 0.8, 3], # Theta
        [0.5, 0.1, 0.3, 3], # Psi
        # Positions -> Angles
        [0.18, 0.0005, 0.6, 1, precision_speed],   # X
        [0.18, 0.0005, 0.6, 1, precision_speed],   # Y
        [1.5, 0.15, 5, 3, 10],   # exception in Z: Position -> thrust
        # Velocities -> Angles
        [3, 0.008, 5, 10, max_angle],   # Velocity X
        [-3, -0.008, -5, 10, max_angle],   # Velocity Y
        [10, 0.016, 4, 20],   # exception in Z: Velocity -> thrust
        ]

    thrust_to_weight_range = [0.7, 1.3]
    skew_factor = 1.0
    # estimated_mass, pid_params
    flight_controller_args = [
        mass*skew_factor,
        pid_params
    ]

    arm_length = 2
    propellor_thrust_coefficient = 0.02
    propellor_power_coefficient = 0.0032
    propellor_radius = 1.3

    # arm_length, ENV, thrust_to_weight_range, propellor_ct, propellor_cq, propellor_r
    flight_controller_kwargs = {
        'arm_length': arm_length,
        'thrust_to_weight_range': thrust_to_weight_range,
        'propellor_thrust_coefficient': propellor_thrust_coefficient,
        'propellor_power_coefficient': propellor_power_coefficient,
        'propellor_radius': propellor_radius,
    }

    speed_target = 30
    speed_closing = 3

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

    motor_natural_frequency = 15
    motor_damping = 1.05

    model_kwargs = {
        'propellor_thrust_coefficient': propellor_thrust_coefficient,
        'propellor_power_coefficient': propellor_power_coefficient,
        'propellor_radius': propellor_radius,
        'thrust_to_weight_range': thrust_to_weight_range,
        'arm_length': arm_length,
        'motor_natural_frequency': motor_natural_frequency,
        'motor_damping': motor_damping
    }

    # Define time, initial values and targets
    time_end = 1200 #s
    times = np.arange(0, time_end + time_step, time_step)

    initial_state = np.zeros(12)
    initial_state[2] = 5

    
    
    # Simulate
    print('Simulating...')
    states, times, setpoints, thruster_values, flight_mode_list, input_array = simulate(times, initial_state, target, \
                                                    flight_computer_args=flight_computer_args, \
                                                    flight_computer_kwargs=flight_computer_kwargs,\
                                                    model_args=model_args, model_kwargs=model_kwargs)

    print('Plotting...')
    # Plot
    plot_figures(states, times, setpoints, thruster_values, flight_mode_list, input_array)
    
    # Speed target
    

    # Flight controller arguments
    # estimated_mass, pid_params, *, arm_length=2, ENV=None, thrust_to_weight_range=None, \
    # propellor_thrust_coefficient=None, propellor_power_coefficient=None, propellor_radius=None

    # Flight computer arguments
    # self, speed_target, *, FlightController_args, FlightController_kwargs

    # Model arguments
    # mass, moment_inertia, moment_inertia_prop, *, \
    # propellor_thrust_coefficient, propellor_power_coefficient, propellor_radius, \
    # thrust_to_weight_range=None, ENV=None, arm_length=2, motor_lag_term_time_constant=0.1