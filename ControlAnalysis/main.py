import matplotlib.pyplot as plt
import numpy as np


from hexacopter_model import HexacopterModel
from hexacopter_flight_computer import FlightComputer


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
    flight_computer = FlightComputer(*flight_computer_args, **flight_computer_kwargs)
    model = HexacopterModel(*model_args, **model_kwargs)
    
    states = [initial_state]
    thruster_values = []
    flight_mode_list = []

    for i in range(1, len(times)):
        current_state = states[-1]
        time_step = times[i] - times[i-1]

        # get inputs from flight plan and axis controller
        thruster_inputs, flight_mode = flight_computer.get_flight(current_state, \
            [setpoint[i] for setpoint in setpoints] ,time_step)
        
        # simulate to next timestep
        inputs = tuple([thruster_inputs],)
        new_state = odeint(self.hexacopter_dynamics, current_state, [0, time_step], args=inputs)[-1]

        # save values for analysis
        states.append(new_state)
        thruster_values.append(thruster_inputs)
        flight_mode_list.append(flight_mode)

        # Check if the drone has crashed
        if new_state[2] < 0:
            hexacopter.crashed = True
            break
        
    times = times[:len(states)]
    
    return np.array(states), times, thruster_values, flight_mode_list


def plot_figures(states, times, setpoints, thruster_values, flight_mode_list):

    """
        Plot the simulation results
    """

    desired_x, desired_y, desired_z, desired_phi, desired_theta, desired_psi, \
    desired_xdot, desired_ydot, desired_zdot, desired_p, desired_q, desired_r = setpoints
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
    plt.subplot(2, 1, 1)
    plt.plot(times, states[:, 0], label='x')
    plt.plot(times, desired_x, 'r--', label='desired x')
    plt.plot(times, states[:, 1], label='y')
    plt.plot(times, desired_y, 'g--', label='desired y')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.ylim([-100, 100])
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(times, states[:, 2], label='z')
    plt.plot(times, desired_z, 'r--', label='desired z')
    plt.xlabel('Time [s]')
    plt.ylabel('Altitude [m]')
    plt.ylim([-20, 100])
    plt.legend()

    plt.tight_layout()
    plt.savefig('ControlAnalysis/figures/position.png', dpi=300)

    # Attitude plot
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(times, states[:, 3], label='phi')
    plt.plot(times, desired_phi, 'r--', label='desired phi')
    plt.xlabel('Time [s]')
    plt.ylabel('Roll [rad]')
    plt.ylim([-3, 3])
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(times, states[:, 4], label='theta')
    plt.plot(times, desired_theta, 'r--', label='desired theta')
    plt.xlabel('Time [s]')
    plt.ylabel('Pitch [rad]')
    plt.ylim([-3, 3])
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(times, states[:, 5], label='psi')
    plt.plot(times, desired_psi, 'r--', label='desired psi')
    plt.xlabel('Time [s]')
    plt.ylabel('Yaw [rad]')
    plt.ylim([-3, 3])
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
    plt.ylim([-10, 10])
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(times, states[:, 8], label='z_dot')
    plt.plot(times, desired_zdot, 'r--', label='desired z_dot')
    plt.xlabel('Time [s]')
    plt.ylabel('Altitude velocity [m/s]')
    plt.ylim([-10, 10])

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
    plt.figure(figsize=(20, 4.8))
    plt.plot(times[:-1], thruster_values)
    plt.legend(['1', '2', '3', '4', '5', '6'])
    plt.xlabel('Time [s]')
    plt.ylabel('Thruster values [N]')
    plt.tight_layout()
    plt.savefig('ControlAnalysis/figures/thrusters.png', dpi=1200)

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
    plt.savefig('ControlAnalysis/figures/flight_mode.png', dpi=1200)


    plt.close('all')




if __name__ == "__main__":

    mass = 60.0
    moment_inertia = np.diag([5, 5, 8])
    moment_inertia_prop = 0.01
    pid_params = [[20, 2, 20, 3, 10], [6, 0.3, 4, 10], [6, 0.3, 4, 10], [8, 0.4, 16, 10]]
    torque_thrust_ratio = 0.1
    omega_thrust_ratio = 0.1

    thrust_to_weight_range = [0.6, 1.3]

    hexacopter = HexacopterModel(mass, moment_inertia, moment_inertia_prop, pid_params, \
        propellor_power_coefficient=0.5, propellor_radius=1.3, propellor_thrust_coefficient=1.2, thrust_to_weight_range=thrust_to_weight_range)

    delta_t = 0.02
    t = np.arange(0, 350, delta_t)

    initial_state = np.zeros(12)
    initial_state[2] = 5

    desired_x = np.ones_like(t)*150
    desired_y = np.ones_like(t)*100

    desired_z = np.zeros_like(t)
    desired_z[t <= 5] = np.linspace(0, 20, len(t[t <= 5]))
    desired_z[(t > 5) & (t <= 10)] = 20
    desired_z[(t > 10) & (t <= 30)] = 30
    desired_z[(t > 30) & (t <= 42)] = 20
    desired_z[(t > 42) & (t <= 60)] = 60
    desired_z[(t > 60) & (t <= 250)] = np.sin(t[(t > 60) & (t <= 250)]*np.pi/10)*5 + 50
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

    desired_states = [desired_x, desired_y, desired_z, desired_phi, desired_theta, desired_psi, \
         desired_xdot, desired_ydot, desired_zdot, desired_p, desired_q, desired_r]


    states, times, thruster_values, flight_mode_list = hexacopter.simulate(t, initial_state, desired_states)
    hexacopter.plot_figures(states, times, desired_states, thruster_values, flight_mode_list)