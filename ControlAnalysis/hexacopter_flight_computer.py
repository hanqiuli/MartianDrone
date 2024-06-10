import sys

import numpy as np

sys.path.append('.')
from hexacopter_flight_controller import FlightController


class FlightComputer:
    def __init__(self, speed_target, *, FlightController_args, FlightController_kwargs):
        self.safe_altitude = 100
        self.default_heading = 0
        self.speed_target = speed_target

        self.land_speed_low = -2
        self.land_speed_high = -10

        self.static_time = 0

        self.flight_controller = FlightController(*FlightController_args, **FlightController_kwargs)

    def get_flight_mode(self, state, target):

        x, y, z, phi, theta, psi, xdot, ydot, zdot, p, q, r = state
        target_x, target_y = target[0:2]

        max_allowed_angle = 45 * np.pi/180
        delta_x = target_x-x
        delta_y = target_y-y
        velocity_magnitude = np.linalg.norm([xdot, ydot])
        landing_radius = 2
        close_radius = 40
        proximity = np.linalg.norm([delta_x, delta_y])

        # Attitude recovery
        if abs(phi) > max_allowed_angle or abs(theta) > max_allowed_angle:
            self.static_time = 10.0
            return ['attitude', 2]  # axis weight set 2
        
        # Landing
        if (proximity < landing_radius) and (velocity_magnitude < 1):
            return ['land', 1]
            
        # Unexpectedly low/climbing
        if z < 40:
            self.static_time = 20.0
            return ['climb', 1]

        # Altitude recovery
        if z < self.safe_altitude - 20:
            return ['altitude', 1]
        
        # Stopping
        if (proximity < close_radius):
            return ['close', 0]

        # Nominal flight
        return ['nominal', 0]

    def get_flight_configuration(self, mode, state, target):
        ''' Returns the PID loop configuration and setpoint to be used'''

        # Static mode requested
        if self.static_time > 0:
            set_point = [*target[0:2], self.safe_altitude, 0, 0, self.default_heading, \
                         None, None, None, None, None, None]
            control_loop_index = 0
            
        # Nomimal mode
        elif mode == 'nominal' or mode == 'altitude':
            delta_pos = target[0:2] - state[0:2]
            x_dot, y_dot = delta_pos/np.linalg.norm(delta_pos) * self.speed_target

            set_point = [*target, self.safe_altitude, None, None, self.default_heading, \
                         x_dot, y_dot, None, None, None, None]
            control_loop_index = 1

        elif mode == 'land':
            target_vertical_speed = self.land_speed_low if state[2] < 40 else self.land_speed_high
            set_point = [*target[0:2], None, None, None, target[2], \
                         None, None, target_vertical_speed, None, None, None]
            control_loop_index = 3

        elif mode == 'close':
            set_point = [*target[0:2], None, None, None, target[2], \
                         None, None, None, None, None, None]
            control_loop_index = 2

        else:
            raise NotImplementedError(f'Flight mode {mode} not implemented')
        
        set_point = np.array(set_point)
        return control_loop_index, set_point
            #land alt close

    # Final flight computer logic
    def get_flight(self, state, target, dt):
        '''Gets the flight control inputs and flight mode'''
        state = np.array(state)
        target = np.array(target)

        flight_mode, weight_set = self.get_flight_mode(state, target)
        control_loop_index, set_point = self.get_flight_configuration(flight_mode, state, target)

        thruster_inputs, set_point = self.flight_controller.get_control(state, set_point, weight_set, control_loop_index, dt)

        self.static_time = max(self.static_time-dt, 0)

        return thruster_inputs, flight_mode, set_point


