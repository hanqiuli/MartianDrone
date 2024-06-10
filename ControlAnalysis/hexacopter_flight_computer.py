import sys

import numpy as np

sys.path.append('.')
from hexacopter_flight_controller import FlightController


def interp(x, a, b, y_a, y_b):
    factor = 2 / (b-a)**3 * (x-a)**3 \
            - 3  /  (b-a)**2 * (x-a)**2 \
            + 1
    
    factor = np.where(x < a, 1, factor)
    factor = np.where(x > b, 0, factor)

    return factor * (y_a - y_b) + y_b

class FlightComputer:
    def __init__(self, speed_target, speed_closing, *, FlightController_args, FlightController_kwargs):
        self.safe_altitude = 100
        self.default_heading = -2.4
        self.speed_target = speed_target
        self.speed_closing = speed_closing

        self.land_speed_low = -0.5
        self.land_speed_high = -5

        self.close_radius = 125
        self.land_radius = 5

        self.static_time = 0

        self.flight_controller = FlightController(*FlightController_args, **FlightController_kwargs)

    def get_flight_mode(self, state, target):

        x, y, z, phi, theta, psi, xdot, ydot, zdot, p, q, r = state
        target_x, target_y = target[0:2]

        max_allowed_angle = 45 * np.pi/180
        delta_x = target_x-x
        delta_y = target_y-y
        velocity_magnitude = np.linalg.norm([xdot, ydot])
        proximity = np.linalg.norm([delta_x, delta_y])

        # Attitude recovery
        if abs(phi) > max_allowed_angle or abs(theta) > max_allowed_angle:
            self.static_time = max(10.0, self.static_time)
            return ['attitude', 2]  # axis weight set 2
        
        # Landing
        if (proximity < self.land_radius) and (velocity_magnitude < 1):
            return ['land', 1]
            
        # Unexpectedly low/climbing
        if z < 40:
            self.static_time = max(20.0, self.static_time)
            return ['climb', 1]

        # Altitude recovery
        if z < self.safe_altitude - 20:
            return ['altitude', 1]
        
        # Stopping
        if (proximity < self.close_radius):
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
            r = float(np.linalg.norm(delta_pos))
            
            b = self.close_radius*2
            a = self.close_radius
            
            speed = interp(r, a, b, self.speed_closing, self.speed_target)

            x_dot, y_dot = delta_pos/r * speed

            set_point = [*target[0:2], self.safe_altitude, None, None, self.default_heading, \
                         x_dot, y_dot, None, None, None, None]
            control_loop_index = 1

        elif mode == 'land':
            target_vertical_speed = self.land_speed_low if state[2] < 40 else self.land_speed_high
            set_point = [*target[0:2], 0, None, None, target[2], \
                         None, None, target_vertical_speed, None, None, None]
            control_loop_index = 3

        elif mode == 'close':
            delta_pos = target[0:2] - state[0:2]
            r = float(np.linalg.norm(delta_pos))
            x_dot, y_dot = delta_pos/r * self.speed_closing
            
            b = self.close_radius
            a = min(self.land_radius*5, self.close_radius/2)

            psi = interp(r, a, b, target[2], self.default_heading)

            set_point = [*target[0:2], self.safe_altitude, None, None, psi, \
                         x_dot, y_dot, None, None, None, None]
            control_loop_index = 1

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

        thruster_inputs, set_point, inputs = self.flight_controller.get_control(state, set_point, weight_set, control_loop_index, dt)

        self.static_time = max(self.static_time-dt, 0)

        return thruster_inputs, flight_mode, set_point, inputs


