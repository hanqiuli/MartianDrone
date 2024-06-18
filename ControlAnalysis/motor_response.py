import matplotlib.pyplot as plt
import numpy as np
import time

class MotorResponse:
    def __init__(self, natural_frequency, damping_ratio=1.0):
        """
        Initialize the SecondOrderCriticallyDampedSystem class.

        :param damping_ratio: The damping ratio of the system.
        :param natural_frequency: The natural frequency of the system.
        """
        self.damping_ratio = damping_ratio
        self.natural_frequency = natural_frequency
        self.output = 37.0  # Initial output value
        self.o_dot = 0.0  # Initial output velocity

    def get_actual_torque(self, input_torque, dt):
        """
        Update the system based on the input force.

        :param input_force: The current input force value.
        :param dt: The timestep for the simulation.
        :return: The updated output value.
        """
        # Euler step for output velocity
        if 1/(2*dt) < self.natural_frequency:
            raise ValueError("Time step too large")
        
        self.o_dot += dt * (input_torque * self.natural_frequency**2 
                            - 2 * self.damping_ratio * self.natural_frequency * self.o_dot 
                            - self.natural_frequency ** 2 * self.output)
        # Euler step for output
        self.output += dt * self.o_dot
        return self.output


if __name__ == "__main__":
    freq = 15
    damping = 1.05
    response = MotorResponse(freq, damping)
    response.output = 0.0

    dt = 0.00001
    times = np.arange(0, 1, dt)
    input_torques = 100*np.ones_like(times)

    actual_torques = np.zeros_like(times)
    t_prev = 0
    for i, t in enumerate(times):
        delta_t = t-t_prev
        t_prev = t
        actual_torques[i] = response.get_actual_torque(input_torques[i], delta_t)


    plt.figure()
    plt.plot(times, actual_torques, label='actual torque')
    plt.plot(times, input_torques, 'r--', label='desired torque')
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.legend()
    plt.show()