# AWG properties https://www.calmont.com/wp-content/uploads/calmont-eng-wire-gauge.pdf 
# Standard libraries
import math
import random
import os

# Third party libraries
import numpy as np
import matplotlib.pyplot as plt
import scipy as sp

# Local imports
# from environment_properties import ENV
time_mars_day = 88775 #Average Martian day duration [sec]

#Inputs
#Power usage data
power_max_motor = 1000 #Max wattage of motors [W]
voltage_motors = 100 #Max power operating voltage of motors [V]

#Subsystem design currents
max_current_motor = power_max_motor/voltage_motors
max_current_payload = 0
max_current_communication = 0
max_current_TCS = 0

#Subsystems relative position from battery
cable_length_motors = 2.8 #Length of propeller arms [m]
cable_length_payload = 0
cable_length_communication = 0
cable_length_TCS = 0

#Cable properties 
#https://metersuk.co.uk/american-wire-gauge-awg-cable-conductor-sizes/ 
#https://www.calmont.com/resources/wire-gauge-table/ 
#Specific masses provided in kg/m
AWG_10_specific_mass = 49/1000
AWG_12_specific_mass = 29.8/1000
AWG_14_specific_mass = 19.1/1000
AWG_16_specific_mass = 11.6/1000
AWG_14_resistance = 8.54/1000 #ohm/m

#Calculate cable mass per subsystem
#Cable_mass = length * specific_mass
cable_mass_per_motor_single_phase = cable_length_motors*AWG_10_specific_mass
cable_mass_per_motor_double_phase = cable_length_motors*AWG_14_specific_mass*2
cable_mass_propulsion = 2*6*cable_mass_per_motor_double_phase #6 motors, one set of redudancy each
print(cable_mass_propulsion)

#Calculate cable loss per subsystem
#Ploss = R * I^2
power_loss_per_motor = (AWG_14_resistance * cable_length_motors) * (power_max_motor/voltage_motors/2)**2
power_loss_percentage = (power_max_motor-power_loss_per_motor)/power_max_motor #Power loss propulsion as a percentage of total input power [%]
print(power_loss_percentage)

#Output
wires_mass = 0
def get_wires_mass():
    wires_mass = cable_mass_propulsion
    return wires_mass

cable_loss = 0
def get_cable_loss():
    cable_loss = power_loss_percentage
    return cable_loss