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
from power_load_profile import profile_power
power_max_motor = profile_power['propulsive_takeoff']*1.4/6 #Max wattage of motors [W], SF 1.4 (ie. max power 7200) to account for T/W 1.5
voltage_motors = 60 #Max power operating voltage of motors [V]
SF_mass = 1.5 #Only the cable mass for the motors was calculated, to account for the total harness (cables from array and battery to PMC, cables from PMC to all other power users) a SF of 1.5 is used for mass calculation
SF_loss = 1.5 #SF on cable loss, we only sized from battery to motors so we need to add loss from power supply to battery

#Subsystem design currents
max_current_motor = power_max_motor/voltage_motors
max_current_payload = 0
max_current_communication = 0
max_current_TCS = 0

#Subsystems relative position from battery
cable_length_motors = 1.76 #Length of propeller arms [m]
cable_length_payload = 0
cable_length_communication = 0
cable_length_TCS = 0

#Cable properties 
#https://metersuk.co.uk/american-wire-gauge-awg-cable-conductor-sizes/ 
#https://www.calmont.com/resources/wire-gauge-table/ 
#Specific masses provided in kg/m
AWG_12_specific_mass = 29.8/1000
AWG_12_resistance = 5.2/1000 #ohm/m
AWG_14_specific_mass = 19.1/1000
AWG_14_resistance = 8.54/1000 #ohm/m
AWG_16_specific_mass = 11.6/1000
AWG_16_resistance = 13.17/1000 #ohm/m

#Calculate cable mass per subsystem
#Cable_mass = length * specific_mass
current_per_wire_motors = max_current_motor/3 #Size for 3 cables to hold max current (design for 1 cable lost)
if current_per_wire_motors<3.7:
    wire_specific_mass = AWG_16_specific_mass
    wire_resistance = AWG_16_resistance
elif current_per_wire_motors<5.9:
    wire_specific_mass = AWG_14_specific_mass
    wire_resistance = AWG_14_resistance
elif current_per_wire_motors<9.3:
    wire_specific_mass = AWG_12_specific_mass
    wire_resistance = AWG_12_resistance
else :
    print('Too much current in wires!')

cable_mass_per_motor_triple_phase = cable_length_motors*wire_specific_mass*4 #Triple phase + 1 redundant
cable_mass_propulsion = 6*cable_mass_per_motor_triple_phase #6 motors, one set of redudancy each

#Calculate cable loss per subsystem
#Ploss = R * I^2
power_loss_per_cable = (wire_resistance * cable_length_motors) * (current_per_wire_motors)**2
power_loss_percentage = ((power_max_motor/3)-power_loss_per_cable)/(power_max_motor/3) #Power loss propulsion as a percentage of total input power [%]

#Output
wires_mass = 0
def get_wires_mass():
    wires_mass = cable_mass_propulsion * SF_mass
    return wires_mass
print('Wire mass: ', get_wires_mass())

cable_loss = 0
def get_cable_loss():
    PCDU_loss = 0.5
    cable_loss = power_loss_percentage*SF_loss + PCDU_loss
    return cable_loss
print('Harness loss: ', get_cable_loss())
