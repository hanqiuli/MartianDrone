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
array_density = 0 #Array density [kg/m^2]
battery_density = 0 # Energy density of the battery [kg/Wh]
array_area = 0 #Array area [m^2]
actual_battery_capacity = 0 #Actual battery capacity [Wh]
wire_length = 0 #Length of cables[m]
wire_density = 0 #Density of cables [kg/m]
mass_EPS_thermal_contol = 0 #Mass of EPS thermal control system, including electronics, wire, battery and array thermal control [kg]
mass_EPS_controller = 0 #Mass of power management computer [kg]

#Calculate component masses
mass_array = array_area*array_density
mass_battery = actual_battery_capacity*battery_density
mass_wires = wire_length*wire_density

#Total mass calculation
mass_EPS = mass_array + mass_battery + mass_wires + mass_EPS_controller + mass_EPS_thermal_contol
print(mass_EPS)