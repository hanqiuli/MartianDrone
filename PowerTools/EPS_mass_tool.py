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
from solar_array_sizing import get_array_mass
from battery_sizing import get_battery_mass
from cable_sizing import get_wires_mass
from solar_array_sizing import get_array_area
from battery_sizing import get_battery_capacity

#Inputs
mass_array = get_array_mass()
mass_battery = get_battery_mass()
mass_wires = get_wires_mass()
mass_PCDU = 2

array_area = get_array_area() #m^2
battery_capacity = get_battery_capacity()/3600 #Wh
specific_cost_array = 200 #kEUR/m^2
specific_cost_battery = 250/1000 #kEUR/Wh

#Total mass calculation
mass_EPS = mass_array + mass_battery + mass_wires + mass_PCDU
print('EPS total mass [kg]: ', mass_EPS)

#Total cost calculation
cost_array = specific_cost_array*array_area
cost_battery = specific_cost_battery*battery_capacity
cost_PCDU = 2000 #kEUR
cost_EPS = cost_array + cost_battery + cost_PCDU
print('EPS total cost [kEUR]: ', cost_EPS)