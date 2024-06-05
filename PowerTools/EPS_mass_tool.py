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

#Inputs
mass_array = get_array_mass()
mass_battery = get_battery_mass()
mass_wires = get_wires_mass()
mass_PCDU = 2 #Need to get this figured out

#Total mass calculation
mass_EPS = mass_array + mass_battery + mass_wires + mass_PCDU
print(mass_EPS)