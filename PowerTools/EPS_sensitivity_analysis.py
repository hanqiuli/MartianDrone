# Standard libraries
import math
import random

# Third party libraries
import numpy as np
import matplotlib.pyplot as plt
import scipy as sp

# Local imports
# from environment_properties import ENV
time_mars_day = 88775 #Average Martian day duration [sec]

#Inputs
solar_flux_variation = 10 #[%] variation from nominal value
power_usage_variation = 10 #[%] variation from nominal value
dust_coverage_factor_variation = 10 #[%] variation from nominal value

#Perform monte carlo simulation by varying these parameters