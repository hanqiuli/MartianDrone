# Standard libraries
import math
import random

# Third party libraries
import numpy as np
import matplotlib.pyplot as plt
import scipy as sp

# Local imports
# from environment_properties import ENV
time_mars_day = 88642

profile_power = {
    # 'name': (power [W])
    'propulsive_takeoff': 100,
    'propulsive_takeoff_with_rock': 120,
    'propulsive_cruise': 70,
    'propulsive_cruise_with_rock': 80,
    'propulsive_scanning': 70,
    'propulsive_landing': 110,
    'propulsive_landing_with_rock': 130,
    'propulsive_hover': 100,
    'payload_collection': 15,
    'communication': 15,
    'thermal_propulsion': 30,
    'thermal_avionics': 10,
    'thermal_battery': 15,
    'avionics': 5,
    'avionics_data_gathering': 5
}
profile_time_1day_mission = {
    # 'name': (durations [s])
    'propulsive_takeoff': 30,
    'propulsive_takeoff_with_rock': 30,
    'propulsive_cruise': 400,
    'propulsive_cruise_with_rock': 600,
    'propulsive_scanning': 200,
    'propulsive_landing': 40,
    'propulsive_landing_with_rock': 40,
    'propulsive_hover': 100,
    'payload_collection': 100,
    'communication': time_mars_day,
    'thermal_propulsion': 30,
    'thermal_avionics': time_mars_day,
    'thermal_battery': time_mars_day,
    'avionics': time_mars_day,
    'avionics_data_gathering': time_mars_day
}


time_series = np.linspace(0, time_mars_day, 88642)

power_baseline = profile_power[''] + profile_power['']


power_series = [profile_power['']]





plt.plot(time_series, power_series)
plt.show


power_max = np.max(power_series) #Maximum power is max of power series during the martian day
daily_energy = ... #Integrate power series over the martian day




# class PowerLoadProfile:
#     def __init__(self, dict):
    
#         # self.M_tip = M_tip

#         # self.V_tip = M_tip * ENV['a']
#         ...

#     def required_params_per_rotor(self, T_required):
#         self.A_disk = T_required / self.T_A_disk
#         self.r_disk = np.sqrt(self.A_disk / np.pi)

# def plot_load_profile(foo, bar_bar, foo_bar=True):
#     '''
#     This function calculates the bar-foo force

#     Positional arguments:
#     foo      [-]      Float       The foo coefficient
#     bar_bar  [m/s]    Array-like  The bar speed

#     Keyword arguments:
#     foo_bar  [-]      Bool        Whether or not foo is at bar

#     Returns:
#     bar_foo  [N]      Array-like  The bar-foo force
#     '''