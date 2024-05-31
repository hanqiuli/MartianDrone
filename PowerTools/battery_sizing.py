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
from solar_array_sizing import get_power_generation_profile
from power_load_profile import get_power_usage_profile

#Define inputs
power_generation_profile = get_power_generation_profile()
power_usage_profile = get_power_usage_profile()
battery_discharge_efficiency = 0.9 #Transmission efficiency between battery and load
depth_of_discharge = 0.9 #90% DOD is acceptable for 1000 cycles https://www.researchgate.net/figure/Depth-of-discharge-versus-cycle-life-of-the-lithium-ion-battery_fig4_318292540

#Get shapes to match
empty_list = [0,0,0]
power_generation_profile = np.append(power_generation_profile, empty_list)

print(len(power_generation_profile))
print(len(power_usage_profile))
net_power_profile = power_generation_profile - power_usage_profile

#Plot the net power profile
time_series = np.linspace(0,time_mars_day,time_mars_day)
plt.plot(time_series, net_power_profile)
plt.title('Net power profile over a Martian day')
plt.xlabel('Mission timeline [s]')
plt.ylabel('Net power [W]')
plt.show()

#Net power profile can now be used to size the battery
#Get cumulative net energy over the day, giving the energy difference from the start of the day
energy_state_profile = np.cumsum(net_power_profile) 
plt.plot(time_series, energy_state_profile)
plt.title('Energy over the day')
plt.xlabel('Mission timeline [s]')
plt.ylabel('Energy state [J]')
plt.show()

#Calculate required battery capacity
battery_capacity = np.max(energy_state_profile)-np.min(energy_state_profile)

#Calculate actual battery capacity, taking into account battery efficiency (only applied to power that is not directly consumed!)
actual_battery_capacity = battery_capacity / (battery_discharge_efficiency * depth_of_discharge)
print('Required actual battery capacity [Wh]', actual_battery_capacity/3600)


#Construct a graph of State of Charge (SOC) of the battery
#Set state of charge=0 at minimum energy state
zero_soc_index = np.argmin(energy_state_profile)
inital_soc = -energy_state_profile[zero_soc_index]
soc_profile = inital_soc + energy_state_profile
soc_profile_Wh = soc_profile/3600
plt.plot(time_series, soc_profile_Wh)
plt.title('State of Charge of the battery over a Martian day')
plt.xlabel('Mission timeline [s]')
plt.ylabel('State of charge of the battery [Wh]')
plt.show()

#Convert SOC profile to a battery charge percentage[%]
soc_percentage = (soc_profile)/(battery_capacity)*100
plt.plot(time_series, soc_percentage)
plt.title('Percentage State of Charge of the battery over a Martian day (from the usable capacity)')
plt.xlabel('Mission timeline [s]')
plt.ylabel('State of charge of the battery [%]')
plt.show()