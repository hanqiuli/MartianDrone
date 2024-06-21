# Standard libraries
import math
import random

# Third party libraries
import numpy as np
import matplotlib.pyplot as plt
import scipy as sp
import scienceplots

# Local imports
# from environment_properties import ENV
time_mars_day = 88775 #Average Martian day duration [sec]
from solar_array_sizing import get_power_generation_profile
from power_load_profile import get_power_usage_profile

#Define inputs
power_generation_profile = get_power_generation_profile()
power_usage_profile = get_power_usage_profile()
battery_efficiency = 0.99 #Battery efficiency of Li ion is near 100%
depth_of_discharge = 0.9 #90% DOD is acceptable for 1000 cycles https://www.researchgate.net/figure/Depth-of-discharge-versus-cycle-life-of-the-lithium-ion-battery_fig4_318292540
EOL_capacity_factor = 0.9 #90% of capacity available after 700 cycles
harness_loss = 0.02 #Total cable loss, based on cable_sizing analysis

#Current state of the art commerical Li-ion specs
#https://iopscience.iop.org/article/10.1149/1945-7111/abf05f
#https://www.nasa.gov/wp-content/uploads/2021/10/3.soa_power_2021.pdf
#https://f.hubspotusercontent10.net/hubfs/6584819/Cell%20Spec%20Sheets/LG/Lithium%20Ion%20LG%20INR%2021700%20M50LT%202021-LSD-MBD-b00002_21700_M50LT_PS_Promotion_V1.pdf
target_battery_energy_density = 231 #Wh/kg
cell_voltage = 3.7 #Cell voltage [V]
bus_voltage = 60 #Battery discharge voltage [V]
cell_capacity = 5 #Cell capacity [Ah]
cell_height = 70 #mm
cell_diameter = 21 #mm
cell_mass = 0.067 #kg
cell_max_discharge_rate = 3 #3C max discharge rate
cell_max_charge_rate = 0.7 #0.7C max charge rate

#Get shapes to match
empty_list = [0,0,0,0,0,0]
power_generation_profile = np.append(power_generation_profile, empty_list)
power_available_profile = power_generation_profile*battery_efficiency*(1-harness_loss)*0.97

print('max_power_generation', np.max(power_generation_profile))

net_power_profile = power_available_profile - power_usage_profile

#Plot the net power profile
time_series = np.linspace(0,2*time_mars_day,2*time_mars_day)
plt.style.use('science')
plt.rcParams.update({'text.usetex': False})
plt.plot(time_series, net_power_profile)
plt.title('Net power profile over a 2 day mission', fontsize=16)
plt.xlabel('Mission timeline [s]', fontsize=16)
plt.ylabel('Net power [W]', fontsize=16)
plt.show()

#Net power profile can now be used to size the battery
#Get cumulative net energy over the day, giving the energy difference from the start of the day
energy_state_profile = np.cumsum(net_power_profile) 

#Calculate required battery capacity
battery_capacity = np.max(energy_state_profile)-np.min(energy_state_profile)

#Calculate actual battery capacity, taking into account battery efficiency (only applied to power that is not directly consumed!)
actual_battery_capacity = battery_capacity / (EOL_capacity_factor * depth_of_discharge) #Actual battery capacity [J]
print('Required actual battery capacity [Wh]', actual_battery_capacity/3600)

def get_battery_capacity():
    actual_battery_capacity = battery_capacity / (EOL_capacity_factor * depth_of_discharge)
    return actual_battery_capacity
#Construct a graph of State of Charge (SOC) of the battery
#Set state of charge=0 at minimum energy state
zero_soc_index = np.argmin(energy_state_profile)
inital_soc = -energy_state_profile[zero_soc_index]
soc_profile = inital_soc + energy_state_profile
soc_profile_Wh = soc_profile/3600

plt.style.use('science')
plt.rcParams.update({'text.usetex': False})
plt.plot(time_series, soc_profile_Wh)
plt.title('State of Charge of the battery over a 2 day mission')
plt.xlabel('Mission timeline [s]', fontsize=16)
plt.ylabel('State of charge of the battery [Wh]', fontsize=16)
plt.show()

#Convert SOC profile to a battery charge percentage[%]
soc_percentage = (soc_profile)/(battery_capacity)*100
soc_percentage_rescaled = [10+ (soc*0.8) for soc in soc_percentage]
plt.style.use('science')
plt.rcParams.update({'text.usetex': False})
plt.plot(time_series/3600, soc_percentage_rescaled)
plt.title('State of Charge profile of the battery over a 2 day mission as a percentage of the total battery capacity', fontsize=16)
plt.xlabel('Mission timeline [hours]', fontsize=16)
plt.ylabel('State of charge of the battery [%]', fontsize=16)
plt.axhline(y=10, color='r', linestyle='--', label='Min SOC (10%)')
plt.axhline(y=90, color='g', linestyle='--', label='Max SOC (90%)')
plt.legend()
plt.show()

#Battery architecture
n_cells_series = np.ceil(bus_voltage/3.6)
string_energy = n_cells_series * cell_capacity * cell_voltage * 3600 #[J]
n_strings = np.ceil(actual_battery_capacity/string_energy)+1
n_cells = n_cells_series*n_strings
battery_mass = n_cells*cell_mass

def get_battery_mass():
    battery_mass = n_cells*cell_mass
    return battery_mass

print('Number of cells per string', n_cells_series)
print('Number of strings', n_strings)
print('Number of cells', n_cells)
print('battery_mass [kg]', battery_mass)

 #Check maximum charge/discharge rate are acceptable
power_charge_max = np.max(net_power_profile)
power_discharge_max = np.min(net_power_profile)
required_charge_rate_max = 3600 / (actual_battery_capacity/power_charge_max)
required_discharge_rate_max = -3600 / (actual_battery_capacity/power_discharge_max)
print('peak discharge rate [C]', np.round(required_discharge_rate_max,2))
print(required_charge_rate_max)
if required_charge_rate_max<cell_max_charge_rate and required_discharge_rate_max<cell_max_discharge_rate:
    print('Cell charge performance is acceptable')
else: 
    print('Cell charge performance is not acceptable')