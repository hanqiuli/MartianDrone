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
from solar_flux_analysis import get_avg_solar_flux
from power_load_profile import get_average_power_mission
from solar_flux_analysis import plot_solar_flux_daily

'''
This file will size the solar arrays to produce the average power required at average solar flux values.

Current assumptions: 
    Solar cell efficiency is not affected by red shift
    Dust will be removed by vibrations and/or blade wash. Atmospheric dust taken into account in average solar flux.
    No reduction in efficieny due to temperature
    Sizing assuming all power goes through battery, conservative as some power will be consumed directly.
    Sources for assumptions: https://ntrs.nasa.gov/api/citations/20040191326/downloads/20040191326.pdf 

Solar cells data: https://satsearch.co/products/azur-space-qj-solar-cell-4g32c-advanced

Validate solar array sizing against similar missions 
'''

#Define inputs
design_charge_time = time_mars_day #Design EPS for 1 mission per day
solar_cell_efficiency = 0.32 #Quadruple junction solar cell efficieny. Source: Azur Space
design_incidence_angle = 15 #Design to charge on a 15 degree slope. Source: Terrain map of Ebberwalde crater
average_solar_flux = 131.14 #Design for average yearly solar flux [W/m^2] source: MCD
radiation_degradation_rate = 0.02 #Degradation rate [% per year] Source: https://www.powerandresources.com/blog/solar-power-is-challenging-on-mars
active_area_factor = 0.9 #90% of cell area is available to generate power, refer to dust analysis section
spectrum_shift_factor = 0 #Loss due to spectrum shift from AM0 conditions, need to get better number on this. Argue this is included in efficiency (future improvements!)
mission_lifetime = 2 #Duration of primary mission [years]
battery_efficiency = 0.99#[%] need to get better numbers on this
harness_loss = 0.02 #[%] need to get better numbers on this
packing_factor = 0.9 #Square cells packing density https://sinovoltaics.com/learning-center/basics/packing-density/#:~:text=The%20solar%20cell's%20packing%20density,module's%20operating%20temperature%20as%20well.
bus_voltage = 100 #Voltage of power generation bus[V]
voltage_per_cell = 3.025 #Voltage at max power per cell
cell_area = 30.18 #Cell surface area [cm^2]
solar_array_density = 1.7 #Density of solar array [kg/m^2] Typical value, used by MSH, alligns with data from Sparkwing Solar Panel	and https://exoterracorp.com/products/power/ 
average_power_required = get_average_power_mission()
print('required_average_power', average_power_required)

#Calculate required power generation at BOL conditions (take into account power losses and radiation degradation rate) Note: dust degradation not taken into account
power_generation_EOL = average_power_required/(battery_efficiency*(1-harness_loss))
power_generation_BOL = power_generation_EOL / ((1-radiation_degradation_rate)**(mission_lifetime))
print('required_average_power_generation_BOL',power_generation_BOL)

#Calculate required solar array area (take into account average flux from MCD (incl. dust), incidence angle, red shift)
design_solar_flux = average_solar_flux * np.cos(np.radians(design_incidence_angle)) * (1-spectrum_shift_factor)
array_reference_area = power_generation_BOL / (design_solar_flux * solar_cell_efficiency * active_area_factor) #This is the area of the cells only! (packing not taken into account)

#Solar array architecture
n_cells_series_min = bus_voltage/voltage_per_cell
n_cells_series = np.ceil(n_cells_series_min)
n_cells = array_reference_area/(cell_area/10000)

n_strings_min = n_cells/n_cells_series
n_strings = np.ceil(n_strings_min)+1 #1 redundant string
actual_n_cells = n_cells_series*n_strings

print('n_cells_series', n_cells_series)
print('n_strings_required', n_strings)
print('n_cells', actual_n_cells)

actual_array_area = actual_n_cells*(cell_area/10000)/packing_factor
print('actual_array_area [m^2]', actual_array_area)

mass_array = actual_array_area * solar_array_density
print('mass_array [kg]', mass_array)

def get_array_mass():
    mass_array = actual_array_area * solar_array_density
    return mass_array

#Get solar flux data over one day for average Martian day
filepath_daily_flux = os.path.join('PowerTools/data', 'solar_flux_160deg.txt')
solar_flux_160_martian_day = plot_solar_flux_daily(filepath_daily_flux) #Solar flux corresponding to day 160 of the year, as a list (one entry per second) [W/m^2]

#Interpolate data to create a list of solar fluxes at every second
data_points = solar_flux_160_martian_day[1]

original_length = len(data_points)
desired_length = 88772

original_indices = np.linspace(0, original_length - 1, original_length)
desired_indices = np.linspace(0, original_length - 1, desired_length)

interpolated_solar_flux_160_martian_day = np.interp(desired_indices, original_indices, data_points)
extended_solar_flux = np.concatenate((interpolated_solar_flux_160_martian_day, interpolated_solar_flux_160_martian_day))

#Compute power generation profile for Martian day 160
power_generation_profile = extended_solar_flux * solar_cell_efficiency * array_reference_area * (1-radiation_degradation_rate)**mission_lifetime * np.cos(np.radians(design_incidence_angle)) * (1-spectrum_shift_factor) * active_area_factor #Power generated by solar array for an average martian day (day 160)

def get_power_generation_profile():
    power_generation_profile_EOL = extended_solar_flux * solar_cell_efficiency * array_reference_area * (1-radiation_degradation_rate)**mission_lifetime * np.cos(np.radians(design_incidence_angle)) * (1-spectrum_shift_factor) * active_area_factor
    return power_generation_profile_EOL

#Dust analysis
#inputs
dust_degradation_rate = 0.002 #Loss due to dust accumulation on array for Martian rovers [% per sol] source: https://ntrs.nasa.gov/api/citations/20040191326/downloads/20040191326.pdf 

time_series = np.linspace(0, 750, 750) #750 sols, nominal mission duration (2 Earth years)
power_generation_lifetime_profile=[]
for i in range(len(time_series)):
    power_generation_time_t = design_solar_flux * solar_cell_efficiency * array_reference_area * (1-radiation_degradation_rate)**mission_lifetime * np.cos(np.radians(design_incidence_angle)) * (1-spectrum_shift_factor) * (1-dust_degradation_rate)**time_series[i]
    power_generation_lifetime_profile.append(power_generation_time_t)

plt.plot(time_series, power_generation_lifetime_profile)
plt.title('Daily average power degradation due to dust accumulation')
plt.xlabel('Sols')
plt.ylabel('Daily average power [W]')
plt.show()

#Dust analysis conclusion, the worst case dust accumulation shows drastic reduction in available power over 750 sols. 
#Assume dust will be mostly shaken off and take a 10% margin on available solar array area. https://ieeexplore.ieee.org/abstract/document/9843820 (they took 15% but we have much larger blades, more downwash)


#Charge time analysis at BOL, EOL, summer, winter