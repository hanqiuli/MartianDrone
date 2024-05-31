# Standard libraries
import math
import random

# Third party libraries
import numpy as np
import matplotlib.pyplot as plt
import scipy as sp

# Local imports
# from environment_properties import ENV
time_mars_day = 88775

'''
This file will output total energy required and 
peak power usage for 1 Martian day based on a 
typical mission profile. The two mission profiles 
considered are a 1 day fly out and return mission 
and a 2 day fly out, charge, fly back mission. 
These outputs will be used as a starting point to size 
the EPS including the solar arrays, battery an cables. This is still a
basic tool, its precision can be increased if we use 
variable power usage for communication f(distance) or
variable thermal power usage f(temperature)
'''

profile_power = {
    # 'name': (power [W])
    'propulsive_takeoff': 5000,
    'propulsive_takeoff_with_rock': 5200,
    'propulsive_cruise': 4000,
    'propulsive_cruise_with_rock': 4200,
    'propulsive_scanning': 4500,
    'propulsive_landing': 5000,
    'propulsive_landing_with_rock': 5200,
    'propulsive_hover': 5000,
    'payload_collection': 15,
    'communication': 15,
    'thermal_propulsion': 30,
    'thermal_avionics': 10,
    'thermal_battery': 15,
    'avionics': 5,
    'avionics_data_gathering': 5
}

profile_time_1_day_mission = {
    # 'name': (durations [s])
    'morning_rest': 8*3600, #Assume takeoff at 8am, will have to optimze this somehow
    'takeoff': 30,
    'cruise': 400,
    'scanning': 150,
    'hover': 50,
    'landing': 30,
    'collection': 100,
    'cooling': 500,
    'takeoff_with_rock': 30,
    'cruise_with_rock': 600,
    'landing_with_rock': 30,
    'deposit_rock': 60,
    'afternoon_rest': time_mars_day-8*3600-30-400-150-50-30-100-500-30-600-30-60
}

profile_time_2_day_mission = {
    # 'name': (durations [s])
    'morning_rest': 8*3600, #Assume takeoff at 8am, will have to optimze this somehow
    'takeoff': 30,
    'cruise': 1000,
    'scanning': 150,
    'hover': 50,
    'landing': 30,
    'collection': 100,
    'mid_mission_rest': time_mars_day,
    'takeoff_with_rock': 30,
    'cruise_with_rock': 1200,
    'landing_with_rock': 30,
    'deposit_rock': 60,
    'afternoon_rest': 2*time_mars_day-8*3600-30-1000-150-50-30-100-time_mars_day-30-1200-30-60
}

#Combine powers to form power usage states
power_usage_baseline = profile_power['communication'] + profile_power['thermal_avionics'] + profile_power['thermal_battery'] + profile_power['avionics'] + profile_power['avionics_data_gathering']
power_usage_takeoff = power_usage_baseline + profile_power['propulsive_takeoff'] + profile_power['thermal_propulsion']
power_usage_cruise = power_usage_baseline + profile_power['propulsive_cruise'] + profile_power['thermal_propulsion']
power_usage_scanning = power_usage_baseline + profile_power['propulsive_scanning'] + profile_power['thermal_propulsion']
power_usage_hover = power_usage_baseline + profile_power['propulsive_hover'] + profile_power['thermal_propulsion']
power_usage_landing = power_usage_baseline + profile_power['propulsive_landing'] + profile_power['thermal_propulsion']
power_usage_collection = power_usage_baseline + profile_power['payload_collection'] + profile_power['thermal_propulsion']
power_usage_cooling = power_usage_baseline + profile_power['thermal_propulsion']
power_usage_takeoff_with_rock = power_usage_baseline + profile_power['propulsive_takeoff_with_rock']+profile_power['thermal_propulsion']
power_usage_cruise_with_rock = power_usage_baseline + profile_power['propulsive_cruise_with_rock']+profile_power['thermal_propulsion']
power_usage_landing_with_rock = power_usage_baseline + profile_power['propulsive_landing_with_rock']+profile_power['thermal_propulsion']
power_usage_deposit_rock = power_usage_baseline + profile_power['payload_collection']

#Construct the power usage profile for the 1 day return mission
time_series_1_day = np.linspace(0, time_mars_day, time_mars_day)
power_series_1_day = []

#Append power usage states to the power series at the relevant times
for i in range(profile_time_1_day_mission['morning_rest']):
    power_series_1_day.append(power_usage_baseline)

for i in range(profile_time_1_day_mission['takeoff']):
    power_series_1_day.append(power_usage_takeoff)

for i in range(profile_time_1_day_mission['cruise']):
    power_series_1_day.append(power_usage_cruise)

for i in range(profile_time_1_day_mission['scanning']):
    power_series_1_day.append(power_usage_scanning)

for i in range(profile_time_1_day_mission['hover']):
    power_series_1_day.append(power_usage_hover)

for i in range(profile_time_1_day_mission['landing']):
    power_series_1_day.append(power_usage_landing)

for i in range(profile_time_1_day_mission['collection']):
    power_series_1_day.append(power_usage_collection)

for i in range(profile_time_1_day_mission['cooling']):
    power_series_1_day.append(power_usage_cooling)

for i in range(profile_time_1_day_mission['takeoff_with_rock']):
    power_series_1_day.append(power_usage_takeoff_with_rock)

for i in range(profile_time_1_day_mission['cruise_with_rock']):
    power_series_1_day.append(power_usage_cruise_with_rock)

for i in range(profile_time_1_day_mission['landing_with_rock']):
    power_series_1_day.append(power_usage_landing_with_rock)

for i in range(profile_time_1_day_mission['deposit_rock']):
    power_series_1_day.append(power_usage_collection)

for i in range(profile_time_1_day_mission['afternoon_rest']):
    power_series_1_day.append(power_usage_baseline)

#Plot power profile 1 day mission
plt.plot(time_series_1_day, power_series_1_day)
plt.title('Power load profile 1 day return mission')
plt.xlabel('Mission duration [s]')
plt.ylabel('Power usage [W]')
plt.show()

#Construct the power usage profile for the 2 day return mission
time_series_2_day = np.linspace(0, 2*time_mars_day, 2*time_mars_day)
power_series_2_day = []

#Append power usage states to the power series at the relevant times
for i in range(profile_time_2_day_mission['morning_rest']):
    power_series_2_day.append(power_usage_baseline)

for i in range(profile_time_2_day_mission['takeoff']):
    power_series_2_day.append(power_usage_takeoff)

for i in range(profile_time_2_day_mission['cruise']):
    power_series_2_day.append(power_usage_cruise)

for i in range(profile_time_2_day_mission['scanning']):
    power_series_2_day.append(power_usage_scanning)

for i in range(profile_time_2_day_mission['hover']):
    power_series_2_day.append(power_usage_hover)

for i in range(profile_time_2_day_mission['landing']):
    power_series_2_day.append(power_usage_landing)

for i in range(profile_time_2_day_mission['collection']):
    power_series_2_day.append(power_usage_collection)

for i in range(profile_time_2_day_mission['mid_mission_rest']):
    power_series_2_day.append(power_usage_baseline)

for i in range(profile_time_2_day_mission['takeoff_with_rock']):
    power_series_2_day.append(power_usage_takeoff_with_rock)

for i in range(profile_time_2_day_mission['cruise_with_rock']):
    power_series_2_day.append(power_usage_cruise_with_rock)

for i in range(profile_time_2_day_mission['landing_with_rock']):
    power_series_2_day.append(power_usage_landing_with_rock)

for i in range(profile_time_2_day_mission['deposit_rock']):
    power_series_2_day.append(power_usage_collection)

for i in range(profile_time_2_day_mission['afternoon_rest']):
    power_series_2_day.append(power_usage_baseline)

#Plot power profile 2 day mission
plt.plot(time_series_2_day, power_series_2_day)
plt.title('Power load profile 2 day return mission')
plt.xlabel('Mission duration [s]')
plt.ylabel('Power usage [W]')
plt.show()

#Calculate maximum power usage
print('max_power_1_day [W]', np.max(power_series_1_day))
print('max_power_2_day [W]', np.max(power_series_2_day))

#Calculate mission energy
mission_energy_1_day = np.trapz(power_series_1_day, dx=1)
mission_energy_2_day = np.trapz(power_series_2_day, dx=1)
average_power = np.max((mission_energy_1_day, mission_energy_2_day))/(time_mars_day)
print('mission_energy_1_day [J]', mission_energy_1_day)
print('mission_energy_1_day [Wh]', mission_energy_1_day/3600)
print('mission_energy_2_day [J]', mission_energy_2_day)
print('mission_energy_2_day [Wh]', mission_energy_2_day/3600)
print('design average power [W]', average_power)
# print(abs(mission_energy_1_day-mission_energy_2_day/2)/mission_energy_1_day*100)

'''
Conclusion from this analysis: the 1 day mission profile
is more constraining in terms of energy usage with the current
values and thus the EPS will be sized for the 1 day mission profile. 
The solar arrays can now be sized to provide mission energy over 1 martian 
day such that a mission can be performed every day.
'''