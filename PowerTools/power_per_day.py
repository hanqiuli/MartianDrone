import os
import matplotlib.pyplot as plt
import numpy as np

class DataExtraction:
    def __init__(self):
        pass

    def extract_data_day(file_name:str):
        """
        Extracts local time and property data from a text file. The text file has to be the data per hour for one day.

        Args:
        file_name: The path to the text file.

        Returns:
        A tuple containing two lists: time and property.
        """
        script_dir = os.path.dirname(__file__)  # Get the script's directory
        data_path = os.path.join(script_dir, file_name)  # Join path components

        time, property = [], []
        with open(data_path, 'r') as f:
            # Skip header lines
            lines = f.readlines()[10:]  # Assuming there are 8 header lines
            for line in lines:
            # Split the line by whitespace
                values = line.split()
        # Convert values to float
                time.append(float(values[0]))
                property.append(float(values[1]))

        return time, property

    def interpolation_per_second(time, property):
        # Interpolation
        interpolated_local_time, interpolated_property = [], []
        for i in range(len(time) - 1):  # Loop through each hour except the last
            # Calculate increment per second for this hour
            time_increment = (time[i + 1] - time[i]) / 3600
            property_increment = (property[i + 1] - property[i]) / 3600

            # Create data points per second for this hour
            for j in range(3699):
                interpolated_local_time.append(time[i] + j * time_increment)
                interpolated_property.append(property[i] + j * property_increment)

        # Add data for the last hour (no interpolation needed)
        interpolated_local_time.extend(time[-1:])
        interpolated_property.extend(property[-1:])

        return interpolated_local_time, interpolated_property

A = 3
# Example usage
local_time, solar_flux = DataExtraction.extract_data_day("data/solar_flux_150deg.txt")

print("Local Time (Martian Hour):", local_time)
print("Incident Solar Flux (W/m2):", solar_flux)
local_time = np.array(local_time)
solar_flux = np.array(solar_flux)

# Example usage
interpolated_local_time, interpolated_solar_flux = DataExtraction.interpolation_per_second(local_time,solar_flux)
interpolated_local_time = np.array(interpolated_local_time)
interpolated_solar_flux = np.array(interpolated_solar_flux)

solar_power = interpolated_solar_flux*A
time_mars_day = 3699*24

plt.plot(interpolated_local_time, solar_power)

def power_usage():
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

    for i in range(profile_time_1_day_mission['afternoon_rest']+1):
        power_series_1_day.append(power_usage_baseline)
    
    return power_series_1_day

power_used = power_usage()
net_power = solar_power - power_used

plt.plot(interpolated_local_time,net_power)
plt.show()