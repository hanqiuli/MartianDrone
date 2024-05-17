'''Module to analyze the solar flux data provided by the Mars Climate Database'''

import os
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


def get_avg_solar_flux(file_name:str = 'Solar_Flux.txt', plotting:bool = False):
    '''Function to extract the average solar flux over the day from the solar flux data'''
    #Define file path
    file_path = os.path.join('DesignTools/data', 'Solar_Flux.txt')
    #Read file
    with open(file_path, 'r', encoding="utf-8") as file:
        lines = file.readlines()

    #Isolate time axis
    time_line = lines[11].strip()
    #First line is the time axis
    time_axis = time_line.split("||")[1].split()
    time_axis = [float(time) for time in time_axis]

    #Extract data
    data_lines = lines[13:]

    #Extract days and flux
    days = []
    flux = []

    for line in data_lines:
        parts = line.split("||")
        days.append(float(parts[0].strip()))
        flux.append([float(x) for x in parts[1].split()])

    #Convert to np arrays
    days = np.array(days)
    flux = np.array(flux)

    #Store in dataframe
    df = pd.DataFrame(flux.T, columns=days, index=time_axis)

    #Create colormap for validation
    plt.imshow(df, cmap='hot', interpolation='nearest')
    plt.yticks(range(len(time_axis)), time_axis, size='small')
    plt.xticks(range(len(days)), days, size='small', rotation=90)
    plt.colorbar()
    plt.xlabel('Martian Day')
    plt.ylabel('Martian Time [h]')
    plt.title('Solar Flux [W/m^2]')
    if plotting:
        plt.show()
    plt.clf()

    #Obtain mean flux over each day
    flux_avg = np.mean(flux, axis=1)
    if plotting:
        plt.plot(days, flux_avg)
        plt.xlabel('Martian Day')
        plt.ylabel('Average Solar Flux [W/m^2]')
        plt.title('Average Solar Flux over the Day')
        plt.grid()
        plt.show()

    return days, flux_avg



# filepath = os.path.join('DesignTools/data', 'solar_flux_single_day.txt')

def plot_solar_flux_daily(filepath):
    '''Function to plot the solar flux data over a single day'''
    with open(filepath, 'r', encoding= "utf-8") as file:
        lines = file.readlines()

    data = lines[10:]

    time = []
    flux = []

    for line in data:
        parts = line.split()
        time.append(float(parts[0]))
        flux.append(float(parts[1]))

    plt.plot(time, flux)
    plt.xlabel('Time [h]')
    plt.ylabel('Solar Flux [W/m^2]')
    plt.title('Solar Flux over Time')
    plt.show()

    #get mean
    flux_mean = np.mean(flux)
    print(f'Mean Solar Flux: {flux_mean} W/m^2')
    return time, flux

if __name__ == '__main__':
    x = get_avg_solar_flux('Solar_Flux.txt', plotting=True)
    print(x)