import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

def get_avg_solar_flux(file_name='Solar_Flux.txt', plotting=False):
    file_path = os.path.join('DesignTools/data', 'Solar_Flux.txt')

    with open(file_path, 'r') as file:
        lines = file.readlines()

    #Extract 12th line
    time_line = lines[11].strip()

    time_axis = time_line.split("||")[1].split()
    time_axis = [float(time) for time in time_axis]

    #Extract data
    data_lines = lines[13:]

    days = []
    flux = []

    for line in data_lines:
        parts = line.split("||")
        days.append(float(parts[0].strip()))
        flux.append([float(x) for x in parts[1].split()])

    #Convert to np arrays
    days = np.array(days)
    flux = np.array(flux)

    #dataframe
    df = pd.DataFrame(flux.T, columns=days, index=time_axis)

    # print(df)

    #create colormap
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

    #get mean flux over the day
    flux_avg = np.mean(flux, axis=1)
    if plotting:
        plt.plot(days, flux_avg)
        plt.xlabel('Martian Day')
        plt.ylabel('Average Solar Flux [W/m^2]')
        plt.title('Average Solar Flux over the Day')
        plt.grid()
        plt.show()

    return days, flux_avg

if __name__ == '__main__':
    x = get_avg_solar_flux('Solar_Flux.txt', plotting=True)
    print(x)
