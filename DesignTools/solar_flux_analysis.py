import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os


def plot_solar_flux(file_name='Solar_Flux.txt', plotting=False):
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

    print(df)

    #create colormap
    plt.imshow(df, cmap='hot', interpolation='nearest')
    plt.yticks(range(len(time_axis)), time_axis, size='small')
    plt.xticks(range(len(days)), days, size='small', rotation=90)
    plt.colorbar()
    plt.xlabel('Martian Day')
    plt.ylabel('Martian Time [h]')
    plt.title('Solar Flux [W/m^2]')

    plt.show()



    #Integrate flux over time per day and plot
    flux_integrated = np.trapz(flux, time_axis, axis=1)
    #Calculate average solar flux
    flux_avg = np.mean(flux_integrated)

    plt.plot(days, flux_integrated)
    plt.xlabel('Martian Day')
    plt.ylabel('Integrated Solar Flux [J/m^2]')
    plt.title('Integrated Solar Flux over Martian Days')
    plt.annotate(f'Average Solar Flux: {flux_avg:.2f} J/m^2', (0.5, 0.5), xycoords='axes fraction')
    plt.grid()
    plt.show()

    return flux_avg

if __name__ == '__main__':
    plot_solar_flux('Solar_Flux.txt', plotting=True)

