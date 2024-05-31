import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def get_data(file_name):
    '''Function to extract the average solar flux over the day from the solar flux data'''
    #Read file
    with open(file_name, 'r', encoding="utf-8") as file:
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
    property = []

    for line in data_lines:
        parts = line.split("||")
        days.append(float(parts[0].strip()))
        property.append([float(x) for x in parts[1].split()])

    #Convert to np arrays
    days = np.array(days)
    property = np.array(property)

    #Store in dataframe
    df = pd.DataFrame(property.T, columns=days, index=time_axis)

    return property, time_axis, days

y = get_data("Ratio_Specific_Heats_100m.txt")[0]
R = get_data("Molecular_Gas_Constant_100m.txt")[0]
T = get_data("Temperature_100m.txt")[0]
time_axis = get_data("Ratio_Specific_Heats_100m.txt")[1]
days = get_data("Ratio_Specific_Heats_100m.txt")[2]
a = np.sqrt(y*R*T)

df = pd.DataFrame(a, columns=days,index=time_axis)

plt.imshow(df, cmap='seismic', interpolation='nearest')
plt.yticks(range(len(time_axis)), time_axis, size='small')
plt.xticks(range(len(days)), days, size='small', rotation=90)
plt.colorbar()
plt.xlabel('Martian Day')
plt.ylabel('Martian Time [h]')
plt.title('Speed of Sound [m/s]')
plt.show()