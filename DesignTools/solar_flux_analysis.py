import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

file_path = os.path.join('DesignTools/data', 'Solar_Flux.txt')

#File format:
#1st row: Martian time [h]
#1st column: Martian Day in degrees
#2nd column: Solar flux [W/m^2]
"""
---- ||    0.00000e+00    1.00000e+00    2.00000e+00    3.00000e+00    4.00000e+00    5.00000e+00    6.00000e+00    7.00000e+00    8.00000e+00    9.00000e+00    1.00000e+01    1.10000e+01    1.20000e+01    1.30000e+01    1.40000e+01    1.50000e+01    1.60000e+01    1.70000e+01    1.80000e+01    1.90000e+01    2.00000e+01    2.10000e+01    2.20000e+01    2.30000e+01    2.40000e+01
-----------------------------------
+000 ||    0.00000e+00    0.00000e+00    0.00000e+00    0.00000e+00    0.00000e+00    0.00000e+00    0.00000e+00    1.03809e+02    2.06841e+02    3.16041e+02    4.01806e+02    4.55173e+02    4.75446e+02    4.56245e+02    4.04306e+02    3.18211e+02    2.13290e+02    1.02706e+02    0.00000e+00    0.00000e+00    0.00000e+00    0.00000e+00    0.00000e+00    0.00000e+00    0.00000e+00
+015 ||    0.00000e+00    0.00000e+00    0.00000e+00    0.00000e+00    0.00000e+00    0.00000e+00    0.00000e+00    8.31888e+01    1.83530e+02    2.87284e+02    3.68386e+02    4.18734e+02    4.37401e+02    4.19212e+02    3.69673e+02    2.88337e+02    1.88926e+02    8.50786e+01    0.00000e+00    0.00000e+00    0.00000e+00    0.00000e+00    0.00000e+00    0.00000e+00    0.00000e+00
+030 ||    0.00000e+00 ... 
"""

# data = pd.read_csv(file_path, sep='\t', header=time, skiprows=11)

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
df = pd.DataFrame(flux, columns=days, index=time_axis)

print(df)

#create colormap
plt.imshow(df, cmap='hot', interpolation='nearest')
plt.colorbar()
plt.show()



#Integrate flux over time per day and plot
flux_integrated = np.trapz(flux, time_axis, axis=1)

plt.plot(days, flux_integrated)
plt.xlabel('Martian Day')
plt.ylabel('Integrated Solar Flux [J/m^2]')
plt.title('Integrated Solar Flux over Martian Days')
plt.grid()
plt.show()

