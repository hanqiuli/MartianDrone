import numpy as np
import matplotlib.pyplot as plt
import os

filepath = os.path.join('DesignTools/data', 'solar_flux_single_day.txt')

with open(filepath, 'r') as file:
    lines = file.readlines()

data = lines[10:]

time = []
flux = []

for line in data:
    parts = line.split()
    time.append(float(parts[0]))
    flux.append(float(parts[1]))

# print(time)

plt.plot(time, flux)
plt.xlabel('Time [h]')
plt.ylabel('Solar Flux [W/m^2]')
plt.title('Solar Flux over Time')
plt.show()

#get mean
flux_mean = np.mean(flux)
print(f'Mean Solar Flux: {flux_mean} W/m^2')

