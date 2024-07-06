import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import scienceplots
plt.style.use('science')
plt.rcParams.update({'text.usetex': False})
import sys
sys.path.append("C://Users//sebas//Desktop//MartianDrone//EnvironmentalPropertyTools")  # Replace with the actual path

from colormap import colormap

masses = {
    'PLD': 5.601,
    'Rock': 3,
    'TCS': 5.354,
    'EPS': 20.175,
    'STR': 3.656 + 0.470 + 3.829,
    'AVN': 1.800,
    'PRO': 2.5 + 1.999,
}

my_cmap = colormap()
cmap = matplotlib.cm.get_cmap('binary')
# cmap = ["#8fd7d7", "#00b0be", "#ff8ca1", "#f45f74", "#bdd373", "#98c127", "#ffcd8e"]
cmap = my_cmap
cmap = cmap(np.linspace(0, 1, len(masses)+2))
cmap = cmap[1:-1]
# Sort masses in ascending order
masses = dict(sorted(masses.items(), key=lambda item: item[1]))

# Convert masses to kilograms with labels
mass_labels = [f"{key} ({value:.2f} kg)" for key, value in masses.items()]

# Create pie chart with labels
plt.figure(figsize=(6, 5))
# Calculate total mass
total_mass = sum(masses.values())

# Convert masses to kilograms with labels
mass_labels = [f"{key} ({value:.2f} kg)" for key, value in masses.items()]
wedgeprops = {'linewidth': 1, 'edgecolor': 'black', 'antialiased': True}  # Adjust linewidth as desired

# Create pie chart with labels, hiding percentages
plt.pie(masses.values(), labels=mass_labels, autopct="",colors=cmap)
plt.wedgelabelstyle = {'edgecolor': 'black'}  # Set wedge edge color to black
plt.show()