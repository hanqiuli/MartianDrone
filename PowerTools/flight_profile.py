import matplotlib.pyplot as plt
import scienceplots

# Define the flight profile data
times = [0, 45, 1045, 1060, 1480, 1510, 1540, 1560]
altitudes = [0, 100, 100, 75, 75, 10, 10, 0]

# Plot the flight profile
plt.style.use('science')
plt.rcParams.update({'text.usetex': False})
plt.figure(figsize=(12, 6))
plt.plot(times, altitudes)

# Annotate the different sections
labels = {
    'Cruise': (545, 100),
    'Birds Eye Scan': (1270, 75),
    'Close Up Scan': (1415, 10)
}

for label, (x, y) in labels.items():
    plt.text(x, y, label, horizontalalignment='center', verticalalignment='bottom')

# Add titles and labels
plt.title('Flight Profile')
plt.xlabel('Time (seconds)')
plt.ylabel('Altitude (meters)')
plt.grid(True)
plt.ylim(0, 120)
plt.xlim(0, 1600)

# Show the plot
plt.show()