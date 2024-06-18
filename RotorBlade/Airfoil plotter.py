import matplotlib.pyplot as plt
import numpy as np

# Define the triangular shape
x = [0, 0.3, 1, 0]
y = [0, 0.05, 0, 0]

# x = [0, 0.4, 1, 0.4, 0]
# y = [0, 0.06, 0, -0.02, 0]

# Create the plot
fig, ax = plt.subplots(figsize=(11.5, 4.5 ))

# Plot the triangle
ax.plot(x, y, color='black', linewidth=0.7)

# Set the axis limits
ax.set_xlim(-0.2, 1.2)
ax.set_ylim(-0.1, 0.2)

# Set the axis labels
ax.set_xlabel('nondimensional chord, x/c', fontsize=12, fontfamily='sans-serif')
ax.set_ylabel('nondimensional height, y/c', fontsize=12, fontfamily='sans-serif')

# Show grid with light gray color
ax.grid(True, color='lightgray')

# Set tick parameters
ax.tick_params(axis='both', which='major', labelsize=10)
ax.set_yticks(np.arange(0, 0.21, 0.1))

ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

# Set the color and line width for the visible spines
ax.spines['bottom'].set_color('black')
ax.spines['left'].set_color('black')
ax.spines['bottom'].set_linewidth(1)
ax.spines['left'].set_linewidth(1)


# Adjust the aspect ratio
ax.set_aspect(aspect='1')

# Display the plot
plt.show()


