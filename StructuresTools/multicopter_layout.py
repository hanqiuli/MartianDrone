import matplotlib.pyplot as plt
import numpy as np

# Input Parameters
n_rotors = 6
rotor_radius = 1.3  # in meters
rotor_clearance_percentage = 10  # percentage of rotor radius
body_radius = 0.7  # in meters

# Calculate rotor clearance
rotor_clearance = rotor_clearance_percentage / 100 * rotor_radius

# Calculate the required distance between the centers of adjacent rotors
distance_between_rotor_centers = 2 * rotor_radius + rotor_clearance

# Calculate the angle between adjacent rotors
angle_between_rotors = 2 * np.pi / n_rotors

# Calculate the arm length
arm_length = distance_between_rotor_centers / (2 * np.sin(np.pi / n_rotors))
actual_arm_length = arm_length - body_radius

# Calculate rotor positions
rotor_positions = []
for i in range(n_rotors):
    angle = i * angle_between_rotors
    x = arm_length * np.cos(angle)
    y = arm_length * np.sin(angle)
    rotor_positions.append((x, y))

# Plotting the multicopter
fig, ax = plt.subplots()

# Plot body
body_circle = plt.Circle((0, 0), body_radius, color='blue', fill=True, alpha=0.5)
ax.add_patch(body_circle)

# Plot arms and rotors
for x, y in rotor_positions:
    # Plot arm
    ax.plot([0, x], [0, y], 'k-')
    
    # Plot rotor
    rotor_circle = plt.Circle((x, y), rotor_radius, color='red', fill=False)
    ax.add_patch(rotor_circle)

# Set equal aspect ratio and limits
ax.set_aspect('equal', 'box')
ax.set_xlim(-arm_length * 1.5, arm_length * 1.5)
ax.set_ylim(-arm_length * 1.5, arm_length * 1.5)
ax.set_title('Top View of Multicopter')
ax.set_xlabel('meters')
ax.set_ylabel('meters')

plt.grid(True)
plt.show()

# Print calculated arm length
print(f"Calculated Arm Length: {actual_arm_length:.2f} meters")


