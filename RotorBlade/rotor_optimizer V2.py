import numpy as np
from scipy.optimize import differential_evolution, NonlinearConstraint
from blade import Blade
from rotor_blade import calculate_blade_properties

############## INPUTS ################
number_of_blades = 3  # Range from 2-8
radial_stations = [0.08, 0.25, 0.75, 1.00]  # Fixed
airfoils = ['Diamond', 'Triangle', 'DEP 0.5', 'DEP 0.7']  # Fixed
rotor_radius = 1.20
maximum_alpha = 10
gravitational_acceleration = 3.71
mass = 48.8547718*1.3 # kg
small_angle_approximation = False
######################################

print("optimizer is cooking something...")

# Define the objective function
def objective(params):
    pitch_params = params[:3]
    chord_params = params[3:7]

    blade = Blade(
        radius_rotor=rotor_radius,
        num_blades=number_of_blades,
        radial_nondim_stations=radial_stations,
        chord_nondim_stations=chord_params,
        pitch_params=pitch_params,
        airfoil_name_stations=airfoils,
        small_angle_approx=small_angle_approximation
    )
    calculate_blade_properties(blade)

    minimize_objective = blade.power_rotor

    return minimize_objective

# Define the constraint function to ensure positive power_rotor
def power_constraint(params):
    pitch_params = params[:3]
    chord_params = params[3:7]

    blade = Blade(
        radius_rotor=rotor_radius,
        num_blades=number_of_blades,
        radial_nondim_stations=radial_stations,
        chord_nondim_stations=chord_params,
        pitch_params=pitch_params,
        airfoil_name_stations=airfoils,
        small_angle_approx=small_angle_approximation
    )
    calculate_blade_properties(blade)

    linear_line = np.linspace(12000, 30000, len(blade.reynolds))
    max_deviation = np.max(blade.reynolds - linear_line)


    return blade.thrust_rotor, (blade.thrust_coefficient_rotor / blade.solidity_rotor), blade.solidity_rotor, blade.power_rotor, max_deviation, chord_params[0] - chord_params[1]



lb = [mass*gravitational_acceleration/6, -np.inf, -np.inf, 0, -np.inf, -np.inf]
ub = [np.inf, 0.16, 0.2, np.inf, 0, 0]

# Create a NonlinearConstraint object to enforce positive power_rotor
nonlinear_constraint = NonlinearConstraint(power_constraint, lb, ub)


# Set the bounds for pitch and chord parameters
bounds = [
    (0.05, 0.3), (0.05, 0.3), (-0.5, -0.05),  # Pitch parameters
    (0.08, 2*radial_stations[0] * np.tan(np.pi / number_of_blades)), (0.08, 2*radial_stations[1] * np.tan(np.pi / number_of_blades)), (0.08, 2*radial_stations[2] * np.tan(np.pi / number_of_blades)), (0.0, 2*radial_stations[3] * np.tan(np.pi / number_of_blades))  # Chord parameters
]

# Differential Evolution optimization
result = differential_evolution(objective, bounds, constraints=nonlinear_constraint)

optimal_params = result.x
optimal_pitch_params = optimal_params[:3]
optimal_chord_params = optimal_params[3:7]

print(result.message)
print("Optimal Pitch Parameters:", ', '.join(map(str, optimal_pitch_params)))
print("Optimal Chord Parameters:", ', '.join(map(str, optimal_chord_params)))
print("Minimum power:", result.fun)

