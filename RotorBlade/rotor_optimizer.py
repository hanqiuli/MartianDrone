import numpy as np
from scipy.optimize import minimize
from blade import Blade
from rotor_blade import calculate_blade_properties
import csv

############## INPUTS ################
number_of_blades = 4 # Range from 2-8
radial_stations = [0.08, 0.25, 0.75, 1.00] # Fixed
airfoils = ['Diamond', 'Triangle', 'DEP 0.5', 'DEP 0.7'] # Fixed
rotor_radius = 1.20
maximum_alpha = 9.5 
gravitational_acceleration = 3.71
mass = 60 #kg
small_angle_approximation = False
######################################

optimal_pitch_params_list = []
optimal_chord_params_list = []
thrust_rotor_list = []
power_list1 = []

for iterations in range(10):

    # Define the objective function
    def objective(params):
        pitch_params = params[:3]
        chord_params = params[3:7]

        blade = Blade(radius_rotor= rotor_radius,
                    num_blades=number_of_blades,
                    radial_nondim_stations=radial_stations,
                    chord_nondim_stations=chord_params,
                    pitch_params=pitch_params,
                    airfoil_name_stations= airfoils,
                    small_angle_approx = small_angle_approximation)
        calculate_blade_properties(blade)

        

        minimize_objective = blade.power_rotor

        return minimize_objective


    # Define the constraint function
    def constraint(params):
        pitch_params = params[:3]
        chord_params = params[3:7]

        blade = Blade(radius_rotor=rotor_radius,
                    num_blades=number_of_blades,
                    radial_nondim_stations=radial_stations,
                    chord_nondim_stations=chord_params,
                    pitch_params=pitch_params,
                    airfoil_name_stations=airfoils,
                    small_angle_approx = small_angle_approximation)
        calculate_blade_properties(blade)

        # Constraint parameters
        reynolds_constraints = []
        max_reynolds_last = 25000
        max_reynolds_first = 10000
        num_reynolds = len(blade.reynolds)
        for i in range(num_reynolds):
            if i == 0:
                reynolds_constraints.append(max_reynolds_first - min(blade.reynolds[i], max_reynolds_first))
            elif i == num_reynolds - 1:
                reynolds_constraints.append(max(blade.reynolds[i], max_reynolds_last) - max_reynolds_last)
            else:
                max_reynolds_intermediate = max_reynolds_first + (
                            max_reynolds_last - max_reynolds_first) * i / (num_reynolds - 1)
                reynolds_constraints.append(max_reynolds_intermediate - min(max_reynolds_intermediate,
                                                                            max(max_reynolds_first, min(blade.reynolds[i],
                                                                                                        max_reynolds_last))))

        # print(blade.thrust_rotor)

        constraints = [
            0.16 - (blade.thrust_coefficient_rotor / blade.solidity_rotor),
            maximum_alpha - np.all(np.rad2deg(blade.alpha)),
            2.5 - max(chord_params) / min(chord_params),
            *reynolds_constraints,
            0.25 - blade.solidity_rotor,
            chord_params[2] - chord_params[3],
            pitch_params[0] - pitch_params[1],
            (mass * gravitational_acceleration) / 6 - blade.thrust_rotor
        ]

        return constraints
    
    def constraint_thrust(params):
            pitch_params = params[:3]
            chord_params = params[3:7]

            blade = Blade(radius_rotor=rotor_radius,
                        num_blades=number_of_blades,
                        radial_nondim_stations=radial_stations,
                        chord_nondim_stations=chord_params,
                        pitch_params=pitch_params,
                        airfoil_name_stations=airfoils,
                        small_angle_approx = small_angle_approximation)
            calculate_blade_properties(blade)

            return blade.thrust_rotor

    initial_guess = np.concatenate([
        np.random.uniform(0, 0.4, 2), np.random.uniform(-0.5, 0.0, 1),
        np.random.uniform(0.10, 0.14, 1), np.random.uniform(0.14, 0.18, 1), np.random.uniform(0.12, 0.16, 1), np.random.uniform(0.08, 0.12, 1)
    ])

    # Adjust constraints bounds to ensure they are properly bounded
    bounds = [
        (0.05, 0.35), (0.05, 0.35), (-0.5, -0.05),  # Pitch parameters
        (0.00, radial_stations[0] * np.tan(np.pi / number_of_blades)),
        (0.0, radial_stations[1] * np.tan(np.pi / number_of_blades)),
        (0.0, radial_stations[2] * np.tan(np.pi / number_of_blades)),
        (0.0, radial_stations[3] * np.tan(np.pi / number_of_blades))  # Chord parameters
    ]

    # Adjust constraints to handle inequality and equality separately
    inequality_constraints = [
        {'type': 'ineq', 'fun': lambda params: constraint(params)[i]} for i in range(7)
    ]

    equality_constraints = [
        {'type': 'eq', 'fun': lambda params: constraint(params)[7]}  # Equality constraint
    ]


    # Combine inequality and equality constraints
    constraints = inequality_constraints + equality_constraints

    result = minimize(objective, initial_guess, method='SLSQP', bounds=bounds, constraints=constraints)

    optimal_params = result.x
    optimal_pitch_params = optimal_params[:3]
    optimal_chord_params = optimal_params[3:7]



    optimal_pitch_params_list.append(optimal_pitch_params)
    optimal_chord_params_list.append(optimal_chord_params)
    thrust_rotor = constraint_thrust(optimal_params)
    thrust_rotor_list.append(thrust_rotor)
    power_list1.append(objective(optimal_params))



    # print(f"Iteration {iterations+1}: Optimal Pitch Parameters: {optimal_pitch_params}, Optimal Chord Parameters: {optimal_chord_params}, Thrust Rotor: {thrust_rotor}, Minimum Powr: {result.fun}")

# Writing the data to CSV file
with open('optimization_results.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Optimal Pitch Parameters', 'Optimal Chord Parameters', 'Thrust Rotor', 'Power'])
    for i in range(len(optimal_pitch_params_list)):
        if thrust_rotor_list[i] >= (mass * gravitational_acceleration) / 6:  # Filtering out rows where thrust rotor < 30
            writer.writerow([optimal_pitch_params_list[i], optimal_chord_params_list[i], thrust_rotor_list[i], power_list1[i]])

print("CSV file generated successfully.")

# print("Optimal Pitch Parameters:", ', '.join(map(str, optimal_pitch_params)))
# print("Optimal Chord Parameters:", ', '.join(map(str, optimal_chord_params)))
# print("Minimum power:", result.fun)




