import numpy as np
from scipy.optimize import minimize
from blade import Blade
from rotor_blade import calculate_blade_properties

############## INPUTS ################
number_of_blades = 4
radial_stations = [0.08, 0.25, 0.75, 1.00]
airfoils = ['Diamond', 'Triangle', 'DEP 0.5', 'DEP 0.7']
rotor_radius = 1.2
maximum_alpha = 10
######################################

optimal_pitch_params_list = []
optimal_chord_params_list = []

# for iterations in range(100):

# Define the objective function
def objective(params):
    pitch_params = params[:3]
    chord_params = params[3:7]

    blade = Blade(radius_rotor= rotor_radius,
                num_blades=number_of_blades,
                radial_nondim_stations=radial_stations,
                chord_nondim_stations=chord_params,
                pitch_params=pitch_params,
                airfoil_name_stations= airfoils)
    calculate_blade_properties(blade)

    # print(np.rad2deg(blade.alpha))
    # obj = (np.multiply(blade.lift_slope,blade.alpha))/blade.drag_coefficient

    return -blade.thrust_coefficient_rotor

# Define the constraint function
def constraint(params):
    pitch_params = params[:3]
    chord_params = params[3:7]

    blade = Blade(radius_rotor=1.20,
                num_blades=number_of_blades,
                radial_nondim_stations=radial_stations,
                chord_nondim_stations=chord_params,
                pitch_params=pitch_params,
                airfoil_name_stations=['Diamond', 'Triangle', 'DEP 0.5', 'DEP 0.7'])
    calculate_blade_properties(blade)

    # Constraint parameters

    reynolds_constraints = []
    max_reynolds_last = 20000
    max_reynolds_first = 5000
    num_reynolds = len(blade.reynolds)
    for i in range(num_reynolds):
        if i == 0:
            reynolds_constraints.append(max_reynolds_first - min(blade.reynolds[i], max_reynolds_first))
        elif i == num_reynolds - 1:
            reynolds_constraints.append(max(blade.reynolds[i], max_reynolds_last) - max_reynolds_last)
        else:
            max_reynolds_intermediate = max_reynolds_first + (max_reynolds_last - max_reynolds_first) * i / (num_reynolds - 1)
            reynolds_constraints.append(max_reynolds_intermediate - min(max_reynolds_intermediate, max(max_reynolds_first, min(blade.reynolds[i], max_reynolds_last))))

    return [0.12 - (blade.thrust_coefficient_rotor / blade.solidity_rotor), 
            maximum_alpha - np.rad2deg(blade.alpha), 2.5 - np.max(chord_params)/chord_params[0], reynolds_constraints, 0.25 - blade.solidity_rotor,
            pitch_params[0] - pitch_params[1], chord_params[2] - chord_params[3], chord_params[1] - chord_params[2]]

initial_guess = np.concatenate([
    np.random.uniform(0, 0.5, 2), np.random.uniform(-0.5, 0.0, 1),
    np.random.uniform(0.0, 0.2, 4)
])



bounds = [
    (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.0),  # Pitch parameters
    (0.00, radial_stations[0] * np.tan(np.pi / number_of_blades)),
    (0.0, radial_stations[1] * np.tan(np.pi / number_of_blades)),
    (0.0, radial_stations[2] * np.tan(np.pi / number_of_blades)),
    (0.0, radial_stations[3] * np.tan(np.pi / number_of_blades))  # Chord parameters
]

constraints = [
    {'type': 'ineq', 'fun': lambda params: constraint(params)[0]},
    {'type': 'ineq', 'fun': lambda params: constraint(params)[1]},
    {'type': 'ineq', 'fun': lambda params: constraint(params)[2]},
    {'type': 'ineq', 'fun': lambda params: constraint(params)[3]},
    {'type': 'ineq', 'fun': lambda params: constraint(params)[4]},
    {'type': 'ineq', 'fun': lambda params: constraint(params)[5]},
    {'type': 'ineq', 'fun': lambda params: constraint(params)[6]},
    {'type': 'ineq', 'fun': lambda params: constraint(params)[7]}
    
]

result = minimize(objective, initial_guess, method='SLSQP', bounds=bounds, constraints=constraints)

optimal_params = result.x
optimal_pitch_params = optimal_params[:3]
optimal_chord_params = optimal_params[3:7]

# print(optimal_params)

# optimal_pitch_params_list.append(optimal_pitch_params[0])

# print(optimal_pitch_params_list)

# import scipy.stats as stats
# import matplotlib.pyplot as plt

# optimal_pitch_params_list.sort()
# hmean = np.mean(optimal_pitch_params_list)
# hstd = np.std(optimal_pitch_params_list)
# pdf = stats.norm.pdf(optimal_pitch_params_list, hmean, hstd)
# plt.plot(optimal_pitch_params_list, pdf)
# plt.show()





print("Optimal Pitch Parameters:", optimal_pitch_params)
print("Optimal Chord Parameters:", optimal_chord_params)
print("Maximum Thrust Coefficient:", -result.fun)






