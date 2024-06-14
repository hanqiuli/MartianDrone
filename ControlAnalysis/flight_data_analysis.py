import sys
import os

import numpy as np
import matplotlib.pyplot as plt

sys.path.append('.')
from main import filepath, plot_figures, find_highest_index


if __name__ == '__main__':
    data_path = ['ControlAnalysis', 'Data']

    i_max = find_highest_index(os.path.join(*data_path), pattern_prefix='flight_data_', pattern_suffix='.npz')+1
    
    r_final_array = np.zeros(i_max)
    v_vertical_final = np.zeros(i_max)
    
    crashed = np.zeros(i_max, dtype=bool)
    mass_array = np.zeros(i_max)
    thruster_array = np.zeros([i_max, 2])
    skew_array = np.zeros(i_max)

    file_exists = np.zeros(i_max, dtype=bool)

    for i in range(i_max):
        try:
            print(f'flight_data_{i}.npz')
            data = np.load(filepath(f'flight_data_{i}.npz', data_path))
        except Exception as e:
            print(e)
            continue
        
        
        r_final = np.linalg.norm(data['states'][-1, 0:2]-data['setpoints'][-1, 0:2])
        r_final_array[i] = r_final
        v_vertical_final[i] = data['states'][-1, 8]
        file_exists[i] = 1
        mass_array[i] = data['mass']
        thruster_array[i, :] = data['thrust']
        skew_array[i] = data['skew_factor']
        crashed[i] = data['crashed']

        # Plot the flight data
        #plot_figures(data['states'], data['times'], data['setpoints'], data['thruster_values'], data['flight_mode_list'], data['input_array'])


    r_mass = r_final_array[0:100][file_exists[0:100]]
    r_thrust = r_final_array[100:200][file_exists[100:200]]
    r_skew = r_final_array[200:300][file_exists[200:300]]
    r_full = r_final_array[300:400][file_exists[300:400]]

    v_mass = v_vertical_final[0:100][file_exists[0:100]]
    v_thrust = v_vertical_final[100:200][file_exists[100:200]]
    v_skew = v_vertical_final[200:300][file_exists[200:300]]
    v_full = v_vertical_final[300:400][file_exists[300:400]]

    crashed_mass = crashed[0:100][file_exists[0:100]]
    crashed_thrust = crashed[100:200][file_exists[100:200]]
    crashed_skew = crashed[200:300][file_exists[200:300]]
    crashed_full = crashed[300:400][file_exists[300:400]]

    mass_mass = mass_array[0:100][file_exists[0:100]]
    mass_thrust = mass_array[100:200][file_exists[100:200]]
    mass_skew = mass_array[200:300][file_exists[200:300]]
    mass_full = mass_array[300:400][file_exists[300:400]]

    thrust_mass = thruster_array[0:100][file_exists[0:100]]
    thrust_thrust = thruster_array[100:200][file_exists[100:200]]
    thrust_skew = thruster_array[200:300][file_exists[200:300]]
    thrust_full = thruster_array[300:400][file_exists[300:400]]

    length = 6
    print("-"*10)
    print(f"Average distance (mass)             : {f'{str(np.mean(r_mass[crashed_mass])):<{length+1}}'.replace(' ', '0')[0:length]}")
    print(f"Average distance (thrust)           : {f'{str(np.mean(r_thrust[crashed_thrust])):<{length+1}}'.replace(' ', '0')[0:length]}")
    print(f"Average distance (skew)             : {f'{str(np.mean(r_skew[crashed_skew])):<{length+1}}'.replace(' ', '0')[0:length]}")
    print(f"Average distance (full)             : {f'{str(np.mean(r_full[crashed_full])):<{length+1}}'.replace(' ', '0')[0:length]}")
    print()

    radius = 10
    print(f"Crash percentage (mass)             : {f'{str(np.mean(crashed_mass * r_mass > radius)):<{length+1}}'.replace(' ', '0')[0:length]}")
    print(f"Crash percentage (thrust)           : {f'{str(np.mean(crashed_thrust * r_thrust > radius)):<{length+1}}'.replace(' ', '0')[0:length]}")
    print(f"Crash percentage (skew)             : {f'{str(np.mean(crashed_skew * r_skew > radius)):<{length+1}}'.replace(' ', '0')[0:length]}")
    print(f"Crash percentage (full)             : {f'{str(np.mean(crashed_full * r_full > radius)):<{length+1}}'.replace(' ', '0')[0:length]}")
    print()

    radius = 10
    print(f"Land percentage (mass)              : {f'{str(np.mean(crashed_mass * r_mass < radius)):<{length+1}}'.replace(' ', '0')[0:length]}")
    print(f"Land percentage (thrust)            : {f'{str(np.mean(crashed_thrust * r_thrust < radius)):<{length+1}}'.replace(' ', '0')[0:length]}")
    print(f"Land percentage (skew)              : {f'{str(np.mean(crashed_skew * r_skew < radius)):<{length+1}}'.replace(' ', '0')[0:length]}")
    print(f"Land percentage (full)              : {f'{str(np.mean(crashed_full * r_full < radius)):<{length+1}}'.replace(' ', '0')[0:length]}")
    print()

    radius = 0.8
    print(f"Land percentage accurate (mass)     : {f'{str(np.mean(crashed_mass * r_mass < radius)):<{length+1}}'.replace(' ', '0')[0:length]}")
    print(f"Land percentage accurate (thrust)   : {f'{str(np.mean(crashed_thrust * r_thrust < radius)):<{length+1}}'.replace(' ', '0')[0:length]}")
    print(f"Land percentage accurate (skew)     : {f'{str(np.mean(crashed_skew * r_skew < radius)):<{length+1}}'.replace(' ', '0')[0:length]}")
    print(f"Land percentage accurate (full)     : {f'{str(np.mean(crashed_full * r_full < radius)):<{length+1}}'.replace(' ', '0')[0:length]}")
    print()
    

    # binning plots of r_final compared to mass
    plt.hist(r_mass[crashed_mass], bins=20, density=True, label='r/mass')

    plt.legend()
    plt.show()
