import sys
import os

import numpy as np
import scipy    # type: ignore
import matplotlib.pyplot as plt
import scienceplots

sys.path.append('.')
from helper_funcs import filepath, plot_figures, find_highest_index


if __name__ == '__main__':
    plt.style.use('science')

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

    r_full = r_final_array[file_exists]

    v_full = v_vertical_final[file_exists]

    crashed_full = crashed[file_exists]

    mass_full = mass_array[file_exists]

    thrust_full = thruster_array[file_exists]

    length = 6
    print("-"*10)
    print(f"Average distance                    : {f'{str(np.mean(r_full[crashed_full])):<{length+1}}'.replace(' ', '0')[0:length]}")

    radius = 10
    print(f"Crash percentage                    : {f'{str(np.mean(crashed_full * (r_full > radius))):<{length+1}}'.replace(' ', '0')[0:length]}")
    print(f"Crash distance                      : {f'{str(np.mean(crashed_full * (r_full > radius))):<{length+1}}'.replace(' ', '0')[0:length]}")

    radius = 10
    print(f"Land percentage                     : {f'{str(np.mean(crashed_full * (r_full < radius))):<{length+1}}'.replace(' ', '0')[0:length]}")

    radius = 0.4
    print(f"Land percentage accurate            : {f'{str(np.mean(crashed_full * (r_full < radius))):<{length+1}}'.replace(' ', '0')[0:length]}")
    print()
    

    # binning plots of r_final compared to mass
    plt.hist(r_full[crashed_full], bins=50, density=True, label='r/mass')
    continuous_kde = scipy.stats.gaussian_kde(r_full[crashed_full])
    r_steps = np.arange(0, 2, 0.0001)
    plt.plot(r_steps, continuous_kde(r_steps))
    plt.show()

