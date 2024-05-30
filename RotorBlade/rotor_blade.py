import numpy as np
import matplotlib.pyplot as plt
from planform import Planform

if __name__ == '__main__':
    heat_ratio_air = 190.96742
    gamma_air = 1.29976
    temp_air = 270.307
    density_air = 0.012718839
    viscosity_air = 1.3854112e-5
    Mach_tip = 0.7
    radius_rotor = 1.00
    num_blades = 4
    radial_nondim_stations = [0.08, 0.25, 0.75, 1.00]
    chord_nondim_stations = [0.12, 0.16, 0.14, 0.10]
    planform = Planform(radius_rotor, num_blades, radial_nondim_stations, chord_nondim_stations)
    planform.calculate_Reynolds(gamma_air, heat_ratio_air, temp_air, density_air, viscosity_air, Mach_tip)
    # region Results
    print(f'Area of the blade: {planform.area_blade} m^2')
    print(f'Mean chord of the blade: {planform.area_blade / radius_rotor} m')
    print(f'Aspect ratio of the blade: {planform.aspect_ratio}')
    print(f'Area of all blades: {planform.area_blades} m^2')
    print(f'Area of the rotor disk: {planform.area_rotor} m^2')
    print(f'Solidity of the rotor disk: {planform.solidity}')
    plt.plot(planform.radial_nondim, planform.leading_edge, label='Leading Edge')
    plt.plot(planform.radial_nondim, planform.trailing_edge, label='Trailing Edge')
    plt.xlabel('$r/R$ [-]')
    plt.ylabel('$c/R$ [-]')
    plt.xlim(0, 1)
    plt.ylim(-0.5, 0.5)
    plt.legend()
    plt.minorticks_on()
    plt.grid(which='both')
    plt.show()
    plt.plot(planform.radial_nondim, planform.Reynolds)
    plt.xlabel('$r/R$ [-]')
    plt.ylabel('$Re$ [-]')
    plt.xlim(0, 1)
    plt.ylim(bottom=0)
    plt.minorticks_on()
    plt.grid(which='both')
    plt.show()
    # endregion