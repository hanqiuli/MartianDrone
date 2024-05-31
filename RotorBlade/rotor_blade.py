import numpy as np
import matplotlib.pyplot as plt
from blade import Blade

if __name__ == '__main__':
    gas_constant_air = 190.96742
    gamma_air = 1.29976
    temp_air = 270.307
    density_air = 0.012718839
    viscosity_air = 1.3854112e-5
    mach_tip = 0.7
    radius_rotor = 1.00
    num_blades = 4
    radial_nondim_stations = [0.08, 0.25, 0.75, 1.00]
    chord_nondim_stations = [0.12, 0.16, 0.14, 0.10]
    blade = Blade(radius_rotor, num_blades, radial_nondim_stations, chord_nondim_stations)
    blade.calculate_reynolds(gamma_air, gas_constant_air, temp_air, density_air, viscosity_air, mach_tip)
    blade.calculate_pitch(3.0, 15.0, 1.0, reverse_pitch=True)
    blade.calculate_inflow()
    blade.calculate_angle_of_attack()
    blade.calculate_thrust_coefficient()
    blade.calculate_power_coefficient()
    blade.calculate_thrust_and_power(density_air)
    # region Results
    print(f'Mean chord of the blade: {blade.mean_chord} m')
    print(f'Area of the blade: {blade.area_blade} m^2')
    print(f'Aspect ratio of the blade: {blade.aspect_ratio}')
    print(f'Area of all blades: {blade.area_blades} m^2')
    print(f'Area of the rotor disk: {blade.area_rotor} m^2')
    print(f'Solidity of the rotor disk: {blade.solidity_rotor}')
    print(f'Thrust coefficient of the rotor: {blade.thrust_coefficient_rotor}')
    print(f'Thrust of the rotor: {blade.thrust_coefficient_rotor * density_air * blade.area_rotor * blade.speed_tip**2} N')
    print(f'Induced power coefficient of the rotor: {blade.power_induced_coefficient_rotor}')
    print(f'Induced power of the rotor: {blade.power_induced} W')

    if 0:   # Planform geometry
        plt.plot(blade.radial_nondim, blade.leading_edge, label='Leading Edge')
        plt.plot(blade.radial_nondim, blade.trailing_edge, label='Trailing Edge')
        plt.xlabel('$r/R$ [-]')
        plt.ylabel('$c/R$ [-]')
        plt.xlim(0, 1)
        plt.ylim(-0.5, 0.5)
        plt.legend()
        plt.minorticks_on()
        plt.grid(which='both')
        plt.show()
    if 0:   # Chord
        plt.plot(blade.radial_nondim, blade.chord)
        plt.xlabel('$r/R$ [-]')
        plt.ylabel('$c$ [m]')
        plt.xlim(0, 1)
        plt.ylim(bottom=0)
        plt.minorticks_on()
        plt.grid(which='both')
        plt.show()
    if 0:   # Solidity
        plt.plot(blade.radial_nondim, blade.solidity)
        plt.xlabel('$r/R$ [-]')
        plt.ylabel('$\\sigma$ [-]')
        plt.xlim(0, 1)
        plt.ylim(bottom=0)
        plt.minorticks_on()
        plt.grid(which='both')
        plt.show()
    if 0:   # Reynolds number
        plt.plot(blade.radial_nondim, blade.reynolds)
        plt.xlabel('$r/R$ [-]')
        plt.ylabel('$Re$ [-]')
        plt.xlim(0, 1)
        plt.ylim(bottom=0)
        plt.minorticks_on()
        plt.grid(which='both')
        plt.show()
    if 1:   # Pitch, inflow angle and angle of attack
        plt.plot(blade.radial_nondim, np.rad2deg(blade.pitch), label='Pitch')
        plt.plot(blade.radial_nondim, np.rad2deg(blade.inflow_angle), label='Inflow Angle')
        plt.plot(blade.radial_nondim, np.rad2deg(blade.angle_of_attack), label='Angle of Attack')
        plt.xlabel('$r/R$ [-]')
        plt.ylabel('Angle [deg]')
        plt.xlim(0, 1)
        plt.ylim(bottom=0)
        plt.minorticks_on()
        plt.grid(which='both')
        plt.legend()
        plt.show()
    if 0:   # Inflow
        plt.plot(blade.radial_nondim, blade.inflow)
        plt.xlabel('$r/R$ [-]')
        plt.ylabel('$\\lambda$ [-]')
        plt.xlim(0, 1)
        plt.ylim(bottom=0)
        plt.minorticks_on()
        plt.grid(which='both')
        plt.show()
    if 1:   # Thrust coefficient and slope
        plt.plot(blade.radial_nondim, blade.thrust_slope, label='Thrust Coefficient Slope')
        plt.plot(blade.radial_nondim, blade.thrust_coefficient, label='Thrust Coefficient')
        plt.xlabel('$r/R$ [-]')
        plt.ylabel('$C_T$, $dC_T/dr$ [-]')
        plt.xlim(0, 1)
        plt.ylim(bottom=0)
        plt.minorticks_on()
        plt.grid(which='both')
        plt.legend()
        plt.show()
    # endregion