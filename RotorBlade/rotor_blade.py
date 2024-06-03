import numpy as np
import matplotlib.pyplot as plt
from blade import Blade


def initialise_blade():
    radius_rotor = 1.20
    num_blades = 4
    radial_nondim_stations = [0.08, 0.25, 0.75, 1.00]
    chord_nondim_stations = [0.12, 0.16, 0.14, 0.10]
    pitch_params = [0.2, 0.1, -0.3]
    airfoil_name_stations = ['Diamond', 'Triangle', 'DEP 0.5', 'DEP 0.7']
    return Blade(radius_rotor, num_blades, radial_nondim_stations, chord_nondim_stations, pitch_params, airfoil_name_stations)

def calculate_blade_properties(blade):
    gamma_air = 1.29976             # Specific heat ratio [-]
    gas_constant_air = 190.96742    # Gas constant [J/(kg K)]
    temp_air = 270.307              # Temperature [K]
    density_air = 0.012718839       # Density [kg/m^3]
    viscosity_air = 1.3854112e-5    # Dynamic viscosity [kg/(m s)]
    mach_tip = 0.7                  # Tip Mach number [-]

    blade.calculate_reynolds(gamma_air, gas_constant_air, temp_air, density_air, viscosity_air, mach_tip)
    blade.calculate_pitch()
    blade.interpolate_airfoil_params()
    blade.calculate_loop()
    blade.calculate_thrust_coefficient()
    blade.calculate_power_coefficient()
    blade.calculate_thrust_and_power(density_air)

def print_blade_properties(blade):
    print(f'{blade.chord_mean                       = :>10.4g} m')
    print(f'{blade.aspect_ratio                     = :>10.4g}')
    print(f'{blade.area_blade                       = :>10.4g} m^2')
    print(f'{blade.area_blades                      = :>10.4g} m^2')
    print(f'{blade.area_rotor                       = :>10.4g} m^2')
    print(f'{blade.solidity_rotor                   = :>10.4g}')
    print(f'{blade.thrust_coefficient_rotor         = :>10.4e}')
    print(f'{blade.thrust_rotor                     = :>10.4g} N')
    print(f'{blade.power_induced_coefficient_rotor  = :>10.4e}')
    print(f'{blade.power_induced_rotor              = :>10.4g} W')

def plot_blade_properties(blade):
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
    if 1:   # Lift slope
        plt.plot(blade.radial_nondim, blade.lift_slope)
        plt.xlabel('$r/R$ [-]')
        plt.ylabel('$dC_L/d\\alpha$ [1/deg]')
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
    if 0:   # Thrust coefficient and slope
        plt.plot(blade.radial_nondim, blade.thrust_slope, label='$dC_T/d(r/R)$')
        plt.plot(blade.radial_nondim, blade.thrust_coefficient, label='$C_T$')
        plt.xlabel('$r/R$ [-]')
        plt.ylabel('$C_T$, $dC_T/d(r/R)$ [-]')
        plt.xlim(0, 1)
        plt.ylim(bottom=0)
        plt.minorticks_on()
        plt.grid(which='both')
        plt.legend()
        plt.show()
    if 0:   # Power coefficient
        plt.plot(blade.radial_nondim, blade.power_induced_coefficient)
        plt.xlabel('$r/R$ [-]')
        plt.ylabel('$C_{P_i}$ [-]')
        plt.xlim(0, 1)
        plt.ylim(bottom=0)
        plt.minorticks_on()
        plt.grid(which='both')
        plt.show()

def main():
    blade = initialise_blade()
    calculate_blade_properties(blade)
    print_blade_properties(blade)
    plot_blade_properties(blade)

if __name__ == '__main__':
    main()