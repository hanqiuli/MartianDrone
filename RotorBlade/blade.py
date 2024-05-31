import numpy as np

class Blade:
    def __init__(self, radius_rotor: float, num_blades: int, radial_nondim_stations: list[float], chord_nondim_stations: list[float]):
        """Initializes the planform of the rotor blade.
        Args:
            radius_rotor: The radius of the rotor disk. [m]
            num_blades: The number of blades per rotor disk. [-]
            radial_nondim_stations: The nondimensional radial stations where the chord is specified. [-]
            chord_nondim_stations: The nondimensional chord at the specified radial stations. [-]
        """
        self.radius_rotor = radius_rotor
        self.num_blades = num_blades
        self.radial_nondim = np.linspace(radial_nondim_stations[0], radial_nondim_stations[-1], 10000)
        self.chord_nondim = np.interp(self.radial_nondim, radial_nondim_stations, chord_nondim_stations)
        self.chord = self.chord_nondim * self.radius_rotor
        self.leading_edge = 0.25 * self.chord_nondim
        self.trailing_edge = -0.75 * self.chord_nondim
        self.mean_chord = np.trapz(self.chord_nondim, self.radial_nondim)
        self.area_blade = np.trapz(self.chord, self.radial_nondim)
        self.area_blades = self.area_blade * self.num_blades
        self.area_rotor = np.pi * self.radius_rotor**2
        self.solidity = self.area_blades / self.area_rotor
        self.aspect_ratio = self.radius_rotor / np.mean(self.chord_nondim)

    def calculate_reynolds(self, gamma_air, gas_constant_air, temp_air, density_air, viscosity_air, mach_tip):
        """Calculates the Reynolds number as a function of the radial position on the rotor blade.
        Args:
            gamma_air: The specific heat ratio of air. [-]
            gas_constant_air: The gas constant of air. [J/(kg K)]
            temp_air: The temperature of the air. [K]
            density_air: The density of the air. [kg/m^3]
            viscosity_air: The dynamic viscosity of the air. [kg/(m s)]
            mach_tip: The tip Mach number. [-]
        """
        speed_sound = np.sqrt(gamma_air * gas_constant_air * temp_air)
        self.speed_tip = mach_tip * speed_sound
        self.speed = self.speed_tip * self.radial_nondim
        self.reynolds = density_air * self.speed * self.chord / viscosity_air