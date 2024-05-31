import numpy as np

class Planform:
    def __init__(self, radius_rotor, num_blades, radial_nondim_stations, chord_nondim_stations):
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
        self.area_blade = np.trapz(self.chord, self.radial_nondim)
        self.area_blades = self.area_blade * self.num_blades
        self.area_rotor = np.pi * self.radius_rotor**2
        self.solidity = self.area_blades / self.area_rotor
        self.aspect_ratio = self.radius_rotor / np.mean(self.chord_nondim)

    def calculate_Reynolds(self, gamma_air, heat_ratio_air, temp_air, density_air, viscosity_air, Mach_tip):
        speed_sound = np.sqrt(gamma_air * heat_ratio_air * temp_air)
        self.speed_tip = Mach_tip * speed_sound
        self.speed = self.speed_tip * self.radial_nondim
        self.Reynolds = density_air * self.speed * self.chord / viscosity_air