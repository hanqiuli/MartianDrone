"""# Rotor Blade
Provides a class that calculates the performance of a rotor blade in hover.
    - The blade is defined by its planform geometry and pitch distribution.
    - The Reynolds number distribution over the rotor blade is calculated.
    - The thrust coefficient of the rotor blade and the rotor are calculated.
    - The thrust of the rotor is calculated based on the thrust coefficient and the speed of the rotor tip.
"""
import numpy as np

class Blade:
    def __init__(self, radius_rotor: float, num_blades: int, radial_nondim_stations: list[float], chord_nondim_stations: list[float]):
        """Initializes the rotor blade.
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
        self.radial = self.radial_nondim * self.radius_rotor
        self.chord = self.chord_nondim * self.radius_rotor

        self.solidity = self.num_blades * self.chord_nondim / np.pi
        self.lift_slope = 2 * np.pi

        self.leading_edge = 0.25 * self.chord_nondim
        self.trailing_edge = -0.75 * self.chord_nondim

        self.mean_chord = np.trapz(self.chord, self.radial_nondim)
        self.area_blade = np.trapz(self.chord, self.radial)
        self.area_blades = self.area_blade * self.num_blades
        self.area_rotor = np.pi * self.radius_rotor**2
        self.solidity_rotor = self.area_blades / self.area_rotor
        self.aspect_ratio = self.radius_rotor / np.mean(self.chord_nondim)

    def calculate_pitch(self, pitch_root: float, twist: float, pitch_tip: float, reverse_pitch = True):
        """Calculates the pitch angle of the rotor blade.
        Args:
            pitch_root: The pitch angle at the root of the rotor blade. [deg]
            twist: The twist of the rotor blade. [deg/m]
            pitch_tip: The pitch angle at the tip of the rotor blade. [deg]
            reverse_pitch: True for linear increasing pitch, False for inverse decreasing pitch. [bool]
        """
        if reverse_pitch:
            self.pitch = pitch_tip / self.radial_nondim
        else:
            self.pitch = pitch_root + twist * self.radial_nondim
        self.pitch = np.deg2rad(self.pitch)

    def calculate_inflow(self):
        """Calculates the nondimensionalised inflow in hover.
        """
        self.inflow = self.solidity * self.lift_slope / 16 * (np.sqrt(1 + 32 * self.pitch * self.radial_nondim / (self.solidity * self.lift_slope)) - 1)
        self.inflow_angle = self.inflow / self.radial_nondim

    def calculate_angle_of_attack(self):
        """Calculates the angle of attack of the rotor blade.
        """
        self.angle_of_attack = self.pitch - self.inflow / self.radial_nondim

    def calculate_thrust_coefficient(self):
        """Calculates the thrust slope of the rotor blade.
        """
        self.thrust_slope = self.solidity * self.lift_slope / 2 * (self.pitch * self.radial_nondim**2 - self.inflow * self.radial_nondim)
        self.thrust_coefficient = np.zeros_like(self.radial_nondim)
        for i in range(len(self.radial_nondim)):
            self.thrust_coefficient[i] = np.trapz(self.thrust_slope[:i], self.radial_nondim[:i])
        self.thrust_coefficient_blade = np.trapz(self.thrust_slope, self.radial_nondim)
        self.thrust_coefficient_rotor = self.thrust_coefficient_blade * self.num_blades

    def calculate_power_coefficient(self):
        """Calculates the power coefficient of the rotor blade.
        """
        self.power_induced_coefficient_blade = np.trapz(self.inflow * self.thrust_slope, self.radial_nondim)
        self.power_induced_coefficient_rotor = self.power_induced_coefficient_blade * self.num_blades

    def calculate_thrust_and_power(self, density_air: float):
        """Calculates the thrust and power of the rotor blade.
        """
        self.thrust = self.thrust_coefficient_rotor * density_air * self.area_rotor * self.speed_tip**2
        self.power_induced = self.power_induced_coefficient_rotor * density_air * self.area_rotor * self.speed_tip**3

    def calculate_reynolds(self, gamma_air: float, gas_constant_air: float, temp_air: float, density_air: float, viscosity_air: float, mach_tip: float):
        """Calculates the Reynolds number distribution over the rotor blade.
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