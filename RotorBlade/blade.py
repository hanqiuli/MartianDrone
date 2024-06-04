"""# Rotor Blade
Provides a class that calculates the performance of a rotor blade in hover.
    - The blade is defined by its planform geometry and pitch distribution.
    - The Reynolds number distribution over the rotor blade is calculated.
    - The thrust coefficient of the rotor blade and the rotor are calculated.
    - The thrust of the rotor is calculated based on the thrust coefficient and the speed of the rotor tip.
"""
import numpy as np
from scipy import integrate, interpolate
from airfoil import Airfoil

class Blade:
    def __init__(self, 
                 radius_rotor: float,
                 num_blades: int, 
                 radial_nondim_stations: list[float], 
                 chord_nondim_stations: list[float], 
                 pitch_params: list[float], 
                 airfoil_name_stations: list[str]) -> None:
        """Initializes:
            radial, radial_nondim: The radial position over the rotor blade. [m, -]
            chord, chord_nondim: The chord distribution over the rotor blade. [m, -]
            solidity: The solidity distribution over the rotor blade. [-]
            leading_edge, trailing_edge: The leading and trailing edge of the rotor blade. [m, m]
            chord_mean: The mean chord of the rotor blade. [m]
            aspect_ratio: The aspect ratio of the rotor blade. [-]
            area_blade: The area of the rotor blade. [m^2]
            area_blades: The total area of the rotor blades. [m^2]
            area_rotor: The area of the rotor disk. [m^2]
            solidity_rotor: The solidity of the rotor disk. [-]
        Args:
            radius_rotor: The radius of the rotor disk. [m]
            num_blades: The number of blades per rotor disk. [-]
            radial_nondim_stations: The nondimensional radial stations where the chord is specified. [-]
            chord_nondim_stations: The nondimensional chord at the specified radial stations. [-]
            pitch_params: The quadratic pitch distribution parameters [root pitch, tip pitch, root pitch slope]. [rad, rad, rad/m]
        """
        if not (0 <= radial_nondim_stations[0] < radial_nondim_stations[-1] <= 1) or radial_nondim_stations != sorted(radial_nondim_stations):
            raise ValueError("Radial nondimensional stations must be between 0 and 1 and in ascending order.")
        if len(radial_nondim_stations) != len(chord_nondim_stations):
            raise ValueError("Radial and chord nondimensional stations must have the same length.")
        if len(pitch_params) != 3:
            raise ValueError("Pitch parameters must be a float list in the form [root pitch, tip pitch, root pitch slope].")
        if len(airfoil_name_stations) != len(radial_nondim_stations):
            raise ValueError("Airfoil name stations must have the same length as the radial nondimensional stations.")
        
        self.radius_rotor = radius_rotor
        self.num_blades = num_blades
        self.radial_nondim_stations = radial_nondim_stations
        self.chord_nondim_stations = chord_nondim_stations
        self.airfoil_stations = [Airfoil(name) for name in airfoil_name_stations]
        self.radial_nondim = np.linspace(self.radial_nondim_stations[0], self.radial_nondim_stations[-1], 1000)
        self.chord_nondim = np.interp(self.radial_nondim, self.radial_nondim_stations, self.chord_nondim_stations)
        self.radial = self.radial_nondim * self.radius_rotor
        self.chord = self.chord_nondim * self.radius_rotor
        self.leading_edge = 0.25 * self.chord_nondim
        self.trailing_edge = -0.75 * self.chord_nondim
        self.solidity = self.num_blades * self.chord_nondim / np.pi
        self.pitch_params = pitch_params

        self.chord_mean = np.trapz(self.chord, self.radial_nondim)
        self.aspect_ratio = self.radius_rotor / self.chord_mean
        self.area_blade = np.trapz(self.chord, self.radial)
        self.area_blades = self.area_blade * self.num_blades
        self.area_rotor = np.pi * self.radius_rotor**2
        self.solidity_rotor = self.area_blades / self.area_rotor

    def calculate_pitch(self):
        """Calculates:
            pitch: The pitch distribution over the rotor blade. [rad]
        """
        theta_root = self.pitch_params[0]
        theta_tip = self.pitch_params[1]
        theta_slope_root = self.pitch_params[2]
        a0 = theta_root
        a1 = theta_slope_root
        a2 = (theta_tip - theta_root - theta_slope_root*(1 - self.radial_nondim[0])) / (1 - self.radial_nondim[0])**2
        quadratic_params = [a2, a1, a0]
        self.pitch = np.polyval(quadratic_params, self.radial_nondim - self.radial_nondim[0])

    def interpolate_airfoil_params(self):
        """Provides a lookup table for the lift slope of the airfoil at a given angle of attack, and interpolates the array of lift slopes over the rotor blade.
        """
        self.cls_stations = []
        self.cds_stations = []
        for airfoil in self.airfoil_stations:
            airfoil.interpolate_cl_cd()
            self.cls_stations.append(airfoil.cls)
            self.cds_stations.append(airfoil.cds)
        self.cls_stations = np.array(self.cls_stations)
        self.cds_stations = np.array(self.cds_stations)
        alphas_stations = np.linspace(0.0, 10.0, 11)
        alphas = np.linspace(0.0, 10.0, 1000)
        self.cls = interpolate.RectBivariateSpline(self.radial_nondim_stations, alphas_stations, self.cls_stations, kx=3, ky=3)(self.radial_nondim, alphas)
        self.cds = interpolate.RectBivariateSpline(self.radial_nondim_stations, alphas_stations, self.cds_stations, kx=3, ky=3)(self.radial_nondim, alphas)
        self.cls = np.array(self.cls)
        self.cds = np.array(self.cds)
        print(self.cls.shape)
        # self.cls = interpolate.interp1d(self.radial_nondim_stations, self.cls_stations, axis=0)(self.radial_nondim)
        # self.cds = interpolate.interp1d(self.radial_nondim_stations, self.cds_stations, axis=0)(self.radial_nondim)
        clinv = 2 * self.cls[0] - np.flip(self.cls, axis=1)
        cdinv = 2 * self.cds[0] - np.flip(self.cds, axis=1)
        self.cls = np.concatenate((clinv[:,:-1], self.cls), axis=1)
        self.cds = np.concatenate((cdinv[:,:-1], self.cds), axis=1)
        print(self.cls.shape)

    def calculate_aerodynamic_coefficients(self):
        """Calculates:
            cl = The lift coefficient distribution over the rotor blade. [-]
            cd = The drag coefficient distribution over the rotor blade. [-]
            alpha = The angle of attack distribution over the rotor blade. [rad]
            inflow = The inflow distribution over the rotor blade. [rad]
            inflow_angle = The inflow angle distribution over the rotor blade. [rad]
        """
        self.alpha = np.zeros_like(self.radial_nondim)
        self.cl = np.zeros_like(self.radial_nondim)
        self.cd = np.zeros_like(self.radial_nondim)
        for r_index, r in enumerate(self.radial_nondim):
            alphas = np.deg2rad(np.linspace(-10.0, 10.0, 1999))
            constraint = 8 * r * (self.pitch[r_index] - alphas)**2 / self.solidity[r_index]
            cls_constraint = self.cls[r_index] - constraint
            alpha_index = np.argmin(np.abs(cls_constraint))
            self.alpha[r_index] = alphas[alpha_index]
            print(alpha_index, self.alpha[r_index]) if r_index == 899 else None
            self.cl[r_index] = self.cls[r_index, alpha_index]
            self.cd[r_index] = self.cds[r_index, alpha_index]
        print(self.cl[899])
        self.inflow_angle = self.pitch - self.alpha
        self.inflow = self.inflow_angle * self.radial_nondim
        # self.cl = 8 * self.inflow ** 2 / (self.solidity * self.radial_nondim)
        # print(self.cl[899])

    def calculate_thrust_coefficient(self):
        """Calculates:
            thrust_slope: The thrust slope distribution over the rotor blade. [-]
            thrust_coefficient: The thrust coefficient distribution over the rotor blade. [-]
            thrust_coefficient_blade: The thrust coefficient of the rotor blade. [-]
            thrust_coefficient_rotor: The thrust coefficient of the rotor disk. [-]
        """
        self.thrust_slope = self.solidity * self.lift_slope / 2 * (self.pitch * self.radial_nondim**2 - self.inflow * self.radial_nondim)
        self.thrust_coefficient = integrate.cumtrapz(self.thrust_slope, self.radial_nondim, initial=0)
        self.thrust_coefficient_blade = np.trapz(self.thrust_slope, self.radial_nondim)
        self.thrust_coefficient_rotor = self.thrust_coefficient_blade * self.num_blades

    def calculate_power_coefficient(self):
        """Calculates:
            power_induced_coefficient: The induced power coefficient distribution over the rotor blade. [-]
            power_induced_coefficient_blade: The induced power coefficient of the rotor blade. [-]
            power_induced_coefficient_rotor: The induced power coefficient of the rotor disk. [-]
        """
        self.power_induced_coefficient = integrate.cumtrapz(self.inflow * self.thrust_slope, self.radial_nondim, initial=0)
        self.power_induced_coefficient_blade = np.trapz(self.inflow * self.thrust_slope, self.radial_nondim)
        self.power_induced_coefficient_rotor = self.power_induced_coefficient_blade * self.num_blades

    def calculate_thrust_and_power(self, density_air: float):
        """Calculates:
            thrust_rotor: The thrust of the rotor disk. [N]
            power_induced_rotor: The induced power of the rotor disk. [W]
        Args:
            density_air: The density of the air. [kg/m^3]
        """
        self.thrust_rotor = self.thrust_coefficient_rotor * density_air * self.area_rotor * self.speed_tip**2
        self.power_induced_rotor = self.power_induced_coefficient_rotor * density_air * self.area_rotor * self.speed_tip**3

    def calculate_reynolds(self, gamma_air: float, gas_constant_air: float, temp_air: float, density_air: float, viscosity_air: float, mach_tip: float):
        """Calculates:
            speed_tip: The speed of the rotor tip. [m/s]
            speed: The speed of the rotor blade. [m/s]
            reynolds: The Reynolds number distribution over the rotor blade. [-]
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



