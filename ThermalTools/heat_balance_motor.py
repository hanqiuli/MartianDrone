import numpy as np
from sympy.solvers import solve
from sympy import Symbol
from scipy.integrate import odeint
import matplotlib.pyplot as plt

ENV = {'rho':      0.017, 
            'g':        3.71, 
            'a':        233.1,
            'Re_min':   10000,
            'Re_max':   50000,
            'mu':       0.0000113}

rho_air     = ENV['rho']    # [kg/m^3]      Air density
g           = ENV['g']      # [m/s^2]       Gravitational acceleration
dyn_vsc_air = ENV['mu']   # [Pa*s]        Air dynamic viscosity
cp_air      = 772       # [J/(kg*K)]    Specific heat capacity of air
k_air       = 0.024     # [W/(m*K)]     Air thermal conductivity
sigma       = 5.67e-8   # Stefan-Boltzmann constant [W/(m^2*K^4)]

kin_vsc_air = dyn_vsc_air / rho_air         # [m^2/s]   Air kinematic viscosity
alpha_air   = k_air / (rho_air * cp_air)    # [m^2/s]   Air thermal diffusivity

class FinParameters:
    def __init__(self):
        # Rotorcraft parameters
        self.power_motor_total = 5500  # [W] Total power of the motors
        self.eff_motor = 0.75  # [-] Motor efficiency

        self.mass_motor_total = 3.125  # [kg] Total mass of the motors
        
        self.num_rotors = 6  # [-] Number of rotors

        # Motor parameters
        self.heat_capacity_motor = 900.0  # [J/(kg*K)] Motor specific heat capacity
        self.conductivity_motor = 237.0  # [W/(m*K)] Motor thermal conductivity
        self.diameter_motor = 0.08  # [m] Motor diameter

        self.heat_motor = (1 - self.eff_motor) * self.power_motor_total/self.num_rotors  # [W] Motor heat
        self.mass_motor = self.mass_motor_total / self.num_rotors  # [kg] Motor mass

        self.velocity_downwash_rotor = 22  # [rad/s] Rotor downwash velocity

        # Fin parameters
        self.length_fin = 0.15  # [m] Fin length
        self.height_fin = 0.04  # [m] Fin height
        self.thickness_fin = 0.002  # [m] Fin thickness
        self.spacing_fin = 0.005  # [m] Fin spacing
        self.density_fin = 2700  # [kg/m^3] Fin density

        self.num_fins = np.floor(np.pi * self.diameter_motor / (self.thickness_fin + self.spacing_fin))  # [-] Number of fins
        self.area_unfinned = np.pi * self.diameter_motor * self.length_fin - self.num_fins * self.length_fin * self.thickness_fin  # [m^2] Unfinned area
        self.area_fin = 2 * self.height_fin * self.length_fin + 2 * self.height_fin * self.thickness_fin + self.length_fin * self.thickness_fin  # [m^2] Fin area
        self.area_fin_crosssection = self.length_fin * self.thickness_fin  # [m^2] Fin cross-sectional area
        self.perimiter_fin = 2 * self.length_fin + 2 * self.thickness_fin  # [m] Fin perimeter
        self.area_fin_total = self.num_fins * self.area_fin + self.area_unfinned  # [m^2] Total surface area
        self.area_fin_crosssection_total = self.area_fin_crosssection * self.num_fins  # [m^2] Total Top cross-sectional surface area


        self.volume_fin = self.num_fins * self.length_fin * self.height_fin * self.thickness_fin  # [m^3] Fin volume
        self.mass_fin = self.volume_fin * self.density_fin  # [kg] Fin mass
        self.mass_fin_total = self.mass_fin * self.num_rotors  # [kg] Total fin mass

    def calc_prandtl_num(self):  # Prandtl number
        self.prandtl_num = kin_vsc_air / alpha_air
        return self.prandtl_num

    def calc_reynolds_num(self):  # Reynolds number
        self.reynolds_num = self.velocity_downwash_rotor * self.length_fin / kin_vsc_air
        return self.reynolds_num

    def calc_nusselt_number(self):  # Nusselt number
        self.nusselt_number = 0.664 * self.reynolds_num ** (1 / 2) * self.prandtl_num ** (1 / 3)
        return self.nusselt_number

    def calc_coefficient_convection(self):  # Convective heat transfer coefficient
        self.coefficient_convection = self.nusselt_number * k_air / self.length_fin
        return self.coefficient_convection

    def calc_efficiency_fin(self):  # Fin efficiency
        const = np.sqrt(self.coefficient_convection * self.perimiter_fin / (k_s * self.A_c))
        u1 = np.sqrt(self.coefficient_convection * self.perimiter_fin * k_s * self.A_c)
        u2 = np.tanh(const * self.height_fin) + self.coefficient_convection / (const * k_s)
        l1 = self.coefficient_convection * self.area_fin
        l2 = 1 + self.coefficient_convection / (const * k_s) * np.tanh(const * self.height_fin)
        return u1 / l1 * u2 / l2

    def calc_thermal_resistance(self, h, eff_fin):  # Thermal resistance
        return 1 / (h * (eff_fin * self.num_fins * self.area_fin + self.area_unfinned))

fins = FinParameters()
area_total = fins.area_fin_total
area_top = fins.area_fin_crosssection_total


# heat_balance_cold = {
#     'temperature_atmosphere': 210,  # [K] - Atmospheric temperature
#     'irradiance_sun': 0,  # [W/m^2] - Solar irradiance
#     'absorptivity': FILL,  # [-] - Absorptance of the motor
#     'f_sr': 1,  # [-] - View factor for solar radiation
#     'area_top': FILL,  # [m^2] - Top surface area of the motor

#     'albedo': 0.4,  # [-] - Albedo of Mars

#     'temperature_effective_mars': 209.8,  # [K] - Effective temperature for radiation of black body of Mars
#     'emissivity_mars': 0.65,  # [-]-  Emissivity of Mars
#     'stefan_boltzmann_constant': 5.6704e-8,  # [W/m^2*K^4] - Stefan-Boltzmann constant

#     # motor properties
#     'heat_rate_internal': {'motor': 0.25*5500},  # [W] - Heat from internal sources

#     'coefficient_convection': FILL,  # [W/m^2*K] - Convection coefficient
#     'area_total': area_fin_total,  # [m^2] - Total surface area of the motor
#     'f_re': FILL,  # [-] - View factor for emitted radiation
#     'emissivity_motor': FILL,  # [-] - Emissivity of the motor
#     'motor_mass': FILL,  # [kg] - Mass of the motor
#     'motor_heat_capacity': FILL,  # [J/kg*K] - Specific heat capacity of the motor
# }

heat_balance_hot = {
    'temperature_atmosphere': 269,  # [K] - Atmospheric temperature
    'temperature_ground': 292.79,  # [K] - Ground temperature
    'irradiance_sun': 646,  # [W/m^2] - Solar irradiance
    'absorptivity': 0.2,  # [-] - Absorptance of the motor
    'f_sr': 1,  # [-] - View factor for solar radiation
    'area_top': area_top,  # [m^2] - Top surface area of the motor

    'albedo': 0.4,  # [-] - Albedo of Mars

    'temperature_effective_mars': 209.8,  # [K] - Effective temperature for radiation of black body of Mars
    'emissivity_mars': 0.65,  # [-]-  Emissivity of Mars
    'stefan_boltzmann_constant': 5.6704e-8,  # [W/m^2*K^4] - Stefan-Boltzmann constant

    # motor properties
    'heat_rate_internal': {'motor': 0.25*5500},  # [W] - Heat from internal sources

    'coefficient_convection': 1,  # [W/m^2*K] - Convection coefficient
    'area_total': area_total,  # [m^2] - Total surface area of the motor
    'f_re': 1,  # [-] - View factor for emitted radiation
    'emissivity_motor': 0.85,  # [-] - Emissivity of the motor
    'motor_mass': 9.5,  # [kg] - Mass of the motor
    'motor_heat_capacity': 1100,  # [J/kg*K] - Heat capacity of the motor

    # Insulator properties
    'thermal_conductivity': 216,  # [W/m*K] - Thermal conductivity of the insulator (Cork)
    'thickness_insulator': 0.1  # [m] - Thickness of the insulator
}

class MotorHeatTransfer:
    """
    This class models the heat transfer of the motor of the Martian Drone motor.
    """

    def __init__(self):
        # Environmental properties
        heat_dictionary = heat_balance_hot
        self.temperature_atmosphere = heat_dictionary['temperature_atmosphere']  # [K] - Atmospheric temperature
        self.temperature_ground = heat_dictionary['temperature_ground']  # [K] - Ground temperature
        self.irradiance_sun = heat_dictionary['irradiance_sun']  # [W/m^2] - Solar irradiance
        self.absorptivity = heat_dictionary['absorptivity']  # [-] - Absorptance of the motor
        self.f_sr = heat_dictionary['f_sr']  # [-] - View factor for solar radiation
        self.area_top = heat_dictionary['area_top']  # [m^2] - Top surface area of the motor
        
        self.albedo = heat_dictionary['albedo']  # [-] - Albedo of Mars
        
        self.temperature_effective_mars = heat_dictionary['temperature_effective_mars']  # [K] - Effective temperature for radiation of black body of Mars
        self.emissivity_mars = heat_dictionary['emissivity_mars']  # [-]-  Emissivity of Mars
        self.stefan_boltzmann_constant = heat_dictionary['stefan_boltzmann_constant']  # [W/m^2*K^4] - Stefan-Boltzmann constant

        # motor properties
        self.heat_rate_internal = sum(heat_dictionary['heat_rate_internal'].values())  # [W] - Total heat from internal sources
        
        self.coefficient_convection = heat_dictionary['coefficient_convection']  # [W/m^2*K] - Convection coefficient
        self.area_total = heat_dictionary['area_total']  # [m^2] - Total surface area of the motor
        self.f_re = heat_dictionary['f_re']  # [-] - View factor for emitted radiation
        self.emissivity_motor = heat_dictionary['emissivity_motor']  # [-] - Emissivity of the motor
        self.motor_mass = heat_dictionary['motor_mass']  # [kg] - Mass of the motor
        self.motor_heat_capacity = heat_dictionary['motor_heat_capacity']  # [J/kg*K] - Heat capacity of the motor

        # Insulator properties
        self.thermal_conductivity = heat_dictionary['thermal_conductivity']  # [W/m*K] - Thermal conductivity of the insulator (Cork)
        self.thickness_insulator = heat_dictionary['thickness_insulator']  # [m] - Thickness of the insulator
    
    def heat_rate_conduction(self, temperature_motor):
        """
        Calculates the heat input due to conduction from the motor to the arm.

        Args:
            temperature_motor: The current temperature of the motor [K].

        Returns:
            The heat input due to conduction [W].
        """

        heat_rate_cond = - self.thermal_conductivity * self.area_total / self.thickness_insulator * (temperature_motor - self.temperature_atmosphere) 
        return 0
    
    def heat_rate_external_input(self, temperature_motor):
        """
        Calculates the total heat input from external sources.

        Args:
            temperature_motor: The current temperature of the motor [K].

        Returns:
            A tuple containing the total heat input [W] and individual components [W].
        """

        heat_rate_sun = self.absorptivity * self.irradiance_sun * self.area_top * self.f_sr
        
        heat_rate_albedo = self.albedo * self.absorptivity * self.irradiance_sun * self.area_top * self.f_sr
        
        j_p = self.emissivity_mars * self.stefan_boltzmann_constant * self.temperature_effective_mars**4
        heat_rate_ir = j_p * self.area_top
        
        heat_rate_cond = self.heat_rate_conduction(temperature_motor)
        
        total_heat_rate_ext = heat_rate_sun + heat_rate_albedo + heat_rate_ir + heat_rate_cond
        return total_heat_rate_ext, (heat_rate_sun, heat_rate_albedo, heat_rate_ir, heat_rate_cond)

    def heat_rate_internal_input(self):
        """
        Calculates the total heat input from internal sources.

        Returns:
            The total heat input from electronics and motor [W].
        """

        return self.heat_rate_internal

    def heat_rate_convection(self, temperature_motor):
        """
        Calculates the heat loss due to convection with the atmosphere.

        Args:
            temperature_motor: The current temperature of the motor [K].

        Returns:
            The heat loss due to convection [W].
        """

        heat_rate_conv = self.coefficient_convection * (temperature_motor - self.temperature_atmosphere) * self.area_total
        return heat_rate_conv

    def heat_rate_out(self, temperature_motor):
        """
        Calculates the heat loss due to radiation to the environment.

        Args:
            temperature_motor: The current temperature of the motor [K].

        Returns:
            The heat loss due to radiation [W].
        """

        heat_rate_out = self.stefan_boltzmann_constant * self.emissivity_motor * (temperature_motor**4 - self.temperature_atmosphere**4) * self.area_total
        return heat_rate_out 

    def heat_rate_balance(self, temperature_motor):
        """
        Calculates the balance equation to solve for the motor's temperature.

        Args:
            temperature_motor: The current temperature of the motor [K].

        Returns:
            The value of the balance equation [W] (should be zero at equilibrium).
        """

        heat_rate_ext, _ = self.heat_rate_external_input(temperature_motor)
        heat_rate_int = self.heat_rate_internal_input()
        heat_rate_conv = self.heat_rate_convection(temperature_motor)
        heat_rate_out = self.heat_rate_out(temperature_motor)
        return heat_rate_ext + heat_rate_int - heat_rate_conv - heat_rate_out

    def temperature_time_derivative(self, temperature_motor, t):
        """
        Calculates the derivative of the temperature of the motor over time.

        Args:
            temperature_motor: The current temperature of the motor [K].
            t: The current time [s].

        Returns:
            The derivative of the temperature of the motor over time [K/s].
        """
        return 1/(self.motor_mass*self.motor_heat_capacity)*self.heat_rate_balance(temperature_motor)

    def solve_temperature_time(self):
        """
        Solves the heat balance equation to find the temperature of the motor over time.

        Returns:
            The temperature of the motor over time [K] [s].
        """
        
        t = np.linspace(0,20*60, 1000)
        temperature = odeint(self.temperature_time_derivative, y0=270, t=t)
        return temperature, t
    
    def plot_temp_time(self):
        """
        Plots the temperature of the motor over time.
        """
        y, t = self.solve_temperature_time()
        plt.plot(t, y)
        plt.xlabel('t')
        plt.ylabel('T(t)')
        plt.show()
        pass

    def solve_equilibrium_temperature(self):
        """
        Solves the heat balance equation to find the equilibrium temperature 
        of the motor.

        Returns:
            The equilibrium temperature of the motor [K].
        """

        x = Symbol('x')
        temperature_equilibrium_solutions = solve(self.heat_rate_balance(x), x)
        return temperature_equilibrium_solutions[1]


def main1():
    motor = MotorHeatTransfer()
    motor.solve_equilibrium_temperature()

def main2():
    motor = MotorHeatTransfer()
    motor.plot_temp_time()

if __name__ == "__main__":
    main2()
