import numpy as np
from sympy.solvers import solve
from sympy import Symbol
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# Physical/environmental constants
g = 3.71             # Gravitational acceleration [m/s^2]
sigma = 5.67e-8           # Stefan-Boltzmann constant [W/(m^2*K^4)]

#HOT
rho_air = 1.3e-2          # Air density [kg/m^3]
dyn_vsc_air = 1.4e-5      # Air dynamic viscosity [Pa*s]
temp_amb = 269            # Ambient temperature [K]
cp_air = 8.3e+2           # Specific heat capacity of air [J/(kg*K)]

#COLD
# rho_air = 1.6e-2          # Air density [kg/m^3]
# dyn_vsc_air = 1.0e-5      # Air dynamic viscosity [Pa*s]
# temp_amb = 204            # Ambient temperature [K]
# cp_air = 7.6e+2           # Specific heat capacity of air [J/(kg*K)]

k_air = 0.0209            # Air thermal conductivity [W/(m*K)]
kinematic_air_viscosity = dyn_vsc_air / rho_air  # Kinematic viscosity [m^2/s]

class FinParameters:
    def __init__(self,num_fins=12):
        # Rotorcraft parameters
        self.num_rotors = 6            # Number of rotors [-]

        # Motor parameters
        self.heat_capacity_motor = 900.0  # Motor specific heat capacity [J/(kg*K)]
        self.conductivity_motor = 209.0   # Motor thermal conductivity [W/(m*K)]
        self.diameter_motor = 0.08        # Motor diameter [m]
        self.velocity_downwash_rotor = 22  # Rotor downwash velocity [rad/s]

        # Fin parameters
        self.length_fin = 0.05             # Fin length [m]
        self.height_fin = 0.20             # Fin height [m]
        self.thickness_fin = 0.002         # Fin thickness [m]
        self.num_fins = num_fins           # Number of fins [-]
        self.density_fin = 2700            # Fin density [kg/m^3]

        self.spacing_fin = np.pi * self.diameter_motor / self.num_fins - self.thickness_fin 
        # self.num_fins = np.floor(np.pi * self.diameter_motor / (self.thickness_fin + self.spacing_fin))  # [-] Number of fins
        self.area_unfinned = np.pi * self.diameter_motor * self.length_fin - self.num_fins * self.length_fin * self.thickness_fin  # Unfinned area [m^2]
        self.area_fin = 2 * self.height_fin * self.length_fin + 2 * self.height_fin * self.thickness_fin + self.length_fin * self.thickness_fin  # Fin area [m^2]
        self.area_fin_crosssection = self.length_fin * self.thickness_fin  # Fin cross-sectional area [m^2]
        self.perimeter_fin = 2 * self.length_fin + 2 * self.thickness_fin  # Fin perimeter [m]
        self.area_fin_total = self.num_fins * self.area_fin + self.area_unfinned  # Total surface area [m^2]
        self.area_fin_crosssection_total = self.area_fin_crosssection * self.num_fins  # Total cross-sectional area [m^2]

        self.volume_fin = self.num_fins * self.length_fin * self.height_fin * self.thickness_fin  # Fin volume [m^3]
        self.mass_fin = self.volume_fin * self.density_fin  # Fin mass [kg]
        self.mass_fin_total = self.mass_fin * self.num_rotors  # Total fin mass [kg]

    def calculate_gebhart(self):
        """Calculate the Gebhart factor for radiation exchange between fins."""
        B12 = 1.0
        B21 = 1.0
        max_iterations = 1000
        tolerance = 1e-6
        eps2 = 0.95  # Fin emissivity
        eps1 = 0.95  # Fin emissivity
        F12 = 1 - np.sin((self.spacing_fin) / (self.diameter_motor) * 0.5)
        F21 = F12

        for _ in range(max_iterations):
            B12_old = B12
            B21_old = B21
            B22 = (1 - eps1) * F21 * B21
            B12 = F12 * eps2 + (1 - eps2) * F12 * B22

            if abs(B12 - B12_old) < tolerance and abs(B22 - B21_old) < tolerance:
                break

        return B12  # Gebhart factor

    def calc_thermal_diffusivity(self):
        """Calculate the thermal diffusivity of air."""
        self.thermal_diffusivity_air = k_air / (rho_air * cp_air)  # [m^2/s]
        return self.thermal_diffusivity_air

    def calc_prandtl_num(self):
        """Calculate the Prandtl number."""
        self.prandtl_num = kinematic_air_viscosity / self.calc_thermal_diffusivity()
        return self.prandtl_num

    def calc_reynolds_num(self):
        """Calculate the Reynolds number."""
        self.reynolds_num = self.velocity_downwash_rotor * self.length_fin / kinematic_air_viscosity
        return self.reynolds_num

    def calc_nusselt_number(self):
        """Calculate the Nusselt number."""
        self.nusselt_number = 0.664 * self.calc_reynolds_num() ** 0.5 * self.calc_prandtl_num() ** (1 / 3)
        return self.nusselt_number

    def calc_coefficient_convection(self):
        """Calculate the convective heat transfer coefficient."""
        self.coefficient_convection = self.calc_nusselt_number() * k_air / self.length_fin
        return self.coefficient_convection

    def calc_efficiency_fin(self):
        """Calculate the efficiency of the fin."""
        const = np.sqrt(self.coefficient_convection * self.perimeter_fin / (self.conductivity_motor * self.area_fin_crosssection))
        numerator_1 = np.sqrt(self.coefficient_convection * self.perimeter_fin * self.conductivity_motor * self.area_fin_crosssection)
        numerator_2 = np.tanh(const * self.height_fin) + self.coefficient_convection / (const * self.conductivity_motor)
        denominator_1 = self.coefficient_convection * self.area_fin
        denominator_2 = 1 + self.coefficient_convection / (const * self.conductivity_motor) * np.tanh(const * self.height_fin)
        self.efficiency_fin = numerator_1 / denominator_1 * numerator_2 / denominator_2
        return self.efficiency_fin

    def calc_thermal_resistance(self):
        """Calculate the thermal resistance."""
        self.thermal_resistance = 1 / (self.calc_coefficient_convection() * (self.calc_efficiency_fin() * self.num_fins * self.area_fin + self.area_unfinned))
        return self.thermal_resistance

fins = FinParameters()
print(fins.calc_prandtl_num())
fin_mass = fins.mass_fin
fin_mass_total = fins.mass_fin_total
print("fin mass: " + str(fin_mass))
print("fin mass total: " + str(fin_mass_total))
area_total = fins.area_fin_total
area_top = fins.area_fin_crosssection_total
thermal_resistance = fins.calc_thermal_resistance()
f_re = 1-fins.calculate_gebhart()
print(f_re)

heat_balance_hot = {
    'temperature_atmosphere': temp_amb,  # [K] - Atmospheric temperature
    'irradiance_sun': 646,  # [W/m^2] - Solar irradiance
    'absorptivity': 0.2,  # [-] - Absorptance of the motor
    'f_sr': 1,  # [-] - View factor for solar radiation
    'area_top': area_top,  # [m^2] - Top surface area of the motor

    'albedo': 0.4,  # [-] - Albedo of Mars

    'temperature_effective_mars': 209.8,  # [K] - Effective temperature for radiation of black body of Mars
    'emissivity_mars': 0.65,  # [-]-  Emissivity of Mars
    'stefan_boltzmann_constant': 5.6704e-8,  # [W/m^2*K^4] - Stefan-Boltzmann constant

    # motor properties
    'heat_rate_internal': {'motor': 0.25*5500/6 * 3/4},  # [W] - Heat from internal sources

    'coefficient_resistance': thermal_resistance,  # [K/W] - Thermal Resistance
    'area_total': area_total,  # [m^2] - Total surface area of the motor
    'f_re': f_re,  # [-] - View factor for emitted radiation
    'emissivity_motor': 0.95,  # [-] - Emissivity of the motor
    'motor_mass': 0.5+fin_mass,  # [kg] - Mass of the motor
    'motor_heat_capacity': 1100,  # [J/kg*K] - Heat capacity of the motor

    # Conductivity properties
    'thermal_conductivity': 216,  # [W/m*K] - Thermal conductivity
}

class MotorHeatTransfer:
    """
    This class models the heat transfer of the motor of the Martian Drone.
    """

    def __init__(self,heat_dictionary=heat_balance_hot):
        # Environmental properties
        self.temperature_atmosphere = heat_dictionary['temperature_atmosphere']  # Atmospheric temperature [K]
        self.irradiance_sun = heat_dictionary['irradiance_sun']  # Solar irradiance [W/m^2]
        self.absorptivity = heat_dictionary['absorptivity']  # Absorptance of the motor [-]
        self.f_sr = heat_dictionary['f_sr']  # View factor for solar radiation [-]
        self.area_top = heat_dictionary['area_top']  # Top surface area of the motor [m^2]
        self.albedo = heat_dictionary['albedo']  # Albedo of Mars [-]
        self.temperature_effective_mars = heat_dictionary['temperature_effective_mars']  # Effective temperature of Mars [K]
        self.emissivity_mars = heat_dictionary['emissivity_mars']  # Emissivity of Mars [-]
        self.stefan_boltzmann_constant = heat_dictionary['stefan_boltzmann_constant']  # Stefan-Boltzmann constant [W/(m^2*K^4)]

        # Motor properties
        self.heat_rate_internal = sum(heat_dictionary['heat_rate_internal'].values())  # Total internal heat [W]
        self.coefficient_resistance = heat_dictionary['coefficient_resistance']  # Heat transfer resistance coefficient [W/K]
        self.area_total = heat_dictionary['area_total']  # Total motor surface area [m^2]
        self.f_re = heat_dictionary['f_re']  # View factor for emitted radiation [-]
        self.emissivity_motor = heat_dictionary['emissivity_motor']  # Motor emissivity [-]
        self.motor_mass = heat_dictionary['motor_mass']  # Motor mass [kg]
        self.motor_heat_capacity = heat_dictionary['motor_heat_capacity']  # Motor heat capacity [J/(kg*K)]

        # Conductivity properties
        self.thermal_conductivity = heat_dictionary['thermal_conductivity']  # Thermal conductivity [W/(m*K)]

    def heat_rate_conduction(self, temperature_motor):
        """
        Placeholder for calculating heat input due to conduction from the motor to the arm.

        Args:
            temperature_motor: Current motor temperature [K].

        Returns:
            Heat input due to conduction [W].
        """
        return 0

    def heat_rate_external_input(self, temperature_motor):
        """
        Calculates the total heat input from external sources.

        Args:
            temperature_motor: Current motor temperature [K].

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
            Total heat input from internal sources [W].
        """
        return self.heat_rate_internal

    def heat_rate_convection(self, temperature_motor):
        """
        Calculates the heat loss due to convection with the atmosphere.

        Args:
            temperature_motor: Current motor temperature [K].

        Returns:
            Heat loss due to convection [W].
        """
        heat_rate_conv = 1 / self.coefficient_resistance * (temperature_motor - self.temperature_atmosphere)
        return heat_rate_conv

    def heat_rate_out(self, temperature_motor):
        """
        Calculates the heat loss due to radiation to the environment.

        Args:
            temperature_motor: Current motor temperature [K].

        Returns:
            Heat loss due to radiation [W].
        """
        heat_rate_out = self.f_re * self.stefan_boltzmann_constant * self.emissivity_motor * \
                        (temperature_motor**4 - self.temperature_atmosphere**4) * self.area_total
        return heat_rate_out 

    def heat_rate_balance(self, temperature_motor):
        """
        Calculates the balance equation to solve for the motor's temperature.

        Args:
            temperature_motor: Current motor temperature [K].

        Returns:
            Balance equation value [W] (should be zero at equilibrium).
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
            temperature_motor: Current motor temperature [K].
            t: Current time [s].

        Returns:
            Derivative of motor temperature over time [K/s].
        """
        return 1 / (self.motor_mass * self.motor_heat_capacity) * self.heat_rate_balance(temperature_motor)

    def solve_temperature_time(self):
        """
        Solves the heat balance equation to find the temperature of the motor over time.

        Returns:
            Tuple containing temperature of the motor over time [K] and time array [s].
        """
        t = np.linspace(0, 20*60, 1000)
        temperature = odeint(self.temperature_time_derivative, y0=temp_amb, t=t)
        return temperature, t
    
    def plot_temp_time(self):
        """
        Plots the temperature of the motor over time.
        """
        temperature, t = self.solve_temperature_time()
        plt.plot(t, temperature)
        plt.xlabel('Time [s]')
        plt.ylabel('Temperature [K]')
        plt.title('Motor Temperature Over Time')
        plt.show()

    def solve_equilibrium_temperature(self):
        """
        Solves the heat balance equation to find the equilibrium temperature of the motor.

        Returns:
            Equilibrium temperature of the motor [K].
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

from scipy.optimize import minimize_scalar

def optimize_fin_amount(target_temperature=350):
    """
    Optimizes the number of fins to achieve a target motor temperature.
    
    Args:
        target_temperature: The desired motor temperature at the end of the time period [K].

    Returns:
        Optimal number of fins.
    """
    
    def objective(num_fins):
        fins.__init__(num_fins)        
        heat_balance_hot['coefficient_resistance'] = fins.calc_thermal_resistance()
        heat_balance_hot['motor_mass'] = 0.5+fins.mass_fin
        heat_balance_hot['area_total'] = fins.area_fin_total
        heat_balance_hot['area_top'] = fins.area_fin_crosssection_total
        f_re = 1-fins.calculate_gebhart()
        heat_balance_hot['f_re'] = f_re
        motor = MotorHeatTransfer(heat_balance_hot)
        temperature, _ = motor.solve_temperature_time()
        print(temperature[-1])
        return abs(temperature[-1] - target_temperature)
    
    result = minimize_scalar(objective, bounds=(1, 50), method='bounded')
    return int(result.x)

if __name__ == "__main__":
    main2()
    # a = optimize_fin_amount()
    # print(a)

