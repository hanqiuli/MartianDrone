import numpy as np
from sympy.solvers import solve
from sympy import Symbol
from scipy.integrate import odeint
import matplotlib.pyplot as plt


# doi:10.1088/1757-899X/1226/1/012113

heat_balance_cold = {
    'temperature_atmosphere': 210,  # [K] - Atmospheric temperature
    'irradiance_sun': 0,  # [W/m^2] - Solar irradiance
    'absorptivity': 0.2,  # [-] - Absorptance of the battery
    'f_sr': 1,  # [-] - View factor for solar radiation
    'area_top': 0.0168*0.0126,  # [m^2] - Top surface area of the battery

    'albedo': 0.4,  # [-] - Albedo of Mars

    'temperature_effective_mars': 209.8,  # [K] - Effective temperature for radiation of black body of Mars
    'emissivity_mars': 0.65,  # [-]-  Emissivity of Mars
    'stefan_boltzmann_constant': 5.6704e-8,  # [W/m^2*K^4] - Stefan-Boltzmann constant

    # battery properties
    'heat_rate_internal': {'electronics': 10},  # [W] - Heat from internal sources

    'coefficient_convection': 1,  # [W/m^2*K] - Convection coefficient
    'area_total': (0.0168*12.6*2+0.021*0.0126),  # [m^2] - Total surface area of the battery
    'f_re': 1,  # [-] - View factor for emitted radiation
    'emissivity_battery': 0.01,  # [-] - Emissivity of the battery
    'battery_mass': 9.5,  # [kg] - Mass of the battery
    'battery_heat_capacity': 1100,  # [J/kg*K] - Specific heat capacity of the battery

    # Insulator properties
    'thermal_conductivity': 0.035/2,  # [W/m*K] - Thermal conductivity of the insulator (Cork)
    'thickness_insulator': 0.1  # [m] - Thickness of the insulator
}

heat_balance_hot = {
    'temperature_atmosphere': 269,  # [K] - Atmospheric temperature
    'irradiance_sun': 646,  # [W/m^2] - Solar irradiance
    'absorptivity': 0.2,  # [-] - Absorptance of the battery
    'f_sr': 1,  # [-] - View factor for solar radiation
    'area_top': 0.0168*0.0126,  # [m^2] - Top surface area of the battery

    'albedo': 0.4,  # [-] - Albedo of Mars

    'temperature_effective_mars': 209.8,  # [K] - Effective temperature for radiation of black body of Mars
    'emissivity_mars': 0.65,  # [-]-  Emissivity of Mars
    'stefan_boltzmann_constant': 5.6704e-8,  # [W/m^2*K^4] - Stefan-Boltzmann constant

        # battery properties
    'heat_rate_internal': {'heater': 50},  # [W] - Heat from internal sources

    'coefficient_convection': 1,  # [W/m^2*K] - Convection coefficient
    'area_total': (0.0168*12.6*2+0.021*0.0126),  # [m^2] - Total surface area of the battery
    'f_re': 1,  # [-] - View factor for emitted radiation
    'emissivity_battery': 0.85,  # [-] - Emissivity of the battery
    'battery_mass': 9.5,  # [kg] - Mass of the battery
    'battery_heat_capacity': 1100,  # [J/kg*K] - Heat capacity of the battery

    # Insulator properties
    'thermal_conductivity': 0.035,  # [W/m*K] - Thermal conductivity of the insulator (Cork)
    'thickness_insulator': 0.1  # [m] - Thickness of the insulator
}

class BatteryHeatTransfer:
    """
    This class models the heat transfer of the battery of the Martian battery.
    """

    def __init__(self):
        # Environmental properties
        heat_dictionary = heat_balance_cold
        self.temperature_atmosphere = heat_dictionary['temperature_atmosphere']  # [K] - Atmospheric temperature
        self.temperature_ground = heat_dictionary['temperature_ground']  # [K] - Ground temperature
        self.irradiance_sun = heat_dictionary['irradiance_sun']  # [W/m^2] - Solar irradiance
        self.absorptivity = heat_dictionary['absorptivity']  # [-] - Absorptance of the battery
        self.f_sr = heat_dictionary['f_sr']  # [-] - View factor for solar radiation
        self.area_top = heat_dictionary['area_top']  # [m^2] - Top surface area of the battery
        
        self.albedo = heat_dictionary['albedo']  # [-] - Albedo of Mars
        
        self.temperature_effective_mars = heat_dictionary['temperature_effective_mars']  # [K] - Effective temperature for radiation of black body of Mars
        self.emissivity_mars = heat_dictionary['emissivity_mars']  # [-]-  Emissivity of Mars
        self.stefan_boltzmann_constant = heat_dictionary['stefan_boltzmann_constant']  # [W/m^2*K^4] - Stefan-Boltzmann constant

        # battery properties
        self.heat_rate_internal = sum(heat_dictionary['heat_rate_internal'].values())  # [W] - Total heat from internal sources
        
        self.coefficient_convection = heat_dictionary['coefficient_convection']  # [W/m^2*K] - Convection coefficient
        self.area_total = heat_dictionary['area_total']  # [m^2] - Total surface area of the battery
        self.f_re = heat_dictionary['f_re']  # [-] - View factor for emitted radiation
        self.emissivity_battery = heat_dictionary['emissivity_battery']  # [-] - Emissivity of the battery
        self.battery_mass = heat_dictionary['battery_mass']  # [kg] - Mass of the battery
        self.battery_heat_capacity = heat_dictionary['battery_heat_capacity']  # [J/kg*K] - Heat capacity of the battery

        # Insulator properties
        self.thermal_conductivity = heat_dictionary['thermal_conductivity']  # [W/m*K] - Thermal conductivity of the insulator (Cork)
        self.thickness_insulator = heat_dictionary['thickness_insulator']  # [m] - Thickness of the insulator

    
    def heat_rate_conduction(self, temperature_battery):
        """
        Calculates the heat input due to conduction from the ground to the battery.

        Args:
            temperature_battery: The current temperature of the battery [K].

        Returns:
            The heat input due to conduction [W].
        """

        heat_rate_cond = - self.thermal_conductivity * self.area_total / self.thickness_insulator * (temperature_battery - self.temperature_atmosphere) 
        return heat_rate_cond
    
    def heat_rate_external_input(self, temperature_battery):
        """
        Calculates the total heat input from external sources.

        Args:
            temperature_battery: The current temperature of the battery [K].

        Returns:
            A tuple containing the total heat input [W] and individual components [W].
        """

        heat_rate_sun = self.absorptivity * self.irradiance_sun * self.area_top * self.f_sr
        heat_rate_albedo = self.albedo * self.absorptivity * self.irradiance_sun * self.area_top * self.f_sr
        j_p = self.emissivity_mars * self.stefan_boltzmann_constant * self.temperature_effective_mars**4
        heat_rate_ir = j_p * self.area_top
        heat_rate_cond = self.heat_rate_conduction(temperature_battery)
        total_heat_rate_ext = heat_rate_sun + heat_rate_albedo + heat_rate_ir + heat_rate_cond
        return total_heat_rate_ext, (heat_rate_sun, heat_rate_albedo, heat_rate_ir, heat_rate_cond)

    def heat_rate_internal_input(self):
        """
        Calculates the total heat input from internal sources.

        Returns:
            The total heat input from electronics and motor [W].
        """

        return self.heat_rate_internal

    def heat_rate_convection(self, temperature_battery):
        """
        Calculates the heat loss due to convection with the atmosphere.

        Args:
            temperature_battery: The current temperature of the battery [K].

        Returns:
            The heat loss due to convection [W].
        """

        heat_rate_conv = self.coefficient_convection * (temperature_battery - self.temperature_atmosphere) * self.area_total
        return heat_rate_conv

    def heat_rate_out(self, temperature_battery):
        """
        Calculates the heat loss due to radiation to the environment.

        Args:
            temperature_battery: The current temperature of the battery [K].

        Returns:
            The heat loss due to radiation [W].
        """

        heat_rate_out = self.stefan_boltzmann_constant * self.emissivity_battery * (temperature_battery**4 - self.temperature_atmosphere**4) * self.area_total
        return heat_rate_out 

    def heat_rate_balance(self, temperature_battery):
        """
        Calculates the balance equation to solve for the battery's temperature.

        Args:
            temperature_battery: The current temperature of the battery [K].

        Returns:
            The value of the balance equation [W] (should be zero at equilibrium).
        """

        heat_rate_ext, _ = self.heat_rate_external_input(temperature_battery)
        heat_rate_int = self.heat_rate_internal_input()
        heat_rate_conv = self.heat_rate_convection(temperature_battery)
        heat_rate_out = self.heat_rate_out(temperature_battery)
        return heat_rate_ext + heat_rate_int - heat_rate_conv - heat_rate_out

    def temperature_time_derivative(self, temperature_battery, t):
        """
        Calculates the derivative of the temperature of the battery over time.

        Args:
            temperature_battery: The current temperature of the battery [K].
            t: The current time [s].

        Returns:
            The derivative of the temperature of the battery over time [K/s].
        """
        return 1/(self.battery_mass*self.battery_heat_capacity)*self.heat_rate_balance(temperature_battery)

    def solve_temperature_time(self):
        """
        Solves the heat balance equation to find the temperature of the battery over time.

        Returns:
            The temperature of the battery over time [K] [s].
        """
        
        t = np.linspace(0, 12*60*60, 10000)
        temperature = odeint(self.temperature_time_derivative, y0=290, t=t)
        return temperature, t
    
    def plot_temp_time(self):
        """
        Plots the temperature of the battery over time.
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
        of the battery.

        Returns:
            The equilibrium temperature of the battery [K].
        """

        x = Symbol('x')
        temperature_equilibrium_solutions = solve(self.heat_rate_balance(x), x)
        return temperature_equilibrium_solutions[1]


def main1():
    battery = BatteryHeatTransfer()
    equilibrium_temp = battery.solve_equilibrium_temperature()
    print(f"Equilibrium temperature of the battery: {equilibrium_temp}")

def main2():
    battery = BatteryHeatTransfer()
    equilibrium_temp = battery.plot_temp_time()

if __name__ == "__main__":
    main2()
