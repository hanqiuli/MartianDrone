# doi:10.1088/1757-899X/1226/1/012113

heat_balance_cold = {
    'temperature_atmosphere': 173,  # [K] - Atmospheric temperature
    'temperature_ground': 170,  # [K] - Ground temperature
    'irradiance_sun': 0,  # [W/m^2] - Solar irradiance
    'absorptivity': 0.2,  # [-] - Absorptance of the drone
    'f_sr': 1,  # [-] - View factor for solar radiation
    'area_top': 1.5,  # [m^2] - Top surface area of the drone

    'albedo': 0.4,  # [-] - Albedo of Mars

    'temperature_effective_mars': 209.8,  # [K] - Effective temperature for radiation of black body of Mars
    'emissivity_mars': 0.65,  # [-]-  Emissivity of Mars
    'stefan_boltzmann_constant': 5.6704e-8,  # [W/m^2*K^4] - Stefan-Boltzmann constant

    # Drone properties
    'heat_rate_internal': {'electronics': 50, 'motor': 0},  # [W] - Heat from internal sources

    'coefficient_convection': 1,  # [W/m^2*K] - Convection coefficient
    'area_total': 4,  # [m^2] - Total surface area of the drone
    'f_re': 1,  # [-] - View factor for emitted radiation
    'emissivity_drone': 0.85,  # [-] - Emissivity of the drone

    # Thermal resistance
    'num_legs': 2,  # [-] - Number of legs
    'thermal_resistance_ground_drone': 0.049,  # [K/W] - Thermal resistance between ground and drone
    'thermal_resistance_suspension_drone': 1.052,  # [K/W] - Thermal resistance between suspension and drone
}

from sympy.solvers import solve
from sympy import Symbol


class MarsDroneHeatTransfer:
    """
    This class models the heat transfer of a drone on the Martian surface.
    """

    def __init__(self):
        # Environmental properties
        heat_dictionary = heat_balance_cold
        self.temperature_atmosphere = heat_dictionary['temperature_atmosphere']  # [K] - Atmospheric temperature
        self.temperature_ground = heat_dictionary['temperature_ground']  # [K] - Ground temperature
        self.irradiance_sun = heat_dictionary['irradiance_sun']  # [W/m^2] - Solar irradiance
        self.absorptivity = heat_dictionary['absorptivity']  # [-] - Absorptance of the drone
        self.f_sr = heat_dictionary['f_sr']  # [-] - View factor for solar radiation
        self.area_top = heat_dictionary['area_top']  # [m^2] - Top surface area of the drone
        
        self.albedo = heat_dictionary['albedo']  # [-] - Albedo of Mars
        
        self.temperature_effective_mars = heat_dictionary['temperature_effective_mars']  # [K] - Effective temperature for radiation of black body of Mars
        self.emissivity_mars = heat_dictionary['emissivity_mars']  # [-]-  Emissivity of Mars
        self.stefan_boltzmann_constant = heat_dictionary['stefan_boltzmann_constant']  # [W/m^2*K^4] - Stefan-Boltzmann constant

        # Drone properties
        self.heat_rate_internal = sum(heat_dictionary['heat_rate_internal'].values())  # [W] - Total heat from internal sources
        
        self.coefficient_convection = heat_dictionary['coefficient_convection']  # [W/m^2*K] - Convection coefficient
        self.area_total = heat_dictionary['area_total']  # [m^2] - Total surface area of the drone
        self.f_re = heat_dictionary['f_re']  # [-] - View factor for emitted radiation
        self.emissivity_drone = heat_dictionary['emissivity_drone']  # [-] - Emissivity of the drone

        # Thermal resistance
        self.num_legs = heat_dictionary['num_legs']  # [-] - Number of legs
        self.thermal_resistance_ground_drone = heat_dictionary['thermal_resistance_ground_drone']  # [K/W] - Thermal resistance between ground and drone
        self.thermal_resistance_suspension_drone = heat_dictionary['thermal_resistance_suspension_drone']  # [K/W] - Thermal resistance between suspension and drone
    
    def heat_rate_external_input(self, temperature_drone):
        """
        Calculates the total heat input from external sources.

        Args:
            temperature_drone: The current temperature of the drone [K].

        Returns:
            A tuple containing the total heat input [W] and individual components [W].
        """

        heat_rate_sun = self.absorptivity * self.irradiance_sun * self.area_top * self.f_sr
        heat_rate_albedo = self.albedo * heat_rate_sun
        j_p = self.emissivity_mars * self.stefan_boltzmann_constant * self.temperature_effective_mars**4
        heat_rate_ir = j_p * self.area_top
        heat_rate_cond = self.num_legs * (self.temperature_ground - temperature_drone) / (self.thermal_resistance_ground_drone + self.thermal_resistance_suspension_drone)
        total_heat_rate_ext = heat_rate_sun + heat_rate_albedo + heat_rate_ir + heat_rate_cond
        return total_heat_rate_ext, (heat_rate_sun, heat_rate_albedo, heat_rate_ir, heat_rate_cond)

    def heat_rate_internal_input(self):
        """
        Calculates the total heat input from internal sources.

        Returns:
            The total heat input from electronics and motor [W].
        """

        return self.heat_rate_internal

    def heat_rate_convection(self, temperature_drone):
        """
        Calculates the heat loss due to convection with the atmosphere.

        Args:
            temperature_drone: The current temperature of the drone [K].

        Returns:
            The heat loss due to convection [W].
        """

        heat_rate_conv = self.coefficient_convection * (temperature_drone - self.temperature_atmosphere) * self.area_total
        return heat_rate_conv

    def heat_rate_out(self, temperature_drone):
        """
        Calculates the heat loss due to radiation to the environment.

        Args:
            temperature_drone: The current temperature of the drone [K].

        Returns:
            The heat loss due to radiation [W].
        """

        heat_rate_out = self.stefan_boltzmann_constant * self.emissivity_drone * (temperature_drone**4 - self.temperature_atmosphere**4) * self.area_total
        return heat_rate_out 

    def heat_rate_balance(self, temperature_drone):
        """
        Calculates the balance equation to solve for the drone's temperature.

        Args:
            temperature_drone: The current temperature of the drone [K].

        Returns:
            The value of the balance equation [W] (should be zero at equilibrium).
        """

        heat_rate_ext, _ = self.heat_rate_external_input(temperature_drone)
        heat_rate_int = self.heat_rate_internal_input()
        heat_rate_conv = self.heat_rate_convection(temperature_drone)
        heat_rate_out = self.heat_rate_out(temperature_drone)
        return heat_rate_ext + heat_rate_int - heat_rate_conv - heat_rate_out

    def solve_equilibrium_temperature(self):
        """
        Solves the heat balance equation to find the equilibrium temperature 
        of the drone.

        Returns:
            The equilibrium temperature of the drone [K].
        """

        x = Symbol('x')
        temperature_equilibrium_solutions = solve(self.heat_rate_balance(x), x)
        return temperature_equilibrium_solutions[1]


def main():
    drone = MarsDroneHeatTransfer()
    equilibrium_temp = drone.solve_equilibrium_temperature()
    print(f"Equilibrium temperature of the drone: {equilibrium_temp}")


if __name__ == "__main__":
    main()
