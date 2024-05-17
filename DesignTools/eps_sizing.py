import numpy as np
from solar_flux_analysis import get_avg_solar_flux
import matplotlib.pyplot as plt

# Solar panel parameters
# Cell Type: Inverted Metamorphic Quadruple Junction (IMM4J)
#Source: https://www.researchgate.net/publication/365501968_Ultra-lightweight_and_flexible_inverted_metamorphic_four_junction_solar_cells_for_space_applications
cell_eff = 0.35 # Cell efficiency
# array_power_density = 200 # Power per unit of array area [W/m^2]
# array_specific_power = 20 # Power per unit of array mass [W/kg]
# degradation = 0.02 # Cell degradation per year


#Martian Surface Parameters

# Source: https://nssdc.gsfc.nasa.gov/planetary/factsheet/marsfact.html
# Source: https://www-mars.lmd.jussieu.fr/mcd_python/
datafile = 'Solar_Flux.txt'
design_solar_flux = get_avg_solar_flux(datafile, plotting=False) # [W/m^2] over the day
t_day = 88775.244  # [s] Length of a Martian day



#Drone Parameters
Power = 7232 # Power consumption of the drone [W]
n_motors = 6 # Number of motors
flight_time = 20*60 # Flight time [s]
g = 3.71 # Mars gravity [m/s^2]
# charging_time = 24 # Charging time [h]

#Battery Parameters
V_bat = 22.2 # Battery voltage [V]
I = Power/V_bat # Current consumption of the drone [A]
battery_capacity_wh = 1929 # Battery capacity [Wh]
e_bat = 250 # Energy density of the battery [Wh/kg]
m_bat = battery_capacity_wh/e_bat # Battery mass [kg]


#Calculations
# def area_to_charging_time(charging_time, battery_capacity, solar_flux):
#     return battery_capacity/(charging_time*solar_flux*cell_eff)
                             

# array_power = cell_eff*design_solar_flux # [W/m^2] The array produces this power per unit of area on average over the day
# array_energy = array_power*charging_time # [Wh/m^2] The array produces this energy per unit of area in the charging time
# # print(array_energy)
# array_area = battery_capacity_wh/array_energy
# # array_mass = array_area*array_specific_power
def harmonic_mean(arr):
    return len(arr) / np.sum(1.0 / arr)


class eps():
    def __init__(self, battery_capacity, charging_time, solar_data:str, cell_efficiency, array_specific_power, mission_duration, battery_energy_density, cell_degradation):
        self.battery_capacity = battery_capacity
        self.battery_energy_density = battery_energy_density
        self.cell_degradation = cell_degradation
        self.charging_time = charging_time
        self.solar_data = solar_data
        self.cell_efficiency = cell_efficiency
        self.array_specific_power = array_specific_power
        self.solar_flux = harmonic_mean(self.get_design_solar_flux(solar_data)[1])
        self.mission_duration = mission_duration
        # self.area = self.area_to_charging_time(charging_time, battery_capacity, self.solar_flux)
        self.cell_efficiency_eol = self.degradation(mission_duration)

    def get_design_solar_flux(self, datafile):
        return get_avg_solar_flux(datafile, plotting=False)  

    def area_to_charging_time(self, charging_time, battery_capacity, solar_flux, EOL:bool=False):
        if EOL:
            e_cell = self.cell_efficiency_eol
        else:
            e_cell = self.cell_efficiency
        # self.area = battery_capacity/(charging_time*solar_flux*e_cell)
        return battery_capacity/(charging_time*solar_flux*e_cell)
        
    
    def degradation(self, mission_duration):
        return self.cell_efficiency*(1-self.cell_degradation)**self.mission_duration

    def charging_time_iteration(self, plotting=False, EOL_design:bool=False):
        charging_time = np.arange(1,72,1)
        area = self.area_to_charging_time(charging_time, self.battery_capacity, self.solar_flux, EOL=EOL_design)
        if plotting:
            plt.plot(charging_time, area)
            plt.xlabel('Charging Time [h]')
            plt.ylabel('Array Area [m^2]')
            plt.title('Array Area vs Charging Time')
            plt.show()
        return charging_time, area
    
    def battery_mass(self):
        return self.battery_capacity/self.battery_energy_density
    
    def charging_time_over_year(self, EOL_design:bool=False):
        days, flux_avg = self.get_design_solar_flux(self.solar_data)
        A = self.area_to_charging_time(self.charging_time, self.battery_capacity, self.solar_flux, EOL=EOL_design)
        charging_time = self.battery_capacity/(A*flux_avg*self.cell_efficiency)
        mean_charging_time = np.mean(charging_time)
        charging_time_eol = self.battery_capacity/(A*flux_avg*self.cell_efficiency_eol)
        mean_charging_time_eol = np.mean(charging_time_eol)

        plt.plot(days, charging_time)
        plt.plot(days, charging_time_eol)
        plt.xlabel('Martian Day')
        plt.ylabel('Charging Time [h]')
        plt.title('Charging Time over the Year for A= ' + str(round(A,3)) + 'm^2')
        plt.legend(['BOL', 'EOL'])
        plt.annotate('Design for EOL: '+str(EOL_design), (0,0), (5, 28), xycoords='axes fraction', textcoords='offset points', va='top')
        plt.annotate('Mean Charging Time BOL :'+str(round(mean_charging_time,2)), (0,0), (5, 50), xycoords='axes fraction', textcoords='offset points', va='top')
        plt.annotate('Mean Charging Time EOL :'+str(round(mean_charging_time_eol,2)), (0,0), (5, 40), xycoords='axes fraction', textcoords='offset points', va='top')
        plt.show()
        


if __name__ == '__main__':
    eps = eps(1929, 24, 'Solar_Flux.txt', 0.35, 20, 2, 250, 0.02)
    print(eps.battery_mass())
    # eps.charging_time_iteration(plotting=True)
    charging_time, area = eps.charging_time_iteration()
    charging_time_eol, area_eol = eps.charging_time_iteration(EOL_design=True)
    plt.plot(charging_time, area)
    plt.plot(charging_time_eol, area_eol)
    plt.xlabel('Charging Time [h]')
    plt.ylabel('Array Area [m^2]')
    plt.title('Array Area vs Charging Time')
    plt.legend(['Design for Initial Efficiency', ' Design for EOL Efficiency'])
    plt.annotate('Initial Efficiency: '+ str(eps.cell_efficiency), (0,0), (300, 30), xycoords='axes fraction', textcoords='offset points', va='top')
    plt.annotate('Degradation: ' + str(eps.cell_degradation), (0,0), (400, 40), xycoords='axes fraction', textcoords='offset points', va='top')
    plt.annotate('Battery Capacity: ' + str(eps.battery_capacity) + 'Wh', (0,0), (400, 50), xycoords='axes fraction', textcoords='offset points', va='top')
    plt.annotate('Mission Duration: ' + str(eps.mission_duration) + ' years', (0,0), (400, 60), xycoords='axes fraction', textcoords='offset points', va='top')
    plt.show()

    eps.charging_time_over_year(EOL_design=True)











    


    
    
