'''Module to size the solar array and battery for a Martian drone mission 
    based on the solar flux data provided by the Mars Climate Database'''

import numpy as np
from solar_flux_analysis import get_avg_solar_flux
import matplotlib.pyplot as plt

# Solar panel parameters
# Cell Type: Inverted Metamorphic Quadruple Junction (IMM4J)
#Source: https://www.researchgate.net/publication/365501968_Ultra-lightweight_and_flexible_inverted_metamorphic_four_junction_solar_cells_for_space_applications
# cell_eff = 0.35 # Cell efficiency
# array_power_density = 200 # Power per unit of array area [W/m^2]
# array_specific_power = 20 # Power per unit of array mass [W/kg]
# degradation = 0.02 # Cell degradation per year


#Martian Surface Parameters

# Source: https://nssdc.gsfc.nasa.gov/planetary/factsheet/marsfact.html
# Source: https://www-mars.lmd.jussieu.fr/mcd_python/
data_file = 'Solar_Flux.txt' #File containing the solar flux data from Mars Climate Database
duration_day = 88775.244  # [s] Length of a Martian day
g = 3.71 # Mars gravity [m/s^2]


#Drone Parameters
power_drone = 7232 # Power consumption of the drone [W]
num_motors = 6 # Number of motors
time_flight = 20*60 # Flight time [s]
time_charging = 24 # Charging time [h]

#Battery Parameters
voltage_battery = 22.2 # Battery voltage [V]
current = power_drone/voltage_battery # Current consumption of the drone [A]
capacity_battery_wh = 2821 # Battery capacity [Wh]
energy_density_battery = 250 # Energy density of the battery [Wh/kg]
mass_battery = capacity_battery_wh/energy_density_battery # Battery mass [kg]

#Define the harmonic mean function
def harmonic_mean(arr):
    '''Function to calculate the harmonic mean of an array'''
    return len(arr) / np.sum(1.0 / arr)


class Eps():
    '''Class to size the solar array and battery for a Martian drone mission'''

    def __init__(self, capacity_battery, time_charging, solar_data:str, cell_efficiency, array_specific_power, mission_duration, battery_energy_density, cell_degradation):
        self.capacity_battery = capacity_battery
        self.battery_energy_density = battery_energy_density
        self.cell_degradation = cell_degradation
        self.time_charging = time_charging
        self.solar_data = solar_data
        self.cell_efficiency = cell_efficiency
        self.array_specific_power = array_specific_power
        self.solar_flux = np.cos(15*np.pi/180)*harmonic_mean(self.get_design_solar_flux(solar_data)[1])
        self.mission_duration = mission_duration
        self.cell_efficiency_eol = self.degradation()

    def get_design_solar_flux(self, datafile):
        '''Function to get the average solar flux over the day from the datafile'''
        return get_avg_solar_flux(datafile, plotting=False)  

    def area_to_time_charging(self, time_to_charge, capacity_battery, solar_flux, EOL:bool=False):
        '''Function to calculate the array area based on the charging time'''
        if EOL:
            e_cell = self.cell_efficiency_eol
        else:
            e_cell = self.cell_efficiency
        # self.area = capacity_battery/(time_charging*solar_flux*e_cell)
        return capacity_battery/(time_to_charge*solar_flux*e_cell)
        
    
    def degradation(self):
        '''Function to calculate the cell efficiency at the end of the mission'''
        return self.cell_efficiency*(1-self.cell_degradation)**self.mission_duration

    def time_charging_iteration(self, plotting=False, EOL_design:bool=False):
        '''Function to iterate over the charging time to find the optimal array area'''
        charging_t = np.arange(1,72,1)
        area = self.area_to_time_charging(charging_t, self.capacity_battery, self.solar_flux, EOL=EOL_design)
        if plotting:
            plt.plot(charging_t, area)
            plt.xlabel('Charging Time [h]')
            plt.ylabel('Array Area [m^2]')
            plt.title('Array Area vs Charging Time')
            plt.show()
        return charging_t, area
    
    def battery_mass(self):
        '''Function to calculate the battery mass based on the battery capacity and energy density'''
        return self.capacity_battery/self.battery_energy_density
    
    def time_charging_over_year(self, EOL_design:bool=False):
        '''Function to calculate the charging time over the year based on the solar flux data and the array area'''
        days, flux_avg = self.get_design_solar_flux(self.solar_data)
        A = self.area_to_time_charging(self.time_charging, self.capacity_battery, self.solar_flux, EOL=EOL_design)
        time_charging = self.capacity_battery/(A*flux_avg*self.cell_efficiency)
        mean_time_charging = np.mean(time_charging)
        time_charging_eol = self.capacity_battery/(A*flux_avg*self.cell_efficiency_eol)
        mean_time_charging_eol = np.mean(time_charging_eol)

        plt.plot(days, time_charging)
        plt.plot(days, time_charging_eol)
        plt.xlabel('Martian Day')
        plt.ylabel('Charging Time [h]')
        plt.title('Charging Time over the Year for A= ' + str(round(A,3)) + 'm^2')
        plt.legend(['BOL', 'EOL'])
        plt.annotate('Design for EOL: '+str(EOL_design), (0,0), (5, 28), xycoords='axes fraction', textcoords='offset points', va='top')
        plt.annotate('Mean Charging Time BOL :'+str(round(mean_time_charging,2)), (0,0), (5, 50), xycoords='axes fraction', textcoords='offset points', va='top')
        plt.annotate('Mean Charging Time EOL :'+str(round(mean_time_charging_eol,2)), (0,0), (5, 40), xycoords='axes fraction', textcoords='offset points', va='top')
        plt.annotate('Initial Efficiency: '+ str(self.cell_efficiency), (0,0), (5, 60), xycoords='axes fraction', textcoords='offset points', va='top')
        plt.annotate('Degradation: ' + str(self.cell_degradation), (0,0), (5, 70), xycoords='axes fraction', textcoords='offset points', va='top')
        plt.show()
    
    def power_density(self, array_area):
        '''Function to calculate the power density of the array'''
        return self.capacity_battery/array_area


if __name__ == '__main__':
    eps = Eps(capacity_battery_wh, time_charging, 'Solar_Flux.txt', 0.3, 20, 2, energy_density_battery, 0.02)
    print(eps.battery_mass())
    # eps.time_charging_iteration(plotting=True)
    charging_duration, array_area = eps.time_charging_iteration()
    charging_duration_eol, array_area_eol = eps.time_charging_iteration(EOL_design=True)
    plt.plot(charging_duration, array_area)
    plt.plot(charging_duration_eol, array_area_eol)
    plt.xlabel('Charging Time [h]')
    plt.ylabel('Array Area [m^2]')
    plt.title('Array Area vs Charging Time')
    plt.legend(['Design for Initial Efficiency', ' Design for EOL Efficiency'])
    plt.annotate('Initial Efficiency: '+ str(eps.cell_efficiency), (0,0), (300, 30), xycoords='axes fraction', textcoords='offset points', va='top')
    plt.annotate('Degradation: ' + str(eps.cell_degradation), (0,0), (300, 40), xycoords='axes fraction', textcoords='offset points', va='top')
    plt.annotate('Battery Capacity: ' + str(eps.capacity_battery) + 'Wh', (0,0), (300, 50), xycoords='axes fraction', textcoords='offset points', va='top')
    plt.annotate('Mission Duration: ' + str(eps.mission_duration) + ' years', (0,0), (300, 60), xycoords='axes fraction', textcoords='offset points', va='top')
    plt.show()

    eps.time_charging_over_year(EOL_design=True)
    print(eps.power_density(eps.area_to_time_charging(time_charging, eps.capacity_battery, eps.solar_flux, EOL=True)))











    


    
    
