import numpy as np

# Solar panel parameters
# Cell Type: Inverted Metamorphic Quadruple Junction (IMM4J)
#Source: https://www.researchgate.net/publication/365501968_Ultra-lightweight_and_flexible_inverted_metamorphic_four_junction_solar_cells_for_space_applications
cell_eff = 0.39 # Cell efficiency



#Martian Surface Parameters

# Source: https://nssdc.gsfc.nasa.gov/planetary/factsheet/marsfact.html
albedo = 0.25 # Mars surface albedo
t_d_min = 14.5-9 # Minimum daylight hours [h]
t_d_max = 16.5-7 # Maximum daylight hours [h]
Solar_flux_min = 258 # Minimum solar flux [W/m^2]
Solar_flux_max = 646 # Maximum solar flux [W/m^2]



#Drone Parameters
Power = 7232 # Power consumption of the drone [W]
n_motors = 6 # Number of motors
flight_time = 20*60 # Flight time [s]
g = 3.71 # Mars gravity [m/s^2]

#Battery Parameters
V_bat = 22.2 # Battery voltage [V]
I = Power/V_bat # Current consumption of the drone [A]
battery_capacity_j = Power*flight_time # Battery capacity [J]
battery_capacity_wh = battery_capacity_j/3600 # Battery capacity [Wh]
battery_capacity_ah = battery_capacity_wh/V_bat # Battery capacity [Ah]
e_bat = 250 # Energy density of the battery [Wh/kg]
m_bat = battery_capacity_wh/e_bat # Battery mass [kg]



#Functions


