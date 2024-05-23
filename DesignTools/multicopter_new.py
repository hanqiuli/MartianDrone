import numpy as np
import matplotlib.pyplot as plt
from environment_properties import ENV
from rotor_new import Rotor

#Define design parameters
M_tip = 0.7     # Tip Mach number
N_rotors = 6    # Number of ROTORS
N_blades = 4    # Number of blades per rotor
# T_A_disk = 2.29*ENV['g']    # Disk loading [N/m^2]
T_A_disk = 2.24 * ENV['g']     # Disk loading [N/m^2] VALIDATION

CT_sigma = 0.115    # Blade loading coefficient
cl_cd = 10          # Section lift-to-drag ratio
k_hover = 1.2       # Induced power ratio in hover

t_flight = 2.5*60    # Hover time in seconds
e_bat = 218.5         # Battery specific energy in Wh/kg VALIDATION
total_eff = 0.7     # Total efficiency of the system VALIDATION

# Define mass constants (in kg)
# Note that these can be changed as needed

# m_payload = 11.87
m_payload = 2.02
# m_avionics = 4.05
m_avionics = 1.36 + 1.2 #controls + avionics
m_solar = 2 * 0.62 # this is actually solar
# m_comms = 2.9

# Define variable masses

m_bat = 1
m_motor = 1
m_rotor_group = 1
m_struct = 1

if __name__ == "__main__":

    # Initialize rotor object
    rotor_instance = Rotor(M_tip, N_rotors, N_blades, T_A_disk=T_A_disk, CT_sigma=CT_sigma, cl_cd=cl_cd, k_hover=k_hover, total_eff=total_eff, e_bat=e_bat, t_flight=t_flight, validating=True)
    m_drone = m_payload + m_avionics + m_solar + m_struct + m_bat + m_motor + m_rotor_group
    m_history = [m_drone]
    power_history = []
    energy_history = []
    i = 0
    while i < 1000 and (i == 0 or np.abs(m_drone - m_history[-2]) > 0.001):
        T_required = (m_drone * ENV['g']) / N_rotors
        rotor_instance.required_params_per_rotor(T_required)
        rotor_instance.calculate_A_blade_total(T_required)
        rotor_instance.calculate_ct_and_solidity(T_required)
        rotor_instance.calculate_power_per_rotor(T_required)
        rotor_instance.calculate_power_total()

        m_bat = rotor_instance.calculate_battery_mass()
        m_motor = rotor_instance.calculate_motor_mass()
        m_rotor_group = rotor_instance.calculate_rotor_group_mass(T_required)
        m_struct = rotor_instance.calculate_struct_mass(T_required)

        m_drone = m_payload + m_avionics + m_solar + m_struct + m_bat + m_motor + m_rotor_group + m_drone * 0.2
        m_history.append(m_drone)
        power_history.append(rotor_instance.P_total)
        energy_history.append(rotor_instance.calculate_total_energy(Wh=True))

        i += 1

    print("--------------------------------------------")
    print(f"    Rotor radius: {rotor_instance.r_disk:.4g} m")
    print(f"    Rotor solidity: {rotor_instance.sigma:.4g}")
    print(f"Total drone mass: {m_drone:.2f} kg")
    print(f"    Rotor group mass: {m_rotor_group:.2f} kg")
    print(f"    Motor mass: {m_motor:.2f} kg")
    print(f"    Battery mass: {m_bat:.2f} kg")
    print(f"    Structural mass: {m_struct:.2f} kg")
    print(f"    Avionics mass: {m_avionics:.2f} kg")
    print(f"    Solar mass: {m_solar:.2f} kg")
    print(f"Total motor power: {power_history[-1]:.2f} W")
    print(f"Total energy: {energy_history[-1]:.2f} Wh")
    print("--------------------------------------------")