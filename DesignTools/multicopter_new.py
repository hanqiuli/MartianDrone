import numpy as np
import matplotlib.pyplot as plt
from environment_properties import ENV
from DesignTools.rotor_new import Rotor

#Define design parameters
M_tip = 0.7     # Tip Mach number
N = 6          # Number of ROTORS
N_blades = 4    # Number of blades per rotor
T_W_max = 1.0   # Thrust margin
T_A_disk = 2.29*ENV['g']  # Disk loading [N/m^2]

# Define aerodynamic constants for rotor
ct_sigma = 0.115        # blade loading coefficient
cl_mean = 6 * ct_sigma  # mean lift coefficient
cd_mean = cl_mean/10    # mean drag coefficient
k_hover = 1.2           # induced/ideal power ratio

# Define flight time and battery specific energy, as well as efficiency
T_flight = 20*60 # s
E_bat = 250      # Wh/kg
total_eff = 0.7

# Define mass constants (in kg)
# Note that these can be changed as needed

m_payload = 11.87
m_avionics = 4.05
m_comms = 2.9

# Define variable masses

m_bat = 1
m_motor = 1
m_rotor_group = 1
m_struct = 1



if __name__ == "__main__":

    # Initialize rotor object
    rotor_instance = Rotor(M_tip, N, N_blades, T_W_max=T_W_max, T_A_disk=T_A_disk, ct_sigma=ct_sigma, cl_mean=cl_mean, cd_mean=cd_mean, k_hover=k_hover, total_eff=total_eff)
    m_drone = m_payload + m_avionics + m_comms + m_struct + m_bat + m_motor + m_rotor_group
    m_history = [m_drone]
    power_history = []
    energy_history = []
    i = 0
    while i < 1000 and (i == 0 or np.abs(m_drone - m_history[-2]) > 0.001):
        # perform calculations...
        T_required = (m_drone * ENV['g']) / N
        rotor_instance.required_params_per_rotor(T_required)
        rotor_instance.calculate_A_blade_total(T_required)
        rotor_instance.calculate_ct_and_solidity(T_required)
        rotor_instance.calculate_power_per_rotor(T_required)
        rotor_instance.calculate_power_total()

        m_bat = rotor_instance.calculate_battery_mass(T_flight, E_bat)
        m_motor = rotor_instance.calculate_motor_mass()
        m_rotor_group = rotor_instance.calculate_rotor_group_mass(T_required)
        m_struct = rotor_instance.calculate_struct_mass(T_required)

        m_drone = m_payload + m_avionics + m_comms + m_struct + m_bat + m_motor + m_rotor_group
        m_history.append(m_drone)
        power_history.append(rotor_instance.P_total)
        energy_history.append(rotor_instance.calculate_total_energy(T_flight, Wh=True))

        i += 1

    print("SEPARATION LINE --------------------------------------------")
    print(f"Endurance: {T_flight} seconds")
    print(f"Total drone mass: {m_drone} kg")
    print(f"Rotor group mass: {m_rotor_group} kg")
    print(f"Motor mass: {m_motor} kg")
    print(f"Battery mass: {m_bat} kg")
    print(f"Structural mass: {m_struct} kg")
    print(f"Total power: {power_history[-1]} W")
    print(f"Total energy: {energy_history[-1]} Wh")
    print(f"Rotor radius: {rotor_instance.r_disk} m")
    print("SEPARATION LINE --------------------------------------------")