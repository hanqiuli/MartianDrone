import numpy as np
import matplotlib.pyplot as plt
from environment_properties import ENV
from rotor import Rotor

#Define design parameters
M_tip = 0.7     # Tip Mach number
N = 6          # Number of ROTORS
N_blades = 4    # Number of blades per rotor
T_W_max = 1   # Thrust margin
T_A_disk = 2.29*ENV['g']  # Disk loading [N/m^2]

# Define aerodynamic constants for rotor
ct_sigma = 0.115        # blade loading coefficient
cl_mean = 6 * ct_sigma  # mean lift coefficient
cd_mean = cl_mean/10    # mean drag coefficient
k_hover = 1.2           # induced/ideal power ratio

# Define flight time and battery specific energy, as well as efficiency
T_flight = 30*60 # s
E_bat = 250      # Wh/kg
total_eff = 0.7

# Define mass constants (in kg)
# Note that these can be changed as needed

m_payload = 11.07
m_avionics = 4.05
m_comms = 2.9

# Define variable masses

m_bat = 1
m_motor = 1
m_rotor_group = 1
m_struct = 1

# Define relevant formulas for multicopter
# Calculate struct mass
def calculate_struct_mass(m_drone):
    return 28 * ((m_drone / 1000) ** (2/3)) + 0.067 * m_drone # NASA MSH Concept design

def calculate_dq_fuselage(m_drone):
    S_wet = 27.5 * (m_drone * ENV['g'] / 1000)**(2/3)
    Cd = 1.2
    return Cd * S_wet

def calculate_dq_rotors(A_disk):
    return 0.0018 * A_disk

def get_total_dq(m_drone, A_disk):
    dq_fuselage = calculate_dq_fuselage(m_drone)
    dq_rotors = calculate_dq_rotors(A_disk)
    return dq_fuselage + dq_rotors + 0.05

if __name__ == "__main__":

    # We iterate through different endurance minutes to see how the drone mass changes
    # with respect to the endurance time

    total_mass_list = []
    total_power_list = []
    radius_list = []

    for mins in np.arange(1, 40, 1):
        T_flight = mins * 60
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
            rotor_instance.figure_of_merit_calc(T_required)
            rotor_instance.calculate_single_blade_chord()

            m_bat = rotor_instance.calculate_battery_mass(T_flight, E_bat)
            m_motor = rotor_instance.calculate_motor_mass()
            m_rotor_group = rotor_instance.calculate_rotor_group_mass(T_required)
            m_struct = calculate_struct_mass(m_drone)

            m_drone = m_payload + m_avionics + m_comms + m_struct + m_bat + m_motor + m_rotor_group
            m_history.append(m_drone)
            power_history.append(rotor_instance.P_total)
            energy_history.append(rotor_instance.calculate_total_energy(T_flight, Wh=True))

            i += 1

        print("SEPARATION LINE --------------------------------------------")
        print(f"Endurance: {mins} minutes")
        print(f"Total drone mass: {m_drone} kg")
        print(f"Rotor group mass: {m_rotor_group} kg")
        print(f"Motor mass: {m_motor} kg")
        print(f"Battery mass: {m_bat} kg")
        print(f"Structural mass: {m_struct}")
        print(f"Total power: {power_history[-1]} W")
        print(f"Total energy: {energy_history[-1]} Wh")
        print(f"Rotor radius: {rotor_instance.r_disk} m")
        print(f"figure of merit: {rotor_instance.figure_of_merit}")
        print(f"Chord length for one blade: {rotor_instance.c_blade} m")
        print("SEPARATION LINE --------------------------------------------")

        if mins == 20:
            print('got here?')
            dq = get_total_dq(m_drone, rotor_instance.A_disk)
            V = np.linspace(0, 60, 100)
            Power_list = dq * ENV['rho'] * 0.5 * V**3
            plt.plot(V, Power_list)
            plt.grid()
            plt.show()

        total_mass_list.append(m_drone)
        total_power_list.append(rotor_instance.P_total)
        radius_list.append(rotor_instance.r_disk)

    
    plt.plot(np.arange(1, 40, 1), total_mass_list)
    plt.grid()
    plt.xlabel('Endurance (minutes)')
    plt.ylabel('Total Mass (kg)')
    plt.show()

    plt.plot(np.arange(1, 40, 1), total_power_list)
    plt.grid()
    plt.xlabel('Endurance (minutes)')
    plt.ylabel('Total Power (W)')
    plt.show()

    plt.plot(np.arange(1, 40, 1), radius_list)
    plt.grid()
    
    #max rotor radius
    max_rotor_radius = (0.9 * 4.5 / 2)
    # max_rotor_radius = 4.5 / 2

    #plot a horizontal line for the limit rotor radius
    plt.axhline(y=max_rotor_radius, color='r', linestyle='--')

    plt.xlabel('Endurance (minutes)')
    plt.ylabel('Rotor Radius (m)')
    plt.show()


    # find the maximum endurance time given the rotor radius constraint
    # find the index of the first element that is greater than the max rotor radius
    try:
        idx = np.where(np.array(radius_list) > max_rotor_radius)[0][0]
        print(f"Maximum endurance time given rotor radius constraint: {idx} minutes")
    except IndexError:
        print("All endurance times are within the rotor radius constraint")