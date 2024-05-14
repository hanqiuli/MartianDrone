import numpy as np
import matplotlib.pyplot as plt
from environment_properties import ENV
from rotor import Rotor

#Define design parameters
M_tip = 0.7     # Tip Mach number
N = 4          # Number of ROTORS
N_blades = 4    # Number of blades per rotor (Not really used rn)
T_W_max = 1.2   # Thrust margin
T_A_disk = 9.81 * 2.29  # Disk loading [N/m^2]


# Define aerodynamic constants for rotor
ct_sigma = 0.115        # blade loading coefficient
cl_mean = 6 * ct_sigma  # mean lift coefficient
cd_mean = cl_mean/10    # mean drag coefficient
k_hover = 1.2           # induced/ideal power ratio

# Define flight time and battery specific energy
T_flight = 30*60 # s
E_bat = 250      # Wh/kg

# Define mass constants (in kg)
# Note that these can be changed as needed

m_payload = 11.87
m_avionics = 4.05
m_comms = 2.9
m_solar = 4

# Define variable masses

m_bat = 1
m_motor = 1
m_rotor_group = 1
m_struct = 1

# Define relevant formulas for multicopter
# Calculate struct mass
def calculate_struct_mass(m_drone):
    return 28 * ((m_drone / 1000) ** (2/3)) + 0.067 * m_drone # NASA MSH Concept design

if __name__ == "__main__":

    # We iterate through different endurance minutes to see how the drone mass changes
    # with respect to the endurance time

    total_mass_list = []
    total_power_list = []
    radius_list = []

    for mins in np.arange(1, 60, 1):
        T_flight = mins * 60
        # Initialize rotor object
        rotor_instance = Rotor(M_tip, N, N_blades)

        m_drone = m_payload + m_avionics + m_comms + m_struct + m_solar + m_bat + m_motor + m_rotor_group
        m_history = [m_drone]
        power_history = []
        energy_history = []

        i = 0
        while i < 100 and (i == 0 or np.abs(m_drone - m_history[-2]) > 0.001):
            # perform calculations...
            T_required = m_drone * ENV['g'] * T_W_max / N
            rotor_instance.required_params_per_rotor(T_required)
            rotor_instance.calculate_A_blade(T_required)
            rotor_instance.calculate_ct_and_solidity(T_required)
            rotor_instance.calculate_power_per_rotor(T_required)
            rotor_instance.calculate_power_total()

            m_bat = rotor_instance.calculate_battery_mass(T_flight, E_bat)
            m_motor = rotor_instance.calculate_motor_mass()
            m_rotor_group = rotor_instance.calculate_rotor_group_mass(T_required)
            m_struct = calculate_struct_mass(m_drone)

            m_drone = m_payload + m_avionics + m_comms + m_struct + m_solar + m_bat + m_motor + m_rotor_group
            m_history.append(m_drone)
            power_history.append(rotor_instance.P_total)
            energy_history.append(rotor_instance.calculate_total_energy(T_flight))

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
        print("SEPARATION LINE --------------------------------------------")

        total_mass_list.append(m_drone)
        total_power_list.append(rotor_instance.P_total)
        radius_list.append(rotor_instance.r_disk)

        # plt.plot(m_history, label='Total Drone')
        # plt.legend()
        # plt.grid()
        # plt.xlabel('Iteration')
        # plt.ylabel('Masses (kg)')
        # plt.show()

        # plt.plot(power_history)
        # plt.grid()
        # plt.xlabel('Iteration')
        # plt.ylabel('Total Power (W)')
        # plt.show()
    
    plt.plot(np.arange(1, 60, 1), total_mass_list)
    plt.grid()
    plt.xlabel('Endurance (minutes)')
    plt.ylabel('Total Mass (kg)')
    plt.show()

    plt.plot(np.arange(1, 60, 1), total_power_list)
    plt.grid()
    plt.xlabel('Endurance (minutes)')
    plt.ylabel('Total Power (W)')
    plt.show()



    plt.plot(np.arange(1, 60, 1), radius_list)
    plt.grid()
    
    #max rotor radius
    max_rotor_radius = 4.5 / 2.2

    #plot a horizontal line for the limit rotor radius
    plt.axhline(y=max_rotor_radius, color='r', linestyle='--')

    plt.xlabel('Endurance (minutes)')
    plt.ylabel('Rotor Radius (m)')
    plt.show()