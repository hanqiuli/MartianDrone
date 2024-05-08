import numpy as np
import matplotlib.pyplot as plt

# Define atmospheric constants
rho = 0.017     # kg/m^3
g = 3.71        # m/s^2
a = 233.1       # m/s
T = 213.15      # K
Re_min = 10000  
Re_max = 50000
mu = 1.13e-5    # Ns/m^2

#Define design parameters
M_tip = 0.7     # Tip Mach number
N = 1           # Number of ROTORS
N_blades = 4    # Number of blades per rotor
T_W_max = 1.2   # Thrust margin
T_A_disk = 8.5  # Disk loading [N/m^2]

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

m_bat = 100
m_motor = 100
m_rotor_group = 100
m_struct = 100

# Define relevant functions

def calculate_V_tip(M_tip, a):
    return M_tip * a

def required_params_per_rotor(m_drone, g, N_rotor, disk_loading, thrust_margin):
    weight = m_drone * g
    T_required = weight * thrust_margin / N_rotor
    A_disk = T_required / disk_loading
    r_disk = np.sqrt(A_disk / np.pi)
    return weight, T_required, A_disk, r_disk

def calculate_A_blade(V_tip, ct_sigma, T_req, rho):
    return T_req / (rho * ct_sigma * V_tip**2)

def calculate_ct_and_solidity(A_disk, rho, V_tip, T_req, ct_sigma):
     ct = T_req / (A_disk * rho * V_tip**2)
     sigma = ct / ct_sigma
     return ct, sigma

def calculate_omega(V_tip, r_disk):
    return V_tip / r_disk

def calculate_power_per_rotor(k_hover, T_req, rho, A_disk, A_blade, V_tip, cd_mean):
    P = k_hover * T_req * np.sqrt((T_req)/(2*rho*A_disk)) + rho * A_blade * V_tip**3 * cd_mean * (1/8)
    return P

def calculate_power_total(P_rotor, N_rotor):
    return P_rotor * N_rotor

def calculate_total_energy(P_total, T_flight):
    return (P_total * T_flight) / 3600 # Convert to Wh

def calculate_battery_mass(E_total, E_specific):
    return E_total / E_specific

def calculate_motor_mass(P_total):
    return 0.432/1000 * P_total


#  Formulas for rotor group mass

def calculate_blade_mass(A_blade, N_rotor):
    # calculates TOTAL blade mass
    return 1.1 * A_blade * N_rotor

def calculate_hub_mass(T_max, N_rotor):
    # calculate TOTAL hub mass
    return 0.05 * T_max * N_rotor

def calculate_shaft_mass(r_rotor, N_rotor):
    # calculate TOTAL shaft mass
    return 0.15 * 0.15 * r_rotor * N_rotor

def calculate_support_mass(r_rotor, N_rotor):
    # calculate TOTAL mass of rotor support arms
    return 0.2 * r_rotor * N_rotor

def calculate_rotor_group_mass(A_blade, T_max, r_rotor, N_rotor):
    # calculate TOTAL rotor group mass
    return calculate_blade_mass(A_blade, N_rotor) + calculate_hub_mass(T_max, N_rotor) + calculate_shaft_mass(r_rotor, N_rotor) + calculate_support_mass(r_rotor, N_rotor)


# Calculate struct mass
def calculate_struct_mass(m_drone):
    return 28 * ((m_drone / 1000) ** (2/3)) + 0.067 * m_drone

m_drone = m_payload + m_avionics + m_comms + m_struct + m_solar + m_bat + m_motor + m_rotor_group
m_history = [m_drone]
m_rotor_group_history = [m_rotor_group]
m_motor_history = [m_motor]
m_bat_history = [m_bat]
m_struct_history = [m_struct]
power_history = []
energy_history = []
r_rotor_history = []
i = 0

while i < 100 and (i == 0 or np.abs(m_drone - m_history[-2]) > 0.001):
    # perform calculations...
    m_struct = calculate_struct_mass(m_drone)
    W, T_req, A_disk, r_disk = required_params_per_rotor(m_drone, g, N, T_A_disk, T_W_max)
    V_tip = calculate_V_tip(M_tip, a)
    A_blade = calculate_A_blade(V_tip, ct_sigma, T_req, rho)
    ct, sigma = calculate_ct_and_solidity(A_disk, rho, V_tip, T_req, ct_sigma)
    omega = calculate_omega(V_tip, r_disk)
    P_rotor = calculate_power_per_rotor(k_hover, T_req, rho, A_disk, A_blade, V_tip, cd_mean)
    P_total = calculate_power_total(P_rotor, N)
    E_total = calculate_total_energy(P_total, T_flight)
    m_bat = calculate_battery_mass(E_total, E_bat)
    m_motor = calculate_motor_mass(P_total)
    m_rotor_group = calculate_rotor_group_mass(A_blade, T_req, r_disk, N)

    # update drone mass estimate
    m_drone = m_payload + m_avionics + m_comms + m_struct + m_solar + m_bat + m_motor + m_rotor_group

    # # Calculate the percentages to see the budgets
    # m_payload_percent = m_payload / m_drone
    # m_avionics_percent = m_avionics / m_drone
    # m_comms_percent = m_comms / m_drone
    # m_struct_percent = m_struct / m_drone
    # m_solar_percent = m_solar / m_drone
    # m_bat_percent = m_bat / m_drone
    # m_motor_percent = m_motor / m_drone
    # m_rotors_percent = m_rotors / m_drone

    # Append mass histories
    m_history.append(m_drone)
    m_rotor_group_history.append(m_rotor_group)
    m_motor_history.append(m_motor)
    m_bat_history.append(m_bat)
    m_struct_history.append(m_struct)

    # Append rotor radius history
    r_rotor_history.append(r_disk)

    # Append power histories

    power_history.append(P_total)
    energy_history.append(E_total)

    i += 1

print(f"Total drone mass: {m_drone} kg")
print(f"Rotor group mass: {m_rotor_group} kg")
print(f"Motor mass: {m_motor} kg")
print(f"Battery mass: {m_bat} kg")
print(f"Structural mass: {m_struct} kg")
print(f"Total power: {power_history[-1]} W")
print(f"Total energy: {energy_history[-1]} Wh")
print(f"Rotor radius: {r_rotor_history[-1]} m")
print(f"Number of rotors: {N}")


plt.plot(m_history, label='Total Drone')
plt.plot(m_rotor_group_history, label='Rotor Group', linestyle='dashed')
plt.plot(m_motor_history, label='Motor', linestyle='dotted')
plt.plot(m_bat_history, label='Battery', linestyle='dashdot')
plt.plot(m_struct_history, label='Structural', linestyle='dashdot')
plt.legend()
plt.grid()
plt.xlabel('Iteration')
plt.ylabel('Masses (kg)')
plt.show()

plt.plot(power_history)
plt.grid()
plt.xlabel('Iteration')
plt.ylabel('Total Power (W)')
plt.show()
