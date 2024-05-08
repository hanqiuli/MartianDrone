import numpy as np
import matplotlib.pyplot as plt

# Define atmospheric constants
rho = 0.017 # kg/m^3
g = 3.71 # m/s^2
a = 233.1 # m/s
T = 213.15 # K
Re_min = 10000
Re_max = 50000
mu = 1.13e-5 # Ns/m^2

#Define design parameters
M_tip = 0.7 # Tip Mach number
N = 6 # Number of ROTORS
N_blades = 2 # Number of blades per rotor
T_W_max = 1.2 # Thrust margin
T_A_disk = 8.5 # Disk loading [N/m^2]

# Define aerodynamic constants for rotor
ct_sigma = 0.115
cl_mean = 6 * ct_sigma # Mean lift coefficient
cd_mean = cl_mean/10 # Mean drag coefficient
k_hover = 1.2 # induced/ideal power ratio

# Define flight time and battery specific energy
T_flight = 1800 # seconds
E_bat = 250 # Wh/kg

# Define mass constants
# Note that these can be changed as needed

m_payload = 8
m_avionics = 1.5
m_comms = 2.9
m_struct = 10
# m_struct = 15
m_solar = 4

# Define variable masses

m_bat = 20
m_motor = 3
m_rotors = 12.66


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


# Iterate battery mass only
m_drone = m_payload + m_avionics + m_comms + m_struct + m_solar + m_bat + m_motor + m_rotors
m_history = [m_drone]
i = 0

while i < 100 and (i == 0 or np.abs(m_drone - m_history[-2]) > 0.001):
    # perform calculations...
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

    # update drone mass estimate
    m_drone = m_payload + m_avionics + m_comms + m_struct + m_solar + m_bat + m_motor + m_rotors
    # Calculate the percentages to see the budgets
    m_payload_percent = m_payload / m_drone
    m_avionics_percent = m_avionics / m_drone
    m_comms_percent = m_comms / m_drone
    m_struct_percent = m_struct / m_drone
    m_solar_percent = m_solar / m_drone
    m_bat_percent = m_bat / m_drone
    m_motor_percent = m_motor / m_drone
    m_rotors_percent = m_rotors / m_drone

    m_history.append(m_drone)

    print(f"Iteration {i+1}: Battery mass = {m_bat:.2f} kg, Motor mass = {m_motor:.2f} kg, Total drone mass = {m_drone:.2f} kg, Power = {P_total:.2f} W, Energy = {E_total:.2f} Wh")

    i += 1

plt.plot(m_history)
plt.xlabel('Iteration')
plt.ylabel('Drone Mass [kg]')
plt.show()