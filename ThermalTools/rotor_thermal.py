import numpy as np
import matplotlib.pyplot as plt
from environment_properties import ENV

# region Constants
rho_air     = ENV['rho']    # [kg/m^3]      Air density
g           = ENV['g']      # [m/s^2]       Gravitational acceleration
dyn_vsc_air = ENV['mu']   # [Pa*s]        Air dynamic viscosity
temp_amb    = 220       # [K]           Ambient temperature
cp_air      = 772       # [J/(kg*K)]    Specific heat capacity of air
k_air       = 0.024     # [W/(m*K)]     Air thermal conductivity
sigma       = 5.67e-8   # Stefan-Boltzmann constant [W/(m^2*K^4)]

kin_vsc_air = dyn_vsc_air / rho_air         # [m^2/s]   Air kinematic viscosity
alpha_air   = k_air / (rho_air * cp_air)    # [m^2/s]   Air thermal diffusivity
beta_air    = 1 / temp_amb                  # [1/K]     Air thermal expansion coefficient

temp_init   = 273.15 - 10   # [K]   Initial motor temperature
temp_max    = 273.15 + 100  # [K]   Maximum motor temperature

# Rotorcraft parameters
power_motor_total   = 7232      # [W]   Total power of the motors
eff_motor           = 0.70      # [-] Motor efficiency

mass                = 49.2      # [kg]  Total mass of the rotorcraft
mass_motor_total    = 3.125     # [kg]  Total mass of the motors
power_rotor_total   = power_motor_total*eff_motor   # [W]   Total power of the rotors
r_rotor             = 1.31      # [m]   Rotor radius
n_rotors            = 6         # [-]   Number of rotors

weight  = mass * g                          # [N]   Maximum thrust
A_disk = np.pi * r_rotor**2                 # [m^2] Single rotor disk area
power_rotor = power_rotor_total / n_rotors  # [W]   Power per rotor

# Motor parameters
cp_motor    = 900.0    # [J/(kg*K)]    Motor specific heat capacity
k_motor     = 237.0     # [W/(m*K)]     Motor thermal conductivity
D_motor     = 0.08      # [m]           Motor diameter

mass_motor  = mass_motor_total / n_rotors   # [kg]          Motor mass
power_motor = power_motor_total / n_rotors  # [W] Motor power

# Cruise parameters
power_factor_cruise = 3/4
t_cruise = 22  # in seconds
# endregion

# region Fin calculations
L_fin   = 0.06  # [m]       Fin length
H_fin   = 0.04  # [m]       Fin height
t_fin   = 0.001 # [m]       Fin thickness
s_fin   = 0.003 # [m]       Fin spacing
rho_fin = 2700  # [kg/m^3]  Fin density

n_fins  = np.floor(np.pi * D_motor / (t_fin + s_fin))           # [-]   Number of fins
A_b     = np.pi * D_motor * L_fin - n_fins * L_fin * t_fin      # [m^2] Unfinned area                                                        # Unfinned area
A_f     = 2 * H_fin * L_fin + 2 * H_fin * t_fin + L_fin * t_fin # [m^2] Fin area
A_c     = L_fin * t_fin                                         # [m^2] Fin cross-sectional area
P_fin   = 2 * L_fin + 2 * t_fin                                 # [m]   Fin perimeter
A_f_tot = n_fins * A_f + A_b                                    # [m^2] Total surface area

volume_fin = n_fins * L_fin * H_fin * t_fin # [m^3] Fin volume
mass_fin = volume_fin * rho_fin             # [kg] Fin mass
mass_fin_total = mass_fin * n_rotors        # [kg] Total fin mass
mass_motor_total = mass_motor * n_rotors    # [kg] Total motor mass

print('--- FIN PARAMETERS ---')
print(f'n_fins: {n_fins}')
print(f'A_b: {A_b} m^2')
print(f'A_f: {A_f} m^2')
print(f'A_c: {A_c} m^2')
print(f'P_fin: {P_fin} m')
print(f'Total area: {A_f_tot} m^2')
print(f'Fin volume: {volume_fin} m^3')
print(f'Fin mass: {mass_fin} kg')
print(f'Total fin mass: {mass_fin_total} kg')
print(f'Total motor mass excl. fins: {mass_motor_total} kg')
# endregion

# region Heat transfer functions
def calc_w_rotor(A_disk, air_density, weight, n_rotors): # Rotor downwash velocity
    return np.sqrt(weight / n_rotors/(2*A_disk*air_density))

def calc_Pr(kin_vsc_air, alpha_air):    # Prandtl number
    return kin_vsc_air / alpha_air

def calc_Re(kin_vsc_air, L_fin, w_rotor): # Reynolds number
    return w_rotor * (L_fin) / kin_vsc_air

def calc_Nu(Re, Pr): # Nusselt number
    return 0.664 * Re**(1/2) * Pr**(1/3)

def calc_h(Nu, k_air, L_fin): # Convective heat transfer coefficient
    return Nu * k_air / L_fin

def calc_eff_fin(h, P_fin, k_s, A_c, A_f): # Fin efficiency
    const = np.sqrt(h * P_fin / (k_s * A_c))
    u1 = np.sqrt(h * P_fin * k_s * A_c)
    u2 = np.tanh(const * H_fin) + h / (const * k_s)
    l1 = h * A_f
    l2 = 1 + h / (const * k_s) * np.tanh(const * H_fin)
    return u1 / l1 * u2 / l2

def calc_R_th(h, eff_fin, n_fins, A_f, A_b): # Thermal resistance
    return 1 / (h * (eff_fin * n_fins * A_f + A_b))
# endregion

# region Convection calculations
w_rotor = calc_w_rotor(A_disk, rho_air, weight, n_rotors)
Pr      = calc_Pr(kin_vsc_air, alpha_air)
Re      = calc_Re(kin_vsc_air, L_fin, w_rotor)
Nu      = calc_Nu(Re, Pr)
h       = calc_h(Nu, k_air, L_fin)
eff_fin = calc_eff_fin(h, P_fin, k_motor, A_c, A_f)
R_th    = calc_R_th(h, eff_fin, n_fins, A_f, A_b)

print('--- CONVECTION PARAMETERS ---')
print(f'w_rotor: {w_rotor} m/s')
print(f'Pr: {Pr}')
print(f'Re: {Re}')
print(f'Nu: {Nu}')
print(f'h: {h} W/(m^2*K)')
print(f'eff_fin: {eff_fin}')
print(f'R_th: {R_th} K/W')
# endregion
print(n_fins)
# region Thermal simulation
time = np.linspace(0, 30*60, 1000)
dt   = time[1] - time[0]

# Temperature rise without cooling
temp_no_cool = np.zeros_like(time)
temp_no_cool[0] = temp_init
for i in range(1, len(time)):
    if time[i] > 60:
        if time[i] <= (60 + t_cruise):
            power_effective = power_motor * (1 - eff_motor) * power_factor_cruise
        else:
            power_effective = power_motor * (1 - eff_motor)
    else:
        power_effective = power_motor * (1 - eff_motor)
    
    temp_no_cool[i] = temp_no_cool[i-1] + power_effective * dt / (mass_motor * cp_motor)

# Calculate temperature rise with convective cooling
temp_conv = np.zeros_like(time)
temp_conv[0] = temp_init
for i in range(1, len(time)):
    # Calculate convective heat transfer from motor to surroundings
    heat_loss = (temp_conv[i-1] - temp_amb) / R_th
    
    if time[i] > 60:
        if time[i] <= (60 + t_cruise):
            power_effective = power_motor * (1 - eff_motor) * power_factor_cruise
        else:
            power_effective = power_motor * (1 - eff_motor)
    else:
        power_effective = power_motor * (1 - eff_motor)
    
    # Calculate change in temperature
    delta_temp = (power_effective - heat_loss) * dt / ((mass_motor + mass_fin) * cp_motor)
    # Update temperature
    temp_conv[i] = temp_conv[i-1] + delta_temp

# Calculate time to reach maximum temperature for no cooling
time_max_no_cool = np.argmax(temp_no_cool >= temp_max) * dt

# Calculate time to reach maximum temperature with convective cooling
time_max_conv = np.argmax(temp_conv >= temp_max) * dt

print('--- TIME TO REACH MAXIMUM TEMPERATURE ---')
print(f'No cooling: {time_max_no_cool/60} min')
print(f'Convective cooling: {time_max_conv/60} min')
# endregion

# region Plotting
plt.figure(figsize=(10, 6))

plt.plot(time/60, temp_no_cool - 273.15, label='Without cooling')
plt.plot(time/60, temp_conv - 273.15, label='With convective cooling')
plt.plot([0, time[-1]/60], [temp_max-273.15, temp_max-273.15], 'k--', label='Maximum temperature')
plt.xlim(left=0)
plt.ylim([temp_init-273.15 - 10, temp_max-273.15 + 20])
plt.xlabel('Time [min]', fontsize=14)
plt.ylabel('Temperature [°C]', fontsize=14)
plt.tick_params(axis='both', which='major', labelsize=14)
plt.legend(fontsize=14)
plt.grid()
plt.show()

plt.plot(time/60, temp_no_cool - 273.15)
plt.plot([0, 5], [temp_max-273.15, temp_max-273.15], 'k--', label='Maximum temperature')
plt.xlim(left=0, right=5)
plt.ylim([temp_init-273.15 - 10, temp_max-273.15 + 20])
plt.xlabel('Time [min]', fontsize=14)
plt.ylabel('Temperature [°C]', fontsize=14)
plt.tick_params(axis='both', which='major', labelsize=14)
plt.grid()
plt.show()
# endregion
