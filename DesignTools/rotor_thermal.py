import numpy as np
import matplotlib.pyplot as plt
from rotor import Rotor

# Martian atmosphere
rho_air     = 0.017     # [kg/m^3]      Air density
g           = 3.71      # [m/s^2]       Gravitational acceleration
temp_amb    = 220       # [K]           Ambient temperature
cp_air      = 772       # [J/(kg*K)]    Specific heat capacity of air
dyn_vsc_air = 0.0000113 # [Pa*s]        Air dynamic viscosity
k_air       = 0.024     # [W/(m*K)]     Air thermal conductivity

kin_vsc_air = dyn_vsc_air / rho_air         # [m^2/s]   Air kinematic viscosity
alpha_air   = k_air / (rho_air * cp_air)    # [m^2/s]   Air thermal diffusivity
beta_air    = 1 / temp_amb                  # [1/K]     Air thermal expansion coefficient

# Motor parameters
power_motor = 4000  # [W]           Motor power
mass_motor  = 1.5   # [kg]          Motor mass
cp_motor    = 1825  # [J/(kg*K)]    Motor specific heat capacity
k_motor     = 216   # [W/(m*K)]     Motor thermal conductivity
eff_motor   = 0.70  # [-]           Motor efficiency
D_motor     = 0.08 # [m]           Motor diameter
temp_init   = 240   # [K]           Initial motor temperature
temp_max    = 373.15  # [K]           Maximum motor temperature

# Rotor parameters
weight  = 100 * g           # [N] Maximum thrust
T_A_disk = 8.5
A_disk  = weight / T_A_disk # [m^2] Disk area
n_rotors = 6                # [-] Number of rotors

# Fin parameters
L_fin   = 0.08  # [m]   Fin length
H_fin   = 0.05  # [m]   Fin height
t_fin   = 0.001 # [m]   Fin thickness
s_fin   = 0.002 # [m]   Fin spacing

N_fin   = np.pi * D_motor / (t_fin + s_fin)                     # [-]   Number of fins
A_b     = np.pi * D_motor * L_fin - N_fin * L_fin * t_fin       # [m^2] Unfinned area                                                        # Unfinned area
A_f     = 2 * H_fin * L_fin + 2 * H_fin * t_fin + L_fin * t_fin # [m^2] Fin area
A_c     = L_fin * t_fin                                         # [m^2] Fin cross-sectional area
P_fin   = 2 * L_fin + 2 * t_fin                                 # [m]   Fin perimeter
A_f_tot = N_fin * A_f + A_b                                     # [m^2] Total surface area

# Heat transfer calculations
def calc_w_rotor(A_disk, air_density, weight, n_rotors): # Rotor downwash velocity
    return np.sqrt(weight/n_rotors/(2*A_disk*air_density))

def calc_Pr(kin_vsc_air, alpha_air):    # Prandtl number
    return kin_vsc_air / alpha_air

def calc_Re(kin_vsc_air, L_fin, w_rotor): # Reynolds number
    return w_rotor * (L_fin) / kin_vsc_air

def calc_Nu(Re, Pr): # Nusselt number
    return 0.664*(Re**0.5)*(Pr**(1/3))

def calc_h(Nu, k_air, L_fin): # Convective heat transfer coefficient
    return Nu * k_air / L_fin

def calc_eff_fin(h, P_fin, k_s, A_c, A_f): # Fin efficiency
    cock = np.sqrt(h * P_fin / (k_s * A_c))
    u1 = np.sqrt(h * P_fin * k_s * A_c)
    u2 = np.tanh(cock * H_fin) + h / (cock * k_s)
    l1 = h * A_f
    l2 = 1 + h / (cock * k_s) * np.tanh(cock * H_fin)
    return u1 / l1 * u2 / l2

def calc_R_th(h, eff_fin, N_fin, A_f, A_b): # Thermal resistance
    return 1 / (h * (eff_fin * N_fin * A_f + A_b))

w_rotor = calc_w_rotor(A_disk, rho_air, weight, n_rotors)
Pr = calc_Pr(kin_vsc_air, alpha_air)
Re = calc_Re(kin_vsc_air, L_fin, w_rotor)
Nu = calc_Nu(Re, Pr)
h = calc_h(Nu, k_air, L_fin)
eff_fin = calc_eff_fin(h, P_fin, k_motor, A_c, A_f)
R_th = calc_R_th(h, eff_fin, N_fin, A_f, A_b)

print(f'A_b: {A_b} m^2')
print(f'A_f: {A_f} m^2')
print(f'A_c: {A_c} m^2')
print(f'P_fin: {P_fin} m')
print(f'w_rotor: {w_rotor} m/s')
print(f'Pr: {Pr}')
print(f'Re: {Re}')
print(f'Nu: {Nu}')
print(f'h: {h} W/(m^2*K)')
print(f'eff_fin: {eff_fin}')
print(f'R_th: {R_th} K/W')
print(f'Total area: {A_f_tot} m^2')

time = np.linspace(0, 1800, 10000)
dt   = time[1] - time[0]

# Temperature rise without cooling
temp_no_cool = np.zeros_like(time)
temp_no_cool[0] = temp_init
for i in range(1, len(time)):
    temp_no_cool[i] = temp_no_cool[i-1] + power_motor * (1-eff_motor) * dt / (mass_motor * cp_motor)

# Calculate temperature rise with convective cooling
temp_conv = np.zeros_like(time)
temp_conv[0] = temp_init
for i in range(1, len(time)):
    # Calculate convective heat transfer from motor to surroundings
    heat_loss = (temp_conv[i-1] - temp_amb) / R_th
    # Calculate change in temperature
    delta_temp = ((power_motor * (1 - eff_motor)) - heat_loss) * dt / (mass_motor * cp_motor)
    # Update temperature
    temp_conv[i] = temp_conv[i-1] + delta_temp

# Plot results
plt.figure(figsize=(10, 6))

plt.plot(time/60, temp_no_cool - 273.15, label='Without Cooling')
plt.plot(time/60, temp_conv - 273.15, label='With Convective Cooling')
plt.plot([0, time[-1]/60], [temp_max-273.15, temp_max-273.15], 'k--', label='Max temperature')

plt.xlabel('Time [min]')
plt.ylabel('Temperature [deg C]')
plt.title('Motor temperature over time with Convective Cooling')
plt.legend()
plt.grid()
plt.show()