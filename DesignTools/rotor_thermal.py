import numpy as np
import matplotlib.pyplot as plt

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

temp_init   = 273.15 - 10   # [K]   Initial motor temperature
temp_max    = 273.15 + 100  # [K]   Maximum motor temperature

# Rotorcraft parameters
mass                = 121.00    # [kg]  Total mass of the rotorcraft
mass_motor_total    = 9.00      # [kg]  Total mass of the motors
power_rotor_total   = 17000.0   # [W]   Total power of the rotors
r_rotor             = 1.00      # [m]   Rotor radius
n_rotors            = 6         # [-]   Number of rotors

weight  = mass * g                          # [N]   Maximum thrust
A_disk = np.pi * r_rotor**2                 # [m^2] Single rotor disk area
power_rotor = power_rotor_total / n_rotors  # [W]   Power per rotor

# Motor parameters
cp_motor    = 1825.0    # [J/(kg*K)]    Motor specific heat capacity
k_motor     = 216.0     # [W/(m*K)]     Motor thermal conductivity
eff_motor   = 0.70      # [-]           Motor efficiency
D_motor     = 0.08      # [m]           Motor diameter

mass_motor  = mass_motor_total / n_rotors   # [kg]          Motor mass
# mass_motor  = 0.5 # [kg]          Motor mass 
power_motor = power_rotor / eff_motor # [W] Motor power

# Fin parameters
L_fin   = 0.08  # [m]       Fin length
H_fin   = 0.05  # [m]       Fin height
t_fin   = 0.001 # [m]       Fin thickness
s_fin   = 0.002 # [m]       Fin spacing
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
mass_motor_total = mass_motor * n_rotors    # [kg] Total motor massº

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

# Heat transfer calculations
def calc_w_rotor(A_disk, air_density, weight, n_rotors): # Rotor downwash velocity
    return np.sqrt(weight/6/n_rotors/(2*A_disk*air_density))

def calc_Pr(kin_vsc_air, alpha_air):    # Prandtl number
    return kin_vsc_air / alpha_air

def calc_Re(kin_vsc_air, L_fin, w_rotor): # Reynolds number
    return w_rotor * (L_fin) / kin_vsc_air

def calc_Nu(Re, Pr): # Nusselt number
    return 0.664 * Re**(1/2) * Pr**(1/3)

def calc_h(Nu, k_air, L_fin): # Convective heat transfer coefficient
    return Nu * k_air / L_fin

def calc_eff_fin(h, P_fin, k_s, A_c, A_f): # Fin efficiency
    cock = np.sqrt(h * P_fin / (k_s * A_c))
    u1 = np.sqrt(h * P_fin * k_s * A_c)
    u2 = np.tanh(cock * H_fin) + h / (cock * k_s)
    l1 = h * A_f
    l2 = 1 + h / (cock * k_s) * np.tanh(cock * H_fin)
    return u1 / l1 * u2 / l2

def calc_R_th(h, eff_fin, n_fins, A_f, A_b): # Thermal resistance
    return 1 / (h * (eff_fin * n_fins * A_f + A_b))

w_rotor = calc_w_rotor(A_disk, rho_air, weight, n_rotors)
Pr = calc_Pr(kin_vsc_air, alpha_air)
Re = calc_Re(kin_vsc_air, L_fin, w_rotor)
Nu = calc_Nu(Re, Pr)
h = calc_h(Nu, k_air, L_fin)
eff_fin = calc_eff_fin(h, P_fin, k_motor, A_c, A_f)
R_th = calc_R_th(h, eff_fin, n_fins, A_f, A_b)

print('--- CONVECTION PARAMETERS ---')
print(f'w_rotor: {w_rotor} m/s')
print(f'Pr: {Pr}')
print(f'Re: {Re}')
print(f'Nu: {Nu}')
print(f'h: {h} W/(m^2*K)')
print(f'eff_fin: {eff_fin}')
print(f'R_th: {R_th} K/W')

time = np.linspace(0, 30*60, 1000)
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
    delta_temp = ((power_motor * (1 - eff_motor)) - heat_loss) * dt / ((mass_motor + mass_fin) * cp_motor)
    # Update temperature
    temp_conv[i] = temp_conv[i-1] + delta_temp

# Plot results
plt.figure(figsize=(10, 6))

plt.plot(time/60, temp_no_cool - 273.15, label='Without cooling')
plt.plot(time/60, temp_conv - 273.15, label='With convective cooling')
plt.plot([0, time[-1]/60], [temp_max-273.15, temp_max-273.15], 'k--', label='Max temperature')

plt.xlabel('Time [min]')
plt.ylabel('Temperature [deg C]')
plt.title('Motor temperature over time with convective cooling')
plt.legend()
plt.grid()
plt.show()