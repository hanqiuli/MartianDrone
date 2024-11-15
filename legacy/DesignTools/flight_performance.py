import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fsolve
from environment_properties import ENV

# region Functions
def calc_parasite_drag_area(m):
    f = 2.5 * 0.15737 * (m/1000)**(2/3)
    return f

def calc_induced_ideal_power_ratio(mu, k_hover):
    k_i = k_hover * np.cosh(6.76 * mu**2)
    return k_i

def calc_profile_power_factor(mu):
    t1 = 0.5 * (1 + 6 * mu**2 + mu**4)
    t2 = 0.25 * (2 + 5 * mu**2) * np.sqrt(1 + mu**2)
    t3 = np.zeros_like(mu)
    mask = mu != 0
    mu_mask = mu[mask]
    t3[mask] = 0.375 * mu_mask**4 * np.log(
        (np.sqrt(1 + mu_mask**2) + 1) / (np.sqrt(1 + mu_mask**2) - 1))
    return t1 + t2 + t3

def calc_inflow_ratio(CT):
    la = np.sqrt(CT / 2)
    return la

def calc_profile_drag_coeff(CT_sigma, cl_cd):
    cd_o = 6 * CT_sigma / cl_cd
    return cd_o

def calc_induced_power_coeff(k_hover, mu, CT):
    k_i = calc_induced_ideal_power_ratio(mu, k_hover)
    la = calc_inflow_ratio(CT)
    CP_i = k_i * CT**2 / (2 * np.sqrt(mu**2 + la**2))
    return CP_i

def calc_profile_power_coeff(mu, sigma, CT_sigma, cl_cd):
    cd_o = calc_profile_drag_coeff(CT_sigma, cl_cd)
    FP_o = calc_profile_power_factor(mu)
    CP_o = sigma * cd_o / 8 * FP_o
    return CP_o

def calc_parasite_power_coeff(mu, m, A):
    f = calc_parasite_drag_area(m)
    CP_p = 0.5 * mu**3 * f / A
    return CP_p
# endregion

# region Constants
# region Aerodynamic Constants
M_tip = 0.7
CT_sigma = 0.115
k_hover = 1.2
cl_cd = 10
M_at_max = 0.95  # Maximum advancing tip Mach number
# endregion

# region Vehicle Constants
m = 46.47571934878282 # kg
sigma = 0.1632232432895824
r_rotor = 1.0376257291135118 # m
n_rotors = 6
# endregion

# region Derived Values
V_tip = M_tip * ENV['a']
A_rotor = np.pi * r_rotor**2
A_tot = n_rotors * A_rotor
CT = CT_sigma * sigma
powdim = ENV['rho'] * A_tot * V_tip**3
# endregion
# endregion

# region Calculations
advance_ratios = np.linspace(0.0, 0.5, 100)
airspeeds = advance_ratios * V_tip
CP_is = calc_induced_power_coeff(k_hover, advance_ratios, CT)
CP_os = calc_profile_power_coeff(advance_ratios, sigma, CT_sigma, cl_cd)
CP_ps = calc_parasite_power_coeff(advance_ratios, m, A_tot)
CPs = CP_is + CP_os + CP_ps
powers = CPs * powdim
# endregion

# region Optimal Airspeeds
min_power_idx = np.argmin(powers)
min_power_airspeed = airspeeds[min_power_idx]

powers_derivative = np.gradient(powers, airspeeds)

def tangent_condition(airspeed):
    power = np.interp(airspeed, airspeeds, powers)
    power_derivative = np.interp(airspeed, airspeeds, powers_derivative)
    return power - airspeed * power_derivative

tangent_airspeed_initial_guess = min_power_airspeed
tangent_airspeed = fsolve(tangent_condition, tangent_airspeed_initial_guess)[0]
tangent_power = np.interp(tangent_airspeed, airspeeds, powers)
tangent_slope = tangent_power / tangent_airspeed
tangent_line_x = np.linspace(0, max(airspeeds), 100)
tangent_line_y = tangent_slope * tangent_line_x

# Calculate the advance ratio and airspeed for the maximum advancing tip Mach number
advance_ratio_max = M_at_max / M_tip - 1
airspeed_max = advance_ratio_max * V_tip

print(f'Power at hover: {powers[0]:.2f} [W]')
print(f'Airspeed for maximum endurance: {min_power_airspeed:.2f} [m/s]')
print(f'Power at maximum endurance: {powers[min_power_idx]:.2f} [W]')
print(f'Theoretical airspeed for maximum range: {tangent_airspeed:.2f} [m/s]')
print(f'Theoretical power at maximum range: {tangent_power:.2f} [W]')
print(f'Maximum advance ratio due to M_at_max: {advance_ratio_max:.3f} [-]')
print(f'Maximum airspeed: {airspeed_max:.2f} [m/s]')
print(f'Power at maximum: {np.interp(airspeed_max, airspeeds, powers):.2f} [W]')
# endregion

# region Plotting
# Plot advance ratios vs powers with vertical line at advance_ratio_max
plt.plot(advance_ratios, powers, color='firebrick')
plt.xlabel('Advance Ratio $\mu$', fontsize=14)
plt.ylabel('Power $P_r$ [W]', fontsize=14)
plt.xlim(left=0)
plt.grid(which='both', linestyle=':', linewidth=0.5)
plt.minorticks_on()
plt.tick_params(axis='both', which='major', labelsize=14)
plt.show()

# Plot power coefficients
labels = ['Induced Power Coefficient $C_{P_i}$',
          'Profile Power Coefficient $C_{P_o}$',
          'Parasite Power Coefficient $C_{P_p}$',
          'Total Power Coefficient $C_P$']
for y, label in zip([CP_is, CP_os, CP_ps, CPs], labels):
    plt.plot(advance_ratios, y, label=label, linewidth=1.5)
plt.xlabel('Advance Ratio $\mu$', fontsize=14)
plt.ylabel('Power Coefficient $C_P$', fontsize=14)
plt.xlim(left=0)
plt.ylim(bottom=0)
plt.ylim(top=np.max(CPs) * 1.2)
plt.grid(which='both', linestyle=':', linewidth=0.5)
plt.minorticks_on()
plt.legend(fontsize=12)
plt.tick_params(axis='both', which='major', labelsize=14)
plt.show()

# # Plot power ratios
# labels = ['Induced Power Ratio $C_{P_i}/C_P$',
#           'Profile Power Ratio $C_{P_o}/C_P$',
#           'Parasite Power Ratio $C_{P_p}/C_P$']
# ratios = [CP_is, CP_os, CP_ps]
# for y, label in zip(np.divide(ratios, CPs), labels):
#     plt.plot(advance_ratios, y, label=label)
# plt.xlabel('Advance Ratio $\mu$')
# plt.ylabel('Power Ratio')
# plt.grid(which='both', linestyle=':', linewidth=0.5)
# plt.minorticks_on()
# plt.legend(prop={'size': 8}, ncol=1)
# plt.show()

# Plot airspeeds vs powers with vertical line at airspeed_max
plt.plot(airspeeds, powers, label='Power Required')
plt.axvline(x=airspeed_max, color='black', linestyle='--', label=f'Max $M_{{at}}$ ({M_at_max})', linewidth=0.5)
plt.scatter([min_power_airspeed], [powers[min_power_idx]], color='orange', label='Max Endurance', zorder=5)
plt.scatter([tangent_airspeed], [tangent_power], color='green', label='Max Range', zorder=4)
plt.plot(tangent_line_x, tangent_line_y, color='green', linewidth=0.5)
plt.xlabel('Airspeed $V$ [m/s]')
plt.ylabel('Power $P_r$ [W]')
plt.xlim(left=0)
plt.ylim(bottom=0)
plt.grid(which='both', linestyle=':', linewidth=0.5)
plt.minorticks_on()
plt.legend()
plt.show()
# endregion
