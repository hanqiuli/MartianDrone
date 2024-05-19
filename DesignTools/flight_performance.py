import numpy as np
import matplotlib.pyplot as plt

# region Functions
def calc_parasite_drag_area(m):
    f = 2.5 * 0.15737 * (m/1000)**(2/3)
    return f

def calc_induced_ideal_power_ratio(mu, k_hover):
    k_i = k_hover * np.cosh(6.76 * mu**2)
    return k_i

def calc_profile_power_factor(mu):
    if mu == 0:
        return 1
    else:
        t1 = 0.5 * (1 + 6 * mu**2 + mu**4)
        t2 = 0.25 * (2 + 5 * mu**2) * np.sqrt(1 + mu**2)
        t3 = 0.375 * mu**4 * np.log((np.sqrt(1 + mu**2) + 1)/(np.sqrt(1 + mu**2) - 1))
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

def calc_profile_power_coeff(mu, sigma, cl_cd):
    cd_o = calc_profile_drag_coeff(CT_sigma, cl_cd)
    Fp_o = calc_profile_power_factor(mu)
    CP_o = sigma * cd_o / 8 * Fp_o
    return CP_o

def calc_parasite_power_coeff(mu, m, A):
    f = calc_parasite_drag_area(m)
    CP_p = 0.5 * mu**3 * f / A
    return CP_p
# endregion

# region Constants
# region Atmospheric Constants
rho = 0.017  # kg/m^3
a = 233.1 # m/s
g = 3.71 # m/s^2
# endregion

# region Aerodynamic Constants
M_tip = 0.7
CT_sigma = 0.115
k_hover = 1.2
cl_cd = 10
# endregion

# region Vehicle Constants
m = 46.47571934878282 # kg
sigma = 0.1632232432895824
r_rotor = 1.0376257291135118 # m
n_rotors = 6
# endregion

# region Derived Values
V_tip = M_tip * a
A_rotor = np.pi * r_rotor**2
A_tot = n_rotors * A_rotor
CT = CT_sigma * sigma
dims = rho * A_tot * V_tip**3
# endregion
# endregion

# region Calculations
advance_ratios = np.linspace(0.0, 0.5, 100)
airspeeds = [mu * V_tip for mu in advance_ratios]
CP_is = []
CP_os = []
CP_ps = []
CPs = []
powers = []
for mu in advance_ratios:
    CP_i = calc_induced_power_coeff(k_hover, mu, CT)
    CP_o = calc_profile_power_coeff(mu, sigma, cl_cd)
    CP_p = calc_parasite_power_coeff(mu, m, A_tot)
    CP = CP_i + CP_o + CP_p
    power = CP * dims
    CP_is.append(CP_i)
    CP_os.append(CP_o)
    CP_ps.append(CP_p)
    CPs.append(CP)
    powers.append(power)
# endregion

# region Plotting
def plot_curve(x, y, xlabel, ylabel):
    plt.plot(x, y)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(which='both', linestyle='--', linewidth=0.5)
    plt.minorticks_on()
    plt.show()

def plot_curves(x, ys, labels, xlabel, ylabel):
    for y, label in zip(ys, labels):
        plt.plot(x, y, label=label)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(which='both', linestyle='--', linewidth=0.5)
    plt.minorticks_on()
    plt.legend(prop={'size': 8}, ncol=1)
    plt.show()

plot_curve(advance_ratios, powers, 'Advance Ratio $\mu$', 'Power $P_r$ [W]')
plot_curve(airspeeds, powers, 'Airspeed $V$ [m/s]', 'Power $P_r$ [W]')
plot_curves(advance_ratios, [CP_is, CP_os, CP_ps, CPs], ['Induced Power Coefficient $C_{P_i}$',
                                                         'Profile Power Coefficient $C_{P_o}$',
                                                         'Parasite Power Coefficient $C_{P_p}$',
                                                         'Total Power Coefficient $C_P$'], 
                                                         'Advance Ratio $\mu$', 'Power Coefficient $C_P$')
plot_curves(advance_ratios, [np.divide(CP_is, CPs), 
                             np.divide(CP_os, CPs), 
                             np.divide(CP_ps, CPs)], ['Induced Power Ratio $C_{P_i}/C_P$',
                                                      'Profile Power Ratio $C_{P_o}/C_P$',
                                                      'Parasite Power Ratio $C_{P_p}/C_P$'],
                                                      'Advance Ratio $\mu$', 'Power Ratio')

# endregion