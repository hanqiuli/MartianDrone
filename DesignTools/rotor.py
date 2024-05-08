import numpy as np
import matplotlib.pyplot as plt
from environment_properties import ENV

env = ENV

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


class Rotor:
    def __init__(self, M_tip, N, N_blades, T_W_max, T_A_disk, ct_sigma, cl_mean, cd_mean, k_hover):
        self.M_tip = M_tip
        self.N = N
        self.N_blades = N_blades
        self.T_W_max = T_W_max
        self.T_A_disk = T_A_disk
        self.ct_sigma = ct_sigma
        self.cl_mean = cl_mean
        self.cd_mean = cd_mean
        self.k_hover = k_hover

    def calculate_V_tip(self, a):
        self.V_tip = self.M_tip * a

    def required_params_per_rotor(self, T_required, thrust_margin):
        self.A_disk = (thrust_margin * T_required) / self.T_A_disk
        self.r_disk = np.sqrt(self.A_disk / np.pi)

    def calculate_A_blade(self, T_required, rho):
        self.A_blade = T_required / (rho * self.ct_sigma * self.V_tip**2)

    def calculate_ct_and_solidity(self, T_required, rho):
        self.ct = T_required / (self.A_disk * rho * self.V_tip**2)
        self.sigma = self.ct / self.ct_sigma

    def calculate_omega(self):
        return self.V_tip / self.r_disk

    def calculate_power_per_rotor(self, T_required, rho):
        self.P_per_rotor = self.k_hover * T_required * np.sqrt((T_required)/(2*rho*self.A_disk)) + \
            rho * self.A_blade * self.V_tip**3 * self.cd_mean * (1/8)

    def calculate_power_total(self):
        '''total power required for all rotors'''
        self.P_total = self.P_per_rotor * self.N

    def calculate_total_energy(self, T_flight):
        self.E_total = (self.P_total * T_flight) / 3600 # Convert to Wh
        return self.E_total


    #  Formulas for masses

    def calculate_battery_mass(self, E_specific):
        return self.E_total / E_specific

    def calculate_motor_mass(self):
        return 0.432/1000 * self.P_total

    def calculate_blade_mass(self):
        # calculates TOTAL blade mass
        return 1.1 * self.A_blade * self.N

    def calculate_hub_mass(self, T_required):
        # calculate TOTAL hub mass
        return 0.05 * T_required * self.N

    def calculate_shaft_mass(self):
        # calculate TOTAL shaft mass
        return 0.15 * 0.15 * self.r_disk * self.N

    def calculate_support_mass(self):
        # calculate TOTAL mass of rotor support arms
        return 0.2 * self.r_disk * self.N

    def calculate_rotor_group_mass(self, T_required):
        # calculate TOTAL rotor group mass
        return self.calculate_blade_mass() + self.calculate_hub_mass(T_required) + \
                self.calculate_shaft_mass() + self.calculate_support_mass()
