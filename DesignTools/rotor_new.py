import numpy as np
import matplotlib.pyplot as plt
from environment_properties import ENV


class Rotor:
    def __init__(self, M_tip, N, N_blades, *, T_W_max=1.2, T_A_disk=8.5, ct_sigma=0.115, cl_mean=0.69, cd_mean=0.069, k_hover=1.2, total_eff = 0.7):
        self.M_tip = M_tip
        self.N = N
        self.N_blades = N_blades
        self.T_W_max = T_W_max
        self.T_A_disk = T_A_disk
        self.ct_sigma = ct_sigma
        self.cl_mean = cl_mean
        self.cd_mean = cd_mean
        self.k_hover = k_hover
        self.total_eff = total_eff

        self.V_tip = M_tip * ENV['a']

    def required_params_per_rotor(self, T_required):
        """This function calculates the required parameters for a single rotor, T_required is the thrust required for a single rotor"""
        self.A_disk = (self.T_W_max * T_required) / self.T_A_disk
        self.r_disk = np.sqrt(self.A_disk / np.pi)

    def calculate_A_blade_total(self, T_required):
        self.A_blade_total = (self.T_W_max * T_required) / (ENV['rho'] * self.ct_sigma * self.V_tip**2)

    def calculate_ct_and_solidity(self, T_required):
        self.ct = (self.T_W_max * T_required) / (self.A_disk * ENV['rho'] * self.V_tip**2)
        self.sigma = self.ct / self.ct_sigma

    def calculate_power_per_rotor(self, T_required):
        self.P_per_rotor = self.k_hover * (self.T_W_max * T_required) * np.sqrt((self.T_W_max * T_required)/(2*ENV['rho']*self.A_disk)) + \
            ENV['rho'] * self.A_blade_total * self.V_tip**3 * self.cd_mean * (1/8)
        return self.P_per_rotor

    def calculate_power_total(self):
        '''total power required for the system'''
        self.P_total = (self.P_per_rotor * self.N) / self.total_eff

    def calculate_total_energy(self, t_flight, Wh=False):
        E_total = self.P_total * t_flight # [J]
        if Wh:
            E_total = E_total / 3600
        return E_total
    
    #  Formulas for masses
    def calculate_battery_mass(self, t_flight, E_specific):
        return self.calculate_total_energy(t_flight, Wh=True) / E_specific

    def calculate_motor_mass(self):
        return 0.432/1000 * self.P_total

    def calculate_blade_mass(self):
        # calculates TOTAL blade mass
        return 1.1 * self.A_blade_total * self.N

    def calculate_hub_mass(self, T_required):
        # calculate TOTAL hub mass
        return 0.05 * T_required / ENV['g'] * self.N

    def calculate_shaft_mass(self):
        # calculate TOTAL shaft mass
        return 0.15 * 0.15 * self.r_disk * self.N

    def calculate_support_mass(self):
        # calculate TOTAL mass of rotor support arms
        return 0.2 * self.r_disk * self.N

    def calculate_rotor_group_mass(self, T_required):
        # calculate TOTAL rotor group mass
        # Based on MSH paper
        return self.calculate_blade_mass() + self.calculate_hub_mass(T_required) + \
                self.calculate_shaft_mass() + self.calculate_support_mass()
    
    def calculate_struct_mass(self, T_required):
        return 28 * (T_required * self.N / ENV['g'] / 1000) ** (2/3) + 0.067 * T_required / ENV['g'] * self.N # NASA MSH Concept design