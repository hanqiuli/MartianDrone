import numpy as np
from environment_properties import ENV

class Rotor:
    def __init__(self, M_tip, N_rotors, N_blades, T_A_disk=8.5, CT_sigma=0.115, cl_cd=10, k_hover=1.2, total_eff=0.7, e_bat=250, t_flight=20*60):
        self.M_tip = M_tip
        self.N_rotors = N_rotors
        self.N_blades = N_blades
        self.T_A_disk = T_A_disk
        self.CT_sigma = CT_sigma
        self.cl_cd = cl_cd
        self.k_hover = k_hover
        self.total_eff = total_eff
        self.e_bat = e_bat
        self.t_flight = t_flight

        self.V_tip = self.M_tip * ENV['a']
        self.cl_mean = 6 * self.CT_sigma
        self.cd_mean = self.cl_mean / self.cl_cd

    def required_params_per_rotor(self, T_required):
        self.A_disk = T_required / self.T_A_disk
        self.r_disk = np.sqrt(self.A_disk / np.pi)

    def calculate_A_blade_total(self, T_required):
        self.A_blade_total = (T_required) / (ENV['rho'] * self.CT_sigma * self.V_tip**2)

    def calculate_ct_and_solidity(self, T_required):
        self.CT = (T_required) / (self.A_disk * ENV['rho'] * self.V_tip**2)
        self.sigma = self.CT / self.CT_sigma

    def calculate_power_per_rotor(self, T_required):
        self.P_per_rotor = self.k_hover * (T_required) * np.sqrt((T_required)/(2*ENV['rho']*self.A_disk)) + \
            ENV['rho'] * self.A_blade_total * self.V_tip**3 * self.cd_mean / 8
        return self.P_per_rotor

    def calculate_power_total(self):
        self.P_total = (self.P_per_rotor * self.N_rotors) / self.total_eff

    def calculate_total_energy(self, Wh=False):
        return self.P_total * self.t_flight / (3599 * float(Wh) + 1)

    def calculate_battery_mass(self):
        return self.calculate_total_energy(Wh=True) / self.e_bat

    def calculate_motor_mass(self):
        return 0.432/1000 * self.P_total

    def calculate_blade_mass(self):
        return 1.1 * self.A_blade_total * self.N_rotors

    def calculate_hub_mass(self, T_required):
        return 0.05 * T_required / ENV['g'] * self.N_rotors

    def calculate_shaft_mass(self):
        return 0.15 * 0.15 * self.r_disk * self.N_rotors

    def calculate_support_mass(self):
        return 0.2 * self.r_disk * self.N_rotors

    def calculate_rotor_group_mass(self, T_required):
        return self.calculate_blade_mass() + self.calculate_hub_mass(T_required) + \
            self.calculate_shaft_mass() + self.calculate_support_mass()

    def calculate_struct_mass(self, T_required):
        return 28 * (T_required * self.N_rotors / ENV['g'] / 1000) ** (2/3) + 0.067 * T_required / ENV['g'] * self.N_rotors