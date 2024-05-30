# test_rotor.py
import sys
sys.path.append("./DesignTools")

import pytest
import numpy as np
from environment_properties import ENV
from rotor_new import Rotor

#region Constants for testing
T_REQUIRED = 1000  
T_FLIGHT = 20 * 60
M_TIP = 0.8
N_ROTORS = 4
N_BLADES = 3
EBAT = 250
#endregion

# Initialize the Rotor instance for testing
rotor = Rotor(M_tip=M_TIP, N_rotors=N_ROTORS, N_blades=N_BLADES, t_flight=T_FLIGHT, e_bat=EBAT)

def test_required_params_per_rotor():
    rotor.required_params_per_rotor(T_REQUIRED)
    assert rotor.A_disk == pytest.approx(T_REQUIRED / rotor.T_A_disk)
    assert rotor.r_disk == pytest.approx(np.sqrt(rotor.A_disk / np.pi))

def test_calculate_A_blade_total():
    rotor.calculate_A_blade_total(T_REQUIRED)
    expected_A_blade_total = T_REQUIRED / (ENV['rho'] * rotor.CT_sigma * rotor.V_tip**2)
    assert rotor.A_blade_total == pytest.approx(expected_A_blade_total)

def test_calculate_ct_and_solidity():
    rotor.required_params_per_rotor(T_REQUIRED)
    rotor.calculate_ct_and_solidity(T_REQUIRED)
    expected_CT = T_REQUIRED / (rotor.A_disk * ENV['rho'] * rotor.V_tip**2)
    expected_sigma = expected_CT / rotor.CT_sigma
    assert rotor.CT == pytest.approx(expected_CT)
    assert rotor.sigma == pytest.approx(expected_sigma)

def test_calculate_power_per_rotor():
    rotor.required_params_per_rotor(T_REQUIRED)
    rotor.calculate_A_blade_total(T_REQUIRED)
    rotor.calculate_ct_and_solidity(T_REQUIRED)
    P_per_rotor = rotor.calculate_power_per_rotor(T_REQUIRED)
    expected_P_per_rotor = rotor.k_hover * T_REQUIRED * np.sqrt(T_REQUIRED / (2 * ENV['rho'] * rotor.A_disk)) + \
        ENV['rho'] * rotor.A_blade_total * rotor.V_tip**3 * rotor.cd_mean / 8
    assert P_per_rotor == pytest.approx(expected_P_per_rotor)

def test_calculate_power_total():
    rotor.required_params_per_rotor(T_REQUIRED)
    rotor.calculate_A_blade_total(T_REQUIRED)
    rotor.calculate_ct_and_solidity(T_REQUIRED)
    rotor.calculate_power_per_rotor(T_REQUIRED)
    rotor.calculate_power_total()
    expected_P_total = (rotor.P_per_rotor * rotor.N_rotors) / rotor.total_eff
    assert rotor.P_total == pytest.approx(expected_P_total)

def test_calculate_total_energy():
    rotor.required_params_per_rotor(T_REQUIRED)
    rotor.calculate_A_blade_total(T_REQUIRED)
    rotor.calculate_ct_and_solidity(T_REQUIRED)
    rotor.calculate_power_per_rotor(T_REQUIRED)
    rotor.calculate_power_total()
    total_energy = rotor.calculate_total_energy(Wh=True)
    expected_total_energy = rotor.P_total * T_FLIGHT / 3600
    assert total_energy == pytest.approx(expected_total_energy)

def test_calculate_battery_mass():
    rotor.required_params_per_rotor(T_REQUIRED)
    rotor.calculate_A_blade_total(T_REQUIRED)
    rotor.calculate_ct_and_solidity(T_REQUIRED)
    rotor.calculate_power_per_rotor(T_REQUIRED)
    rotor.calculate_power_total()
    battery_mass = rotor.calculate_battery_mass()
    expected_battery_mass = rotor.calculate_total_energy(Wh=True) / rotor.e_bat
    assert battery_mass == pytest.approx(expected_battery_mass)

def test_calculate_motor_mass():
    rotor.required_params_per_rotor(T_REQUIRED)
    rotor.calculate_A_blade_total(T_REQUIRED)
    rotor.calculate_ct_and_solidity(T_REQUIRED)
    rotor.calculate_power_per_rotor(T_REQUIRED)
    rotor.calculate_power_total()
    motor_mass = rotor.calculate_motor_mass()
    expected_motor_mass = 0.432 / 1000 * rotor.P_total
    assert motor_mass == pytest.approx(expected_motor_mass)

def test_calculate_blade_mass():
    rotor.required_params_per_rotor(T_REQUIRED)
    rotor.calculate_A_blade_total(T_REQUIRED)
    blade_mass = rotor.calculate_blade_mass()
    expected_blade_mass = 1.1 * rotor.A_blade_total * rotor.N_rotors
    assert blade_mass == pytest.approx(expected_blade_mass)

def test_calculate_hub_mass():
    hub_mass = rotor.calculate_hub_mass(T_REQUIRED)
    expected_hub_mass = 0.05 * T_REQUIRED / ENV['g'] * rotor.N_rotors
    assert hub_mass == pytest.approx(expected_hub_mass)

def test_calculate_shaft_mass():
    rotor.required_params_per_rotor(T_REQUIRED)
    shaft_mass = rotor.calculate_shaft_mass()
    expected_shaft_mass = 0.15 * 0.15 * rotor.r_disk * rotor.N_rotors
    assert shaft_mass == pytest.approx(expected_shaft_mass)

def test_calculate_support_mass():
    rotor.required_params_per_rotor(T_REQUIRED)
    support_mass = rotor.calculate_support_mass()
    expected_support_mass = 0.2 * rotor.r_disk * rotor.N_rotors
    assert support_mass == pytest.approx(expected_support_mass)

def test_calculate_rotor_group_mass():
    rotor.required_params_per_rotor(T_REQUIRED)
    rotor.calculate_A_blade_total(T_REQUIRED)
    rotor_group_mass = rotor.calculate_rotor_group_mass(T_REQUIRED)
    expected_rotor_group_mass = rotor.calculate_blade_mass() + rotor.calculate_hub_mass(T_REQUIRED) + \
        rotor.calculate_shaft_mass() + rotor.calculate_support_mass()
    assert rotor_group_mass == pytest.approx(expected_rotor_group_mass)

def test_calculate_struct_mass():
    struct_mass = rotor.calculate_struct_mass(T_REQUIRED)
    expected_struct_mass = 28 * (T_REQUIRED * rotor.N_rotors / ENV['g'] / 1000) ** (2 / 3) + 0.067 * T_REQUIRED / ENV['g'] * rotor.N_rotors
    assert struct_mass == pytest.approx(expected_struct_mass)

if __name__ == "__main__":
    pytest.main()