import numpy as np
import matplotlib.pyplot as plt
from rotor_new import Rotor
from environment_properties import ENV

if __name__ == "__main__":
    # Validation procedures, using values retrieved from MSH paper of ingenuity.
    gross_weight = 1.8 # [kg]
    number_rotors = 2 # [-]
    design_ctsigma = 0.1
    design_mtip = 0.7
    disk_loading = 0.8 * 3.711  # [N/m^2]

    hover_power = 3

    # things to validate
    solidity = 0.148
    rotor_radius = 0.605 # [m]
    A_disk = np.pi * rotor_radius**2

    # Calculate the thrust required for a single rotor
    thrust_required = (gross_weight * 3.711) / number_rotors

    # print rpm
    RPM = (design_mtip * ENV['a'] / rotor_radius) * 60 / (2 * np.pi)
    print(f"RPM: {RPM}")

    # Create a rotor object
    rotor = Rotor(M_tip=design_mtip, N_rotors=number_rotors, N_blades=2, T_A_disk=disk_loading, CT_sigma=design_ctsigma, validating=True)

    # Calculate the required parameters for a single rotor
    rotor.required_params_per_rotor(thrust_required)
    print(f"A_disk: {rotor.A_disk}")
    print(f"r_disk: {rotor.r_disk}")
    print(f"Actual RPM: {rotor.V_tip / (2 * np.pi * rotor.r_disk) * 60}")
    print(f"radius_diff_in_percent: {100 * (rotor.r_disk - rotor_radius) / rotor_radius}")
    rotor.calculate_ct_and_solidity(thrust_required)
    print(f"sigma: {rotor.sigma * number_rotors}")
    print(f"solidity_diff_in_percent: {100 * (rotor.sigma * number_rotors - solidity) / solidity}")
    rotor.calculate_A_blade_total(thrust_required)

    # Calculate the power required
    rotor.calculate_power_per_rotor(thrust_required)
    rotor.calculate_power_total()
    print(f"P_total: {rotor.P_total}")
   
