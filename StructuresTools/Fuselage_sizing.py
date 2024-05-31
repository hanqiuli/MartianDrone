import sys
sys.path.append("./SETools")

import numpy
import pandas
import os

from SE_data import SEData, data_path, backup_path  

data_path = os.path.join('SETools/data', 'configuration_data.csv')
backup_path = os.path.join('SETools/data', 'backup')

data = SEData(data_path, backup_path)
print(data.get_all_data())


def battery_energy_density(dimensions, capacity_wh):
    # dimensions = [length, width, height]
    volume = dimensions[0] * dimensions[1] * dimensions[2]
    energy_density = capacity_wh / volume
    return energy_density






if __name__ == "__main__":
    
    energy_density = battery_energy_density([0.067, 0.0195, 0.076], 9*3.6 )

    battery_capacity = data.get_subsystem_property("PWR","Battery Capacity (Wh)")

    battery_volume = battery_capacity / energy_density
    
    data.update_subsystem_property("PWR","Volume", battery_volume)
    data.update_subsystem_property("PLD","Volume", 0.00675)
    data.update_subsystem_property("AVI","volume", 0.00675)
    data.update_subsystem_property("Avi","mass", 1)

    