import numpy as np
import pandas as pd
import os
import shutil
from datetime import datetime

data_path = os.path.join('SETools/data', 'configuration_data.csv')
backup_path = os.path.join('SETools/data', 'backup')

class SE_data:
    '''Class to store and retrieve the design data for all the subsystems'''
    def __init__(self, data_path: str = data_path, backup_path: str = backup_path):
        self.data_path = data_path
        self.backup_path = backup_path
        if os.path.exists(self.data_path):
            self.data = pd.read_csv(self.data_path)
        else:
            self.data = pd.DataFrame(columns=["Property"])
        if not os.path.exists(self.backup_path):
            os.makedirs(self.backup_path)
    
    def create_backup(self):
        '''Function to create a backup of the current configuration file'''
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
        backup_file = os.path.join(self.backup_path, f'configuration_data_{timestamp}.csv')
        shutil.copy2(self.data_path, backup_file)
        print(f"Backup created at {backup_file}")

    def clear_values(self, subsystem: str = None):
        '''Function to clear values in the configuration file completely or for a single subsystem'''
        self.create_backup()
        if subsystem:
            if subsystem in self.data.columns:
                self.data[subsystem] = np.nan
            else:
                raise ValueError(f"Subsystem {subsystem} does not exist.")
        else:
            for col in self.data.columns[1:]:
                self.data[col] = np.nan
        self.data.to_csv(self.data_path, index=False)
        print(f"Values cleared for {'all subsystems' if not subsystem else subsystem}")

    def add_subsystem(self, subsystem: str, data: dict):
        '''Function to add a new subsystem to the data'''
        if subsystem not in self.data.columns:
            self.data[subsystem] = np.nan
        for prop, value in data.items():
            if prop not in self.data["Property"].values:
                self.data = pd.concat([self.data, pd.DataFrame({"Property": [prop]})], ignore_index=True)
            self.data.loc[self.data["Property"] == prop, subsystem] = value
        self.data.to_csv(self.data_path, index=False)

    def add_subsystem_property(self, subsystem: str, property: str, value):
        '''Function to add a new property to a subsystem'''
        if property not in self.data["Property"].values:
            self.data = pd.concat([self.data, pd.DataFrame({"Property": [property]})], ignore_index=True)
        if subsystem not in self.data.columns:
            self.data[subsystem] = np.nan
        self.data.loc[self.data["Property"] == property, subsystem] = value
        self.data.to_csv(self.data_path, index=False)

    def get_subsystem(self, subsystem: str):
        '''Function to get the data of a subsystem'''
        if subsystem in self.data.columns:
            return self.data[["Property", subsystem]].dropna()
        else:
            raise ValueError(f"Subsystem {subsystem} does not exist.")

    def get_subsystem_property(self, subsystem: str, property: str):
        '''Function to get a property of a subsystem'''
        if property in self.data["Property"].values:
            return self.data.loc[self.data["Property"] == property, subsystem].values[0]
        else:
            raise ValueError(f"Property {property} does not exist.")

    def get_all_subsystems(self):
        '''Function to get all the subsystems'''
        return self.data.columns[1:]

    def get_all_properties(self, subsystem: str):
        '''Function to get all the properties of a subsystem'''
        if subsystem in self.data.columns:
            return self.data[self.data[subsystem].notna()]["Property"].values
        else:
            raise ValueError(f"Subsystem {subsystem} does not exist.")
    
    def get_all_data(self):
        '''Function to get all the data'''
        return self.data
    
# Test the class
data = SE_data()
subsystems = ['EPS', 'ADCS', 'CDH', 'COM', 'GNC', 'PROP', 'STR', 'TCS']
properties = ['mass', 'power', 'volume', 'cost']
for subsystem in subsystems:
    data.add_subsystem(subsystem, {property: np.random.randint(1, 10) for property in properties})

print(data.get_all_data())

# Clear values for a single subsystem and for all subsystems
data.clear_values('EPS')
print(data.get_all_data())

data.clear_values()
print(data.get_all_data())
