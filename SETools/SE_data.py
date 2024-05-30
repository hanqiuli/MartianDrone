import numpy as np
import pandas as pd
import os
import shutil
from datetime import datetime

data_path = os.path.join('SETools/data', 'configuration_data.csv')
backup_path = os.path.join('SETools/data', 'backup')
test_data_path = os.path.join('SETools/data', 'test_configuration_data.csv')
test_backup_path = os.path.join('SETools/data', 'test_backup')

class SEData:
    '''Class to store and retrieve the design data for all the subsystems'''
    def __init__(self, data_path: str, backup_path: str):
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
            if subsystem.upper() in self.data.columns:
                self.data[subsystem.upper()] = np.nan
            else:
                raise ValueError(f"Subsystem {subsystem} does not exist.")
        else:
            for col in self.data.columns[1:]:
                self.data[col] = np.nan
        self.data.to_csv(self.data_path, index=False)
        print(f"Values cleared for {'all subsystems' if not subsystem else subsystem}")

    def add_subsystem(self, subsystem: str, data: dict):
        '''Function to add a new subsystem to the data'''
        if subsystem.upper() not in self.data.columns:
            self.data[subsystem.upper()] = np.nan
        for prop, value in data.items():
            if prop.upper() not in self.data["Property"].values:
                self.data = pd.concat([self.data, pd.DataFrame({"Property": [prop.upper()]})], ignore_index=True)
            self.data.loc[self.data["Property"] == prop.upper(), subsystem.upper()] = value
        self.data.to_csv(self.data_path, index=False)

    def add_subsystem_property(self, subsystem: str, property: str, value):
        '''Function to add a new property to a subsystem'''
        if property.upper() not in self.data["Property"].values:
            self.data = pd.concat([self.data, pd.DataFrame({"Property": [property.upper()]})], ignore_index=True)
        if subsystem.upper() not in self.data.columns:
            self.data[subsystem.upper()] = np.nan
        self.data.loc[self.data["Property"] == property.upper(), subsystem.upper()] = value
        self.data.to_csv(self.data_path, index=False)

    def remove_subsystem(self, subsystem: str):
        '''Function to remove a subsystem from the data'''
        if subsystem.upper() in self.data.columns:
            self.data.drop(subsystem.upper(), axis=1, inplace=True)
            self.data.to_csv(self.data_path, index=False)
        else:
            raise ValueError(f"Subsystem {subsystem} does not exist.")
        
    def remove_subsystem_property(self, subsystem: str, property: str):
        '''Function to remove a property from a subsystem'''
        if property.upper() in self.data["Property"].values:
            self.data.loc[self.data["Property"] == property.upper(), subsystem.upper()] = np.nan
            self.data.to_csv(self.data_path, index=False)
        else:
            raise ValueError(f"Property {property} does not exist.")
    
    def update_subsystem_property(self, subsystem: str, property: str, value):
        '''Function to update the value of a property in a subsystem'''
        if subsystem.upper() in self.data.columns:
            if property.upper() in self.data["Property"].values:
                self.data.loc[self.data["Property"] == property.upper(), subsystem.upper()] = value
                self.data.to_csv(self.data_path, index=False)
            else:
                raise ValueError(f"Property {property} does not exist.")
        else:
            raise ValueError(f"Subsystem {subsystem} does not exist.")

    def get_subsystem(self, subsystem: str):
        '''Function to get the data of a subsystem'''
        if subsystem.upper() in self.data.columns:
            return self.data[["Property", subsystem.upper()]].dropna()
        else:
            raise ValueError(f"Subsystem {subsystem} does not exist.")

    def get_subsystem_property(self, subsystem: str, property: str):
        '''Function to get a property of a subsystem'''
        if property.upper() in self.data["Property"].values:
            return self.data.loc[self.data["Property"] == property.upper(), subsystem.upper()].values[0]
        else:
            raise ValueError(f"Property {property} does not exist.")

    def get_all_properties(self, subsystem: str):
        '''Function to get all the properties of a subsystem (headers)'''
        if subsystem.upper() in self.data.columns:
            return self.data[self.data[subsystem.upper()].notna()]["Property"].values
        else:
            raise ValueError(f"Subsystem {subsystem} does not exist.")
    
    def get_all_data(self):
        '''Function to get all the data'''
        return self.data
    


if __name__ == '__main__':
    data = SEData(data_path, backup_path)
    subsystems = ["PWR", "STR", "PRO", "AVI", "COM", "PLD", "THE"]
    properties = ["mass", "power", "volume", "cost" ]

    for subsystem in subsystems:
        data.add_subsystem(subsystem, {prop: np.nan for prop in properties})
    