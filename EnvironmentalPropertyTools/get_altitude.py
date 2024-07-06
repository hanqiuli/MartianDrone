import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import RegularGridInterpolator
import scienceplots
plt.style.use('science')
plt.rcParams.update({'text.usetex': False})

from colormap import colormap
my_cmap = colormap()

class DataGathering:
    def __init__(self, file_name):
        self.file_name = file_name
        self.property, self.latitude, self.longitude = self.get_data()

    def get_data(self):
        '''Extract the property over the day from the datafile'''
        with open(self.file_name, 'r', encoding="utf-8") as file:
            lines = file.readlines()

        # Isolate time axis
        time_line = lines[11].strip()
        latitude = time_line.split("||")[1].split()
        latitude = [float(time) for time in latitude]
        self.latitude = latitude

        # Extract data
        data_lines = lines[13:]
        longitude = []
        property = []

        for line in data_lines:
            parts = line.split("||")
            longitude.append(float(parts[0].strip()))
            property.append([float(x) for x in parts[1].split()])

        # Convert to numpy arrays
        longitude = np.array(longitude)
        self.longitude = longitude
        property = np.array(property)
        self.property = property
        
        # self.interp_longitude = np.arange(min(self.longitude), max(self.longitude), 1)
        # self.interp_latitude = np.arange(min(self.latitude), max(self.latitude), 1/12)

        return property, latitude, longitude   
    
    def interpolate_data(self, new_longitude, new_latitude):
        """Interpolates data at new points using the created interpolator.

        Args:
            new_longitude: Array of new longitude for which to interpolate.
            new_latitude: Array of new time points for which to interpolate.

        Returns:
            Interpolated property values at the new points.
        """
        interp = RegularGridInterpolator((self.longitude, self.latitude), self.property, method='cubic')

        # Create the grid of points where we want to interpolate
        mesh_new_longitude, mesh_new_latitude = np.meshgrid(new_longitude, new_latitude, indexing='ij')
        points = np.array([mesh_new_longitude.ravel(), mesh_new_latitude.ravel()]).T

        # Perform interpolation
        interpolated_values = interp(points).reshape(len(new_longitude), len(new_latitude))
        return interpolated_values

    def plot_data(self, propertylabel='Property [?]', xlabel='X-axis', ylabel='Y-axis', cmap='coolwarm'):
        '''Plot the given data'''
        print(self.property)
        plt.figure(figsize=(5, 4))
        plt.contourf(self.property.T, cmap=my_cmap, levels=40)
        plt.yticks(np.linspace(0,len(latitude)-1,8), np.round(np.linspace(-24.5, -23.5, 8),1) ,size='small')
        plt.xticks(np.linspace(0,len(longitude)-1,8), np.round(np.linspace(326.1, 327.3, 8),1), size='small', rotation=90)
        plt.colorbar(label=propertylabel)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.tight_layout()
        plt.savefig('EnvironmentalPropertyTools/plots/'+data_file+'.pdf')
        # plt.show()

data_file = "altitude_abov_areoid"
# Data file
dg = DataGathering("EnvironmentalPropertyTools/data/"+data_file+'.txt')  # Replace with your data file name

property, latitude, longitude = dg.get_data()
print(property.shape, len(latitude), len(longitude))
dg.plot_data(propertylabel='Altitude above the areoid (Mars Geoid) [$m$]', xlabel='Longitude East [deg]', ylabel='Latitude North [deg]')