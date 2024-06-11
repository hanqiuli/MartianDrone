import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import RegularGridInterpolator

class DataGathering:
    def __init__(self, file_name):
        self.file_name = file_name
        self.property, self.time_axis, self.days = self.get_data()

    def get_data(self):
        '''Extract the property over the day from the datafile'''
        with open(self.file_name, 'r', encoding="utf-8") as file:
            lines = file.readlines()

        # Isolate time axis
        time_line = lines[11].strip()
        time_axis = time_line.split("||")[1].split()
        time_axis = [float(time) for time in time_axis]
        self.time_axis = time_axis

        # Extract data
        data_lines = lines[13:]
        days = []
        property = []

        for line in data_lines:
            parts = line.split("||")
            days.append(float(parts[0].strip()))
            property.append([float(x) for x in parts[1].split()])

        # Convert to numpy arrays
        days = np.array(days)
        self.days = days
        property = np.array(property)
        self.property = property
        
        self.interp_days = np.arange(min(self.days), max(self.days), 1)
        self.interp_time = np.arange(min(self.time_axis), max(self.time_axis), 1/12)
        print(repr(self.interp_time))
        return property, time_axis, days   
    
    def interpolate_data(self, new_days, new_time_axis):
        """Interpolates data at new points using the created interpolator.

        Args:
            new_days: Array of new days for which to interpolate.
            new_time_axis: Array of new time points for which to interpolate.

        Returns:
            Interpolated property values at the new points.
        """
        interp = RegularGridInterpolator((self.days, self.time_axis), self.property, method='cubic')

        # Create the grid of points where we want to interpolate
        mesh_new_days, mesh_new_time_axis = np.meshgrid(new_days, new_time_axis, indexing='ij')
        points = np.array([mesh_new_days.ravel(), mesh_new_time_axis.ravel()]).T

        # Perform interpolation
        interpolated_values = interp(points).reshape(len(new_days), len(new_time_axis))
        return interpolated_values

    def plot_data(self, title='Data Plot', xlabel='X-axis', ylabel='Y-axis', cmap='seismic'):
        '''Plot the given data'''
        
        plt.contourf(self.property.T, cmap=cmap, levels=20)
        plt.yticks(range(len(self.time_axis)), self.time_axis, size='small')
        plt.xticks(range(len(self.days)), self.days, size='small', rotation=90)
        plt.colorbar()
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.title(title)
        plt.show()

    def find_extreme_day(self, extreme='max'):
        """Finds the day with extreme average temperature

        Returns:
            tuple: (extreme_average_temperature, day_data, day_index)
        """
        # Calculate daily average temperature
        interpolated_data = self.interpolate_data(self.interp_days, self.interp_time)
        interpolated_data = np.maximum(interpolated_data,0)
        daily_average_property = np.mean(interpolated_data,axis=1)

        if extreme.lower() == 'max':
            # Find the day with maximum average temperature day
            day_index = np.argmax(daily_average_property)
        elif extreme.lower() == 'min':
            # Find the day with maximum average temperature day
            day_index = np.argmin(daily_average_property)
        else:
            raise ValueError("extreme should be either 'max' or 'min'")
        # Extract data for the day with minimum average temperature
        day_data = interpolated_data[day_index, :]
        extreme_average = daily_average_property[day_index]
        return extreme_average, day_data, self.interp_days[day_index]



# Example usage
dg = DataGathering("EnvironmentalPropertyTools/data/100m/temperature_100m.txt")  # Replace with your data file name

extreme = "max"
min_temp, day_data, day_index = dg.find_extreme_day(extreme)

print(f"Day with {extreme} average property: {day_index} deg")
print(f"Average temperature: {min_temp} K")
print(f"Data for the day: {repr(day_data)}")

temp, time, days = dg.get_data()
dg.plot_data(title='Temperature [K]', xlabel='Martian Day', ylabel='Martian Time [h]')