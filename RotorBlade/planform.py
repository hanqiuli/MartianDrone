import numpy as np

class Planform:
    def __init__(self, radius_rotor, number_of_blades, radial_stations, chord_distribution):
        self.radius_rotor = radius_rotor
        self.number_of_blades = number_of_blades
        self.radial_stations = radial_stations
        self.chord_distribution = chord_distribution
        self.radial_position = np.linspace(radial_stations[0], radial_stations[1], 0.01)
        self.chord_distribution = self.get_chord_distribution()

    def get_chord_distribution(self):
        pass


