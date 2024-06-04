import numpy as np
import matplotlib.pyplot as plt

airfoils = {
    'Diamond': {
        'alpha': [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
        'cl': [0.00498, 0.01099, 0.017, 0.02165, 0.0263, 0.02955, 0.0328, 0.0353, 0.0378, 0.0365, 0.0352],
        'cd': [0.00273, 0.00283, 0.00292, 0.00321, 0.00351, 0.00402, 0.00453, 0.00545, 0.00638, 0.00746, 0.00855]},
    'Triangle': {
        'alpha': [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
        'cl': [0.09642, 0.16033, 0.22645, 0.28375, 0.34986, 0.39614, 0.44463, 0.50193, 0.59229, 0.79725, 0.84353],
        'cd': [0.05675, 0.05895, 0.05895, 0.06336, 0.06556, 0.06777, 0.07658, 0.08760, 0.09862, 0.14490, 0.17576]},
    'DEP 0.5': {
        'alpha': [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
        'cl': [0.24, 0.342, 0.452, 0.562, 0.691, 0.801, 0.925, 1.034, 1.192, 1.23, 1.309],
        'cd': [0.0317, 0.0329, 0.0345, 0.0363, 0.0388, 0.0445, 0.0638, 0.0834, 0.1243, 0.1411, 0.1806]},
    'DEP 0.7': {
        'alpha': [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
        'cl': [0.297, 0.444, 0.584, 0.724, 0.898, 1.058, 1.292, 1.352, 1.432, 1.192, 1.011],
        'cd': [0.0338, 0.0358, 0.0387, 0.0431, 0.0572, 0.0920, 0.1470, 0.1711, 0.1997, 0.1180, 0.0836]},
}

class Airfoil:
    def __init__(self, name: str):
        """Initializes:
            alpha, cl, cd: The angle of attack, lift coefficient, and drag coefficient of the airfoil. [deg, -, -]
        Args:
            name: The name of the airfoil. [-]
        """
        if name not in airfoils:
            raise ValueError("Airfoil name not found.")
        
        self.name = name
        self.alphas = airfoils[self.name]['alpha']
        self.cls = airfoils[self.name]['cl']
        self.cds = airfoils[self.name]['cd']

    def interpolate_cl_cd(self):
        """Interpolates the lift and drag coefficients of the airfoil at a given angle of attack.
        Args:
            alpha: The angle of attack. [deg]
        Returns:
            cl, cd: The lift and drag coefficients of the airfoil. [-, -]
        """
        self.alpha = np.linspace(self.alphas[0], self.alphas[-1], 1000)
        self.cl = np.interp(self.alpha, self.alphas, self.cls)
        self.cd = np.interp(self.alpha, self.alphas, self.cds)
    
if __name__ == '__main__':
    airfoil = Airfoil('Diamond')
    airfoil.interpolate_cl_cd()
    plt.plot(airfoil.alpha, airfoil.cl, label='Lift Coefficient')
    plt.xlabel('Angle of Attack [deg]')
    plt.ylabel('Lift Coefficient [-]')
    plt.legend()
    plt.minorticks_on()
    plt.grid(which='both')
    