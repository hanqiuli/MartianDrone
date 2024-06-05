import numpy as np
import matplotlib.pyplot as plt

# airfoils = {
#     'Diamond 0.22': {
#         'alpha': [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
#         'cl': [0.00498, 0.01099, 0.017, 0.02165, 0.0263, 0.02955, 0.0328, 0.0353, 0.0378, 0.0365, 0.0352],
#         'cd': [0.00273, 0.00283, 0.00292, 0.00321, 0.00351, 0.00402, 0.00453, 0.00545, 0.00638, 0.00746, 0.00855]},
#     'Triangle': {
#         'alpha': [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
#         'cl': [0.09642, 0.16033, 0.22645, 0.28375, 0.34986, 0.39614, 0.44463, 0.50193, 0.59229, 0.79725, 0.84353],
#         'cd': [0.05675, 0.05895, 0.05895, 0.06336, 0.06556, 0.06777, 0.07658, 0.08760, 0.09862, 0.14490, 0.17576]},
#     'DEP 0.5': {
#         'alpha': [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
#         'cl': [0.24, 0.342, 0.452, 0.562, 0.691, 0.801, 0.925, 1.034, 1.192, 1.23, 1.309],
#         'cd': [0.0317, 0.0329, 0.0345, 0.0363, 0.0388, 0.0445, 0.0638, 0.0834, 0.1243, 0.1411, 0.1806]},
#     'DEP 0.7': {
#         'alpha': [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
#         'cl': [0.297, 0.444, 0.584, 0.724, 0.898, 1.058, 1.292, 1.352, 1.432, 1.192, 1.011],
#         'cd': [0.0338, 0.0358, 0.0387, 0.0431, 0.0572, 0.0920, 0.1470, 0.1711, 0.1997, 0.1180, 0.0836]},
#     'CLF5605 0.5': {
#         'alpha': [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
#         'cl': [0.46738794435857806, 0.5972179289026275, 0.7162287480680062, 0.8309119010819166, 0.9520865533230294, 1.0667697063369397, 1.1857805255023184, 1.246367851622875, 1.2009273570324575, 1.1554868624420402, 1.1360123647604328],
#         'cd': [0.03693623639191291, 0.03926905132192846, 0.041601866251944015, 0.04471228615863142, 0.04782270606531882, 0.05248833592534992, 0.05870917573872473, 0.07542768273716952, 0.10847589424572317, 0.13724727838258166, 0.16368584758942456]}, 
#     'CLF5605 0.7': {
#         'alpha': [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
#         'cl': [0.5863987635239567, 0.7508500772797527, 0.9023183925811438, 1.0624420401854715, 1.1403400309119012, 1.1684698608964452, 1.1836166924265843, 1.1901081916537868, 1.1965996908809893, 1.2030911901081915, 1.209582689335394],
#         'cd': [0.038102643856920686, 0.04082426127527216, 0.04432348367029549, 0.05171073094867807, 0.06531881804043546, 0.0828149300155521, 0.10186625194401244, 0.12363919129082426, 0.14502332814930016, 0.16796267496111975, 0.19129082426127528]}, 
#     'NACA6904': {
#         'alpha': [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
#         'cl': [0.297, 0.444, 0.584, 0.724, 0.898, 1.058, 1.292, 1.352, 1.432, 1.192, 1.011],
#         'cd': [0.0338, 0.0358, 0.0387, 0.0431, 0.0572, 0.0920, 0.1470, 0.1711, 0.1997, 0.1180, 0.0836]},
#     'NACA0096': {
#         'alpha': [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
#         'cl': [0.297, 0.444, 0.584, 0.724, 0.898, 1.058, 1.292, 1.352, 1.432, 1.192, 1.011],
#         'cd': [0.0338, 0.0358, 0.0387, 0.0431, 0.0572, 0.0920, 0.1470, 0.1711, 0.1997, 0.1180, 0.0836]},
#     'CP 0.7': {
#         'alpha': [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
#         'cl': [0.577710843373494, 0.7231927710843373, 0.858132530120482, 0.9930722891566265, 1.1385542168674698, 1.1954819277108435, 1.1343373493975903, 1.0816265060240964, 1.1153614457831325, 1.1427710843373493, 1.155421686746988],
#         'cd': [0.036891679748822605, 0.04081632653061225, 0.046310832025117744, 0.05533751962323391, 0.06554160125588697, 0.09576138147566719, 0.10949764521193094, 0.12284144427001571, 0.14756671899529042, 0.17111459968602827, 0.19505494505494506]}
# }

airfoils = {
    'Diamond': {
        'lift_slope': 0.235056,
        'alpha_max': 8.0,
        'delta_0': 0.0028,
        'delta_1': -0.00472937,
        'delta_2': 0.214743
    },
    'Triangle': {
        'lift_slope': 3.551392,
        'alpha_max': 8.0,
        'delta_0': 0.058512,
        'delta_1': -0.0929,
        'delta_2': 2.67
    },
    'DEP 0.5': {
        'lift_slope': 6.818198,
        'alpha_max': 8.0,
        'delta_0': 0.037876,
        'delta_1': -0.551383,
        'delta_2': 7.976333
    },
    'DEP 0.7': {
        'lift_slope': 8.128839,
        'alpha_max': 8.0,
        'delta_0': 0.032837,
        'delta_1': -0.2434046,
        'delta_2': 10.8753
    },
    'CLF5605 0.5': {
        'lift_slope': 5.253589057,
        'alpha_max': 8.0,
        'delta_0': 0.0387,
        'delta_1': -0.0587,
        'delta_2': 2.7275
    },
    'CLF5605 0.7': {
        'lift_slope': 4.370242224,
        'alpha_max': 8.0,
        'delta_0': 0.0354,
        'delta_1': 0.1677,
        'delta_2': 4.2828
    }
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
        self.lift_slope = airfoils[self.name]['lift_slope']
        self.alpha_max = airfoils[self.name]['alpha_max']
        self.delta_0 = airfoils[self.name]['delta_0']
        self.delta_1 = airfoils[self.name]['delta_1']
        self.delta_2 = airfoils[self.name]['delta_2']

# class Airfoil:
#     def __init__(self, name: str):
#         """Initializes:
#             alpha, cl, cd: The angle of attack, lift coefficient, and drag coefficient of the airfoil. [deg, -, -]
#         Args:
#             name: The name of the airfoil. [-]
#         """
#         if name not in airfoils:
#             raise ValueError("Airfoil name not found.")
        
#         self.name = name
#         self.alphas = airfoils[self.name]['alpha']
#         self.cls = airfoils[self.name]['cl']
#         self.cds = airfoils[self.name]['cd']

#     def interpolate_cl_cd(self):
#         """Interpolates the lift and drag coefficients of the airfoil at a given angle of attack.
#         Args:
#             alpha: The angle of attack. [deg]
#         Returns:
#             cl, cd: The lift and drag coefficients of the airfoil. [-, -]
#         """
#         self.alpha = np.linspace(self.alphas[0], self.alphas[-1], 100)
        
#         self.cl = np.interp(self.alpha, self.alphas, self.cls)
#         self.cd = np.interp(self.alpha, self.alphas, self.cds)
    
#     def calculate_lift_slope(self):
#         """Calculates the lift slope of the airfoil.
#         Returns:
#             lift_slope: The lift slope of the airfoil. [1/deg]
#         """
#         self.lift_slope = np.gradient(self.cl, self.alpha)
    
# if __name__ == '__main__':
#     airfoillist = [Airfoil(name) for name in airfoils.keys()]
#     for airfoil in airfoillist:
#         airfoil.interpolate_cl_cd()
#         airfoil.calculate_lift_slope()
#         plt.plot(airfoil.alpha, airfoil.cl, label=f'$c_l$ {airfoil.name}')
#         # plt.plot(airfoil.alpha, airfoil.cd, label='Drag Coefficient '+airfoil.name)
#     plt.xlabel('Angle of Attack [deg]')
#     plt.ylabel('Coefficient [-]')
#     plt.legend()
#     plt.minorticks_on()
#     plt.grid(which='both')
#     plt.show()
#     print(airfoil.cl)