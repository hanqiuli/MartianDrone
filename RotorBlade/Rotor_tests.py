import unittest
import numpy as np
from blade import Blade


class TestBlade(unittest.TestCase):

    def setUp(self):
        # Sample input data for initializing a Blade object
        self.radius_rotor = 1.0
        self.num_blades = 5
        self.radial_nondim_stations = [0.05, 0.2, 0.5, 1.00]
        self.chord_nondim_stations = [0.12, 0.21, 0.14, 0.12]
        self.pitch_params = [0.24, 0.12, -0.12]
        self.airfoil_name_stations = ['Diamond', 'Triangle', 'DEP 0.5', 'DEP 0.7']
        self.small_angle_approx = True

        self.blade = Blade(
            radius_rotor=self.radius_rotor,
            num_blades=self.num_blades,
            radial_nondim_stations=self.radial_nondim_stations,
            chord_nondim_stations=self.chord_nondim_stations,
            pitch_params=self.pitch_params,
            airfoil_name_stations=self.airfoil_name_stations,
            small_angle_approx=self.small_angle_approx
        )

    def test_initialization(self):
        # Check if the properties are correctly set during initialization
        self.assertEqual(self.blade.radius_rotor, self.radius_rotor)
        self.assertEqual(self.blade.num_blades, self.num_blades)
        self.assertListEqual(self.blade.radial_nondim_stations, self.radial_nondim_stations)
        self.assertListEqual(self.blade.chord_nondim_stations, self.chord_nondim_stations)
        self.assertListEqual(self.blade.pitch_params, self.pitch_params)
        self.assertEqual(self.blade.small_angle_approx, self.small_angle_approx)


    def test_calculate_reynolds(self):
        # Sample input data for calculate_reynolds
        gamma_air = np.random.uniform(0,1,1)
        gas_constant_air = np.random.uniform(200,500,1)
        temp_air = np.random.uniform(100,500,1)
        density_air = np.random.uniform(0,0.04,1)
        viscosity_air = 1.81e-5
        mach_tip = np.random.uniform(0,1,1)

        a = np.sqrt(gamma_air*gas_constant_air*temp_air)

        self.blade.calculate_reynolds(gamma_air, gas_constant_air, temp_air, density_air, viscosity_air, mach_tip)
        
        self.assertTrue(hasattr(self.blade, 'speed_tip'))
        self.assertTrue(hasattr(self.blade, 'reynolds'))
        self.assertAlmostEqual(self.blade.speed_tip, mach_tip*a, places=5)

# Running the tests
if __name__ == '__main__':
    unittest.main()

