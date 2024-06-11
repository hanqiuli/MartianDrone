import numpy as np
from sympy.solvers import solve
from sympy import Symbol
from scipy.integrate import odeint
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


# doi:10.1088/1757-899X/1226/1/012113

heat_balance_cold = {
    'temperature_atmosphere': 210,  # [K] - Atmospheric temperature
    'irradiance_sun': 0,  # [W/m^2] - Solar irradiance
    'absorptivity': 0.2,  # [-] - Absorptance of the battery
    'f_sr': 1,  # [-] - View factor for solar radiation
    'area_top': 0.0168*0.0126,  # [m^2] - Top surface area of the battery

    'albedo': 0.4,  # [-] - Albedo of Mars

    'temperature_effective_mars': 209.8,  # [K] - Effective temperature for radiation of black body of Mars
    'emissivity_mars': 0.65,  # [-]-  Emissivity of Mars
    'stefan_boltzmann_constant': 5.6704e-8,  # [W/m^2*K^4] - Stefan-Boltzmann constant

    # battery properties
    'heat_rate_internal': {'electronics': 10},  # [W] - Heat from internal sources

    'coefficient_convection': 1,  # [W/m^2*K] - Convection coefficient
    'area_total': (0.0168*12.6*2+0.021*0.0126),  # [m^2] - Total surface area of the battery
    'f_re': 1,  # [-] - View factor for emitted radiation
    'emissivity_battery': 0.01,  # [-] - Emissivity of the battery
    'battery_mass': 9.5,  # [kg] - Mass of the battery
    'battery_heat_capacity': 1100,  # [J/kg*K] - Specific heat capacity of the battery

    # Insulator properties
    'thermal_conductivity': 0.035,  # [W/m*K] - Thermal conductivity of the insulator (Cork)
    'thickness_insulator': 0.05,  # [m] - Thickness of the insulator
    'thermal_conductivity_air': 0.0209,  # [W/m*K] - Thermal conductivity of the insulator (Cork)
    'thickness_insulator_air': 0.03,  # [m] - Thickness of the insulator
}

heat_balance_hot = {
    'temperature_atmosphere': np.array([225.07991434, 224.86182942, 224.64078064, 224.41711538,
       224.19118099, 223.96332486, 223.73389433, 223.50323679,
       223.2716996 , 223.03963013, 222.80737574, 222.5752838 ,
       222.34370168, 222.11297675, 221.88345637, 221.65548792,
       221.42941875, 221.20559624, 220.98436775, 220.76608065,
       220.55108232, 220.3397201 , 220.13234138, 219.92929352,
       219.73092389, 219.53743365, 219.34843913, 219.16341044,
       218.98181772, 218.80313109, 218.62682067, 218.45235658,
       218.27920894, 218.10684789, 217.93474353, 217.762366  ,
       217.58918542, 217.41483628, 217.23961051, 217.06396446,
       216.88835441, 216.7132367 , 216.53906764, 216.36630354,
       216.19540072, 216.02681549, 215.86100417, 215.69842307,
       215.53952852, 215.384634  , 215.23348176, 215.08567122,
       214.94080179, 214.7984729 , 214.65828397, 214.51983441,
       214.38272366, 214.24655112, 214.11091622, 213.97541838,
       213.83965702, 213.70400153, 213.57190121, 213.44757534,
       213.33524318, 213.23912401, 213.16343709, 213.1124017 ,
       213.09023711, 213.1011626 , 213.14939742, 213.23916086,
       213.37467219, 213.55858584, 213.78729698, 214.05563592,
       214.35843298, 214.6905185 , 215.04672279, 215.42187617,
       215.81080897, 216.20835151, 216.60933412, 217.00858711,
       217.40094082, 217.78285689, 218.15732235, 218.52895554,
       218.9023748 , 219.28219849, 219.67304495, 220.07953252,
       220.50627957, 220.95790443, 221.43902546, 221.954261  ,
       222.50822939, 223.10401527, 223.73856834, 224.4073046 ,
       225.10564004, 225.82899065, 226.57277242, 227.33240134,
       228.1032934 , 228.8808646 , 229.66053092, 230.43770836,
       231.2078129 , 231.96704251, 232.71472305, 233.45096232,
       234.17586817, 234.88954841, 235.59211087, 236.28366336,
       236.96431372, 237.63416978, 238.29333934, 238.94193024,
       239.5800503 , 240.2079704 , 240.82661369, 241.43706635,
       242.04041459, 242.63774459, 243.23014255, 243.81869468,
       244.40448715, 244.98860618, 245.57213795, 246.15616867,
       246.74178452, 247.32965655, 247.91879519, 248.50779572,
       249.09525342, 249.67976357, 250.25992143, 250.8343223 ,
       251.40156146, 251.96023416, 252.50893571, 253.04626137,
       253.57080643, 254.0816484 , 254.5797938 , 255.0667314 ,
       255.54394994, 256.01293819, 256.4751849 , 256.93217884,
       257.38540877, 257.83636343, 258.28653161, 258.73740204,
       259.19046349, 259.64657734, 260.10409545, 260.56074229,
       261.01424235, 261.4623201 , 261.90270002, 262.33310659,
       262.75126429, 263.15489759, 263.54173097, 263.90948891,
       264.25589589, 264.57946308, 264.88184837, 265.16549635,
       265.43285163, 265.68635877, 265.92846239, 266.16160705,
       266.38823736, 266.6107979 , 266.83173326, 267.05348804,
       267.27850681, 267.50805786, 267.73870417, 267.96583241,
       268.18482926, 268.39108139, 268.57997547, 268.74689817,
       268.88723616, 268.99637613, 269.06970473, 269.10260864,
       269.09047453, 269.03033463, 268.92580343, 268.78214094,
       268.60460721, 268.39846226, 268.16896613, 267.92137885,
       267.66096046, 267.39297098, 267.12267045, 266.8553189 ,
       266.59617636, 266.34857415, 266.10812871, 265.86852773,
       265.62345894, 265.36661005, 265.09166877, 264.79232282,
       264.4622599 , 264.09516773, 263.68473402, 263.2246465 ,
       262.70859285, 262.13243349, 261.50071951, 260.82017469,
       260.09752282, 259.33948766, 258.55279301, 257.74416264,
       256.92032032, 256.08798986, 255.25389501, 254.42475956,
       253.6073073 , 252.80713587, 252.0253384 , 251.2618819 ,
       250.51673337, 249.78985981, 249.08122822, 248.39080561,
       247.71855898, 247.06445533, 246.42846166, 245.81054498,
       245.21067229, 244.62850789, 244.06250532, 243.51081541,
       242.971589  , 242.44297693, 241.92313002, 241.41019912,
       240.90233506, 240.39768867, 239.8944108 , 239.39065227,
       238.88456392, 238.37492021, 237.86299006, 237.35066601,
       236.83984058, 236.33240633, 235.83025579, 235.3352815 ,
       234.84937599, 234.37443181, 233.91234149, 233.46499758,
       233.0342926 , 232.62157442, 232.22601214, 231.84623018,
       231.48085297, 231.12850492, 230.78781046, 230.457394  ,
       230.13587998, 229.82189281, 229.51405692, 229.21099672,
       228.91133663, 228.61370109, 228.31671451, 228.01900131,
       227.71918592, 227.41589275, 227.10774623, 226.79337078,
       226.47139083, 226.14043078, 225.79911507, 225.44606812]),  # [K] - Atmospheric temperature
    'irradiance_sun': np.array([0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
       0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
       0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
       1.81641605e-17, 2.26016941e-02, 4.42550653e-02, 6.40117909e-02,
       8.09235480e-02, 9.40420138e-02, 1.02418865e-01, 1.05105780e-01,
       1.01154435e-01, 8.96165073e-02, 6.95436741e-02, 3.99876126e-02,
       6.07534012e-18, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
       0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
       0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
       9.02217163e-17, 1.79074961e-01, 3.85651283e-01, 6.01710834e-01,
       8.09235480e-01, 9.90207086e-01, 1.12660752e+00, 1.20041865e+00,
       1.19362233e+00, 1.08820045e+00, 8.66134850e-01, 5.09407413e-01,
       3.15094488e-17, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
       0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
       0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
       0.00000000e+00, 2.52576441e+00, 5.68539610e+00, 9.47548943e+00,
       1.38926387e+01, 1.89334384e+01, 2.45944827e+01, 3.08723660e+01,
       3.77636828e+01, 4.52650272e+01, 5.33729937e+01, 6.20841766e+01,
       7.13951703e+01, 8.12916863e+01, 9.17159052e+01, 1.02599125e+02,
       1.13872643e+02, 1.25467757e+02, 1.37315765e+02, 1.49347964e+02,
       1.61495653e+02, 1.73690130e+02, 1.85862691e+02, 1.97944635e+02,
       2.09867259e+02, 2.21577056e+02, 2.33081293e+02, 2.44402432e+02,
       2.55562936e+02, 2.66585266e+02, 2.77491885e+02, 2.88305254e+02,
       2.99047836e+02, 3.09742093e+02, 3.20410487e+02, 3.31075479e+02,
       3.41759532e+02, 3.52477707e+02, 3.63215458e+02, 3.73950839e+02,
       3.84661905e+02, 3.95326708e+02, 4.05923302e+02, 4.16429741e+02,
       4.26824078e+02, 4.37084368e+02, 4.47188663e+02, 4.57115018e+02,
       4.66841486e+02, 4.76349022e+02, 4.85630182e+02, 4.94680426e+02,
       5.03495212e+02, 5.12069998e+02, 5.20400241e+02, 5.28481402e+02,
       5.36308937e+02, 5.43878306e+02, 5.51184966e+02, 5.58224375e+02,
       5.64991993e+02, 5.71484626e+02, 5.77704472e+02, 5.83655081e+02,
       5.89340000e+02, 5.94762776e+02, 5.99926957e+02, 6.04836091e+02,
       6.09493726e+02, 6.13903410e+02, 6.18068691e+02, 6.21993115e+02,
       6.25680232e+02, 6.29131283e+02, 6.32338290e+02, 6.35290970e+02,
       6.37979039e+02, 6.40392212e+02, 6.42520208e+02, 6.44352742e+02,
       6.45879529e+02, 6.47090288e+02, 6.47974734e+02, 6.48522583e+02,
       6.48723552e+02, 6.48570986e+02, 6.48072747e+02, 6.47240326e+02,
       6.46085214e+02, 6.44618902e+02, 6.42852881e+02, 6.40798641e+02,
       6.38467673e+02, 6.35871469e+02, 6.33021520e+02, 6.29929315e+02,
       6.26606347e+02, 6.23060855e+02, 6.19288077e+02, 6.15280002e+02,
       6.11028616e+02, 6.06525906e+02, 6.01763861e+02, 5.96734468e+02,
       5.91429714e+02, 5.85841587e+02, 5.79962074e+02, 5.73783162e+02,
       5.67296839e+02, 5.60498819e+02, 5.53399720e+02, 5.46013884e+02,
       5.38355656e+02, 5.30439379e+02, 5.22279398e+02, 5.13890056e+02,
       5.05285696e+02, 4.96480663e+02, 4.87489299e+02, 4.78325950e+02,
       4.69004958e+02, 4.59538582e+02, 4.49930731e+02, 4.40183231e+02,
       4.30297906e+02, 4.20276581e+02, 4.10121081e+02, 3.99833230e+02,
       3.89414852e+02, 3.78867774e+02, 3.68193818e+02, 3.57394810e+02,
       3.46472575e+02, 3.35429671e+02, 3.24271596e+02, 3.13004583e+02,
       3.01634862e+02, 2.90168665e+02, 2.78612226e+02, 2.66971775e+02,
       2.55253544e+02, 2.43463766e+02, 2.31608672e+02, 2.19694494e+02,
       2.07727464e+02, 1.95720093e+02, 1.83710009e+02, 1.71741116e+02,
       1.59857322e+02, 1.48102533e+02, 1.36520655e+02, 1.25155594e+02,
       1.14051256e+02, 1.03251548e+02, 9.28003746e+01, 8.27416434e+01,
       7.31192600e+01, 6.39726935e+01, 5.53236635e+01, 4.71894524e+01,
       3.95873426e+01, 3.25346165e+01, 2.60485566e+01, 2.01464451e+01,
       1.48455646e+01, 1.01631975e+01, 6.11662603e+00, 2.72313274e+00,
       0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
       0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
       0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
       0.00000000e+00, 5.49670943e-01, 9.34594094e-01, 1.17421174e+00,
       1.28796617e+00, 1.29529966e+00, 1.21565451e+00, 1.06847299e+00,
       8.73197402e-01, 6.49270021e-01, 4.16133137e-01, 1.93229034e-01,
       1.74872288e-16, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
       0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
       0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
       9.24007043e-17, 4.31482310e-02, 7.50404017e-02, 9.66997904e-02,
       1.09149675e-01, 1.13413334e-01, 1.10514046e-01, 1.01475089e-01,
       8.73197402e-02, 6.90712789e-02, 4.77529829e-02, 2.43881306e-02,
       0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
       0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
       0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00]),  # [W/m^2] - Solar irradiance
    'absorptivity': 0.2,  # [-] - Absorptance of the battery
    'f_sr': 1,  # [-] - View factor for solar radiation
    'area_top': 0.0168*0.0126,  # [m^2] - Top surface area of the battery

    'albedo': 0.4,  # [-] - Albedo of Mars

    'temperature_effective_mars': 209.8,  # [K] - Effective temperature for radiation of black body of Mars
    'emissivity_mars': 0.65,  # [-]-  Emissivity of Mars
    'stefan_boltzmann_constant': 5.6704e-8,  # [W/m^2*K^4] - Stefan-Boltzmann constant

        # battery properties
    'heat_rate_internal': {'heater': 50},  # [W] - Heat from internal sources

    'coefficient_convection': 1,  # [W/m^2*K] - Convection coefficient
    'area_total': (0.0168*12.6*2+0.021*0.0126),  # [m^2] - Total surface area of the battery
    'f_re': 1,  # [-] - View factor for emitted radiation
    'emissivity_battery': 0.85,  # [-] - Emissivity of the battery
    'battery_mass': 9.5,  # [kg] - Mass of the battery
    'battery_heat_capacity': 1100,  # [J/kg*K] - Heat capacity of the battery

    # Insulator properties
    'thermal_conductivity': 0.035,  # [W/m*K] - Thermal conductivity of the insulator (Cork)
    'thickness_insulator': 0.05,  # [m] - Thickness of the insulator
    'thermal_conductivity_air': 0.0209,  # [W/m*K] - Thermal conductivity of the insulator (Cork)
    'thickness_insulator_air': 0.03,  # [m] - Thickness of the insulator
}

class BatteryHeatTransfer:
    """
    This class models the heat transfer of the battery of the Martian battery.
    """

    def __init__(self):
        # Environmental properties
        heat_dictionary = heat_balance_hot
        self.temperature_atmosphere = heat_dictionary['temperature_atmosphere']  # [K] - Atmospheric temperature
        self.irradiance_sun = heat_dictionary['irradiance_sun']  # [W/m^2] - Solar irradiance
        self.absorptivity = heat_dictionary['absorptivity']  # [-] - Absorptance of the battery
        self.f_sr = heat_dictionary['f_sr']  # [-] - View factor for solar radiation
        self.area_top = heat_dictionary['area_top']  # [m^2] - Top surface area of the battery
        
        self.albedo = heat_dictionary['albedo']  # [-] - Albedo of Mars
        
        self.temperature_effective_mars = heat_dictionary['temperature_effective_mars']  # [K] - Effective temperature for radiation of black body of Mars
        self.emissivity_mars = heat_dictionary['emissivity_mars']  # [-]-  Emissivity of Mars
        self.stefan_boltzmann_constant = heat_dictionary['stefan_boltzmann_constant']  # [W/m^2*K^4] - Stefan-Boltzmann constant

        # battery properties
        self.heat_rate_internal = sum(heat_dictionary['heat_rate_internal'].values())  # [W] - Total heat from internal sources
        
        self.coefficient_convection = heat_dictionary['coefficient_convection']  # [W/m^2*K] - Convection coefficient
        self.area_total = heat_dictionary['area_total']  # [m^2] - Total surface area of the battery
        self.f_re = heat_dictionary['f_re']  # [-] - View factor for emitted radiation
        self.emissivity_battery = heat_dictionary['emissivity_battery']  # [-] - Emissivity of the battery
        self.battery_mass = heat_dictionary['battery_mass']  # [kg] - Mass of the battery
        self.battery_heat_capacity = heat_dictionary['battery_heat_capacity']  # [J/kg*K] - Heat capacity of the battery

        # Insulator properties
        self.thermal_conductivity_ins = heat_dictionary['thermal_conductivity']  # [W/m*K] - Thermal conductivity of the insulator (Cork)
        self.thickness_insulator_ins = heat_dictionary['thickness_insulator']  # [m] - Thickness of the insulator
        self.thermal_conductivity_air = heat_dictionary['thermal_conductivity_air']  # [W/m*K] - Thermal conductivity of the insulator (Cork)
        self.thickness_insulator_air = heat_dictionary['thickness_insulator_air']  # [m] - Thickness of the insulator

    
    def heat_rate_conduction(self, temperature_battery):
        """
        Calculates the heat input due to conduction from the ground to the battery.

        Args:
            temperature_battery: The current temperature of the battery [K].

        Returns:
            The heat input due to conduction [W].
        """
        thickness_insulator = self.thickness_insulator_ins+self.thickness_insulator_air
        thermal_insulator = thickness_insulator/(self.thickness_insulator_ins/self.thermal_conductivity_ins + self.thickness_insulator_air/self.thermal_conductivity_air)
        heat_rate_cond_ins = - thermal_insulator * self.area_total / thickness_insulator * (temperature_battery - self.temperature_atmosphere) 
        return heat_rate_cond_ins
    
    def heat_rate_external_input(self, temperature_battery):
        """
        Calculates the total heat input from external sources.

        Args:
            temperature_battery: The current temperature of the battery [K].

        Returns:
            A tuple containing the total heat input [W] and individual components [W].
        """

        heat_rate_sun = self.absorptivity * self.irradiance_sun * self.area_top * self.f_sr
        heat_rate_albedo = self.albedo * self.absorptivity * self.irradiance_sun * self.area_top * self.f_sr
        j_p = self.emissivity_mars * self.stefan_boltzmann_constant * self.temperature_effective_mars**4
        heat_rate_ir = j_p * self.area_top
        heat_rate_cond = self.heat_rate_conduction(temperature_battery)
        total_heat_rate_ext = heat_rate_sun + heat_rate_albedo + heat_rate_ir + heat_rate_cond
        return total_heat_rate_ext, (heat_rate_sun, heat_rate_albedo, heat_rate_ir, heat_rate_cond)

    def heat_rate_internal_input(self):
        """
        Calculates the total heat input from internal sources.

        Returns:
            The total heat input from electronics and motor [W].
        """

        return self.heat_rate_internal

    def heat_rate_convection(self, temperature_battery):
        """
        Calculates the heat loss due to convection with the atmosphere.

        Args:
            temperature_battery: The current temperature of the battery [K].

        Returns:
            The heat loss due to convection [W].
        """

        heat_rate_conv = self.coefficient_convection * (temperature_battery - self.temperature_atmosphere) * self.area_total
        return heat_rate_conv

    def heat_rate_out(self, temperature_battery):
        """
        Calculates the heat loss due to radiation to the environment.

        Args:
            temperature_battery: The current temperature of the battery [K].

        Returns:
            The heat loss due to radiation [W].
        """

        heat_rate_out = self.stefan_boltzmann_constant * self.emissivity_battery * (temperature_battery**4 - self.temperature_atmosphere**4) * self.area_total
        return heat_rate_out 

    def heat_rate_balance(self, temperature_battery):
        """
        Calculates the balance equation to solve for the battery's temperature.

        Args:
            temperature_battery: The current temperature of the battery [K].

        Returns:
            The value of the balance equation [W] (should be zero at equilibrium).
        """

        heat_rate_ext, _ = self.heat_rate_external_input(temperature_battery)
        heat_rate_int = self.heat_rate_internal_input()
        heat_rate_conv = self.heat_rate_convection(temperature_battery)
        heat_rate_out = self.heat_rate_out(temperature_battery)
        return heat_rate_ext + heat_rate_int - heat_rate_conv - heat_rate_out

    def temperature_time_derivative(self, t, temperature_battery, t_list):
        """
        Calculates the derivative of the temperature of the battery over time.

        Args:
            t: The current time [s].
            temperature_battery: The current temperature of the battery [K].

        Returns:
            The derivative of the temperature of the battery over time [K/s].
        """
        index = np.argmin(np.abs(t_list-t))
        temperature_derivative = (1/(self.battery_mass*self.battery_heat_capacity)*self.heat_rate_balance(temperature_battery))
        return temperature_derivative[index]

    def solve_temperature_time(self):
        """
        Solves the heat balance equation to find the temperature of the battery over time.

        Returns:
            The temperature of the battery over time [K] [s].
        """
        t = np.array([ 0.        ,  0.08333333,  0.16666667,  0.25      ,  0.33333333,
        0.41666667,  0.5       ,  0.58333333,  0.66666667,  0.75      ,
        0.83333333,  0.91666667,  1.        ,  1.08333333,  1.16666667,
        1.25      ,  1.33333333,  1.41666667,  1.5       ,  1.58333333,
        1.66666667,  1.75      ,  1.83333333,  1.91666667,  2.        ,
        2.08333333,  2.16666667,  2.25      ,  2.33333333,  2.41666667,
        2.5       ,  2.58333333,  2.66666667,  2.75      ,  2.83333333,
        2.91666667,  3.        ,  3.08333333,  3.16666667,  3.25      ,
        3.33333333,  3.41666667,  3.5       ,  3.58333333,  3.66666667,
        3.75      ,  3.83333333,  3.91666667,  4.        ,  4.08333333,
        4.16666667,  4.25      ,  4.33333333,  4.41666667,  4.5       ,
        4.58333333,  4.66666667,  4.75      ,  4.83333333,  4.91666667,
        5.        ,  5.08333333,  5.16666667,  5.25      ,  5.33333333,
        5.41666667,  5.5       ,  5.58333333,  5.66666667,  5.75      ,
        5.83333333,  5.91666667,  6.        ,  6.08333333,  6.16666667,
        6.25      ,  6.33333333,  6.41666667,  6.5       ,  6.58333333,
        6.66666667,  6.75      ,  6.83333333,  6.91666667,  7.        ,
        7.08333333,  7.16666667,  7.25      ,  7.33333333,  7.41666667,
        7.5       ,  7.58333333,  7.66666667,  7.75      ,  7.83333333,
        7.91666667,  8.        ,  8.08333333,  8.16666667,  8.25      ,
        8.33333333,  8.41666667,  8.5       ,  8.58333333,  8.66666667,
        8.75      ,  8.83333333,  8.91666667,  9.        ,  9.08333333,
        9.16666667,  9.25      ,  9.33333333,  9.41666667,  9.5       ,
        9.58333333,  9.66666667,  9.75      ,  9.83333333,  9.91666667,
       10.        , 10.08333333, 10.16666667, 10.25      , 10.33333333,
       10.41666667, 10.5       , 10.58333333, 10.66666667, 10.75      ,
       10.83333333, 10.91666667, 11.        , 11.08333333, 11.16666667,
       11.25      , 11.33333333, 11.41666667, 11.5       , 11.58333333,
       11.66666667, 11.75      , 11.83333333, 11.91666667, 12.        ,
       12.08333333, 12.16666667, 12.25      , 12.33333333, 12.41666667,
       12.5       , 12.58333333, 12.66666667, 12.75      , 12.83333333,
       12.91666667, 13.        , 13.08333333, 13.16666667, 13.25      ,
       13.33333333, 13.41666667, 13.5       , 13.58333333, 13.66666667,
       13.75      , 13.83333333, 13.91666667, 14.        , 14.08333333,
       14.16666667, 14.25      , 14.33333333, 14.41666667, 14.5       ,
       14.58333333, 14.66666667, 14.75      , 14.83333333, 14.91666667,
       15.        , 15.08333333, 15.16666667, 15.25      , 15.33333333,
       15.41666667, 15.5       , 15.58333333, 15.66666667, 15.75      ,
       15.83333333, 15.91666667, 16.        , 16.08333333, 16.16666667,
       16.25      , 16.33333333, 16.41666667, 16.5       , 16.58333333,
       16.66666667, 16.75      , 16.83333333, 16.91666667, 17.        ,
       17.08333333, 17.16666667, 17.25      , 17.33333333, 17.41666667,
       17.5       , 17.58333333, 17.66666667, 17.75      , 17.83333333,
       17.91666667, 18.        , 18.08333333, 18.16666667, 18.25      ,
       18.33333333, 18.41666667, 18.5       , 18.58333333, 18.66666667,
       18.75      , 18.83333333, 18.91666667, 19.        , 19.08333333,
       19.16666667, 19.25      , 19.33333333, 19.41666667, 19.5       ,
       19.58333333, 19.66666667, 19.75      , 19.83333333, 19.91666667,
       20.        , 20.08333333, 20.16666667, 20.25      , 20.33333333,
       20.41666667, 20.5       , 20.58333333, 20.66666667, 20.75      ,
       20.83333333, 20.91666667, 21.        , 21.08333333, 21.16666667,
       21.25      , 21.33333333, 21.41666667, 21.5       , 21.58333333,
       21.66666667, 21.75      , 21.83333333, 21.91666667, 22.        ,
       22.08333333, 22.16666667, 22.25      , 22.33333333, 22.41666667,
       22.5       , 22.58333333, 22.66666667, 22.75      , 22.83333333,
       22.91666667, 23.        , 23.08333333, 23.16666667, 23.25      ,
       23.33333333, 23.41666667, 23.5       , 23.58333333, 23.66666667,
       23.75      , 23.83333333, 23.91666667])

        t *= 3600
        y0 = np.array([self.temperature_atmosphere[0]])
        y0 = np.array([260])
        print(y0.shape)
        temperature = solve_ivp(self.temperature_time_derivative, y0=y0, t_span=[t[0], t[-1]], t_eval=t, args=[t]).y.T
        return temperature, t
    
    def plot_temp_time(self):
        """
        Plots the temperature of the battery over time.
        """
        y, t = self.solve_temperature_time()
        plt.plot(t, y)
        plt.xlabel('t')
        plt.ylabel('T(t)')
        plt.show()
        pass



    def solve_equilibrium_temperature(self):
        """
        Solves the heat balance equation to find the equilibrium temperature 
        of the battery.

        Returns:
            The equilibrium temperature of the battery [K].
        """

        x = Symbol('x')
        temperature_equilibrium_solutions = solve(self.heat_rate_balance(x), x)
        return temperature_equilibrium_solutions[1]


def main1():
    battery = BatteryHeatTransfer()
    equilibrium_temp = battery.solve_equilibrium_temperature()
    print(f"Equilibrium temperature of the battery: {equilibrium_temp}")

def main2():
    battery = BatteryHeatTransfer()
    equilibrium_temp = battery.plot_temp_time()

if __name__ == "__main__":
    main2()
