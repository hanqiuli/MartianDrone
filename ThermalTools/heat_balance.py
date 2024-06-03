# doi:10.1088/1757-899X/1226/1/012113

"""
Variables HOT CASE on ground (2m):
Boundary Conditions
----------------
T_gnd = 306K
T_atm_1m = 274K



"""
from sympy.solvers import solve
from sympy import Symbol

# -------------HOT CASE----------------
temp_atm = 269
temp_gnd = 292.79

i_s = 646 # MAX 646
absorptivity = 0.2
f_sr = 1
area_top = 1.5

albedo = 0.4

temp_eff = 209.8
em_mars = 0.65
sigma = 5.6704E-08

q_electronics = 200
q_motor = 0.3*7300

h = 3
area_total = 4

f_re = 1
em_drone = 0.85

n_legs = 0
R_gnd = 0.049
R_susp = 1.052

# ----------------MSL VERIFICATION/VALIDATION----------------
# temp_atm = 270.73
# temp_gnd = 292.79

# i_s = 587.424
# absorptivity = 0.2
# f_sr = 1
# area_top = 1.598

# albedo = 0.4

# temp_eff = 209.8
# em_mars = 0.65
# sigma = 5.6704E-08

# q_electronics = 301.988
# q_motor = 2000

# h = 1
# area_total = 6.39

# f_re = 1
# em_drone = 0.85

# n_legs = 6
# R_gnd = 0.049
# R_susp = 1.052

# ----------------COLD CASE----------------
# temp_atm = 173
# temp_gnd = 170

# i_s = 0
# absorptivity = 0.2
# f_sr = 1
# area_top = 1.5

# albedo = 0.4

# temp_eff = 209.8
# em_mars = 0.65
# sigma = 5.6704E-08

# q_electronics = 50
# q_motor = 0

# h = 1
# area_total = 4

# f_re = 1
# em_drone = 0.85

# n_legs = 2
# R_gnd = 23.40513869
# R_susp = 0

def heat_external_input(x):
    # heat_sun_irradiation: Q_sun = a * I_s * A_top * F_s/r
    q_sun = absorptivity * i_s * area_top * f_sr
    # heat_albedo: Q_albedo = b * a * I_s * A_bottom * F_s/r
    q_albedo = albedo * absorptivity * i_s * area_top * f_sr
    # heat thermal radiation: Q_IR = J_p * A_bottom; J_p = e * s * T_eff ^4; e ~= 1 - b
    j_p = em_mars * sigma * temp_eff**4
    q_ir = j_p * area_top 

    # heat conduction: Q_cond = n_legs* (T_gnd - T_drone)/(R_gnd - R_susp)
    q_cond = n_legs * (temp_gnd - x)/(R_gnd + R_susp)
    return q_sun + q_albedo + q_ir + q_cond, (q_sun,q_albedo,q_ir,q_cond)

def heat_internal_input():
    # heat electrical systems
    
    # heat propulsion motor
    return q_electronics + q_motor

def heat_convection(x):
    # Qconv = h*(T_drone - T_atm)*Asph
    """
    This function calculates the heat convection from the atmosphere

    Args:
    Asph: is the surface of a sphere equal to that of the drone
    h is the convection coefficient, for Curiosity it was taken as 1 W/(m^2 K) as its hard to evaluate
    
    Returns:

    """
    q_conv = h * (x - temp_atm) * area_total
    return q_conv

def heat_out(x):
    # Qout = s*e*(T_drone ^4 - T_amb ^4)*Asph
    """
    This function calculates the heat convection from the atmosphere

    Args:
    e: emmisivity
    S: Stefan-Boltzmann constant

    Returns:

    """
    q_out = sigma * em_drone *(x**4 - temp_atm**4) * area_total
    return q_out


def heat_balance(x):
    """
    Calculate the balance temperature of the drone

    Returns:
    T_drone
    """
    f = heat_external_input(x)[0] + heat_internal_input() - heat_convection(x) - heat_out(x)
    return f

x = Symbol('x')
sol = solve(heat_balance(x),x)
print(sol)

# temp_rover = 331
# q_ext_inp = heat_external_input(temp_rover)
# q_int_inp = heat_internal_input()
# q_conv = heat_convection(temp_rover) 
# q_out = heat_out(temp_rover)
# print(q_ext_inp, q_int_inp, q_conv, q_out)
# print(heat_balance(temp_rover))