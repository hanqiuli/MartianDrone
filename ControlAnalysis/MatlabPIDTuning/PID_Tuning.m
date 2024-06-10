% Constants
g = 3.71; % m/s^2

% Parameters
mass = 60.0;  % kg
arm_length = 2.0;  % m
torque_thrust_ratio = 0.1;  % m^-1

% Estimated weight
estimated_weight = mass * g;

% Thrust range per motor
hover_thrust_per_motor = estimated_weight / 6;
thrust_min = hover_thrust_per_motor * 0.25;
thrust_max = hover_thrust_per_motor * 1.5;

Jxx = 5.0;  % kg*m^2
Jyy = 5.0;  % kg*m^2
Jzz = 8.0;  % kg*m^2





