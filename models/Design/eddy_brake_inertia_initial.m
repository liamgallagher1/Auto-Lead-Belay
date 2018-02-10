clear;

r_max = 5; % In
r_min = 1; % In
r_o = 2.46; % Relative to the brake arms system

I_in = 8.77; % lb in^2

I_o = 34.44; % lb in^2
m_o = 1.7; % lb

% Parallel axis theorem
I_min = I_in + 3 * ((r_min + r_o)^2 * m_o + I_o);
I_max = I_in + 3 * ((r_max + r_o)^2 * m_o + I_o);

% 1 lb in^2 = 0.000292639653 m^2 kg
I_min = I_min *  0.000292639653; % m^2 kg
I_max = I_max *  0.000292639653; % m^2 kg

