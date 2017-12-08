function [dydt] = ode_func_with_coriolis_controlled(t, Y)
% ODE_FUNC_NO_CORIOLIS Summary of this function goes here
% Detailed explanation goes here

% TODO change units
J_i = 8.77 * 0.000292639653;%  [m^2 kg] % lb in^2
J_o = 34.4 * 0.000292639653; % lb in^2 
m_o = 1.7 * 0.453592; % [kg]
n = 3;

r_o = 2.5 * 0.0254; % [m] 

% solve for reasonable spring constant
r_max = 5 * r_o; 
omega_max = 5;
k_s = m_o * (r_max / (r_max - r_o)) * omega_max^2;

theta = Y(1);
theta_dot = Y(2);
r_s = Y(3);
r_dot = Y(4);
T_m = Y(5);

dydt = zeros(4, 1);
dydt(1) = theta_dot;
dydt(3) = r_dot;

% Acceleration in the radial direction. 
% TODO add limits
if (r_o + r_s >= r_max)
    r_dot = 0;
    a_r = 0;
    r_s = r_max - r_o;
    dydt(3) = r_dot;
    dydt(4) = a_r;
else
    a_r = (m_o * theta_dot^2 * (r_o + r_s) - k_s * r_s) / m_o; 
    dydt(4) = a_r;
end

% get radial acceleration
T_m = torque_signal(t);

% Some parallel axis theorem stuff going down
J_outer = n * (m_o * (r_o + r_s)^2 + J_o);


% Neglecting coriolis force, but its fine
alpha = (T_m - 2 * n * (r_o + r_s) * m_o * theta_dot * r_dot) / ...
    (J_i + n * m_o * (r_o + r_s)^2 + J_outer);
dydt(2) = alpha;

end

