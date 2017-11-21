function [tau_max, P_max, omega_max] = motor_requirements(feed_length_m, time_s, ...
    r2, r1, l, material)
% Simple model of motor requirements just to spin a spool.
% TODO add requirements with force considerations only. 

% Max linear rope speed [m/s]
v_max = 2 * feed_length_m / time_s;
omega_max = v_max / r2; 
a_max = v_max / (time_s / 2); 
alpha_max = a_max / r2;

[J_spool, ~] = spool_inertia_calc(r2, r1, l, material);
tau_max = alpha_max * J_spool;
P_max = omega_max * tau_max;

end

