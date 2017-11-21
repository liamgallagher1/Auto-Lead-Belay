clear;

% Motor sizing based on the assumption of bang bang control
s = 1.25; % [m], maximum rope pull length
t = 1;   % [s], ideally the time we should be able to output that in

% Assuming zero dampening. 
t_accel = t / 2;
t_deccel = t / 2;

% s = 0.5 * v_max * t_accel + 0.5 * v_max * t_decel
% s = v_max * (0.5 * (t_accel + t_decel)) 

% Peak rope feed rate
v_max = 2 * s / (t_accel + t_deccel);
accel_lin = v_max / t_accel;


% System caracteristics
total_inertia = 0.1463;
r_min = 0.15; 
accel_ang = accel_lin / r_min;

tau_max = total_inertia * accel_ang;





