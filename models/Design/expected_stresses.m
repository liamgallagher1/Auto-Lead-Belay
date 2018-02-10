function [bending_stress, hoop_stress, torsional_shear] = ...
    expected_stresses(r2, r1, length, material)
% For a spool of a given geometry, outputs the maxium expected stress from 
% three prominant modes. 
% Makes assumptions about expected max forces. 

%% Assumed forces, pressures, torsions

F_max = 8500; % [N] Static
N = 1; % number of rope wraps

rope_diam = 0.01; % [m] 
P_max = F_max / (pi * 2 * r2 * N * rope_diam); % [Pa]

[~, ~, max_r] = spool_inertia_calc(r2, r1, length, material);
T_max = 3.16 * 10^3 * max_r; % Max torque from expected forces


%% Bending Calculations

% Area moment of inertia, I_x, not I_z [m^4]
I = pi / 4 * (r2^4 - r1^4); 
M = F_max * length / 2; % Max moment [N m]
bending_stress = M * r2 / I; 

%% Compression Calculation

% This might require a double check
hoop_stress = (r2^2 + r1^2) / (r2^2 - r1^2) * P_max; 

%% Torsional Calculations

% Polar moment, I_z, not I_x [m^4]
J = pi / 2 * (r2^4 - r1^4); 
torsional_shear = T_max * r2 / J; 

% TODO shear calculations but not really it won't shear.
end

