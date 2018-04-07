function [A, B, C, D] = upper_pulley_dynamics_vals(k_e, k_m, R, J_sys, b)
%UPPER_PULLEY_DYNAMICS Gray Box Modeling of uppey pulley dyanmics
% TODO account for Coulomb friction

A = [0, 1; 0, -k_e / J_sys - b];
B = [0; k_m / R];

C = [1, 0; 0, -k_e / R];
D = [0; 1/R];

end

