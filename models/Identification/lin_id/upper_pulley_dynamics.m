function [A, B, C, D] = upper_pulley_dynamics(a_22, b_2, c_22, d_2, Ts)
%UPPER_PULLEY_DYNAMICS Gray Box Modeling of uppey pulley dyanmics
% TODO account for Coulomb friction

A = [0, 1; 0, a_22];
B = [0; b_2];

C = [1, 0; 0, c_22];
D = [0; d_2];
end