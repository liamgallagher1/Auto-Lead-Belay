function [A, B, C, D] = upper_pulley_dynamics_expanded(a22, b21, b22, c2, d2, Ts)
%UPPER_PULLEY_DYNAMICS Gray Box Modeling of uppey pulley dyanmics
% TODO account for Coulomb friction

A = [0, 1 ; 
    0, a22];

B = [0, 0; 
    b21, b22];

C = [1, 0; 
    c2, 0];

D = [0, 0;
    d2, 0];

end