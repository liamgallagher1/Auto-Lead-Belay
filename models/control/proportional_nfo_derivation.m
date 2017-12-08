clear;
% Derivation of control of our design. 

% Inertia compensating approach. 
% Our device will have an inertia, J_r, but we want the climber too feel 
% like it has some lesser inertia, J_f. 

syms J_i J_o r_o m_o k num_wings omega T_m T_m_n alpha J_f

r_s = m_o * omega^2 * r_o / (k - m_o * omega^2);

J_r = J_i * num_wings * (J_o  + m_o * (r_o + r_s)^2);

T_m_n = (J_r * alpha - T_m) * (J_r / J_f - 1);
T_m_n = simplify(T_m_n, 500);