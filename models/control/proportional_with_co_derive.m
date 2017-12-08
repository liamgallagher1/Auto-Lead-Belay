clear;

syms T_m m omega r_dot F_c R J_i n m_o r J_o J_f

alpha = (T_m - n * 2 * m_o * omega * r_dot + F_c * R) / (J_i + n * (m_o * r^2 + J_o));

T_m = solve(F_c * R == alpha * J_f, T_m);
% leads to 
% T_m = 2*m_o*n*omega*r_dot - F_c*R + (F_c*R*(J_i + n*(m_o*r^2 + J_o)))/J_f
