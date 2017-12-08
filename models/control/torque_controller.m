function [T_m_now] = torque_controller(t, J_f, J_i, J_o, m_o, r, r_dot, omega, alpha, T_m_prev)
% Returns a torque that, if output, will make the climber feel a system
% With lesser inertia, J_f, feel?
J_r = J_i + n * (m_o * r^2 + J_o);

F_c_R = J_r * alpha - T_m_prev;

% Something like this? 
T_m_now = n * 2 * m_o * omega * r_dot + F_c_R  * (J_r / J_f - 1);

end