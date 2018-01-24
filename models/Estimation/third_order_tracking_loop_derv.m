clear;
clc;
syms s z T x k_p k_i

% Design a second order tracking loop

% x_hat = 1/s v_hat
% v_hat = 1/s a_hat
% a_hat = (ki/s + kp + kd s) (x - x_hat)
% leads to (TODO double check)
% x_hat = (kd s^2 + kp s + ki) / (s^3 + kd s^2 + kp s + ki) x

% Definition modified from https://www.embeddedrelated.com/showarticle/530.php
    
k_i = 900; k_p = 40; k_d = 1; % Idk
Ts = 0.001; % 1 ms  
     
sys_theta = tf([k_d, k_p, k_i], ...
               [1, k_d, k_p, k_i]);
% omega_hat / (theta - theta_hat)
sys_omega = tf([k_d, k_p, k_i], ...
               [1, 0, 0]);
% alpha_hat / (theta - theta_hat)
sys_alpha = tf([k_d, k_p, k_i], ...
               [1, 0]);
sysd_theta = c2d(sys_theta, Ts, 'tustin');
sysd_omega = c2d(sys_omega, Ts, 'tustin');
sysd_alpha = c2d(sys_alpha, Ts, 'tustin');
[num_theta, denom_theta] = tfdata(sysd_theta);
num_theta = num_theta{1}; denom_theta = denom_theta{1};
[num_omega, denom_omega] = tfdata(sysd_theta);
num_omega = num_omega{1}; denom_omega = denom_omega{1};
[num_alpha, denom_alpha] = tfdata(sysd_theta);
num_alpha = num_alpha{1}; denom_alpha = denom_alpha{1};



