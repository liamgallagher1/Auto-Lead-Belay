clear;
clc;
syms s z T x k_p k_i

% Design a second order tracking loop

% x_hat = 1/s v_hat
% v_hat = (ki/s + kp) (x - x_hat)

% Definition from https://www.embeddedrelated.com/showarticle/530.php
x_hat = ((k_p / k_i) * s + 1) /...
        (1 / k_i^2 * s^2 + k_p / k_i * s + 1) * x;
    
    
k_i = 900; k_p = 40; % Idk
Ts = 0.001; % 1 ms  
     
%theta_hat / theta
sys_theta = tf([k_p / k_i, 1], ...
         [1 / k_i^2, k_p / k_i, 1]);
% Note that this can be done better in terms of x and x_hat 

% omega_hat / (theta - theta_hat)
sys_omega = tf([k_p, k_i], ...
               [1, 0]);
% alpha_hat / (theta - theta_hat)
sys_alpha = tf([k_p, k_i], ...
               1);
     
sysd_theta = c2d(sys_theta, Ts, 'tustin');
sysd_omega = c2d(sys_omega, Ts, 'tustin');
sysd_alpha = c2d(sys_alpha, Ts, 'tustin');

[num_theta, denom_theta] = tfdata(sysd_theta);
num_theta = num_theta{1}; denom_theta = denom_theta{1};
[num_omega, denom_omega] = tfdata(sysd_theta);
num_omega = num_omega{1}; denom_omega = denom_omega{1};
[num_alpha, denom_alpha] = tfdata(sysd_theta);
num_alpha = num_alpha{1}; denom_alpha = denom_alpha{1};



