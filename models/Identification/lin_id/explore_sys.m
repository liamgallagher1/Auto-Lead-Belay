clear;
%load('logs/one_dir_est_sys.mat')
%load('logs/sigmoid_sys_est.mat');
load('logs/sigmoid_sys_est_2.mat'); 

Ts = 0.0005;


A = est_sys.A;
B = est_sys.B;
C = est_sys.C;
D = est_sys.D;

friction_term = B(2, 2);
v_offset = friction_term / B(2, 1);

B = B(:, 1);
D = D(:, 2);


%u = K(xd)(x ? xd) + ud

% cost of a meter squared
q_rope_m = 1;
% Cost of a meter per second squared
q_rope_ms = 0; % 10;
% cost of a volt squared
r_a_2 = 1;

spool_radius = 0.09;

q_spool_rad = q_rope_m / spool_radius^2;
q_spool_vel = q_rope_ms / spool_radius^2;

% Controller Mats
Q = [q_spool_rad, 0; 
    0           , q_spool_vel];
R = r_a_2;

% continuous system
sys = ss(A,B,C,D);
sys_d = c2d(sys, Ts);


% Get our gains, lets go
K = dlqr(sys_d.A, sys_d.B, Q, R);
