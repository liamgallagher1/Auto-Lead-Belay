clear;
load('logs/one_dir_est_sys.mat')

A = est_sys.A;
B = est_sys.B;
C = est_sys.C;
D = est_sys.D;

%u = K(xd)(x ? xd) + ud


Q = [q_spool_rad, 0; 
    0           , q_vel];
