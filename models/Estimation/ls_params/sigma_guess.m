clear;

Ts = 0.0005;

% Test with avery
%filename = '../../../pi/motor_driver/logs/timestamp_test39_17_12.csv';

% First test without std deque
%filename = '../../../pi/motor_driver/logs/timestamp_test48_16_15.csv';
%filename = '../../../pi/motor_driver/logs/new_stamptest_clean.csv';
% First test using true ISR
filename = '../../../pi/motor_driver/logs/timestamp_test48_17_48.csv';
%filename = '../../../pi/motor_driver/logs/timestamp_test48_17_48.csv';

%time_len = (1146444 / 12);
%time_len =  (1838100 / 12);
%time_len = 153175;
time_len = 1041960 / 12;



% Load data
%A = csvread(filename, 2, 0);%, 2, 0, [2 0, 3, time_len - 2]);
%A = csvread(filename, 2, 0, [2 0, 2, time_len - 2]);
A = csvread(filename, 2, 0); %, 2, 0, [0, 0, 0, time_len - 20]); 
%A = [A(1:time_len)'; A(time_len+1:2*time_len)'; A(2 * time_len + 1:end)'];
% pull time stamps
stamps_s = A(1, 1:end-1);
stamps_ns = A(2, 1:end-1); 
counts = -1 * A(3, 1:end-1);
times = stamps_s - stamps_s(1) + 10^-9 * (stamps_ns - stamps_ns(1));
% Clean data by removing all duplicates, however they happened.
time_diff = diff(times);
times_to_keep = time_diff ~= 0;
times = times(times_to_keep);
counts = counts(times_to_keep);
time_stamps = times;


% save('time_stamps_avery.mat', 'time_stamps', 'counts');

num_segments = 15;

% Therefore
%best_smoothness = 0.9999947;
best_smoothness = 0.85; 

% Smooth it
[coefs, start_indx, end_indx, breaks] = ...
    fast_smooth(times, counts, best_smoothness, num_segments);
%save('smothess_avery_test.csv', 'coefs', 'breaks');
%load('smothess_avery_test.mat');

% counts per revolution
CPR = 2048 * 4;

coefs = coefs * 2 * pi / CPR;

theta_hat = mkpp(breaks, coefs); % in counts


% Go from 3rd order coefficents to 2nd order velocity
first_derv_mat = [3, 0, 0; 0, 2, 0; 0, 0, 1; 0, 0, 0];
% Go from 2nd order to 1st order acceleration
second_derv_mat = [2, 0; 0, 1; 0, 0];

vel = coefs * first_derv_mat;
omega_hat = mkpp(breaks, vel);

accel = vel * second_derv_mat;
alpha_hat = mkpp(breaks, accel);

% Discretize the estimates
time = min(breaks):Ts:max(breaks);
theta_t = ppval(theta_hat, time);
omega_t = ppval(omega_hat, time);
alpha_t = ppval(alpha_hat, time);
save('x_v_a_48_17_48.mat', 'time', 'theta_t', 'omega_t', 'alpha_t');
%load('x_v_a_circ_buff_guess.mat');

% Evaluate the estimate at the corresponding points.
subplot(3, 1, 1); 
plot(time_stamps, counts * 2 * pi / CPR, 'gx'); 
hold on;
plot(time, theta_t);
xlabel('Time [s]');
ylabel('Theta. Estimate [rad]');
legend('Stamped Encoder Counts', 'Smoothing Spline Fit');

subplot(3, 1, 2);
plot(time, omega_t);
xlabel('Time [s]');
ylabel('Vel. Estimate [rad/s]');
legend('Smoothed Spline Fit');

subplot(3, 1, 3);
plot(time, alpha_t);
xlabel('Time [s]');
ylabel('Accel. Estimate [rad/s^2]');
legend('Smoothed Acceleration Estimate');

figure;
% Might be useful
L = length(time); 
Y = fft(alpha_t);
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

Fs = 1/Ts;

f = Fs*(0:(L/2))/L;
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')


