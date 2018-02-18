clear;

% Motor count, pos radians, vel est, accel est, U of t, raw adc, amplified adc
filename = '../../pi/motor_driver/logs/chirp_sys_id33_0_26.csv';
A = csvread(filename, 2, 0);

[N, M] = size(A);
A = A(1:floor(N / 20), :);

time_s = A(:, 1);
time_s = time_s - time_s(1);
time_ns = A(:, 2);
time_ns = time_ns - time_ns(1);
time = time_s + 10^-9 * time_ns;

motor_count = A(:, 3);
motor_pos_rads = A(:, 4);
motor_vel = A(:, 5);
motor_accel = A(:, 6);
duty_cycle = A(:, 7);
raw_adc = A(:, 8);
amplified_adc = A(:, 9);

