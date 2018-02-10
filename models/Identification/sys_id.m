clear;

% Motor count, pos radians, vel est, accel est, U of t, raw adc, amplified adc
filename = '../../pi/motor_driver/logs/chirp_sys_id33_0_26.csv';
A = csvread(filename, 2, 0);

[N, M] = size(A);
A = A(1:floor(N / 10), :);

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


pos_fit = fit(time, motor_pos_rads,'smoothingspline', 'SmoothingParam', 0.994);
%pp = spline(time, motor_pos_rads);
pp = pos_fit.p;
[breaks,coefs,l,k,d] = unmkpp(pp);
dpp = mkpp(breaks,repmat(k-1:-1:1,d*l,1).*coefs(:,1:k-1),d);
vel_spline_est = ppval(dpp, time);

[breaks,coefs,l,k,d] = unmkpp(dpp);
ddpp = mkpp(breaks,repmat(k-1:-1:1,d*l,1).*coefs(:,1:k-1),d);
accel_spline_est = ppval(ddpp, time);

subplot(3, 1, 1);
plot(time, motor_pos_rads);
subplot(3, 1, 2);
%plot(time, motor_vel);
%hold on;
plot(time, vel_spline_est);
subplot(3, 1, 3);
%plot(time, motor_accel);
%hold on;
plot(time, accel_spline_est);
