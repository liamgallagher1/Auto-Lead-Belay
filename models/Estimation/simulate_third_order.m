clear;
clf;

%% Simulates the affects of using a second order tracking loop
PPR = 2048;

% Set up hypothetical velocity profile
max_speed_ms = 1.25;
diam_m = 0.25; 
max_speed_rs = 1.25 / pi / diam_m;

time = 0:0.001:1.0;
velocity_profile_rs = 2 * max_speed_rs * [0:0.001:0.499, 0.5:-0.001:0.0];

% encoder counts seen at each time step
counts = tics(time, velocity_profile_rs, PPR);
% corresponding radial position observations from counts
theta_obs_rad = 2 * pi / (4 * PPR) * counts;

% encoder state estimate
theta_meas_rad = zeros(length(time), 1);
theta_meas_rad(1:2) = theta_obs_rad(1:2);
omega_meas_rs = zeros(length(time), 1);
alpha_meas_rss = zeros(length(time), 1);

% difference between observations and measurements
theta_err_rad = theta_obs_rad - theta_meas_rad;

% Estimator constants
k_i = 10000; k_p = 900; k_d = 80;
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
[num_omega, denom_omega] = tfdata(sysd_omega);
num_omega = num_omega{1}; denom_omega = denom_omega{1};
[num_alpha, denom_alpha] = tfdata(sysd_alpha);
num_alpha = num_alpha{1}; denom_alpha = denom_alpha{1};

% Implement the differencing
for i = 4:length(time)
    % Calculate position estimate
    theta_meas_rad(i) = ...
        -denom_theta(2) * theta_meas_rad(i - 1) - denom_theta(3) * theta_meas_rad(i - 2) - denom_theta(4) * theta_meas_rad(i - 3) +...
        num_theta(1) * theta_obs_rad(i) + num_theta(2) * theta_obs_rad(i -1) + num_theta(3) * theta_obs_rad(i - 2) + num_theta(4) * theta_obs_rad(i - 3);
    theta_err_rad(i) = theta_obs_rad(i) - theta_meas_rad(i);
    
    % Calculate velocity estimate
    omega_meas_rs(i) = ...
        -denom_omega(2) * omega_meas_rs(i - 1) -denom_omega(3) * omega_meas_rs(i - 2) ...
        + num_omega(1) * theta_err_rad(i) + num_omega(2) * theta_err_rad(i - 1) + num_omega(3) * theta_err_rad(i - 2);
    
     % Calculate velocity estimate
    alpha_meas_rss(i) = ...
        -denom_alpha(2) * alpha_meas_rss(i - 1) -denom_alpha(3) * alpha_meas_rss(i - 2) ...
        + num_alpha(1) * theta_err_rad(i) + num_alpha(2) * theta_err_rad(i - 1) + num_alpha(3) * theta_err_rad(i - 2);
    % Calculate acceleration estimate
    
end

% diverges, shit
subplot(3, 1, 1);
plot(time, theta_meas_rad);
hold on;
plot(time, theta_obs_rad);
legend('Estimated angle', 'Actual angle');
title('State Estimation using second order tracking loop, IIR');
xlabel('Time [s]');
ylabel('$\hat{\theta}$', 'Interpreter','latex');

subplot(3, 1, 2);
plot(time, omega_meas_rs);
hold on;
plot(time, velocity_profile_rs);
legend('Estimated velocity', 'Actual Velocity');
xlabel('Time [s]');
ylabel('$\hat{\omega}$', 'Interpreter','latex');


subplot(3, 1, 3);
plot(time, alpha_meas_rss);
hold on;
plot([0, .49999, 0.50001, 1.0], 2 * max_speed_rs * [1, 1, -1, -1]);
ylim(3 * max_speed_rs * [-1, 1]);
legend('Estimated acceleration', 'Actual acceleration');

xlabel('Time [s]');
ylabel('$\hat{\alpha}$', 'Interpreter','latex');


