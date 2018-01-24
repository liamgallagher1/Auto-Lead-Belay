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
Ts = 0.001; % 1 ms
sample_freq_hz = 1 / Ts;

% elliptic filter
passband_freq_hz = 25;
normalized_freq_rad = .6;% 2 * passband_freq_hz / sample_freq_hz; 


order = 6;
% TODO learn what these mean
passband_ripple_db = 5; 
stopband_attenuation_db = 50;

% get chebysvel type 2 digital filter
% filter options: https://www.mathworks.com/help/signal/ug/iir-filter-design.html
[b_theta,a_theta] =...
    cheby2(order, stopband_attenuation_db, normalized_freq_rad);

% Make into continuous system
sysd_theta =  tf(a_theta, b_theta, Ts,'variable','z^-1');
sysc_theta = d2c(sysd_theta, 'tustin');
freqz(b_theta, a_theta)

[num_theta, denom_theta] = tfdata(sysc_theta);
sysc_omega = tf([num_theta, 0], denom_theta);
sysd_omega = c2d(sysc_omega, Ts, 'tustin');

sysc_alpha = tf([num_theta, 0, 0], denom_theta);
sysd_alpha = c2d(sysc_alpha, Ts, 'tustin');



%theta_hat / theta
sys_theta = tf([k_p / k_i, 1], ...
         [1 / k_i, k_p / k_i, 1]);
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
% The constant difference equation terms to use
[num_theta, denom_theta] = tfdata(sysd_theta);
num_theta = num_theta{1}; denom_theta = denom_theta{1};
[num_omega, denom_omega] = tfdata(sysd_omega);
num_omega = num_omega{1}; denom_omega = denom_omega{1};
[num_alpha, denom_alpha] = tfdata(sysd_alpha);
num_alpha = num_alpha{1}; denom_alpha = denom_alpha{1};

bode(sys_theta);
figure;

% Implement the differencing
for i = 4:length(time)
    % Calculate position estimate
    theta_meas_rad(i) = ...
        -denom_theta(2) * theta_meas_rad(i - 1) - denom_theta(3) * theta_meas_rad(i - 2) + ...
        num_theta(1) * theta_obs_rad(i) + num_theta(2) * theta_obs_rad(i -1) + num_theta(3) * theta_obs_rad(i - 2);
    theta_err_rad(i) = theta_obs_rad(i) - theta_meas_rad(i);
    
    % Calculate velocity estimate
    omega_meas_rs(i) = ...
        -denom_omega(2) * omega_meas_rs(i - 1) ...
        + num_omega(1) * theta_err_rad(i) + num_omega(2) * theta_err_rad(i - 1);
    
     % Calculate velocity estimate
    alpha_meas_rss(i) = ...
        -denom_alpha(2) * alpha_meas_rss(i - 1) ...
        + num_alpha(1) * theta_err_rad(i) + num_alpha(2) * theta_err_rad(i - 1);
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


