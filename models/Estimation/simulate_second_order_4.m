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

% % TODO, easier direct diff of digital tf?
% [num_theta, denom_theta] = tfdata(sysc_theta);
% sysc_omega = tf([num_theta, 0], denom_theta);
% sysd_omega = c2d(sysc_omega, Ts, 'tustin');
% [b_omega, a_omega] = tfdata(sysd_omega);
% 
% sysc_alpha = tf([num_theta, 0, 0], denom_theta);
% sysd_alpha = c2d(sysc_alpha, Ts, 'tustin');
% [b_alpha, a_alpha] = tfdata(sysd_alpha);


theta_filter_rad = filter(b_theta, a_theta, theta_obs_rad);
% omega_filter_rs = filter(b_omega, a_omega, theta_obs_rad);
% alpha_filter_rss = filter(b_alpha, a_alpha, theta_obs_rad);

% diverges, shit
subplot(3, 1, 1);
plot(time, theta_filter_rad);
hold on;
plot(time, theta_obs_rad);
legend('Estimated angle', 'Actual angle');
title('State Estimation using second order tracking loop, IIR');
xlabel('Time [s]');
ylabel('$\hat{\theta}$', 'Interpreter','latex');

subplot(3, 1, 2);
plot(time, omega_filter_rs);
hold on;
plot(time, velocity_profile_rs);
legend('Estimated velocity', 'Actual Velocity');
xlabel('Time [s]');
ylabel('$\hat{\omega}$', 'Interpreter','latex');


subplot(3, 1, 3);
plot(time, alpha_filter_rss);
hold on;
plot([0, .49999, 0.50001, 1.0], 2 * max_speed_rs * [1, 1, -1, -1]);
ylim(3 * max_speed_rs * [-1, 1]);
legend('Estimated acceleration', 'Actual acceleration');

xlabel('Time [s]');
ylabel('$\hat{\alpha}$', 'Interpreter','latex');


