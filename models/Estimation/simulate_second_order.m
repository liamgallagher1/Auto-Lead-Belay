clear;

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

% Estimator constants
k_i = 900; k_p = 40;
Ts = 0.00001; % 1 ms
   
sys_theta = tf([k_p / k_i, 1], ...
         [1 / k_i^2, k_p / k_i, 1]);
% Note that this can be done better in terms of x and x_hat     
sys_omega = tf([k_p / k_i, 1, 1], ...
         [1 / k_i^2, k_p / k_i, 1]);
     
sys_alpha = tf([k_p / k_i, 1, 1, 1], ...
         [1 / k_i^2, k_p / k_i, 1]);

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

% Implement the differencing
for i = 4:length(time)
    % Calculate position estimate
    theta_meas_rad(i) = ...
        -denom_theta(2) * theta_meas_rad(i - 1) - denom_theta(3) * theta_meas_rad(i - 2) + ...
        num_theta(1) * theta_obs_rad(i) + num_theta(2) * theta_obs_rad(i -1) + num_theta(3) * theta_obs_rad(i - 2);
    
    % Calculate velocity estimate
    omega_meas_rs(i) = ...
        -denom_omega(2) * omega_meas_rs(i - 1) - denom_omega(3) * omega_meas_rs(i - 2) + ...
        num_omega(1) * theta_obs_rad(i) + num_omega(2) * theta_obs_rad(i - 1) + num_omega(3) * theta_obs_rad(i - 2);
    
     % Calculate velocity estimate
    alpha_meas_rss(i) = ...
        -denom_alpha(2) * alpha_meas_rss(i - 1) - denom_alpha(3) * alpha_meas_rss(i - 2) -  denom_alpha(4) * alpha_meas_rss(i - 3) +...
        num_alpha(1) * theta_obs_rad(i) + num_alpha(2) * theta_obs_rad(i - 1) + num_alpha(3) * theta_obs_rad(i - 2) + + num_alpha(4) * theta_obs_rad(i - 3);
    
    % Calculate acceleration estimate
end

% diverges, shit
subplot(3, 1, 1);
plot(time, theta_meas_rad);
subplot(3, 1, 2);
plot(time, omega_meas_rs);
subplot(3, 1, 3);
plot(time, alpha_meas_rss);

